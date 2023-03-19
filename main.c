#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "pot.h"
#include "cup.h"

#ifndef ASPECT
#define ASPECT 2.25
#endif

#ifndef HEIGHT
#define HEIGHT 64
#endif
#define WIDTH ((int)(HEIGHT * ASPECT))

float buffer[WIDTH][HEIGHT][4] = {0};


typedef struct
{
    float x;
    float y;
} Vector2D;

typedef struct
{
    float x;
    float y;
    float z;
} Vector3D;

typedef struct
{
    Vector3D color;
    Vector3D normal;
} SurfaceAttributes;

typedef struct
{
    Vector3D position;
    float rotXZ;
    float rotYZ;
    float focalLength;
} Camera;

typedef struct
{
    Vector3D lightVector;
    Vector3D direct;
    Vector3D ambient;
    Camera camera;
} SceneAttributes;

#define MIN(a, b) (a < b ? a : b)
#define MAX(a, b) (a > b ? a : b)

float min(float a, float b, float c, float d)
{
    return MIN(MIN(a, b), MIN(c, d));
}

float max(float a, float b, float c, float d)
{
    return MAX(MAX(a, b), MAX(c, d));
}

float get_det(Vector2D a, Vector2D b)
{
    return a.x * b.y - a.y * b.x;
}

float dot2(Vector2D a, Vector2D b)
{
    return a.x * b.x + a.y * b.y;
}

float dot(Vector3D a, Vector3D b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector3D cross(Vector3D a, Vector3D b)
{
    Vector3D ret = {a.y * b.z - a.z * b.y,
                    a.z * b.x - a.x * b.z,
                    a.x * b.y - a.y * b.x};
    return ret;
}

Vector3D normalize(Vector3D a)
{
    float l = sqrtf(dot(a, a));
    a.x /= l;
    a.y /= l;
    a.z /= l;
    return a;
}

Vector2D barycentric(Vector2D p, Vector2D a, Vector2D b, Vector2D c)
{
    Vector2D r = {p.x - c.x, p.y - c.y};
    Vector2D A = {a.x - c.x, a.y - c.y};
    Vector2D B = {b.x - c.x, b.y - c.y};

    float det = get_det(A, B);
    if (det == 0)
    {
        Vector2D z = {0, 0};
        return z;
    }

    Vector2D iA = {B.y, -B.x};
    Vector2D iB = {-A.y, A.x};

    Vector2D l = {dot2(iA, r) / det, dot2(iB, r) / det};
    return l;
}

void triangleShader(float bufferColor[4], int x, int y, Vector2D uv, float iza, float izb, float izc, SurfaceAttributes attribs, SceneAttributes scene)
{
    float bufferIz = bufferColor[3];
    float iz = uv.x * iza + uv.y * izb + (1. - uv.x - uv.y) * izc;
    if ((uv.x >= 0) && (uv.y >= 0) && ((uv.x + uv.y) <= 1.) && iz >= bufferIz)
    {
        float illumination = 1.;
        if (dot(attribs.normal, attribs.normal) > 0.)
        {
            illumination = MAX(0., dot(attribs.normal, scene.lightVector)) + .05 * MAX(0., -attribs.normal.y);
        }
        bufferColor[0] = (illumination * scene.direct.x + scene.ambient.x) * attribs.color.x;
        bufferColor[1] = (illumination * scene.direct.y + scene.ambient.y) * attribs.color.y;
        bufferColor[2] = (illumination * scene.direct.z + scene.ambient.z) * attribs.color.z;
        bufferColor[3] = iz;
    }
}

void triangle2D(float buffer[WIDTH][HEIGHT][4], Vector2D a, Vector2D b, Vector2D c, float iza, float izb, float izc, SurfaceAttributes attribs, SceneAttributes scene)
{
    if (iza < 0. || izb < 0. || izc < 0.)
        return;
    // triangle AABB estimation
    int xMin = (int)(MAX(min(a.x, b.x, c.x, (float)(WIDTH - 1)), 0));
    int xMax = (int)(MIN(max(a.x, b.x, c.x, 0), (float)(WIDTH - 1)));
    if (xMax < 0 || xMin >= WIDTH)
        return;

    int yMin = (int)(MAX(min(a.y, b.y, c.y, (float)(HEIGHT - 1)), 0));
    int yMax = (int)(MIN(max(a.y, b.y, c.y, 0), (float)(HEIGHT - 1)));
    if (yMax < 0 || yMin >= HEIGHT)
        return;

    for (int x = xMin; x <= xMax; x++)
    {
        for (int y = yMin; y <= yMax; y++)
        {
            Vector2D p = {x, y};
            Vector2D uv = barycentric(p, a, b, c);
            triangleShader(buffer[x][y], x, y, uv, iza, izb, izc, attribs, scene);
        }
    }
}

Vector3D getWorldPos(Vector3D modelPos, Vector3D origin, float rotXY, float rotXZ, float rotYZ)
{
    modelPos.x = -modelPos.x;
    modelPos.y = -modelPos.y;
    modelPos.z = -modelPos.z;

    Vector2D xy = {modelPos.x, modelPos.y};
    modelPos.x = cosf(rotXY) * xy.x + sinf(rotXY) * xy.y;
    modelPos.y = -sinf(rotXY) * xy.x + cosf(rotXY) * xy.y;

    Vector2D xz = {modelPos.x, modelPos.z};
    modelPos.x = cosf(rotXZ) * xz.x + sinf(rotXZ) * xz.y;
    modelPos.z = -sinf(rotXZ) * xz.x + cosf(rotXZ) * xz.y;

    Vector2D yz = {modelPos.y, modelPos.z};
    modelPos.y = cosf(rotYZ) * yz.x + sinf(rotYZ) * yz.y;
    modelPos.z = -sinf(rotYZ) * yz.x + cosf(rotYZ) * yz.y;

    Vector3D ret = {modelPos.x - origin.x,
                    modelPos.y - origin.y,
                    modelPos.z - origin.z};
    return ret;
}

Vector3D getViewPos(Vector3D wolrdPos, Camera cam)
{
    wolrdPos.x -= cam.position.x;
    wolrdPos.y -= cam.position.y;
    wolrdPos.z -= cam.position.z;

    float rotXZ = cam.rotXZ;
    float rotYZ = cam.rotYZ;

    Vector2D xz = {wolrdPos.x, wolrdPos.z};
    wolrdPos.x = cosf(rotXZ) * xz.x + sinf(rotXZ) * xz.y;
    wolrdPos.z = -sinf(rotXZ) * xz.x + cosf(rotXZ) * xz.y;

    Vector2D yz = {wolrdPos.y, wolrdPos.z};
    wolrdPos.y = cosf(rotYZ) * yz.x + sinf(rotYZ) * yz.y;
    wolrdPos.z = -sinf(rotYZ) * yz.x + cosf(rotYZ) * yz.y;

    Vector3D ret = {wolrdPos.x,
                    wolrdPos.y,
                    wolrdPos.z};
    return ret;
}

Vector3D getViewDir(Vector3D wolrdDir, Camera cam)
{

    float rotXZ = cam.rotXZ;
    float rotYZ = cam.rotYZ;

    Vector2D xz = {wolrdDir.x, wolrdDir.z};
    wolrdDir.x = cosf(rotXZ) * xz.x + sinf(rotXZ) * xz.y;
    wolrdDir.z = -sinf(rotXZ) * xz.x + cosf(rotXZ) * xz.y;

    Vector2D yz = {wolrdDir.y, wolrdDir.z};
    wolrdDir.y = cosf(rotYZ) * yz.x + sinf(rotYZ) * yz.y;
    wolrdDir.z = -sinf(rotYZ) * yz.x + cosf(rotYZ) * yz.y;

    return (Vector3D){wolrdDir.x,
                      wolrdDir.y,
                      wolrdDir.z};
}

Vector3D getNormal(Vector3D a, Vector3D b, Vector3D c)
{
    Vector3D A = {a.x - c.x,
                  a.y - c.y,
                  a.z - c.z};
    Vector3D B = {b.x - c.x,
                  b.y - c.y,
                  b.z - c.z};
    return normalize(cross(B, A));
}

Vector3D project(Vector3D p, const float screenZ)
{
    Vector3D ret = {0, 0, 0};
    p.z = p.z;
    ret.x = (.5 + .5 * screenZ * p.x / p.z) * WIDTH;
    ret.y = (.5 + .5 * screenZ * p.y / p.z) * HEIGHT;
    ret.z = screenZ / p.z;
    return ret;
}

void triangle3D(Vector3D A, Vector3D B, Vector3D C, Vector3D color, SceneAttributes scene)
{
    Vector3D normal = getNormal(A, B, C);

    // Fix normals (ideally this step should not be required, but whatever)
    if (dot(normal, (Vector3D){A.x - scene.camera.position.x, A.y - scene.camera.position.y, A.z - scene.camera.position.z}) < 0.)
        normal = (Vector3D){-normal.x, -normal.y, -normal.z};

    A = getViewPos(A, scene.camera);
    B = getViewPos(B, scene.camera);
    C = getViewPos(C, scene.camera);

    Vector3D Ap = project(A, scene.camera.focalLength);
    Vector3D Bp = project(B, scene.camera.focalLength);
    Vector3D Cp = project(C, scene.camera.focalLength);

    Vector2D a = {Ap.x, Ap.y};
    Vector2D b = {Bp.x, Bp.y};
    Vector2D c = {Cp.x, Cp.y};

    SurfaceAttributes attribs = {color, normal};

    triangle2D(buffer, a, b, c, Ap.z, Bp.z, Cp.z, attribs, scene);
}

void print(float buffer[WIDTH][HEIGHT][4]);

void from_obj(int t)
{

    const float cameraRotateSpeed = .01f;
    float cameraRotXZ = t * cameraRotateSpeed;
    float cameraRotYZ = -0.5;
    const float cameraDistance = 10;
    const Vector3D cameraPosition = {cameraDistance * sinf(cameraRotXZ), cameraDistance * sinf(cameraRotYZ), -cameraDistance * cosf(cameraRotXZ) * cos(cameraRotYZ)};
    const float cameraFocalLength = 3.;

    Camera camera = {cameraPosition, cameraRotXZ, cameraRotYZ, cameraFocalLength};

    Vector3D lightVec = normalize((Vector3D){0, 2, .3});

    SceneAttributes scene = {lightVec, {.999, .999, .995}, {.005, .005, .01}, camera};

    for (int f = 0; f < faces_count_pot; f++)
    {
        const float rotateXY = 0.4;
        const float rotateXZ = 0.0;
        const float rotateYZ = 0.1;
        const Vector3D worldOffset = {-1, 2, 0};

        Vector3D A = (Vector3D){vertices_pot[faces_pot[f][0]][0], vertices_pot[faces_pot[f][0]][1], vertices_pot[faces_pot[f][0]][2]};
        Vector3D B = (Vector3D){vertices_pot[faces_pot[f][1]][0], vertices_pot[faces_pot[f][1]][1], vertices_pot[faces_pot[f][1]][2]};
        Vector3D C = (Vector3D){vertices_pot[faces_pot[f][2]][0], vertices_pot[faces_pot[f][2]][1], vertices_pot[faces_pot[f][2]][2]};

        A = getWorldPos(A, worldOffset, rotateXY, rotateXZ, rotateYZ);
        B = getWorldPos(B, worldOffset, rotateXY, rotateXZ, rotateYZ);
        C = getWorldPos(C, worldOffset, rotateXY, rotateXZ, rotateYZ);

        triangle3D(A, B, C, (Vector3D){1, .5, .5}, scene);

#ifdef STEP_RENDER
        printf("\033[%zuA", (size_t)HEIGHT);
        printf("\033[%zuD", (size_t)WIDTH);
        print(buffer);
#endif
    }

    for (int f = 0; f < faces_count_cup; f++)
    {
        const float rotateXZ = 0.5;
        const float rotateYZ = 0;
        const float rotateXY = 0;
        const Vector3D worldOffset = {2, .5, 0};

        // normals are broken because of inconsistent vertex order
        Vector3D A = (Vector3D){vertices_cup[faces_cup[f][0]][0], vertices_cup[faces_cup[f][0]][1], vertices_cup[faces_cup[f][0]][2]};
        Vector3D B = (Vector3D){vertices_cup[faces_cup[f][1]][0], vertices_cup[faces_cup[f][1]][1], vertices_cup[faces_cup[f][1]][2]};
        Vector3D C = (Vector3D){vertices_cup[faces_cup[f][2]][0], vertices_cup[faces_cup[f][2]][1], vertices_cup[faces_cup[f][2]][2]};

        A = getWorldPos(A, worldOffset, rotateXY, rotateXZ, rotateYZ);
        B = getWorldPos(B, worldOffset, rotateXY, rotateXZ, rotateYZ);
        C = getWorldPos(C, worldOffset, rotateXY, rotateXZ, rotateYZ);

        triangle3D(A, B, C, (Vector3D){1, 1, 1}, scene);

#ifdef STEP_RENDER
        printf("\033[%zuA", (size_t)HEIGHT);
        printf("\033[%zuD", (size_t)WIDTH);
        print(buffer);
#endif
    }
    {
        // floor
        Vector3D A = (Vector3D){-4, 0, -4};
        Vector3D B = (Vector3D){-4, 0, 4};
        Vector3D C = (Vector3D){4, 0, -4};
        triangle3D(A, B, C, (Vector3D){.6, .5, .1}, scene);
#ifdef STEP_RENDER
        printf("\033[%zuA", (size_t)HEIGHT);
        printf("\033[%zuD", (size_t)WIDTH);
        print(buffer);
#endif
        A = (Vector3D){4, 0, 4};
        triangle3D(A, B, C, (Vector3D){.6, .5, .1}, scene);
    }
}
#ifndef FRAMES
#define FRAMES 1
#endif


char *ascii = " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
int asciiLen = 92;

char *color = "\033";

const char *colors[8] = {"[1;30m", "[1;31m", "[1;32m", "[1;33m", "[1;34m", "[1;35m", "[1;36m", "[1;37m"};
const char *bgcolors[8] = {"[1;40m", "[1;41m", "[1;42m", "[1;43m", "[1;44m", "[1;45m", "[1;46m", "[1;47m"};
const int black = 0;
const int red = 1;
const int green = 2;
const int yellow = 3;
const int blue = 4;
const int magenta = 5;
const int cyan = 6;
const int white = 7;

char *defaultColor = "[0m";

float lumaR = 0.2126;
float lumaG = 0.7152;
float lumaB = 0.0722;

float bayer(int x, int y, int level)
{
    if (level <= 0)
    {
        x &= 1;
        y &= 1;
        return x == 0 ? (y == 0 ? 0 : .5f) : (y == 0 ? .75f : .25f);
    }
    return bayer(x / 2, y / 2, level - 1) * .25f + bayer(x, y, 0);
}

Vector2D toCocg(Vector3D rgb)
{
    float co = rgb.x - rgb.z;
    float tmp = rgb.z + co / 2;
    float cg = rgb.y - tmp;
    return (Vector2D){co, cg};
}

void print_color(float buffer[WIDTH][HEIGHT][4])
{
    for (int j = 0; j < HEIGHT; j++)
    {
        for (int i = 0; i < WIDTH; i++)
        {
            float db = bayer(i, j, 4);
            float dr = bayer(i, j, 5);
            float dg = bayer(i, j, 6);

            float r = buffer[i][j][0];
            r = r < 0. ? 0. : r;
            float g = buffer[i][j][1];
            g = g < 0. ? 0. : g;
            float b = buffer[i][j][2];
            b = b < 0. ? 0. : b;
            float luma = (r * lumaR + g * lumaG + b * lumaB);
            if (luma == 0)
            {
                printf(" ");
                continue;
            }

            Vector2D chroma = toCocg((Vector3D){r / luma, g / luma, b / luma});
            Vector2D chromaR = toCocg((Vector3D){1, 0, 0});
            Vector2D chromaG = toCocg((Vector3D){0, 1, 0});
            Vector2D chromaB = toCocg((Vector3D){0, 0, 1});

            Vector2D rg = barycentric(chroma, chromaR, chromaG, chromaB);

            int colorFlag = 0;

            if (rg.x > dr)
            {
                colorFlag |= 1;
            }
            if (rg.y > dg)
            {
                colorFlag |= 2;
            }
            if (1 - rg.x - rg.y > db)
            {
                colorFlag |= 4;
            }
            const int lut[8] = {white, red, green, yellow, blue, magenta, cyan, white};
            const float lum[8] = {1, lumaR, lumaG, lumaG + lumaR, lumaB, lumaB + lumaR, lumaB + lumaG, lumaB + lumaG + lumaR};
            int charColor = lut[colorFlag];

            int charid = (int)(sqrtf(luma) * asciiLen);
            if (charid >= asciiLen)
                charid = asciiLen - 1;
            if (charid < 0)
                charid = 0;
            printf("%s%s%c", color, colors[charColor], ascii[charid]);
        }
        printf("\n%s%s", color, defaultColor);
    }
}

void print_grayscale(float buffer[WIDTH][HEIGHT][4])
{
    for (int j = 0; j < HEIGHT; j++)
    {
        for (int i = 0; i < WIDTH; i++)
        {
            float db = bayer(i, j, 4);
            float dr = bayer(i, j, 5);
            float dg = bayer(i, j, 6);

            float r = buffer[i][j][0];
            r = r < 0. ? 0. : r;
            float g = buffer[i][j][1];
            g = g < 0. ? 0. : g;
            float b = buffer[i][j][2];
            b = b < 0. ? 0. : b;

            float luma = (r * lumaR + g * lumaG + b * lumaB);

            int charid = (int)(sqrtf(luma) * asciiLen);
            if (charid >= asciiLen)
                charid = asciiLen - 1;
            if (charid < 0)
                charid = 0;
            printf("%c", ascii[charid]);
        }
        printf("\n%s%s", color, defaultColor);
    }
}


#define GRAYSCALE 0
#define COLOR 1

#ifndef RENDER_TARGET
#define RENDER_TARGET GRAYSCALE
#endif

void print(float buffer[WIDTH][HEIGHT][4]){
#if RENDER_TARGET == GRAYSCALE
    print_grayscale(buffer);
#else
    print_color(buffer);
#endif
}

void clearBuffer(float buffer[WIDTH][HEIGHT][4])
{
    for (int i = 0; i < WIDTH; i++)
    {
        for (int j = 0; j < HEIGHT; j++)
        {
            buffer[i][j][0] = 0;
            buffer[i][j][1] = 0;
            buffer[i][j][2] = 0;
            buffer[i][j][3] = 0;
        }
    }
}

int main()
{
    for (int t = 0; t < FRAMES; t++)
    {
        from_obj(t);
        print(buffer);
        usleep(1000 * 1000 / 60);
        printf("\033[%zuA", (size_t)HEIGHT);
        printf("\033[%zuD", (size_t)WIDTH);
        clearBuffer(buffer);
    }
}
