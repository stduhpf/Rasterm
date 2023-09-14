#include <stdio.h>
#include <math.h>

#ifdef WIN32
#include <windows.h>

void usleep(__int64 usec)
{
    HANDLE timer;
    LARGE_INTEGER ft;

    ft.QuadPart = -(10 * usec); // Convert to 100 nanosecond interval, negative value indicates relative time

    timer = CreateWaitableTimer(NULL, TRUE, NULL);
    SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
    WaitForSingleObject(timer, INFINITE);
    CloseHandle(timer);
}

#else
#include <unistd.h>
#endif

#include "pot.h"
#include "cup.h"

#ifndef ASPECT
#define ASPECT 2.25
#endif

#ifndef HEIGHT
#define HEIGHT 64
#endif
#ifndef WIDTH
#define WIDTH ((int)(HEIGHT * ASPECT))
#endif

#define PERSPECTIVE_UV // enable perspective texture mapping

#define RASTERM_IMPLEMENTATION
#include "rasterm.h"

void chessboardShader(float bufferColor[4], int x, int y, Vector2D uv, float inverseDepth, SurfaceAttributes attribs, SceneAttributes scene)
{
    float illumination = 1.;
    if (dot(attribs.normal, attribs.normal) > 0.)
    {
        illumination = MAX(0., dot(attribs.normal, scene.lightVector)) + .05 * MAX(0., -attribs.normal.y);
    }
    float texture = (fmodf((uv.x * 4.0f), 1.0f) > .5 ^ fmodf((uv.y * 4.0f), 1.0f) > .5) ? 1. : .01;
    bufferColor[0] = (illumination * scene.direct.x + scene.ambient.x) * attribs.color.x * texture;
    bufferColor[1] = (illumination * scene.direct.y + scene.ambient.y) * attribs.color.y * texture;
    bufferColor[2] = (illumination * scene.direct.z + scene.ambient.z) * attribs.color.z * texture;
    bufferColor[3] = inverseDepth;
}

void print(float buffer[WIDTH][HEIGHT][4]);
void from_obj(float buffer[WIDTH][HEIGHT][4], int t)
{

    const float cameraRotateSpeed = .01f;
    float cameraRotXZ = t * cameraRotateSpeed;
    float cameraRotYZ = -0.5;
    const float cameraDistance = 15;
    const Vector3D cameraPosition = {cameraDistance * sinf(cameraRotXZ), cameraDistance * sinf(cameraRotYZ), -cameraDistance * cosf(cameraRotXZ) * cos(cameraRotYZ)};
    const float cameraFocalLength = 3.;

    Camera camera = {cameraPosition, cameraRotXZ, cameraRotYZ, cameraFocalLength};

    Vector3D lightVec = normalize((Vector3D){0, 2, .3});

    SceneAttributes scene = {lightVec, (Vector3D){.999, .999, .995}, (Vector3D){.005, .005, .01}, camera};

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

        triangle3D(buffer, A, B, C, (Vector3D){1, .5, .5}, scene);

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

        triangle3D(buffer, A, B, C, (Vector3D){1, 1, 1}, scene);

#ifdef STEP_RENDER
        printf("\033[%zuA", (size_t)HEIGHT);
        printf("\033[%zuD", (size_t)WIDTH);
        print(buffer);
#endif
    }
    {
        attachFragmentShader(&chessboardShader);
        // floor
        Vector3D A = (Vector3D){-4, 0, 4};
        Vector3D B = (Vector3D){4, 0, -4};
        Vector3D C = (Vector3D){-4, 0, -4};
        triangle3D(buffer, A, B, C, (Vector3D){.6, .5, .1}, scene);
#ifdef STEP_RENDER
        printf("\033[%zuA", (size_t)HEIGHT);
        printf("\033[%zuD", (size_t)WIDTH);
        print(buffer);
#endif
        C = (Vector3D){4, 0, 4};
        triangle3D(buffer, A, B, C, (Vector3D){.6, .5, .1}, scene);
        resetFragmentShader();
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

void print(float buffer[WIDTH][HEIGHT][4])
{
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
    float buffer[WIDTH][HEIGHT][4];
    memset(buffer, 0, sizeof(buffer));
    // attachFragmentShader(&customFragmentShader);
    for (int t = 0; t < FRAMES; t++)
    {
        from_obj(buffer, t);
        print(buffer);
        usleep(1000 * 1000 / 60);
        printf("\033[%zuA", (size_t)HEIGHT);
        printf("\033[%zuD", (size_t)WIDTH);
        clearBuffer(buffer);
    }
}
