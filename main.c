#include <stdio.h>
#include <math.h>

// #include <omp.h>

#ifdef _WIN32
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

#ifndef PX_ASPECT
#define PX_ASPECT 2.21
#endif

#ifndef ASPECT_RATIO
#define ASPECT_RATIO 1
#endif

#ifndef HEIGHT
#define HEIGHT 64
#endif
#ifndef WIDTH
#define WIDTH ((int)(HEIGHT * PX_ASPECT * ASPECT_RATIO))
#endif

#define PERSPECTIVE_UV // enable perspective texture mapping

#define RASTERM_IMPLEMENTATION
#include "rasterm.h"

#include "printImg.h"

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

    // #pragma omp parallel for
    ModelTransform potTransform = {(Vector3D){-1, 2, 0},
                                   0.1,
                                   0.0,
                                   0.4,
                                   (Vector3D){-1, -1, -1}};
    for (int f = 0; f < faces_count_pot; f++)
    {

        Vector3D A = (Vector3D){vertices_pot[faces_pot[f][0]][0], vertices_pot[faces_pot[f][0]][1], vertices_pot[faces_pot[f][0]][2]};
        Vector3D B = (Vector3D){vertices_pot[faces_pot[f][1]][0], vertices_pot[faces_pot[f][1]][1], vertices_pot[faces_pot[f][1]][2]};
        Vector3D C = (Vector3D){vertices_pot[faces_pot[f][2]][0], vertices_pot[faces_pot[f][2]][1], vertices_pot[faces_pot[f][2]][2]};

        A = getWorldPos(A, potTransform);
        B = getWorldPos(B, potTransform);
        C = getWorldPos(C, potTransform);

        triangle3D(buffer, A, B, C, (Vector3D){1, .5, .5}, scene);

#ifdef STEP_RENDER
        printf("\033[%zuA", (size_t)HEIGHT);
        printf("\033[%zuD", (size_t)WIDTH);
        print(buffer);
#endif
    }
    ModelTransform cupTransform = {(Vector3D){2, .5, 0},
                                   0.0,
                                   0.5,
                                   0.0,
                                   (Vector3D){-1, -1, -1}};
    for (int f = 0; f < faces_count_cup; f++)
    {

        // normals are broken because of inconsistent vertex order
        Vector3D A = (Vector3D){vertices_cup[faces_cup[f][0]][0], vertices_cup[faces_cup[f][0]][1], vertices_cup[faces_cup[f][0]][2]};
        Vector3D B = (Vector3D){vertices_cup[faces_cup[f][1]][0], vertices_cup[faces_cup[f][1]][1], vertices_cup[faces_cup[f][1]][2]};
        Vector3D C = (Vector3D){vertices_cup[faces_cup[f][2]][0], vertices_cup[faces_cup[f][2]][1], vertices_cup[faces_cup[f][2]][2]};

        A = getWorldPos(A, cupTransform);
        B = getWorldPos(B, cupTransform);
        C = getWorldPos(C, cupTransform);

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

#define GRAYSCALE 0
#define COLOR 1
#define FULL_COLOR 2

#ifndef RENDER_TARGET
#define RENDER_TARGET GRAYSCALE
#endif

void print(float buffer[WIDTH][HEIGHT][4])
{
#if RENDER_TARGET == GRAYSCALE
    print_grayscale(buffer);
#elif RENDER_TARGET == COLOR
    print_color(buffer);
#else
    print_fc(buffer);
#endif
}

void clearBuffer(float buffer[WIDTH][HEIGHT][4])
{
// #pragma omp parallel for
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
