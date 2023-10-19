#include <stdio.h>
#include <math.h>

#include <omp.h>

#ifndef FRAMES
#define FRAMES 1
#endif // FRAMES

#define GRAYSCALE 0
#define COLOR 1
#define FULL_COLOR 2
#define GDI 3

#ifndef RENDER_TARGET
#define RENDER_TARGET GRAYSCALE
#endif // RENDER_TARGET

#ifdef _WIN32

#if RENDER_TARGET == GDI
#define RENDER_GUI
#include <time.h>
#endif // RENDER_TARGET

#define WIN32_LEAN_AND_MEAN
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
#endif // _WIN32

#include "pot.h"
#include "cup.h"

#ifndef PX_ASPECT
#ifdef RENDER_GUI
#define PX_ASPECT 1
#else
#define PX_ASPECT 2.21
#endif // RENDER_GUI
#endif // PX_ASPECT

#ifndef ASPECT_RATIO
#define ASPECT_RATIO 1
#endif // ASPECT_RATIO

#ifndef HEIGHT
#define HEIGHT 64
#endif // HEIGHT
#ifndef WIDTH
#define WIDTH ((int)(HEIGHT * PX_ASPECT * ASPECT_RATIO))
#endif // WIDTH

#if HEIGHT * HEIGHT * 4 * 4 > 500000
// automatically enable dynamic alloc if stack size is likely to be exceeded
#define DYNAMIC_ALLOC
#endif // HEIGHT

#define PERSPECTIVE_UV // enable perspective texture mapping

#define RASTERM_IMPLEMENTATION
#include "rasterm.h"

#include "printImg.h"
#include "parseObj.c"

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

void gouraudFragmentShader(float bufferColor[4], int x, int y, Vector2D uv, float inverseDepth, SurfaceAttributes attribs, SceneAttributes scene)
{
    float illumination = 1.;

    float w = (1. - uv.x - uv.y);
    Vector3D normal = (Vector3D){
        uv.x * attribs.normalA.x + uv.y * attribs.normalB.x + w * attribs.normalC.x,
        uv.x * attribs.normalA.y + uv.y * attribs.normalB.y + w * attribs.normalC.y,
        uv.x * attribs.normalA.z + uv.y * attribs.normalB.z + w * attribs.normalC.z};

    float normalLength = dot(attribs.normal, attribs.normal);
    if (dot(attribs.normal, attribs.normal) > 0.)
    {
        SCALE_VEC3(normal, 1. / sqrtf(normalLength));
        illumination = MAX(0., dot(normal, scene.lightVector)) + .05 * MAX(0., -normal.y);
    }
    bufferColor[0] = (illumination * scene.direct.x + scene.ambient.x) * attribs.color.x;
    bufferColor[1] = (illumination * scene.direct.y + scene.ambient.y) * attribs.color.y;
    bufferColor[2] = (illumination * scene.direct.z + scene.ambient.z) * attribs.color.z;
    bufferColor[3] = inverseDepth;
}

bool objLoaded = false;
Vector3D *vertices = NULL;
Vector3D *normals = NULL;
Face *faces = NULL;
int faceCount;

void print(float buffer[WIDTH][HEIGHT][4]);
void from_obj(float *buffer, int t)
{
#ifdef LOG_FPS
    clock_t begin = clock();
#endif
    Framebuffer fBuffer = {(float *)buffer, WIDTH, HEIGHT};
    const float cameraRotateSpeed = .01f;
    float cameraRotXZ = t * cameraRotateSpeed;
    float cameraRotYZ = -0.5;
    const float cameraDistance = 15;
    const Vector3D cameraPosition = {cameraDistance * sinf(cameraRotXZ), cameraDistance * sinf(cameraRotYZ), -cameraDistance * cosf(cameraRotXZ) * cos(cameraRotYZ)};
    const float cameraFocalLength = 3.;

    Camera camera = {cameraPosition, cameraRotXZ, cameraRotYZ, cameraFocalLength};

    Vector3D lightVec = normalize((Vector3D){0, 2, .3});

    SceneAttributes scene = {lightVec, (Vector3D){.999, .999, .995}, (Vector3D){.005, .005, .01}, camera};

    ModelTransform potTransform = {(Vector3D){-1, 2, 0},
                                   0.1,
                                   0.0,
                                   0.4,
                                   (Vector3D){-0.15, -0.15, -0.15}};

#pragma omp parallel
    {
        if (objLoaded)
        {
            // printf("%d faces\n", faceCount);
            attachFragmentShader(&gouraudFragmentShader);
            for (int f = 0; f < faceCount; f++)
            {
                Vector3D A = vertices[faces[f].A - 1];
                Vector3D B = vertices[faces[f].B - 1];
                Vector3D C = vertices[faces[f].C - 1];

                Vector3D normal = getNormal(A, B, C);

                A = getWorldPos(A, potTransform);
                B = getWorldPos(B, potTransform);
                C = getWorldPos(C, potTransform);
                SurfaceAttributes attributes = (SurfaceAttributes){
                    (Vector3D){1, .5, .5}, normal,
                    normals[faces[f].An - 1],
                    normals[faces[f].Bn - 1],
                    normals[faces[f].Cn - 1]};
                triangle3D(fBuffer, A, B, C, attributes, scene);
            }
            resetFragmentShader();
        }
        else
        {
#pragma omp for
            for (int f = 0; f < faces_count_pot; f++)
            {

                Vector3D A = (Vector3D){vertices_pot[faces_pot[f][0]][0], vertices_pot[faces_pot[f][0]][1], vertices_pot[faces_pot[f][0]][2]};
                Vector3D B = (Vector3D){vertices_pot[faces_pot[f][1]][0], vertices_pot[faces_pot[f][1]][1], vertices_pot[faces_pot[f][1]][2]};
                Vector3D C = (Vector3D){vertices_pot[faces_pot[f][2]][0], vertices_pot[faces_pot[f][2]][1], vertices_pot[faces_pot[f][2]][2]};

                A = getWorldPos(A, potTransform);
                B = getWorldPos(B, potTransform);
                C = getWorldPos(C, potTransform);

                SurfaceAttributes attributes = {0};
                attributes.color = (Vector3D){1, .5, .5};
                attributes.normal = getNormal(A, B, C);
                attributes.normalA = attributes.normal;
                attributes.normalB = attributes.normal;
                attributes.normalC = attributes.normal;
                triangle3D(fBuffer, A, B, C, attributes, scene);

#ifdef STEP_RENDER
                printf("\033[%zuA", (size_t)HEIGHT);
                printf("\033[%zuD", (size_t)WIDTH);
                print(buffer);
#endif // STEP_RENDER
            }
        }
        ModelTransform cupTransform = {(Vector3D){2, .5, 0},
                                       0.0,
                                       0.5,
                                       0.0,
                                       (Vector3D){-1, -1, -1}};
#pragma omp for
        for (int f = 0; f < faces_count_cup; f++)
        {

            // normals are broken because of inconsistent vertex order
            Vector3D A = (Vector3D){vertices_cup[faces_cup[f][0]][0], vertices_cup[faces_cup[f][0]][1], vertices_cup[faces_cup[f][0]][2]};
            Vector3D B = (Vector3D){vertices_cup[faces_cup[f][1]][0], vertices_cup[faces_cup[f][1]][1], vertices_cup[faces_cup[f][1]][2]};
            Vector3D C = (Vector3D){vertices_cup[faces_cup[f][2]][0], vertices_cup[faces_cup[f][2]][1], vertices_cup[faces_cup[f][2]][2]};

            A = getWorldPos(A, cupTransform);
            B = getWorldPos(B, cupTransform);
            C = getWorldPos(C, cupTransform);

            SurfaceAttributes attributes = {0};
            attributes.color = (Vector3D){1, 1, 1};
            attributes.normal = getNormal(A, B, C);
            // Fix normals (ideally this step should not be required, but whatever)
            attributes.normalA = attributes.normal;
            attributes.normalB = attributes.normal;
            attributes.normalC = attributes.normal;
            triangle3D(fBuffer, A, B, C, attributes, scene);

#ifdef STEP_RENDER
            printf("\033[%zuA", (size_t)HEIGHT);
            printf("\033[%zuD", (size_t)WIDTH);
            print(buffer);
#endif // STEP_RENDER
        }
    }
    {
        attachFragmentShader(&chessboardShader);
        // floor
        Vector3D A = (Vector3D){-4, 0, 4};
        Vector3D B = (Vector3D){4, 0, -4};
        Vector3D C = (Vector3D){-4, 0, -4};

        SurfaceAttributes attributes = {0};
        attributes.color = (Vector3D){.6, .5, .1};
        attributes.normal.y = attributes.normalA.y = attributes.normalB.y = attributes.normalC.y = 1;
        triangle3D(fBuffer, A, B, C, attributes, scene);
#ifdef STEP_RENDER
        printf("\033[%zuA", (size_t)HEIGHT);
        printf("\033[%zuD", (size_t)WIDTH);
        print(buffer);
#endif // STEP_RENDER
        C = (Vector3D){4, 0, 4};
        triangle3D(fBuffer, A, B, C, attributes, scene);
        resetFragmentShader();
    }
#ifdef LOG_FPS
    clock_t end = clock();
    double ft = (double)(end - begin) / CLOCKS_PER_SEC;
    static double maxft = 0;
    static double avgft = 0;
    if (ft > maxft)
        maxft = ft;
    maxft = .98 * maxft + .02 * ft;
    avgft = t == 0 ? ft : .98 * avgft + .02 * ft;
    printf("current framerate = %f fps\tsmooth_avg = %f fps\tsmmoth_low = %f fps\n", 1. / ft, 1. / avgft, 1. / maxft);
#endif // RENDER_GUI
}

void print(float buffer[WIDTH][HEIGHT][4])
{
#if RENDER_TARGET == GRAYSCALE
    print_grayscale(buffer);
#elif RENDER_TARGET == COLOR
    print_color(buffer);
#else
    print_fc(buffer);
#endif // RENDER_TARGET
}

void clearBuffer(Framebuffer buffer)
{
    // #pragma omp parallel for
    for (int i = 0; i < buffer.width; i++)
    {
        for (int j = 0; j < buffer.height; j++)
        {
            float *fragment = framebufferAt(buffer, i, j);
            fragment[0] = 0;
            fragment[1] = 0;
            fragment[2] = 0;
            fragment[3] = 0;
        }
    }
}

void loadObj()
{
    char *path = "teapot.obj";
    faceCount = 0;
    int vertexCount = 0, normalCount = 0;
    if (countObjects(path, &vertexCount, &normalCount, &faceCount))
    {
        vertices = malloc(sizeof(Vector3D) * vertexCount);
        normals = malloc(sizeof(Vector3D) * normalCount);
        faces = malloc(sizeof(Face) * faceCount);

        objLoaded = parseObjects(path, vertices, normals, faces);
        if (objLoaded)
            printf("%s loaded\n", path);
    }
}

void unloadObj()
{
    free(vertices);
    free(normals);
    free(faces);
}

#ifndef RENDER_GUI
int main()
{
    loadObj();
#ifdef DYNAMIC_ALLOC
    float *buffer = malloc(sizeof(float[WIDTH][HEIGHT][4]));
#else
    float buffer[WIDTH][HEIGHT][4];
    clearBuffer((Framebuffer){buffer, WIDTH, HEIGHT});
#endif // DYNAMIC_ALLOC

    for (int t = 0; t < FRAMES; t++)
    {
        from_obj(buffer, t);
        print(buffer);
        usleep(1000 * 1000 / 60);
        printf("\033[%zuA", (size_t)HEIGHT);
        printf("\033[%zuD", (size_t)WIDTH);
        clearBuffer((Framebuffer){buffer, WIDTH, HEIGHT});
    }
#ifdef DYNAMIC_ALLOC
    free(buffer);
#endif // DYNAMIC_ALLOC
    unloadObj();
}

#else

#ifdef _WIN32
float *buffer;

void CreateDIBAndCopyData(HDC hdc, HBITMAP *hBitmap, uint8_t **dibData)
{
    Framebuffer fBuffer = (Framebuffer){buffer, WIDTH, HEIGHT};
    // Create a BITMAPINFO structure for the DIB
    BITMAPINFO bmi;
    ZeroMemory(&bmi, sizeof(BITMAPINFO));
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth = WIDTH;
    bmi.bmiHeader.biHeight = -HEIGHT; // Negative height to make it top-down
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 32; // 32-bit (RGBA)
    bmi.bmiHeader.biCompression = BI_RGB;

    // Create a DIB section
    *hBitmap = CreateDIBSection(hdc, &bmi, DIB_RGB_COLORS, (void **)dibData, NULL, 0);
    // Copy data from your floating-point buffer to the DIB
    for (int y = 0; y < HEIGHT; y++)
    {
        for (int x = 0; x < WIDTH; x++)
        {
            uint8_t *pixel = *dibData + (y * WIDTH + x) * 4;
            float *fragment = framebufferAt(fBuffer, x, y);
            float r = sqrtf(fragment[0]);
            float g = sqrtf(fragment[1]);
            float b = sqrtf(fragment[2]);
            float a = fragment[3];

            pixel[0] = (uint8_t)(b * 255); // Blue
            pixel[1] = (uint8_t)(g * 255); // Green
            pixel[2] = (uint8_t)(r * 255); // Red
            pixel[3] = (uint8_t)(a * 255); // Alpha
        }
    }
}

void PaintWindow(HWND hwnd)
{

    PAINTSTRUCT ps;
    HDC hdc = BeginPaint(hwnd, &ps);

    HBITMAP hBitmap;
    uint8_t *dibData;

    // Create the DIB and copy data from the buffer
    CreateDIBAndCopyData(hdc, &hBitmap, &dibData);

    HDC memDC = CreateCompatibleDC(hdc);
    HBITMAP hOldBitmap = (HBITMAP)SelectObject(memDC, hBitmap);

    // Display the bitmap on the window
    BitBlt(hdc, 0, 0, WIDTH, HEIGHT, memDC, 0, 0, SRCCOPY);

    // Clean up
    SelectObject(memDC, hOldBitmap);
    DeleteObject(hBitmap);
    DeleteDC(memDC);

    EndPaint(hwnd, &ps);
}

int frame = 0;

LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    switch (uMsg)
    {
    case WM_PAINT:
    {
        PaintWindow(hwnd);
        return 0;
    }
    case WM_CLOSE:
    {
        PostQuitMessage(0);
        return 0;
    }
    case WM_TIMER:
    {
        // Update the buffer for the current frame
        clearBuffer((Framebuffer){buffer, WIDTH, HEIGHT});
        from_obj(buffer, ++frame);

        // Repaint the window to display the updated frame
        InvalidateRect(hwnd, NULL, FALSE);
        UpdateWindow(hwnd);

        // Increment the frame index
        return 0;
    }
    }
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

#ifndef FPS
#define FPS 60
#endif // FPS

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    loadObj();
    buffer = malloc(sizeof(float[WIDTH][HEIGHT][4]));
    from_obj(buffer, 0);
    // Register the window class
    WNDCLASSEX wc = {sizeof(WNDCLASSEX), CS_CLASSDC, WindowProc, 0L, 0L, GetModuleHandle(NULL), NULL, NULL, NULL, NULL, ("YourWindowClass"), NULL};
    RegisterClassEx(&wc);

    // Create the window
    HWND hwnd = CreateWindowEx(0, wc.lpszClassName, "Image Display", WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, WIDTH, HEIGHT,
                               NULL,      // Parent window
                               NULL,      // Menu
                               hInstance, // Instance handle
                               NULL       // Additional application data
    );

    // Show the window
    ShowWindow(hwnd, nCmdShow);
    UpdateWindow(hwnd);
    SetTimer(hwnd, 1, 1000 / FPS, NULL);

    // Main message loop
    MSG msg;
    while (GetMessage(&msg, NULL, 0, 0))
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    // Clean up and exit
    UnregisterClass(wc.lpszClassName, wc.hInstance);
    unloadObj();
    return 0;
}
#endif // _WIN32
#endif // RENDER_GUI