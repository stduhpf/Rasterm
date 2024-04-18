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

#ifdef TEXTURED
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#endif // TEXTURED

// Shaders

// Function to calculate the shading of a chessboard pattern on a surface
void chessboardShader(float bufferColor[4], int x, int y, Vector2D uv, float inverseDepth, SurfaceAttributes attribs, SceneAttributes scene)
{
    // Initialize illumination to 1.0 (full brightness)
    float illumination = 1.;

    // If the surface has a non-zero normal vector...
    if (dot(attribs.normal, attribs.normal) > 0.)
    {
        // Calculate the dot product of the normal and lightVector to get the directional intensity of the lighting on the surface
        illumination = MAX(0., dot(attribs.normal, scene.lightVector)) + .05 * MAX(0., -attribs.normal.y);
    }

    // Calculate a texture value based on the UV coordinates of the surface (creating a chessboard pattern)
    float texture = (fmodf((uv.x * 4.0f), 1.0f) > .5 ^ fmodf((uv.y * 4.0f), 1.0f) > .5) ? 1. : .01;

    // Apply the calculated illumination and texture to the color of the surface, storing the result in bufferColor array
    bufferColor[0] = (illumination * scene.direct.x + scene.ambient.x) * attribs.color.x * texture;
    bufferColor[1] = (illumination * scene.direct.y + scene.ambient.y) * attribs.color.y * texture;
    bufferColor[2] = (illumination * scene.direct.z + scene.ambient.z) * attribs.color.z * texture;

    // Store the inverseDepth in the fourth element of the bufferColor array
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
    if (normalLength > 0.)
    {
        SCALE_VEC3(normal, 1. / sqrtf(normalLength));
        illumination = MAX(0., dot(normal, scene.lightVector)) + .05 * MAX(0., -normal.y);
    }
    bufferColor[0] = (illumination * scene.direct.x + scene.ambient.x) * attribs.color.x;
    bufferColor[1] = (illumination * scene.direct.y + scene.ambient.y) * attribs.color.y;
    bufferColor[2] = (illumination * scene.direct.z + scene.ambient.z) * attribs.color.z;
    bufferColor[3] = inverseDepth;
}

void debugUVs(float bufferColor[4], int x, int y, Vector2D uv, float inverseDepth, SurfaceAttributes attribs, SceneAttributes scene)
{
    float illumination = 1.;

    float w = (1. - uv.x - uv.y);
    Vector2D texture_uv = (Vector2D){
        (uv.x * attribs.uvA.x + uv.y * attribs.uvB.x + w * attribs.uvC.x),
        (uv.x * attribs.uvA.y + uv.y * attribs.uvB.y + w * attribs.uvC.y)};

    float normalLength = dot(attribs.normal, attribs.normal);
    bufferColor[0] = texture_uv.x;
    bufferColor[1] = texture_uv.y;
    bufferColor[2] = 0;
    bufferColor[3] = inverseDepth;
}

int img_width = 0;
int img_height = 0;
unsigned char *texture = NULL;
void texturedGouraudShader(float bufferColor[4], int x, int y, Vector2D uv, float inverseDepth, SurfaceAttributes attribs, SceneAttributes scene)
{
    float illumination = 1.;

    float w = (1. - uv.x - uv.y);
    Vector2D texture_uv = (Vector2D){
        (uv.x * attribs.uvA.x + uv.y * attribs.uvB.x + w * attribs.uvC.x),
        (uv.x * attribs.uvA.y + uv.y * attribs.uvB.y + w * attribs.uvC.y)};
    int X = (int)(texture_uv.x * img_width) % img_width;
    int Y = (int)(texture_uv.y * img_height) % img_height;
    unsigned char *data = texture + 4 * (X + Y * img_width);

    Vector3D normal = (Vector3D){
        uv.x * attribs.normalA.x + uv.y * attribs.normalB.x + w * attribs.normalC.x,
        uv.x * attribs.normalA.y + uv.y * attribs.normalB.y + w * attribs.normalC.y,
        uv.x * attribs.normalA.z + uv.y * attribs.normalB.z + w * attribs.normalC.z};

    float normalLength = dot(attribs.normal, attribs.normal);
    if (normalLength > 0.)
    {
        SCALE_VEC3(normal, 1. / sqrtf(normalLength));
        illumination = MAX(0., dot(normal, scene.lightVector)) + .05 * MAX(0., -normal.y);
    }

    bufferColor[0] = (illumination * scene.direct.x + scene.ambient.x) * ((float)*(data)) / 255.;
    bufferColor[1] = (illumination * scene.direct.y + scene.ambient.y) * ((float)*(data + 1)) / 255.;
    bufferColor[2] = (illumination * scene.direct.z + scene.ambient.z) * ((float)*(data + 2)) / 255.;
    bufferColor[3] = inverseDepth;
}

void depthOnlyShader(float bufferColor[4], int x, int y, Vector2D uv, float inverseDepth, SurfaceAttributes attribs, SceneAttributes scene)
{
    bufferColor[0] = inverseDepth;
    bufferColor[1] = inverseDepth;
    bufferColor[2] = inverseDepth;
    bufferColor[3] = inverseDepth;
}

#ifdef SHADOWMAP_DEMO
#ifndef SHADOW_RES
#define SHADOW_RES 256
#endif // SHADOW_RES
float *shadowBuffer;
void hardShadowChessboardShader(float bufferColor[4], int x, int y, Vector2D uv, float inverseDepth, SurfaceAttributes attribs, SceneAttributes scene)
{
    float w = (1. - uv.x - uv.y);

    FrameBuffer shadowfBuffer = (FrameBuffer){(float *)shadowBuffer, SHADOW_RES, SHADOW_RES};
    float cameraRotXZ = 0;
    float cameraRotYZ = -1.4219;
    const float cameraDistance = 1500;
    const Vector3D cameraPosition = {cameraDistance * sinf(cameraRotXZ), cameraDistance * sinf(cameraRotYZ), -cameraDistance * cosf(cameraRotXZ) * cos(cameraRotYZ)};
    const float cameraFocalLength = 250.;
    Camera shadowCamera = {cameraPosition, cameraRotXZ, cameraRotYZ, cameraFocalLength};

    Vector3D P = {uv.x * attribs.A.x + uv.y * attribs.B.x + w * attribs.C.x,
                  uv.x * attribs.A.y + uv.y * attribs.B.y + w * attribs.C.y,
                  uv.x * attribs.A.z + uv.y * attribs.B.z + w * attribs.C.z};

    Vector3D Pp = vertexShader(P, shadowCamera);
    float sz;
    if (Pp.x < 0 || Pp.y < 0 || Pp.x > 1 || Pp.y > 1)
        sz = Pp.z;
    else
        sz = frameBufferAt(shadowfBuffer, (size_t)(Pp.x * SHADOW_RES), (size_t)(Pp.y * SHADOW_RES))[3];

    float illumination = 1.;
    if (dot(attribs.normal, attribs.normal) > 0.)
    {
        illumination = MAX(0., dot(attribs.normal, scene.lightVector)) + .05 * MAX(0., -attribs.normal.y);
    }
    float texture = (fmodf((uv.x * 4.0f), 1.0f) > .5 ^ fmodf((uv.y * 4.0f), 1.0f) > .5) ? 1. : .01;
    if (fabsf(sz - Pp.z) > .00001)
        illumination = 0;

    bufferColor[0] = (illumination * scene.direct.x + scene.ambient.x) * attribs.color.x * texture;
    bufferColor[1] = (illumination * scene.direct.y + scene.ambient.y) * attribs.color.y * texture;
    bufferColor[2] = (illumination * scene.direct.z + scene.ambient.z) * attribs.color.z * texture;
    bufferColor[3] = inverseDepth;
}
#endif // SHADOWMAP_DEMO
// global variables

bool objLoaded = false;
Vector3D *vertices = NULL;
Vector3D *normals = NULL;
Face *faces = NULL;
Vector2D *uvs = NULL;
int faceCount;

void print(float buffer[WIDTH][HEIGHT][4]);
void from_obj(float *buffer, int t)
{
#ifdef LOG_FPS
    clock_t begin = clock();
#endif // LOG_FPS
    FrameBuffer fBuffer = {(float *)buffer, WIDTH, HEIGHT};
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
    {
        if (objLoaded)
        {
// printf("%d faces\n", faceCount);
#ifdef TEXTURED
            attachFragmentShader(&texturedGouraudShader);
#else
            attachFragmentShader(&gouraudFragmentShader);
#endif // TEXTURED

            attachModelTransform(&potTransform);
            // attachFragmentShader(&debugUVs);
            for (int f = 0; f < faceCount; f++)
            {
                Vector3D A = vertices[faces[f].A - 1];
                Vector3D B = vertices[faces[f].B - 1];
                Vector3D C = vertices[faces[f].C - 1];

                Vector3D normal = getNormal(A, B, C);


                SurfaceAttributes attributes = (SurfaceAttributes){
                    (Vector3D){1, .5, .5},
                    A,
                    B,
                    C,
                    normal,
                    normals[faces[f].An - 1],
                    normals[faces[f].Bn - 1],
                    normals[faces[f].Cn - 1],
                    uvs[faces[f].Auv - 1],
                    uvs[faces[f].Buv - 1],
                    uvs[faces[f].Cuv - 1],
                };
                triangle3D(fBuffer, A, B, C, attributes, scene);
            }
            resetFragmentShader();
        }
        else
        {
            attachModelTransform(&potTransform);
            for (int f = 0; f < faces_count_pot; f++)
            {

                Vector3D A = (Vector3D){vertices_pot[faces_pot[f][0]][0], vertices_pot[faces_pot[f][0]][1], vertices_pot[faces_pot[f][0]][2]};
                Vector3D B = (Vector3D){vertices_pot[faces_pot[f][1]][0], vertices_pot[faces_pot[f][1]][1], vertices_pot[faces_pot[f][1]][2]};
                Vector3D C = (Vector3D){vertices_pot[faces_pot[f][2]][0], vertices_pot[faces_pot[f][2]][1], vertices_pot[faces_pot[f][2]][2]};


                SurfaceAttributes attributes = {0};
                attributes.color = (Vector3D){1, .5, .5};
                attributes.A = A, attributes.B = B, attributes.C = C;
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
        attachModelTransform(&cupTransform);
        for (int f = 0; f < faces_count_cup; f++)
        {

            // normals are broken because of inconsistent vertex order
            Vector3D A = (Vector3D){vertices_cup[faces_cup[f][0]][0], vertices_cup[faces_cup[f][0]][1], vertices_cup[faces_cup[f][0]][2]};
            Vector3D B = (Vector3D){vertices_cup[faces_cup[f][1]][0], vertices_cup[faces_cup[f][1]][1], vertices_cup[faces_cup[f][1]][2]};
            Vector3D C = (Vector3D){vertices_cup[faces_cup[f][2]][0], vertices_cup[faces_cup[f][2]][1], vertices_cup[faces_cup[f][2]][2]};


            SurfaceAttributes attributes = {0};
            attributes.color = (Vector3D){1, 1, 1};
            attributes.A = A, attributes.B = B, attributes.C = C;
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
#ifdef SHADOWMAP_DEMO
        attachFragmentShader(&hardShadowChessboardShader);
#else
        attachFragmentShader(&chessboardShader);
#endif
        // floor
        Vector3D A = (Vector3D){-4, 0, 4};
        Vector3D B = (Vector3D){4, 0, -4};
        Vector3D C = (Vector3D){-4, 0, -4};
        resetModelTransform();
        SurfaceAttributes attributes = {0};
        attributes.color = (Vector3D){.6, .5, .45};
        attributes.A = A, attributes.B = B, attributes.C = C;
        attributes.normal.y = attributes.normalA.y = attributes.normalB.y = attributes.normalC.y = 1;
        triangle3D(fBuffer, A, B, C, attributes, scene);
#ifdef STEP_RENDER
        printf("\033[%zuA", (size_t)HEIGHT);
        printf("\033[%zuD", (size_t)WIDTH);
        print(buffer);
#endif // STEP_RENDER
        C = (Vector3D){4, 0, 4};
        attributes.C = C;
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

#ifdef SHADOWMAP_DEMO
void shadowPass(int t)
{
    attachFragmentShader(&depthOnlyShader);
    FrameBuffer fBuffer = {(float *)shadowBuffer, SHADOW_RES, SHADOW_RES};
    float cameraRotXZ = 0;
    float cameraRotYZ = -1.4219;
    const float cameraDistance = 1500;
    Vector3D lightVec = normalize((Vector3D){0, 2, .3});
    const Vector3D cameraPosition = {cameraDistance * sinf(cameraRotXZ), cameraDistance * sinf(cameraRotYZ), -cameraDistance * cosf(cameraRotXZ) * cos(cameraRotYZ)};
    const float cameraFocalLength = 250.;

    Camera camera = {cameraPosition, cameraRotXZ, cameraRotYZ, cameraFocalLength};

    SceneAttributes scene = {lightVec, (Vector3D){0}, (Vector3D){0}, camera};

    ModelTransform potTransform = {(Vector3D){-1, 2, 0},
                                   0.1,
                                   0.0,
                                   0.4,
                                   (Vector3D){-0.15, -0.15, -0.15}};
    {
        if (objLoaded)
        {

            for (int f = 0; f < faceCount; f++)
            {
                Vector3D A = vertices[faces[f].A - 1];
                Vector3D B = vertices[faces[f].B - 1];
                Vector3D C = vertices[faces[f].C - 1];

                Vector3D normal = getNormal(A, B, C);

                attachModelTransform(&potTransform);

                SurfaceAttributes attributes = (SurfaceAttributes){
                    (Vector3D){1, .5, .5},
                    A,
                    B,
                    C,
                    normal,
                    normals[faces[f].An - 1],
                    normals[faces[f].Bn - 1],
                    normals[faces[f].Cn - 1],
                    uvs[faces[f].Auv - 1],
                    uvs[faces[f].Buv - 1],
                    uvs[faces[f].Cuv - 1],
                };
                triangle3D(fBuffer, A, B, C, attributes, scene);
            }
        }
        else
        {
            for (int f = 0; f < faces_count_pot; f++)
            {

                Vector3D A = (Vector3D){vertices_pot[faces_pot[f][0]][0], vertices_pot[faces_pot[f][0]][1], vertices_pot[faces_pot[f][0]][2]};
                Vector3D B = (Vector3D){vertices_pot[faces_pot[f][1]][0], vertices_pot[faces_pot[f][1]][1], vertices_pot[faces_pot[f][1]][2]};
                Vector3D C = (Vector3D){vertices_pot[faces_pot[f][2]][0], vertices_pot[faces_pot[f][2]][1], vertices_pot[faces_pot[f][2]][2]};

                attachModelTransform(&potTransform);

                SurfaceAttributes attributes = {0};
                attributes.color = (Vector3D){1, .5, .5};
                attributes.A = A, attributes.B = B, attributes.C = C;
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
        for (int f = 0; f < faces_count_cup; f++)
        {

            // normals are broken because of inconsistent vertex order
            Vector3D A = (Vector3D){vertices_cup[faces_cup[f][0]][0], vertices_cup[faces_cup[f][0]][1], vertices_cup[faces_cup[f][0]][2]};
            Vector3D B = (Vector3D){vertices_cup[faces_cup[f][1]][0], vertices_cup[faces_cup[f][1]][1], vertices_cup[faces_cup[f][1]][2]};
            Vector3D C = (Vector3D){vertices_cup[faces_cup[f][2]][0], vertices_cup[faces_cup[f][2]][1], vertices_cup[faces_cup[f][2]][2]};

            attachModelTransform(&cupTransform);

            SurfaceAttributes attributes = {0};
            attributes.color = (Vector3D){1, 1, 1};
            attributes.A = A, attributes.B = B, attributes.C = C;
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
        // floor
        Vector3D A = (Vector3D){-4, 0, 4};
        Vector3D B = (Vector3D){4, 0, -4};
        Vector3D C = (Vector3D){-4, 0, -4};
        resetModelTransform();
        SurfaceAttributes attributes = {0};
        attributes.color = (Vector3D){.6, .5, .45};
        attributes.A = A, attributes.B = B, attributes.C = C;
        attributes.normal.y = attributes.normalA.y = attributes.normalB.y = attributes.normalC.y = 1;
        triangle3D(fBuffer, A, B, C, attributes, scene);
#ifdef STEP_RENDER
        printf("\033[%zuA", (size_t)HEIGHT);
        printf("\033[%zuD", (size_t)WIDTH);
        print(buffer);
#endif // STEP_RENDER
        C = (Vector3D){4, 0, 4};
        attributes.C = C;
        triangle3D(fBuffer, A, B, C, attributes, scene);
    }
}
#endif // SHADOWMAP_DEMO

void clearBuffer(FrameBuffer buffer)
{
    // #pragma omp parallel for
    for (int i = 0; i < buffer.width; i++)
    {
        for (int j = 0; j < buffer.height; j++)
        {
            float *fragment = frameBufferAt(buffer, i, j);
            fragment[0] = 0;
            fragment[1] = 0;
            fragment[2] = 0;
            fragment[3] = 0;
        }
    }
}

void loadObj()
{
#ifdef SHADOWMAP_DEMO
    shadowBuffer = malloc(sizeof(float[SHADOW_RES][SHADOW_RES][4]));
#endif // SHADOWMAP_DEMO

#ifdef TEXTURED
    char *filename = "texture1.png";
    texture = stbi_load(filename, &img_width, &img_height, 0, 4);
#endif // TEXTURED
    char *path = "teapot.obj";
    faceCount = 0;
    int vertexCount = 0, normalCount = 0, uvCount = 0;
    if (countObjects(path, &vertexCount, &normalCount, &faceCount, &uvCount))
    {
        vertices = malloc(sizeof(Vector3D) * vertexCount);
        normals = malloc(sizeof(Vector3D) * normalCount);
        faces = malloc(sizeof(Face) * faceCount);
        uvs = malloc(sizeof(Vector2D) * uvCount);

        objLoaded = parseObjects(path, vertices, normals, faces, uvs);
        // free(uvs);
        if (objLoaded)
            printf("%s loaded\n", path);
    }
}

void unloadObj()
{
    free(vertices);
    free(normals);
    free(faces);
    free(uvs);
#ifdef TEXTURED
    stbi_image_free(texture);
#endif // TEXTURED

#ifdef SHADOWMAP_DEMO
    free(shadowBuffer);
#endif // SHADOWMAP_DEMO
}

#ifndef RENDER_GUI
int main()
{
    loadObj();
#ifdef DYNAMIC_ALLOC
    float *buffer = malloc(sizeof(float[WIDTH][HEIGHT][4]));
#else
    float buffer[WIDTH][HEIGHT][4];
    clearBuffer((FrameBuffer){buffer, WIDTH, HEIGHT});
#endif // DYNAMIC_ALLOC
    for (int t = 0; t < FRAMES; t++)
    {
#ifdef SHADOWMAP_DEMO
        shadowPass(t);
#endif // SHADOWMAP_DEMO
        from_obj(buffer, t);
        print(buffer);
        usleep(1000 * 1000 / 60);
        printf("\033[%zuA", (size_t)HEIGHT);
        printf("\033[%zuD", (size_t)WIDTH);
        clearBuffer((FrameBuffer){buffer, WIDTH, HEIGHT});
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
#ifndef DEBUG_SHADOWMAP
    FrameBuffer fBuffer = (FrameBuffer){buffer, WIDTH, HEIGHT};
    for (int y = 0; y < HEIGHT; y++)
    {
        for (int x = 0; x < WIDTH; x++)
        {
            uint8_t *pixel = *dibData + (y * WIDTH + x) * 4;
            float *fragment = frameBufferAt(fBuffer, x, y);
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
#else
    FrameBuffer shadowfBuffer = (FrameBuffer){shadowBuffer, SHADOW_RES, SHADOW_RES};
    for (int y = 0; y < HEIGHT; y++)
    {
        for (int x = 0; x < WIDTH; x++)
        {
            uint8_t *pixel = *dibData + (y * WIDTH + x) * 4;
            float *fragment = frameBufferAt(shadowfBuffer, x * SHADOW_RES / WIDTH, y * SHADOW_RES / HEIGHT);
            float r = sqrtf(200. / (1. / fragment[0] - 1.));
            float g = sqrtf(200. / (1. / fragment[1] - 1.));
            float b = sqrtf(200. / (1. / fragment[2] - 1.));
            float a = fragment[3];

            pixel[0] = (uint8_t)(b * 255); // Blue
            pixel[1] = (uint8_t)(g * 255); // Green
            pixel[2] = (uint8_t)(r * 255); // Red
            pixel[3] = (uint8_t)(a * 255); // Alpha
        }
    }
#endif // DEBUG_SHADOWMAP
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
        clearBuffer((FrameBuffer){buffer, WIDTH, HEIGHT});
#ifdef SHADOWMAP_DEMO
        // shadowPass(frame);
#endif // SHADOWMAP_DEMO
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
#ifdef SHADOWMAP_DEMO
    shadowPass(0);
#endif // SHADOWMAP_DEMO

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