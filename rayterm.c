#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// #include "pot.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#ifndef ASPECT_RATIO
#define ASPECT_RATIO 1
#endif // ASPECT_RATIO

#ifndef HEIGHT
#define HEIGHT 360
#endif // HEIGHT
#ifndef WIDTH
#define WIDTH ((int)(HEIGHT * ASPECT_RATIO))
#endif // WIDTH

#define RASTERM_IMPLEMENTATION
#include "rasterm.h"
#include "parseObj.c"

typedef struct
{
    Vector3D *A;
    Vector3D *B;
    Vector3D *C;
} Triangle;

typedef struct LinkedListStruct
{
    Triangle *T;
    struct LinkedListStruct *next;
} LinkedListNode;
#ifdef MIN
#undef MIN
#endif
#ifdef MAX
#undef MAX
#endif
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define DEPTH 6
const int octree_span = 1 << DEPTH;
const float world_size = 60;
const float world_offset = -world_size / 2;
// const float voxel_size = world_size / (float)octree_span;

typedef struct
{
    // bounding box
    Vector3D world_offset;
    Vector3D world_size;
    // content
    intptr_t *data;
} Octree;

bool objLoaded = false;
Vector3D *vertices = NULL;
Vector3D *normals = NULL;
Face *faces = NULL;
Vector2D *uvs = NULL;
int faceCount;

//[A,B,C,D,E,F,G,[AA,BA,CA,DA,EA,FA,GA],[AB,BB,CB,DB,EB,FB]]
//^A                     ^B
// 8+8*8+8*8*8
// (8^n-1)*8/7

void octreeCoords(Vector3D worldCoords, int *x, int *y, int *z, Octree bb)
{
    *x = (int)((float)octree_span * (worldCoords.x - bb.world_offset.x) / bb.world_size.x);
    *y = (int)((float)octree_span * (worldCoords.y - bb.world_offset.y) / bb.world_size.y);
    *z = (int)((float)octree_span * (worldCoords.z - bb.world_offset.z) / bb.world_size.z);
}
void octreeCoordsDepth(Vector3D worldCoords, int *x, int *y, int *z, Octree bb, int depth)
{
    octreeCoords(worldCoords, x, y, z, bb);
    *x >>= depth;
    *y >>= depth;
    *z >>= depth;
}

Vector3D voxelOrig(int x, int y, int z, Octree bb)
{
    return (Vector3D){bb.world_offset.x + (float)x * bb.world_size.x / (float)octree_span,
                      bb.world_offset.y + (float)y * bb.world_size.y / (float)octree_span,
                      bb.world_offset.z + (float)z * bb.world_size.z / (float)octree_span};
}

Vector3D voxelOrigDepth(int x, int y, int z, Octree bb, int depth)
{
    x <<= depth;
    y <<= depth;
    z <<= depth;
    return voxelOrig(x, y, z, bb);
}
/**
 * @brief Checks if an arbitrary 2D segment intersects an other segment aligned with the y axis
 *
 * @param ax starting x value of the segment
 * @param ay starting y value of the segment
 * @param bx ending x value of the segment
 * @param by ending y value of the segment
 * @param edgeX x coordinate of all points in the second segment
 * @param minY starting y value of the other segment
 * @param maxY ending y value of the other segment
 * @return true if there is an intersection
 */
bool edgeIntersect(float ax, float ay, float bx, float by, float edgeX, float minY, float maxY)
{
    float da = (ax - edgeX);
    float db = (bx - edgeX);
    if (da * db <= 0)
    {
        // points are on opposite sides of the edge line
        // projection:
        float dea = (ax - edgeX);
        float dba = (ax - bx);

        float ga = dea / (dba);
        float py = by + (ay - by) * ga;

        if (py >= minY && py <= maxY)
            return true;
    }
    return false;
}
/**
 * @brief Checks if a 3D triangle intersects an axis-aligned square in the x plane at any point
 *
 * @param A a vertex of the triangle
 * @param B a vertex of the triangle
 * @param C a vertex of the triangle
 * @param faceX x coordinate of all points in the square
 * @param minY lower bounds of the square along the y axis
 * @param maxY higher bounds of the square along the y axis
 * @param minZ lower bounds of the square along the z axis
 * @param maxZ higher bounds of the square along the z axis
 * @return true if there is an intersection
 */
bool faceIntersect(Vector3D A, Vector3D B, Vector3D C, float faceX, float minY, float maxY, float minZ, float maxZ)
{
    float dA = (A.x - faceX);
    float dB = (B.x - faceX);
    float dC = (C.x - faceX);
    float AB = dA * dB;
    float AC = dA * dC;
    if (AB <= 0 || AC <= 0)
    {
        // one point is on the other side of the face => plane intersection
        Vector3D *a, *b, *c;
        if (AB > 0)
            // A and B are on the same side => C is the odd one
            a = &A, b = &B, c = &C;
        else if (AC > 0)
            // A and C are on the same side => B is the odd one
            a = &A, b = &C, c = &B;
        else
            // B and C are on the same side => A is the odd one
            a = &C, b = &B, c = &A;
        //  now we assume a and b are on the same side of the plane

        // projection:
        float dfa = (a->x - faceX);
        float dfb = (b->x - faceX);
        float dca = (a->x - c->x);
        float dcb = (b->x - c->x);

        float ga = dfa / (dca);
        float gb = dfb / (dcb);

        float ay = c->y + (a->y - c->y) * ga;
        float az = c->z + (a->z - c->z) * ga;
        float by = c->y + (b->y - c->y) * gb;
        float bz = c->z + (b->z - c->z) * gb;

        float dy = by - ay, dz = bz - az;
        // now we do 2D intersection of segment and cube

        bool pointInPixel = ay >= minY && ay <= maxY && az >= minZ && az <= maxZ;
        pointInPixel = pointInPixel || by >= minY && by <= maxY && bz >= minZ && bz <= maxZ;
        if (pointInPixel)
            return true;
        // iterate face edges
        else if (edgeIntersect(ay, az, by, bz, minY, minZ, maxZ))
            return true;
        else if (edgeIntersect(ay, az, by, bz, maxY, minZ, maxZ))
            return true;
        else if (edgeIntersect(az, ay, bz, by, minZ, minY, maxY))
            return true;
        else if (edgeIntersect(az, ay, bz, by, maxZ, minY, maxY))
            return true;
    }
    return false;
}
/**
 * @brief Checks if a 3D triangle intersects an axis-aligned voxel at any point
 *
 * @param A a vertex of the triangle
 * @param B a vertex of the triangle
 * @param C a vertex of the triangle
 * @param vMin lower bounds of the voxel (AABB)
 * @param vMax higher bounds of the voxel (AABB)
 * @return true if there is an intersection
 */
bool voxelIntersect(Vector3D *A, Vector3D *B, Vector3D *C, Vector3D vMin, Vector3D vMax)
{
    bool vertexInVoxel = A->x >= vMin.x && A->x <= vMax.x && A->y >= vMin.y && A->y <= vMax.y && A->z >= vMin.z && A->z <= vMax.z;
    vertexInVoxel = vertexInVoxel || (B->x >= vMin.x && B->x <= vMax.x && B->y >= vMin.y && B->y <= vMax.y && B->z >= vMin.z && B->z <= vMax.z);
    vertexInVoxel = vertexInVoxel || (C->x >= vMin.x && C->x <= vMax.x && C->y >= vMin.y && C->y <= vMax.y && C->z >= vMin.z && C->z <= vMax.z);

    Vector3D Ay = (Vector3D){A->y, A->x, A->z}, By = (Vector3D){B->y, B->x, B->z}, Cy = (Vector3D){C->y, C->x, C->z};
    Vector3D Az = (Vector3D){A->z, A->y, A->x}, Bz = (Vector3D){B->z, B->y, B->x}, Cz = (Vector3D){C->z, C->y, C->x};
    if (vertexInVoxel)
        return true;
    // iterate voxel faces
    else if (faceIntersect(*A, *B, *C, vMin.x, vMin.y, vMax.y, vMin.z, vMax.z))
        return true;
    else if (faceIntersect(*A, *B, *C, vMax.x, vMin.y, vMax.y, vMin.z, vMax.z))
        return true;
    else if (faceIntersect(Ay, By, Cy, vMin.y, vMin.x, vMax.x, vMin.z, vMax.z))
        return true;
    else if (faceIntersect(Ay, By, Cy, vMax.y, vMin.x, vMax.x, vMin.z, vMax.z))
        return true;
    else if (faceIntersect(Az, Bz, Cz, vMin.x, vMin.y, vMax.y, vMin.z, vMax.z))
        return true;
    else if (faceIntersect(Az, Bz, Cz, vMax.x, vMin.y, vMax.y, vMin.z, vMax.z))
        return true;
    return false;
}

float minstep;

// TODO: fix that shit
// Somehow it works ok with positive rd, but breaks for negative? (I think?)
Vector3D voxelSkipLOD(Vector3D p, Vector3D rd, int depth, Octree octree)
{
    // ray offsets
    int sx = (rd.x >= 0 ? 1 : 0), sy = (rd.y >= 0 ? 1 : 0), sz = (rd.z >= 0 ? 1 : 0);

    // current voxel coords
    int x, y, z;
    octreeCoordsDepth(p, &x, &y, &z, octree, depth);

    // add offsets
    int xn = (x + sx);
    int yn = (y + sy);
    int zn = (z + sz);

    // Vector3D p0 = voxelOrig(xn, yn, zn, octree);

    // possible intersections (only one component is needed)
    Vector3D pe = voxelOrigDepth(xn, yn, zn, octree, depth);

    // distance to edges
    float dx = (pe.x - p.x) / (rd.x);
    float dy = (pe.y - p.y) / (rd.y);
    float dz = (pe.z - p.z) / (rd.z);

    // if (dz <= 0.000001)
    // {
    //     printf("wtf: dz=%f, \tsz = %d, \tpz = %f, \tp.z = %f, \trd.z=%f\n", dz, sz, pe.z, p.z, rd.z);
    // }

    float dmin = MIN(dx, dy);
    dmin = MIN(dmin, dz);
    // float dmax = MAX(MAX(dx, dy), dz);
    // float dmid = dx + dy + dz - (dmin + dmax);
    // if (dmin <= 0)
    //     dmin = 0;
    // if (dmid <= 0)
    //     dmid = dmax;

    // between min and mid to avoind landing just on the edge
    float d = dmin;

    // just in case
    if (d <= 0.)
    {
        d = .0;
    }

    d += .1;

    minstep = MIN(d, minstep);

    p.x += d * rd.x;
    p.y += d * rd.y;
    p.z += d * rd.z;

    return p;
}

int minB;

/**
 * @brief check if a ray intersects some non-empty voxel in an octree structure
 *
 * @param ro ray origin
 * @param rd ray direction
 * @param octree octree structure
 * @return LinkedListNode* the content of the intersected voxel (NULL if no hit)
 */
LinkedListNode *rayCast_voxel_octree(Vector3D ro, Vector3D rd, Octree octree)
{
    bool hit = false;
    minB = DEPTH;
    minstep = 1e6;

    // TODO: refactor octree system to allow flexibility over its bounds
    while (ro.x >= octree.world_offset.x && ro.x < octree.world_size.x + octree.world_offset.x && ro.y >= octree.world_offset.y && ro.y < octree.world_size.y + octree.world_offset.y && ro.z >= octree.world_offset.z && ro.z < octree.world_size.z + octree.world_offset.z)
    {
        // printf("p: %f, %f, %f\n", ro.x, ro.y, ro.z);
        int xi, yi, zi;
        octreeCoords(ro, &xi, &yi, &zi, octree);
        intptr_t *root = octree.data;
        intptr_t cell_off = 0;
        int w = 1;
        // TODO: smarter starting LOD depending on previous step?
        for (int b = DEPTH - 1; b >= 0; b--)
        {
            minB = MIN(b, minB);

            int cx = (xi >> b) % 2;
            int cy = (yi >> b) % 2;
            int cz = (zi >> b) % 2;

            int loc = cx + 2 * cy + 4 * cz;
            intptr_t *cell = root + cell_off;
            if (!cell[loc])
            {
                // printf("skipping at level %d\n", b);
                ro = voxelSkipLOD(ro, rd, b, octree);

                break;
            }
            else if (b == 0)
            {
                // printf("hit something!\n");
                return (LinkedListNode *)cell[loc];
            }
            else
            {
                // printf("deeper...\n");
                w *= 8;
                root += w;
                cell_off += loc;
                cell_off *= 8;
            }
        }
    }
    return NULL;
}

void build_octree(Octree octree, Triangle *triangles, int triCount, LinkedListNode *nodes)
{
    LinkedListNode *lastNodePtr = nodes;
    //  Voxelize triangles
    for (int f = 0; f < triCount; f++)
    {
        Vector3D *A = vertices + (faces[f].A - 1);
        Vector3D *B = vertices + (faces[f].B - 1);
        Vector3D *C = vertices + (faces[f].C - 1);
        triangles[f] = (Triangle){A, B, C};

        // triangle's AABB
        Vector3D AABB_min = (Vector3D){MIN(A->x, MIN(B->x, C->x)),
                                       MIN(A->y, MIN(B->y, C->y)),
                                       MIN(A->z, MIN(B->z, C->z))};
        Vector3D AABB_max = (Vector3D){MAX(A->x, MAX(B->x, C->x)),
                                       MAX(A->y, MAX(B->y, C->y)),
                                       MAX(A->z, MAX(B->z, C->z))};
        int xMin, yMin, zMin;
        octreeCoords(AABB_min, &xMin, &yMin, &zMin, octree);
        int xMax, yMax, zMax;
        octreeCoords(AABB_max, &xMax, &yMax, &zMax, octree);

        // iterate voxels within triangle's AABB
        for (int x = xMin; x <= xMax; x++)
        {
            for (int y = yMin; y <= yMax; y++)
            {
                for (int z = zMin; z <= zMax; z++)
                {
                    // get voxel bounds
                    Vector3D vMin = voxelOrig(x, y, z, octree);
                    Vector3D vMax = (Vector3D){vMin.x + octree.world_size.x / octree_span, vMin.y + octree.world_size.y / octree_span, vMin.z + octree.world_size.z / octree_span};

                    // check if some part of the current triangles is inside the current voxel
                    if (voxelIntersect(A, B, C, vMin, vMax))
                    {
                        intptr_t *root = octree.data;
                        intptr_t cell_off = 0;
                        int w = 1;
                        for (int b = DEPTH - 1; b >= 0; b--)
                        {
                            int cx = (x >> b) % 2;
                            int cy = (y >> b) % 2;
                            int cz = (z >> b) % 2;

                            int loc = cx + 2 * cy + 4 * cz;
                            intptr_t *cell = root + cell_off;
                            if (b == 0)
                            {
                                // append triangle at the beginning of this voxel's linked list
                                LinkedListNode *nextNode = (LinkedListNode *)cell[loc];
                                *(lastNodePtr) = (LinkedListNode){(Triangle *)(triangles + f), nextNode};
                                cell[loc] = (uintptr_t)lastNodePtr++;
                                continue;
                            }
                            else
                                cell[loc]++;
                            w *= 8;
                            root += w;
                            cell_off += loc;
                            cell_off *= 8;
                        }
                    }
                }
            }
        }
    }
}

int main_render(float *buffer, int frame)
{
    FrameBuffer fBuffer = (FrameBuffer){buffer, WIDTH, HEIGHT};

    // initialize triangle buffer
    Triangle *triangles = malloc(faceCount * sizeof(Triangle));
    LinkedListNode *nodes = malloc(faceCount * 256 * sizeof(LinkedListNode));

    // create voxel octree
    size_t tree_size = 8 * ((1 << (3 * DEPTH)) - 1) / 7;
    intptr_t *octreeData = malloc(tree_size * sizeof(intptr_t));
    if (!octreeData || !triangles || !nodes)
    {
        printf("malloc fail\n");
        exit(-1);
    }
    memset(octreeData, 0, tree_size * sizeof(intptr_t));

    Octree octree = (Octree){(Vector3D){world_offset, world_offset, world_offset},
                             (Vector3D){world_size, world_size, world_size},
                             octreeData};

    build_octree(octree, triangles, faceCount, nodes);

    // printf("\nvoxelization ok\n");

    //  Traverse octree ( arbitrary ray)

    Vector3D p = (Vector3D){0.01, 0.01, (30 - frame)};
    // Vector3D rd = normalize((Vector3D){-.3, .05, -.5});
    // LinkedListNode *hit = rayCast_voxel_octree(p, rd, octree);
    // exit(0);

    for (int j = 0; j < HEIGHT; j++)
    {
        float v = ((float)HEIGHT / 2. - j) / (float)HEIGHT;
        for (int i = 0; i < WIDTH; i++)
        {
            float *px = frameBufferAt(fBuffer, i, j);
            float u = (i - (float)WIDTH / 2.) / (float)HEIGHT;
            Vector3D rd = normalize((Vector3D){-u, v, -.5});

            LinkedListNode *hit = rayCast_voxel_octree(p, rd, octree);

            if (hit)
            {
                float x = 1000. + 30.14 * hit->T->B->x;
                float y = 1000. + 30.14 * hit->T->B->y;
                float z = 1000. + 30.14 * hit->T->B->z;
                px[0] = (x - (int)x), px[1] = (y - (int)y), px[2] = (z - (int)z);
            }
            else
            {
                px[0] = 1. - (minB) / (float)(DEPTH), px[1] = 1. - exp2f(-minstep * .1), px[2] = 0;
            }
        }
    }

    if (0)
    {
        int j = 15;
        float y = world_offset + world_size * (31 - j) / 32.;
        for (int i = 0; i < 64; i++)
        {
            float x = world_offset + world_size * i / 64.;
            bool hit = false;
            for (int k = 0; k < octree_span; k++)
            {
                float z = world_offset + world_size * k / (float)octree_span;
                int xi, yi, zi;
                octreeCoords((Vector3D){x, y, z}, &xi, &yi, &zi, octree);
                // printf("x:%d\t,y:%d\t,z:%d\n", xi, yi, zi);
                intptr_t *root = octree.data;
                intptr_t cell_off = 0;
                int w = 1;
                for (int b = DEPTH - 1; b >= 0; b--)
                {
                    int cx = (xi >> b) % 2;
                    int cy = (yi >> b) % 2;
                    int cz = (zi >> b) % 2;

                    int loc = cx + 2 * cy + 4 * cz;
                    intptr_t *cell = root + cell_off;
                    if (b == 0)
                    {
                        if (cell[loc])
                        {
                            LinkedListNode *node = (LinkedListNode *)cell[loc];
                            int n = 0;
                            while (node->next)
                            {
                                n++;
                                node = node->next;
                            }
                            if (n <= 9)
                                printf("%d", n);
                            else
                                printf("#");
                            hit = true;
                        }
                        else
                        {
                            printf(" ");
                        }
                        break;
                    }
                    if (!cell[loc])
                    {
                        for (int s = k; s < (((k >> b) + 1) << b); s++)
                            printf("-");
                        printf(">");
                        k = ((k >> b) + 1) << b;
                        break;
                    }
                    w *= 8;
                    root += w;
                    cell_off += loc;
                    cell_off *= 8;
                }
            }
            // if (hit)
            //     printf("#");
            // else
            //     printf(" ");
            printf("\n");
        }
        exit(-1);
    }

    free(octreeData);
    free(nodes);
    free(triangles);
}

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
    printf("%d faces\n\n", faceCount);
}
void unloadObj()
{
    free(vertices);
    free(normals);
    free(faces);
    free(uvs);
}

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

        main_render(buffer, ++frame);

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
#define FPS 5
#endif // FPS

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    loadObj();
    buffer = malloc(sizeof(float[WIDTH][HEIGHT][4]));

    main_render(buffer, 0);
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