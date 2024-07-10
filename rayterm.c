#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <time.h>
#include <omp.h>

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

typedef struct
{
    Triangle *T;
    float t;
    Vector3D P;
    Vector3D N;
    Vector2D UV;
} HitResult;
 


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

/**
 * @brief Converts world coordinates to octree space coordinates.
 *
 * This function takes a 3D vector of world coordinates and an octree bounding box as input,
 * and returns a 3D vector of octree space coordinates. It does this by subtracting the world
 * offset from the world coordinates, dividing the result by the world size, and then multiplying
 * by the octree span.
 *
 * @param world_coords The 3D vector of world coordinates to be converted.
 * @param bb The octree bounding box.
 * @return Vector3D The 3D vector of octree space coordinates.
 */
Vector3D toOctreeSpace(Vector3D world_coords, Octree bb){
    Vector3D octree_coords = (Vector3D){
        octree_span * (world_coords.x - bb.world_offset.x) / bb.world_size.x,
        octree_span * (world_coords.y - bb.world_offset.y) / bb.world_size.y,
        octree_span * (world_coords.z - bb.world_offset.z) / bb.world_size.z
    };
    return octree_coords;
}

/**
 * @brief Converts octree space coordinates to world coordinates.
 *
 * This function takes a 3D vector of octree space coordinates and an octree bounding box as input,
 * and returns a 3D vector of world coordinates. It does this by dividing the octree space coordinates
 * by the octree span, multiplying the result by the world size, and then adding the world offset.
 *
 * @param octree_coords The 3D vector of octree space coordinates to be converted.
 * @param bb The octree bounding box.
 * @return Vector3D The 3D vector of world coordinates.
 */
Vector3D octreeToWorldSpace(Vector3D octree_coords, Octree bb){
    Vector3D world_coords;
    world_coords.x = (octree_coords.x / octree_span) * bb.world_size.x + bb.world_offset.x;
    world_coords.y = (octree_coords.y / octree_span) * bb.world_size.y + bb.world_offset.y;
    world_coords.z = (octree_coords.z / octree_span) * bb.world_size.z + bb.world_offset.z;
    return world_coords;
}

/**
 * @brief Calculates the integer coordinates for a given 3D vector.
 *
 * This function calculates the integer coordinates for a given 3D vector.
 * It does this by casting the x, y, and z components of the vector to integers
 * and storing the results in the x, y, and z pointers.
 *
 * @param coords The 3D vector for which to calculate the integer coordinates.
 * @param x A pointer to an integer that will be set to the x coordinate.
 * @param y A pointer to an integer that will be set to the y coordinate.
 * @param z A pointer to an integer that will be set to the z coordinate.
 */
void intCoords(Vector3D coords, int *x, int *y, int *z)
{
    *x = (int)floorf(coords.x);
    *y = (int)floorf(coords.y);
    *z = (int)floorf(coords.z);
}

/**
 * @brief Calculates the integer coordinates for a given 3D vector at a specific level of detail (LoD).
 *
 * This function calculates the integer coordinates for a given 3D vector at a specific level of detail (LoD).
 * It does this by casting the x, y, and z components of the vector to integers and then shifting these
 * values right by the specified LoD value. This effectively reduces the resolution of the integer coordinates,
 * allowing for coarser-grained representations of the 3D vector at lower levels of detail.
 *
 * @param coords The 3D vector for which to calculate the integer coordinates.
 * @param x A pointer to an integer that will be set to the x coordinate.
 * @param y A pointer to an integer that will be set to the y coordinate.
 * @param z A pointer to an integer that will be set to the z coordinate.
 * @param lod The level of detail at which to calculate the integer coordinates.
 */
void intCoordsLod(Vector3D coords, int *x, int *y, int *z, int lod)
{
    *x = (int)floorf(coords.x) >> lod;
    *y = (int)floorf(coords.y) >> lod;
    *z = (int)floorf(coords.z) >> lod;
}

/**
 * @brief Calculates the octree coordinates for a given world coordinate.
 *
 * This function calculates the octree coordinates for a given world coordinate.
 * It does this by subtracting the world offset from the world coordinate,
 * dividing the result by the world size, and then multiplying by the octree span.
 * The resulting value is cast to an integer and stored in the x, y, and z pointers.
 *
 * @param worldCoords The world coordinates for which to calculate the octree coordinates.
 * @param x A pointer to an integer that will be set to the x octree coordinate.
 * @param y A pointer to an integer that will be set to the y octree coordinate.
 * @param z A pointer to an integer that will be set to the z octree coordinate.
 * @param bb The bounding box of the octree.
 */
void octreeCoords(Vector3D worldCoords, int *x, int *y, int *z, Octree bb)
{
    Vector3D octreeCoords = toOctreeSpace(worldCoords, bb);
    intCoords(octreeCoords, x, y, z);
}

/**
 * @brief Calculates the octree coordinates for a given world coordinate at a specific level of detail (LoD).
 *
 * This function calculates the octree coordinates for a given world coordinate at a specific level of detail (LoD).
 * It first calculates the octree coordinates without considering the LoD, and then shifts these coordinates right
 * by the specified LoD value. This effectively reduces the resolution of the octree coordinates, allowing for
 * coarser-grained representations of the world at lower levels of detail.
/**
 * @brief Calculates the octree coordinates for a given world coordinate at a specific level of detail (LoD).
 *
 * @param y A pointer to an integer that will be set to the y octree coordinate.
 * @param z A pointer to an integer that will be set to the z octree coordinate.
 * @param bb The bounding box of the octree.
 * @param lod The level of detail at which to calculate the octree coordinates.
 */
void octreeCoordsLod(Vector3D worldCoords, int *x, int *y, int *z, Octree bb, int lod)
{
    octreeCoords(worldCoords, x, y, z, bb);
    *x >>= lod;
    *y >>= lod;
    *z >>= lod;
}

/**
 * @brief Converts integer coordinates to floating-point coordinates.
 *
 * This function takes three integer values representing x, y, and z coordinates
 * and returns a Vector3D structure with these coordinates converted to floating-point values.
 *
 * @param x The x coordinate as an integer.
 * @param y The y coordinate as an integer.
 * @param z The z coordinate as an integer.
 * @return A Vector3D structure with the x, y, and z coordinates converted to floating-point values.
 */
Vector3D floatCoords(int x, int y, int z)
{
    return (Vector3D){(float)x, (float)y, (float)z};
}

/**
 * @brief Converts integer coordinates to floating-point coordinates at a specific level of detail (LoD).
 *
 * This function takes three integer values representing x, y, and z coordinates and a level of detail (LoD).
 * It then shifts these integer coordinates left by the specified LoD value, effectively increasing the resolution
 * of the coordinates. The resulting values are then converted to floating-point values and returned as a Vector3D structure.
 *
 * @param x The x coordinate as an integer.
 * @param y The y coordinate as an integer.
 * @param z The z coordinate as an integer.
 * @param lod The level of detail at which to convert the integer coordinates to floating-point coordinates.
 * @return A Vector3D structure with the x, y, and z coordinates converted to floating-point values at the specified level of detail.
 */
Vector3D floatCoordsLod(int x, int y, int z, int lod)
{
    x <<= lod;
    y <<= lod;
    z <<= lod;
    return floatCoords(x, y, z);
}

/**
 * @brief Calculates the world coordinates of the origin of a voxel in the octree.
 *
 * This function calculates the world coordinates of the origin of a voxel in the octree,
 * given its integer coordinates (x, y, z) and the bounding box of the octree (bb).
 * It does this by adding the world offset of the octree to the scaled integer coordinates,
 * where the scaling is determined by the size of the world and the span of the octree.
 *
 * @param x The x coordinate of the voxel in the octree.
 * @param y The y coordinate of the voxel in the octree.
 * @param z The z coordinate of the voxel in the octree.
 * @param bb The bounding box of the octree.
 * @return A Vector3D structure with the world coordinates of the origin of the voxel.
 */
Vector3D voxelOrig(int x, int y, int z, Octree bb)
{
    Vector3D octreeCoords = floatCoords(x, y, z);
    return octreeToWorldSpace(octreeCoords, bb);
}

/**
 * @brief Calculates the world coordinates of the origin of a voxel in the octree at a specific level of detail (LoD).
 *
 * This function calculates the world coordinates of the origin of a voxel in the octree at a specific level of detail (LoD).
 * It does this by first shifting the integer coordinates (x, y, z) left by the specified LoD value, effectively increasing
 * the resolution of the coordinates. The resulting values are then used to calculate the world coordinates of the origin
 * of the voxel using the voxelOrig function.
 *
 * @param x The x coordinate of the voxel in the octree.
 * @param y The y coordinate of the voxel in the octree.
 * @param z The z coordinate of the voxel in the octree.
 * @param bb The bounding box of the octree.
 * @param lod The level of detail at which to calculate the world coordinates of the voxel origin.
 * @return A Vector3D structure with the world coordinates of the origin of the voxel at the specified level of detail.
 */
Vector3D voxelOrigLod(int x, int y, int z, Octree bb, int lod)
{
    x <<= lod;
    y <<= lod;
    z <<= lod;
    return voxelOrig(x, y, z, bb);
}

/**
 * @brief Calculates the intersection of a ray with a triangle.
 *
 * This function determines if a ray, defined by its origin (ro) and direction (rd), intersects with a triangle.
 * The triangle is defined by three vertices (A, B, C). If the ray intersects the triangle, the function returns
 * the distance from the ray's origin to the intersection point. If the ray does not intersect the triangle,
 * the function returns -1.
 *
 * @param ro The origin of the ray.
 * @param rd The direction of the ray.
 * @param tri A pointer to a Triangle structure that contains the vertices of the triangle.
 * @return The distance from the ray's origin to the intersection point if the ray intersects the triangle, or -1 if it does not.
 */
HitResult rayTriangleIntersect(Vector3D ro, Vector3D rd, Triangle * tri){

    // TODO: expose u and v (barycentric coordinates) to the caller

    HitResult result = {.T = tri,.t = -1};

    // maybe normal too
    Vector3D a = (Vector3D){tri->B->x - tri->A->x, tri->B->y - tri->A->y, tri->B->z - tri->A->z};
    Vector3D b = (Vector3D){tri->C->x - tri->A->x, tri->C->y - tri->A->y, tri->C->z - tri->A->z};

    Vector3D o = (Vector3D){ro.x - tri->A->x, ro.y - tri->A->y, ro.z - tri->A->z};

    Vector3D n = cross(a, b);

    float h = -dot(o, n)/dot(rd, n);

    Vector3D p = (Vector3D){o.x + h*rd.x, o.y + h*rd.y, o.z + h*rd.z};

    //now we calculate u and v
    float u = dot(cross(p, b), n)/dot(n, n);
    float v = dot(cross(a, p), n)/dot(n, n);

    if(u >= 0 && v >= 0 && u + v <= 1){
        result.t = h;
        result.UV = (Vector2D){u, v};
        result.P = (Vector3D){p.x + tri->A->x, p.y + tri->A->y, p.z + tri->A->z};
        result.N = normalize(n);
     }
    return result;
}


/**
 * @brief Checks if an arbitrary 2D segment intersects an other segment aligned with the y axis
 *
 * This function determines if a line segment, defined by its starting and ending x and y coordinates,
 * intersects with a vertical line segment aligned with the y axis. The intersection is checked by
 * projecting the line segment onto the y axis and seeing if the projected segment overlaps with the
 * vertical segment.
 *
 * @param ax Starting x value of the segment
 * @param ay Starting y value of the segment
 * @param bx Ending x value of the segment
 * @param by Ending y value of the segment
 * @param edgeX x coordinate of all points in the second segment
 * @param minY Starting y value of the other segment
 * @param maxY Ending y value of the other segment
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
        float py = ay + (by - ay) * ga;

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
    
    // check signs
    float AB = dA * dB;
    float AC = dA * dC;
    if (AB <= 0 || AC <= 0)
    {
        // one point is alone on the other side of the face => plane intersection
        Vector3D *a, *b, *c;
        if (AB > 0)
            // A and B are on the same side => C is the odd one
            a = &A, b = &B, c = &C;
        else if (AC > 0)
            // A and C are on the same side => B is the odd one
            a = &C, b = &A, c = &B;
        else
            // B and C are on the same side => A is the odd one
            a = &B, b = &C, c = &A;
        //  now we assume a and b are on the same side of the plane

        // projection:
        // we project a and b onto the plane in the direction of c
        // this gives us the segment where the 3D triangle intersects the plane
        float dfc = (c->x - faceX);
        float dca = (c->x - a->x);
        float dcb = (c->x - b->x);

        float ga = dfc / dca;
        float gb = dfc / dcb;

        float ay = c->y + (a->y - c->y) * ga;
        float az = c->z + (a->z - c->z) * ga;
        float by = c->y + (b->y - c->y) * gb;
        float bz = c->z + (b->z - c->z) * gb;

        // vector from projected a to projected b
        float dy = by - ay, dz = bz - az;


        // now we do 2D intersection of segment and cube

        bool pointAInFace = ay >= minY && ay <= maxY && az >= minZ && az <= maxZ;
        bool pointBInFace = by >= minY && by <= maxY && bz >= minZ && bz <= maxZ;
        bool pointInFace = pointAInFace || pointBInFace;

        if (pointInFace)
            // this means at least one triangle edge gos through the face
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
    bool vAInVoxel = A->x >= vMin.x && A->x <= vMax.x && A->y >= vMin.y && A->y <= vMax.y && A->z >= vMin.z && A->z <= vMax.z;
    bool vBInVoxel = B->x >= vMin.x && B->x <= vMax.x && B->y >= vMin.y && B->y <= vMax.y && B->z >= vMin.z && B->z <= vMax.z;
    bool vCInVoxel = C->x >= vMin.x && C->x <= vMax.x && C->y >= vMin.y && C->y <= vMax.y && C->z >= vMin.z && C->z <= vMax.z;

    bool vertexInVoxel = vAInVoxel || vBInVoxel || vCInVoxel;

    Vector3D Ay = (Vector3D){A->y, A->z, A->x}, By = (Vector3D){B->y, B->z, B->x}, Cy = (Vector3D){C->y, C->z, C->x};
    Vector3D Az = (Vector3D){A->z, A->x, A->y}, Bz = (Vector3D){B->z, B->x, B->y}, Cz = (Vector3D){C->z, C->x, C->y};
    if (vertexInVoxel)
        return true;
    // iterate voxel faces
    else if (faceIntersect(*A, *B, *C, vMin.x, vMin.y, vMax.y, vMin.z, vMax.z))
        return true;
    else if (faceIntersect(*A, *B, *C, vMax.x, vMin.y, vMax.y, vMin.z, vMax.z))
        return true;
    else if (faceIntersect(Ay, By, Cy, vMin.y, vMin.z, vMax.z, vMin.x, vMax.x))
        return true;
    else if (faceIntersect(Ay, By, Cy, vMax.y, vMin.z, vMax.z, vMin.x, vMax.x))
        return true;
    else if (faceIntersect(Az, Bz, Cz, vMin.z, vMin.x, vMax.x, vMin.y, vMax.y))
        return true;
    else if (faceIntersect(Az, Bz, Cz, vMax.z, vMin.x, vMax.x, vMin.y, vMax.y))
        return true;
    return false;
}



/**
 * @brief Calculates the next voxel to traverse in the octree structure based on the ray's direction and the current position.
 *
 * This function takes the current position (p), the ray direction (rd), the level of detail (lod), and the octree structure.
 * It calculates the current voxel coordinates, the ray offsets, and the distances to the edges of the voxel.
 * It then determines the minimum distance (d) to the next intersection plane and updates the minimum and maximum step values.
 * Finally, it returns the new position (p) after traversing the minimum distance (d) along the ray direction (rd).
 *
 * @param p The current position.
 * @param rd The ray direction.
 * @param lod The level of detail.
 * @param octree The octree structure.
 * @return Vector3D The new position after traversing the minimum distance along the ray direction.
 */
Vector3D voxelSkipLOD(Vector3D p, Vector3D rd, int lod, Octree octree)
{
    // calls++;
    
    // current voxel coords in integers
    int x, y, z;
    octreeCoordsLod(p, &x, &y, &z, octree, lod);

    // ray direction offsets 
    int sx = (rd.x >= 0 ? 1 : 0),
        sy = (rd.y >= 0 ? 1 : 0),
        sz = (rd.z >= 0 ? 1 : 0);

    // point in the 3 possible axis-aligned intersections planes in world space
    Vector3D pe = voxelOrigLod(x + sx, y + sy, z + sz, octree, lod);

    const float eps = 5e-6;
    float overshoot = 1.005;
    // distance to planes
    float dx = fabs(rd.x) > eps ? overshoot * (pe.x - p.x) / (rd.x) : 1e6;
    float dy = fabs(rd.y) > eps ? overshoot * (pe.y - p.y) / (rd.y) : 1e6;
    float dz = fabs(rd.z) > eps ? overshoot * (pe.z - p.z) / (rd.z) : 1e6;


    if(dx<=0) dx = 1e6;
    if(dy<=0) dy = 1e6;
    if(dz<=0) dz = 1e6;


    float d = MIN(dx, dy);
          d = MIN(d, dz);

    // assert(d>=0);
    // minstep = MIN(d, minstep);
    // maxstep = MAX(d, maxstep);

    d += eps;

    Vector3D ret = (Vector3D){ p.x + d * rd.x,
                               p.y + d * rd.y,
                               p.z + d * rd.z};

    // int retx, rety, retz;
    // octreeCoordsLod(ret, &retx, &rety, &retz, octree, lod);

    // if (retx == x && rety == y && retz == z){
    //     // shouldn't be possible, but here we are
    //     // printf("broken\n");
    //     broken = true;
    // }

    return ret;
}

// int minB;
// int triChecks;

bool inBounds(Vector3D p, Octree octree){
    return p.x >= octree.world_offset.x && p.x < octree.world_size.x + octree.world_offset.x 
        && p.y >= octree.world_offset.y && p.y < octree.world_size.y + octree.world_offset.y 
        && p.z >= octree.world_offset.z && p.z < octree.world_size.z + octree.world_offset.z;
}

/**
 * @brief check if a ray intersects some non-empty voxel in an octree structure
 *
 * @param ro ray origin
 * @param rd ray direction
 * @param octree octree structure
 * @return LinkedListNode* the content of the intersected voxel (NULL if no hit)
 */
HitResult rayCast_voxel_octree(Vector3D ro, Vector3D rd, Octree octree)
{
    bool hit = false;
    // minB = DEPTH;
    // minstep = 1e6;
    // maxstep = 0.;

    int maxLod = DEPTH - 1;
    // TODO: refactor octree system to allow flexibility over its bounds
    while (inBounds(ro, octree))
    {
        // if(broken){
        //     return NULL;
        // }
        int cellX, cellY, cellZ;
        octreeCoords(ro, &cellX, &cellY, &cellZ, octree);

        intptr_t *root = octree.data;
        intptr_t cell_off = 0;

        // TODO: smarter starting LOD depending on previous step?
        for (int b = maxLod; b >= 0; b--)
        {
            // minB = MIN(b, minB);

            int cx = (cellX >> b) & 1;
            int cy = (cellY >> b) & 1;
            int cz = (cellZ >> b) & 1;

            int loc = cx + (cy<<1) + (cz<<2);
            intptr_t *cell = root + cell_off;
            if (!cell[loc])
            {

                // 0 means empty, skip this cell and its children
                ro = voxelSkipLOD(ro, rd, b, octree);
                break;
            }
            else if (b == 0)
            {
                LinkedListNode * node = ((LinkedListNode *)cell[loc]);
                bool hit = false;
                #ifdef VOXELIZE_RENDER
                    HitResult res = rayTriangleIntersect(ro, rd, node->T);
                    res.P = ro;
                    return res;
                #endif
                HitResult res = (HitResult){.T = NULL, .t = 1e6};
                while (node != NULL)
                {
                    // triChecks++;
                    HitResult tri = rayTriangleIntersect(ro, rd, node->T);
                    if(tri.t>0 && tri.t<res.t)
                        res = tri;
                    node = node->next;
                }
                if(res.T != NULL)
                    return res;
                
        
                // no triangle intersection, keep going to next leaf
                ro = voxelSkipLOD(ro, rd, 0, octree);
                break;
            }
            else
            {
                // printf("deeper...\n");
                root += 1 << 3 * (DEPTH-b);
                cell_off += loc;
                cell_off *= 8;
            }
        }
    }
    return (HitResult){.T =NULL, .t = -1};
    // no hits
}

/**
 * @brief Builds an octree structure from a set of triangles.
 *
 * This function takes an octree structure, an array of triangles, the number of triangles,
 * and a linked list of nodes. It iterates over each triangle, calculates its AABB (Axis-Aligned Bounding Box),
 * and checks for intersections with voxels within the octree. If an intersection is found, the triangle is added
 * to the linked list of the corresponding voxel.
 *
 * @param octree The octree structure to be built.
 * @param triangles An array of triangles to be voxelized and added to the octree.
 * @param triCount The number of triangles in the array.
 * @param nodes A linked list of nodes used to store the triangles in the octree.
 */
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
        Vector3D AABB_min = (Vector3D){MIN(A->x, (MIN(B->x, C->x))),
                                       MIN(A->y, (MIN(B->y, C->y))),
                                       MIN(A->z, (MIN(B->z, C->z)))};
        Vector3D AABB_max = (Vector3D){MAX(A->x, (MAX(B->x, C->x))),
                                       MAX(A->y, (MAX(B->y, C->y))),
                                       MAX(A->z, (MAX(B->z, C->z)))};
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
                    Vector3D vMax = voxelOrig(x + 1, y + 1, z + 1, octree);

                    // check if some part of the current triangles is inside the current voxel
                    if (voxelIntersect(A, B, C, vMin, vMax))
                    {
                        intptr_t *root = octree.data;
                        intptr_t cell_off = 0;
                        for (int b = DEPTH - 1; b >= 0; b--)
                        {
                            int cx = (x >> b) & 1;
                            int cy = (y >> b) & 1;
                            int cz = (z >> b) & 1;

                            int loc = cx + (cy<<1) + (cz<<2);
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
                            root += 1 << 3 * (DEPTH-b);
                            cell_off += loc;
                            cell_off *= 8;
                        }
                    }
                }
            }
        }
    }
}

time_t start0 = 0;

int main_render(float *buffer, int frame)
{

    //TODO avoid rebuilding and allocating octree every frame

    time_t start = clock();
    if(start0 == 0)
        start0 = start;

    FrameBuffer fBuffer = (FrameBuffer){buffer, WIDTH, HEIGHT};

    // initialize triangle buffer
    Triangle *triangles = malloc(faceCount * sizeof(Triangle));
    LinkedListNode *nodes = malloc(faceCount * 512 * sizeof(LinkedListNode));

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

    time_t octree_build_end = clock();

    // printf("\nvoxelization ok\n");

    //  Traverse octree ( arbitrary ray)

    float a = (float)frame*.1;
    float d = 30.;
    Vector3D p = (Vector3D){sinf(a)*d, 0.01, -cosf(a)*d};
    // Vector3D rd = normalize((Vector3D){-.3, .05, -.5});
    // LinkedListNode *hit = rayCast_voxel_octree(p, rd, octree);
    // exit(0);

    // shadow direction
    Vector3D sd = normalize((Vector3D){0, 1, .5});

    #pragma omp parallel for
    for (int j = 0; j < HEIGHT; j++)
    {
        float v = ((float)HEIGHT / 2. - j) / (float)HEIGHT;
        for (int i = 0; i < WIDTH; i++)
        {
            float *px = frameBufferAt(fBuffer, i, j);
            float u = (i - (float)WIDTH / 2.) / (float)HEIGHT;
            // normalize is not required, but it feels wrong to omit it
            Vector3D rd = normalize((Vector3D){u, v, .5});

            float x = rd.x*cosf(a) - rd.z*sinf(a);
            float z = rd.x*sinf(a) + rd.z*cosf(a);
            rd.x = x;
            rd.z = z;


            HitResult hit = rayCast_voxel_octree(p, rd, octree);

            if (hit.T)
            {
                float l = MAX(dot(hit.N, sd),0);
                
                Vector3D rayStart = (Vector3D){hit.P.x + sd.x * .001, hit.P.y + sd.y * .001, hit.P.z + sd.z * .001};
                HitResult shadowHit = rayCast_voxel_octree(rayStart, sd, octree);

                if(shadowHit.T)
                    l *= 0.;
                l+=.05;

                px[0] = l, px[1] = l, px[2] = l;
            }
            else
            {
                px[0] = .5-.25*dot(rd,sd), px[1] = .5, px[2] = .5;   
            }
        }
    }
    time_t end = clock();

    #define AVG_WINDOW 100
    if (frame && frame % AVG_WINDOW == 0){
        printf("avg frame time: %f s\n\n", (float)(end - start0) / (CLOCKS_PER_SEC*AVG_WINDOW) );
        printf("FPS: %f\n", (float)AVG_WINDOW / ((float)(end - start0) / (CLOCKS_PER_SEC)) );
        start0 = end;
    }


    // printf("octree build time: %f s\n", (float)(octree_build_end - start) / CLOCKS_PER_SEC);
    // printf("rendering time: %f s\n", (float)(end - octree_build_end) / CLOCKS_PER_SEC);
    // printf("frame time: %f s\n\n", (float)(end - start) / CLOCKS_PER_SEC);

    if (0)
    {
        // debugging 2D slice of the octree
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