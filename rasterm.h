#ifndef RASTERM
#define RASTERM

#ifndef ASPECT_RATIO
#define ASPECT_RATIO 1
#endif // ASPECT_RATIO
#include <stdint.h>

typedef struct
{
    float *data;
    size_t width;
    size_t height;
} Framebuffer;

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
    Vector3D normalA;
    Vector3D normalB;
    Vector3D normalC;
    Vector2D uvA;
    Vector2D uvB;
    Vector2D uvC;
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

typedef struct
{
    Vector3D origin;
    float rotYZ;
    float rotXZ;
    float rotXY;
    Vector3D scale;
} ModelTransform;

typedef void (*FragmentShader)(float bufferColor[4], int x, int y, Vector2D uv, float inverseDepth, SurfaceAttributes attribs, SceneAttributes scene);
typedef Vector3D (*VertexShader)(Vector3D Vertex, Camera camera);

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define SCALE_VEC3(v3, s) \
    {                     \
        (v3).x *= (s);    \
        (v3).y *= (s);    \
        (v3).z *= (s);    \
    }

float *framebufferAt(Framebuffer buffer, size_t i, size_t j);

float min4(float a, float b, float c, float d);
float max4(float a, float b, float c, float d);
/* 2D outer product */
float get_det(Vector2D a, Vector2D b);
float dot2(Vector2D a, Vector2D b);
float dot(Vector3D a, Vector3D b);
Vector3D cross(Vector3D a, Vector3D b);
/* Returns a scaled copy of vector a with euclidean length = 1 */
Vector3D normalize(Vector3D a);

/* Get barycentric coordinates of point p in triangle abc (c being the "origin" of the barycentric coordinates) */
Vector2D barycentric(Vector2D p, Vector2D a, Vector2D b, Vector2D c);
/* Called during rasterization to check wether some pixel is actually part of the triangle, and if it passes the depth check.
 Also calls the currently enabled fragment shader to each pixel that passes the test  */
void triangleShader(float bufferColor[4], int x, int y, Vector2D uv, float iza, float izb, float izc, SurfaceAttributes attribs, SceneAttributes scene);
/* Rasterize 2D triangle abc onto the frame buffer, performing depth culling with the inverse z values of each vertex */
void triangle2D(Framebuffer buffer, Vector2D a, Vector2D b, Vector2D c, float iza, float izb, float izc, SurfaceAttributes attribs, SceneAttributes scene);
/* Converts points in model space to points in world space */
Vector3D model2WorldTransform(Vector3D modelPos, ModelTransform *transform);
/* Converts points in world space to points in view space */
Vector3D getViewPos(Vector3D worldPos, Camera cam);
/* Converts vectors in world space to vectors in view space */
Vector3D getViewDir(Vector3D worldDir, Camera cam);
/* Returns a vector normal to the surface passing through the 3 points a b and c */
Vector3D getNormal(Vector3D a, Vector3D b, Vector3D c);
/* Converts from view space to screen space (x, y, inverse depth) */
Vector3D project(Vector3D p, const float screenZ);
/* Project and rasterize the 3D triangle ABC in the scene onto the frame buffer */
void triangle3D(Framebuffer buffer, Vector3D A, Vector3D B, Vector3D C, SurfaceAttributes attribs, SceneAttributes scene);
/* Set the "vertex shader" that will be used on each vertex during the next call to triangle3D() */
void attachVertexShader(VertexShader fs);
/* Reset the vertex shader to the default simple shader */
void resetVertexShader();
/* Set the "fragment shader" that will be used during the next call to triangleShader() during rasterization */
void attachFragmentShader(FragmentShader fs);
/* Reset the fragment shader to the default flat lambert shading */
void resetFragmentShader();
/* Set the transform for placing the model in the world */
void attachModelTransform(ModelTransform *mt);
/* Reset the model transform to the default (no transform) */
void resetModelTransform();

#define RASTERM_IMPLEMENTATION
#ifdef RASTERM_IMPLEMENTATION

ModelTransform *modelTransform = (ModelTransform *)NULL;
void attachModelTransform(ModelTransform *mt)
{
    modelTransform = mt;
}
void resetModelTransform()
{
    modelTransform = (ModelTransform *)NULL;
}

/* default shader: simple lambertian diffuse with solid color */
static void defaultFragmentShader(float bufferColor[4], int x, int y, Vector2D uv, float inverseDepth, SurfaceAttributes attribs, SceneAttributes scene)
{
    float illumination = 1.;

    float w = (1. - uv.x - uv.y);
    Vector3D normal = attribs.normal;
    if (dot(attribs.normal, attribs.normal) > 0.)
    {
        illumination = MAX(0., dot(normal, scene.lightVector)) + .05 * MAX(0., -normal.y);
    }
    bufferColor[0] = (illumination * scene.direct.x + scene.ambient.x) * attribs.color.x;
    bufferColor[1] = (illumination * scene.direct.y + scene.ambient.y) * attribs.color.y;
    bufferColor[2] = (illumination * scene.direct.z + scene.ambient.z) * attribs.color.z;
    bufferColor[3] = inverseDepth;
}
FragmentShader fragmentShader = &defaultFragmentShader;
void attachFragmentShader(FragmentShader fs)
{
    fragmentShader = fs;
}
#ifndef PERSPECTIVE_UV
#define attachFragmentShader(fs) _Pragma("message \"\n\nNOTE:\n\tThe rasterizer uses affine mapping by default.\n\tDefine PERSPECTIVE_UV if your custom shader needs the correct texture coordinates.\n\"") \
    attachFragmentShader(fs)
#endif // PERSPECTIVE_UV
void resetFragmentShader()
{
    fragmentShader = &defaultFragmentShader;
}

/* default vertex shader: apply model transform and camera transform */
static Vector3D defaultVertexShader(Vector3D V, Camera c)
{
    if (modelTransform)
        V = model2WorldTransform(V, modelTransform);
    V = getViewPos(V, c);
    return project(V, c.focalLength);
}
VertexShader vertexShader = &defaultVertexShader;
void attachVertexShader(VertexShader vs)
{
    vertexShader = vs;
}
void resetVertexShader()
{
    vertexShader = &defaultVertexShader;
}


float *framebufferAt(Framebuffer buffer, size_t i, size_t j)
{
    return buffer.data + (i * buffer.height + j) * 4;
}

float min4(float a, float b, float c, float d)
{
    return MIN(MIN(a, b), MIN(c, d));
}

float max4(float a, float b, float c, float d)
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

#ifdef FAST_MATH
float Q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threeHalves = 1.5F;

    x2 = number * 0.5F;
    y = number;
    i = *(long *)&y;           // evil floating point bit level hacking
    i = 0x5f3759df - (i >> 1); // what the fuck?
    y = *(float *)&i;
    y = y * (threeHalves - (x2 * y * y)); // 1st iteration
                                          // y  = y * ( threeHalves - ( x2 * y * y ) );   // 2nd iteration, this can be removed

    return y;
}
#endif // FAST_MATH

Vector3D normalize(Vector3D a)
{
#ifdef FAST_MATH
    float il = Q_rsqrt(dot(a, a));
    SCALE_VEC3(a, il);
#else
    float l = sqrtf(dot(a, a));
    float il = 1. / l;
    SCALE_VEC3(a, il);
#endif // FAST_MATH
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
#ifdef PERSPECTIVE_UV
        uv.x *= iza / iz;
        uv.y *= izb / iz;
#endif // PERSPECTIVE_UV
        (*fragmentShader)(bufferColor, x, y, uv, iz, attribs, scene);
    }
}

void triangle2D(Framebuffer buffer, Vector2D a, Vector2D b, Vector2D c, float iza, float izb, float izc, SurfaceAttributes attribs, SceneAttributes scene)
{
    if (iza < 0. || izb < 0. || izc < 0.)
        return;
    // triangle AABB estimation
    int xMin = (int)(MAX(min4(a.x, b.x, c.x, (float)(buffer.width - 1)), 0));
    int xMax = (int)(MIN(max4(a.x, b.x, c.x, 0), (float)(buffer.width - 1)));
    if (xMax < 0 || xMin >= buffer.width)
        return;

    int yMin = (int)(MAX(min4(a.y, b.y, c.y, (float)(buffer.height - 1)), 0));
    int yMax = (int)(MIN(max4(a.y, b.y, c.y, 0), (float)(buffer.height - 1)));
    if (yMax < 0 || yMin >= buffer.height)
        return;

    for (int x = xMin; x <= xMax; x++)
    {
        for (int y = yMin; y <= yMax; y++)
        {
            Vector2D p = {x, y};
            Vector2D uv = barycentric(p, a, b, c);
            triangleShader(framebufferAt(buffer, x, y), x, y, uv, iza, izb, izc, attribs, scene);
        }
    }
}

Vector3D model2WorldTransform(Vector3D modelPos, ModelTransform *transform)
{
    modelPos.x = modelPos.x * transform->scale.x;
    modelPos.y = modelPos.y * transform->scale.y;
    modelPos.z = modelPos.z * transform->scale.z;

    Vector2D xy = {modelPos.x, modelPos.y};
    modelPos.x = cosf(transform->rotXY) * xy.x + sinf(transform->rotXY) * xy.y;
    modelPos.y = -sinf(transform->rotXY) * xy.x + cosf(transform->rotXY) * xy.y;

    Vector2D xz = {modelPos.x, modelPos.z};
    modelPos.x = cosf(transform->rotXZ) * xz.x + sinf(transform->rotXZ) * xz.y;
    modelPos.z = -sinf(transform->rotXZ) * xz.x + cosf(transform->rotXZ) * xz.y;

    Vector2D yz = {modelPos.y, modelPos.z};
    modelPos.y = cosf(transform->rotYZ) * yz.x + sinf(transform->rotYZ) * yz.y;
    modelPos.z = -sinf(transform->rotYZ) * yz.x + cosf(transform->rotYZ) * yz.y;

    Vector3D ret = {modelPos.x - transform->origin.x,
                    modelPos.y - transform->origin.y,
                    modelPos.z - transform->origin.z};
    return ret;
}

Vector3D getViewPos(Vector3D worldPos, Camera cam)
{
    worldPos.x -= cam.position.x;
    worldPos.y -= cam.position.y;
    worldPos.z -= cam.position.z;

    float rotXZ = cam.rotXZ;
    float rotYZ = cam.rotYZ;

    Vector2D xz = {worldPos.x, worldPos.z};
    worldPos.x = cosf(rotXZ) * xz.x + sinf(rotXZ) * xz.y;
    worldPos.z = -sinf(rotXZ) * xz.x + cosf(rotXZ) * xz.y;

    Vector2D yz = {worldPos.y, worldPos.z};
    worldPos.y = cosf(rotYZ) * yz.x + sinf(rotYZ) * yz.y;
    worldPos.z = -sinf(rotYZ) * yz.x + cosf(rotYZ) * yz.y;

    Vector3D ret = {worldPos.x,
                    worldPos.y,
                    worldPos.z};
    return ret;
}

Vector3D getViewDir(Vector3D worldDir, Camera cam)
{

    float rotXZ = cam.rotXZ;
    float rotYZ = cam.rotYZ;

    Vector2D xz = {worldDir.x, worldDir.z};
    worldDir.x = cosf(rotXZ) * xz.x + sinf(rotXZ) * xz.y;
    worldDir.z = -sinf(rotXZ) * xz.x + cosf(rotXZ) * xz.y;

    Vector2D yz = {worldDir.y, worldDir.z};
    worldDir.y = cosf(rotYZ) * yz.x + sinf(rotYZ) * yz.y;
    worldDir.z = -sinf(rotYZ) * yz.x + cosf(rotYZ) * yz.y;

    return (Vector3D){worldDir.x,
                      worldDir.y,
                      worldDir.z};
}

Vector3D getNormal(Vector3D a, Vector3D b, Vector3D c)
{
    Vector3D A = {a.x - c.x,
                  a.y - c.y,
                  a.z - c.z};
    Vector3D B = {c.x - b.x,
                  c.y - b.y,
                  c.z - b.z};
    return normalize(cross(B, A));
}

Vector3D project(Vector3D p, const float screenZ)
{
    Vector3D ret = {0, 0, 0};
    p.z = p.z;
    ret.x = (.5 + .5 * screenZ * p.x / p.z);
    ret.y = ((.5 + .5 * screenZ * p.y / p.z) * ASPECT_RATIO + (1.0f - ASPECT_RATIO) * .5);
    ret.z = screenZ / p.z;
    return ret;
}

void triangle3D(Framebuffer buffer, Vector3D A, Vector3D B, Vector3D C, SurfaceAttributes attribs, SceneAttributes scene)
{
    Vector3D Ap = vertexShader(A, scene.camera);
    Vector3D Bp = vertexShader(B, scene.camera);
    Vector3D Cp = vertexShader(C, scene.camera);

    Vector2D a = {Ap.x * buffer.width, Ap.y * buffer.height};
    Vector2D b = {Bp.x * buffer.width, Bp.y * buffer.height};
    Vector2D c = {Cp.x * buffer.width, Cp.y * buffer.height};

    triangle2D(buffer, a, b, c, Ap.z, Bp.z, Cp.z, attribs, scene);
}

#endif // RASTERM_IMPLEMENTATION
#endif // RASTERM