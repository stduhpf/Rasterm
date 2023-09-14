#ifndef HEIGHT
#define HEIGHT 64
#endif
#ifndef WIDTH
#define WIDTH 64
#endif

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

typedef void (*FragmentShader)(float bufferColor[4], int x, int y, Vector2D uv, float inverseDepth, SurfaceAttributes attribs, SceneAttributes scene);

#define MIN(a, b) (a < b ? a : b)
#define MAX(a, b) (a > b ? a : b)

float min4(float a, float b, float c, float d);
float max4(float a, float b, float c, float d);
float get_det(Vector2D a, Vector2D b);
float dot2(Vector2D a, Vector2D b);
float dot(Vector3D a, Vector3D b);
Vector3D cross(Vector3D a, Vector3D b);
Vector3D normalize(Vector3D a);
Vector2D barycentric(Vector2D p, Vector2D a, Vector2D b, Vector2D c);
void triangleShader(float bufferColor[4], int x, int y, Vector2D uv, float iza, float izb, float izc, SurfaceAttributes attribs, SceneAttributes scene);
void triangle2D(float buffer[WIDTH][HEIGHT][4], Vector2D a, Vector2D b, Vector2D c, float iza, float izb, float izc, SurfaceAttributes attribs, SceneAttributes scene);
Vector3D getWorldPos(Vector3D modelPos, Vector3D origin, float rotXY, float rotXZ, float rotYZ);
Vector3D getViewPos(Vector3D wolrdPos, Camera cam);
Vector3D getViewDir(Vector3D wolrdDir, Camera cam);
Vector3D getNormal(Vector3D a, Vector3D b, Vector3D c);
Vector3D project(Vector3D p, const float screenZ);
void triangle3D(float buffer[WIDTH][HEIGHT][4], Vector3D A, Vector3D B, Vector3D C, Vector3D color, SceneAttributes scene);
void attachFragmentShader(FragmentShader fs);
void resetFragmentShader();

#ifdef RASTERM_IMPLEMENTATION

static void defaultFragmentShader(float bufferColor[4], int x, int y, Vector2D uv, float inverseDepth, SurfaceAttributes attribs, SceneAttributes scene)
{
    float illumination = 1.;
    if (dot(attribs.normal, attribs.normal) > 0.)
    {
        illumination = MAX(0., dot(attribs.normal, scene.lightVector)) + .05 * MAX(0., -attribs.normal.y);
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
#endif
void resetFragmentShader()
{
    fragmentShader = &defaultFragmentShader;
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
#ifdef PERSPECTIVE_UV
        uv.x *= iza / iz;
        uv.y *= izb / iz;
#endif
        (*fragmentShader)(bufferColor, x, y, uv, iz, attribs, scene);
    }
}

void triangle2D(float buffer[WIDTH][HEIGHT][4], Vector2D a, Vector2D b, Vector2D c, float iza, float izb, float izc, SurfaceAttributes attribs, SceneAttributes scene)
{
    if (iza < 0. || izb < 0. || izc < 0.)
        return;
    // triangle AABB estimation
    int xMin = (int)(MAX(min4(a.x, b.x, c.x, (float)(WIDTH - 1)), 0));
    int xMax = (int)(MIN(max4(a.x, b.x, c.x, 0), (float)(WIDTH - 1)));
    if (xMax < 0 || xMin >= WIDTH)
        return;

    int yMin = (int)(MAX(min4(a.y, b.y, c.y, (float)(HEIGHT - 1)), 0));
    int yMax = (int)(MIN(max4(a.y, b.y, c.y, 0), (float)(HEIGHT - 1)));
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

void triangle3D(float buffer[WIDTH][HEIGHT][4], Vector3D A, Vector3D B, Vector3D C, Vector3D color, SceneAttributes scene)
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

#endif