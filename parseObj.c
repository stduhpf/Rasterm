#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#ifndef RASTERM
#include "rasterm.h"
#endif // RASTERM

typedef struct
{
    int A;
    int B;
    int C;
    int An;
    int Bn;
    int Cn;
    int Auv;
    int Buv;
    int Cuv;
} Face;

bool countObjects(char *filename, int *vertexCount, int *normalCount, int *faceCount, int *uvCount)
{
    FILE *fptr;

    // Open a file in read mode
    fptr = fopen(filename, "r");

    if (fptr == NULL)
    {
        printf("file can't be opened \n");
        return false;
    }
    char ch;
    do
    {
        switch (ch = fgetc(fptr))
        {
        case '\n':
            continue;
        case '#':
        {
            // ignore comments
            break;
        }
        case 'v':
        {
            switch (ch = fgetc(fptr))
            {
            case ' ':
            {
                (*vertexCount)++;
                break;
            }
            case 't':
            {
                (*uvCount)++;
                break;
            }
            case 'n':
            {
                (*normalCount)++;
                break;
            }
            }
            break;
        }
        case 'f':
        {
            (*faceCount)++;
            break;
        }
        }
        do
        {
            if (ch == EOF)
                break;
            ch = fgetc(fptr);
            // printf("%c", ch);
        } while (ch != '\n');
    } while (ch != EOF);
    fclose(fptr);
    return true;
}

bool parseObjects(char *filename, Vector3D *vertices, Vector3D *normals, Face *faces, Vector2D *uvs)
{
    int vertexIndex = 0;
    int normalIndex = 0;
    int faceIndex = 0;
    int uvIndex = 0;
    FILE *fptr;

    // Open a file in read mode
    fptr = fopen(filename, "r");

    if (fptr == NULL)
    {
        printf("file can't be opened \n");
        return false;
    }
    char ch;
    char numBuffer[16] = "";
    do
    {
        switch (ch = fgetc(fptr))
        {
        case '\n':
            continue;
        case '#':
        {
            break;
        }
        case 'v':
        {
            switch (ch = fgetc(fptr))
            {
            case ' ':
            {
                float coords[3];
                for (int i = 0; i < 3; ++i)
                {
                    while ((ch = fgetc(fptr)) == ' ')
                        if (ch == EOF)
                            break;
                    ;
                    size_t c = 0;
                    while (ch != ' ')
                    {
                        numBuffer[c++] = ch;
                        ch = fgetc(fptr);

                        if (ch == '\n')
                            break;
                    }
                    numBuffer[c] = '\0';
                    coords[i] = atof(numBuffer);
                }
                // printf("Vertex %d\tx=%f\ty=%f\tz=%f\n", vertexIndex + 1, coords[0], coords[1], coords[2]);
                vertices[vertexIndex] = (Vector3D){coords[0], coords[1], coords[2]};
                vertexIndex++;
                break;
            }
            case 't':
            {
                float coords[2];
                for (int i = 0; i < 2; ++i)
                {
                    while ((ch = fgetc(fptr)) == ' ')
                        if (ch == EOF)
                            break;
                    ;
                    size_t c = 0;
                    while (ch != ' ')
                    {
                        numBuffer[c++] = ch;
                        ch = fgetc(fptr);

                        if (ch == '\n')
                            break;
                    }
                    numBuffer[c] = '\0';
                    coords[i] = atof(numBuffer);
                }
                // printf("UV %d\tu=%f\tv=%f\n", uvIndex + 1, coords[0], coords[1]);
                uvs[uvIndex] = (Vector2D){coords[0], coords[1]};
                uvIndex++;
                break;
            }
            case 'n':
            {
                float components[3];
                for (int i = 0; i < 3; ++i)
                {
                    while ((ch = fgetc(fptr)) == ' ')
                        if (ch == EOF)
                            break;

                    size_t c = 0;
                    while (ch != ' ')
                    {
                        numBuffer[c++] = ch;
                        ch = fgetc(fptr);

                        if (ch == '\n')
                            break;
                    }
                    numBuffer[c] = '\0';
                    components[i] = atof(numBuffer);
                }
                // printf("Vertex Normal %d\tx=%f\ty=%f\tz=%f\n", normalIndex + 1, components[0], components[1], components[2]);
                normals[normalIndex] = (Vector3D){components[0], components[1], components[2]};
                normalIndex++;
                break;
            }
            }
            break;
        }
        case 'f':
        {
            int faceVertices[3];
            int faceUVs[3] = {0, 0, 0};
            int faceNormals[3] = {0, 0, 0};
            for (int i = 0; i < 3; ++i)
            {
                while ((ch = fgetc(fptr)) == ' ')
                    if (ch == EOF)
                        break;
                ;
                size_t c = 0;
                bool vertexSet = false;
                bool uvSet = false;

                while (ch != ' ')
                {
                    numBuffer[c++] = ch;
                    ch = fgetc(fptr);
                    if (ch == '/') // TODO: support vertex uv?
                    {
                        numBuffer[c] = '\0';
                        faceVertices[i] = atoi(numBuffer);
                        vertexSet = true;
                        c = 0;
                        ch = fgetc(fptr);
                        while (ch != ' ')
                        {
                            numBuffer[c++] = ch;
                            ch = fgetc(fptr);
                            if (ch == '/')
                            {
                                numBuffer[c] = '\0';
                                faceUVs[i] = atoi(numBuffer);
                                uvSet = true;
                                c = 0;
                                ch = fgetc(fptr);
                                while (ch != ' ')
                                {
                                    numBuffer[c++] = ch;
                                    ch = fgetc(fptr);
                                    if (ch == '\n')
                                        break;
                                }
                                numBuffer[c] = '\0';
                                faceNormals[i] = atoi(numBuffer);
                            }
                            if (ch == '\n')
                                break;
                        }
                        if (!uvSet)
                        {
                            numBuffer[c] = '\0';
                            faceUVs[i] = atoi(numBuffer);
                        }
                    }
                    if (ch == '\n')
                        break;
                }
                if (!vertexSet)
                {
                    numBuffer[c] = '\0';
                    faceVertices[i] = atoi(numBuffer);
                }
            }
            // printf("face\tA=%d\tB=%d\tC=%d\t\tAn=%d\tBn=%d\tCn=%d\n", faceVertices[0], faceVertices[1], faceVertices[2], faceNormals[0], faceNormals[1], faceNormals[2]);
            faces[faceIndex].A = faceVertices[0];
            faces[faceIndex].B = faceVertices[1];
            faces[faceIndex].C = faceVertices[2];
            faces[faceIndex].An = faceNormals[0];
            faces[faceIndex].Bn = faceNormals[1];
            faces[faceIndex].Cn = faceNormals[2];
            faces[faceIndex].Auv = faceUVs[0];
            faces[faceIndex].Buv = faceUVs[1];
            faces[faceIndex].Cuv = faceUVs[2];

            faceIndex++;
            break;
        }
        default:
            break;
        }
        while (ch != '\n')
        {
            if (ch == EOF)
                break;
            ch = fgetc(fptr);
        }
    } while (ch != EOF);

    fclose(fptr);
    return true;
}