#include "rasterm.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

char *ascii = " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
int asciiLen = 92;

char *color = "\033";

char *full_color = "[1;48;2";
char *full_color256 = "[1;48;5";

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

#define BUFFER_LENGTH 2048

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
    int termColorBuffer[WIDTH][HEIGHT][2];
#pragma omp parallel for
    for (int i = 0; i < WIDTH; i++)
    {
        for (int j = 0; j < HEIGHT; j++)
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
                termColorBuffer[i][j][0] = 0;
                termColorBuffer[i][j][1] = 0;
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
            termColorBuffer[i][j][0] = charColor;
            termColorBuffer[i][j][1] = charid;
        }
    }
    char lineBuffer[BUFFER_LENGTH] = "";
    for (int j = 0; j < HEIGHT; j++)
    {
        lineBuffer[0] = 0;
        for (int i = 0; i < WIDTH; i++)
        {
            int charColor = termColorBuffer[i][j][0];
            int charid = termColorBuffer[i][j][1];
            int s = strlen(lineBuffer);
            if (s + 16 >= BUFFER_LENGTH)
            {
                printf("%s", lineBuffer);
                s = 0;
                lineBuffer[0] = 0;
            }
            sprintf(lineBuffer + s, "%s%s%c", color, colors[charColor], ascii[charid]);
        }
        printf("%s%s%s\n", lineBuffer, color, defaultColor);
    }
}

void print_grayscale(float buffer[WIDTH][HEIGHT][4])
{
    char lineBuffer[BUFFER_LENGTH] = "";
    for (int j = 0; j < HEIGHT; j++)
    {
        lineBuffer[0] = 0;
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
            int s = strlen(lineBuffer);

            if (s + 16 >= BUFFER_LENGTH)
            {
                printf("%s", lineBuffer);
                s = 0;
                lineBuffer[0] = 0;
            }

            sprintf(lineBuffer + s, "%c", ascii[charid]);
        }
        printf("%s\n%s%s", lineBuffer, color, defaultColor);
    }
}

// #define DITHER
#define LINEAR_DITHER

void print_fc(float buffer[WIDTH][HEIGHT][4])
{
    uint8_t fullColorBuffer[WIDTH][HEIGHT];
    for (int i = 0; i < WIDTH; i++)
    {
        for (int j = 0; j < HEIGHT; j++)
        {
            float r = buffer[i][j][0];
            r = r < 0. ? 0. : r;
            float g = buffer[i][j][1];
            g = g < 0. ? 0. : g;
            float b = buffer[i][j][2];
            b = b < 0. ? 0. : b;

#ifdef DITHER
            float dr = bayer(i, j, 5);
            float dg = bayer(i, j, 6);
            float db = bayer(i, j, 4);

#ifdef LINEAR_DITHER
            float R = sqrtf(r);
            float G = sqrtf(g);
            float B = sqrtf(b);

            R = (int)(R * 6) / 6.0f;
            G = (int)(G * 6) / 6.0f;
            B = (int)(B * 6) / 6.0f;

            float r0 = (int)R * (int)R;
            float r1 = r0 + 2 * (int)R + 1;

            float g0 = (int)G * (int)G;
            float g1 = g0 + 2 * (int)G + 1;

            float b0 = (int)B * (int)B;
            float b1 = b0 + 2 * (int)B + 1;

            float rf = (r - r0) / (r1 - r0);
            float gf = (g - g0) / (g1 - g0);
            float bf = (b - b0) / (b1 - b0);
#else

            r = sqrtf(r) * 6;
            g = sqrtf(g) * 6;
            b = sqrtf(b) * 6;

            float R = (int)(r) / 6.0f;
            float G = (int)(g) / 6.0f;
            float B = (int)(b) / 6.0f;

            float rf = r / 6.0f - R;
            float gf = g / 6.0f - G;
            float bf = b / 6.0f - B;
#endif // LINEAR_DITHER

            r = R + (int)(rf > dr) / 6.0f;
            g = G + (int)(gf > dg) / 6.0f;
            b = B + (int)(bf > db) / 6.0f;
#else
            r = sqrtf(r);
            g = sqrtf(g);
            b = sqrtf(b);
#endif // DITHER

            r = r > 1. ? 1. : r;
            g = g > 1. ? 1. : g;
            b = b > 1. ? 1. : b;

            uint8_t c215 = ((int)(r * 5 + .5) * 36) + (int)(g * 5 + .5) * 6 + (int)(b * 5 + .5);
            fullColorBuffer[i][j] = 16 + c215;
        }
    }
    char lineBuffer[BUFFER_LENGTH] = "";
    lineBuffer[0] = 0;
    for (int j = 0; j < HEIGHT; j++)
    {
        for (int i = 0; i < WIDTH; i++)
        {
            uint8_t c256 = fullColorBuffer[i][j];
            int s = strlen(lineBuffer);

            if (s + 16 >= BUFFER_LENGTH)
            {
                printf("%s", lineBuffer);
                s = 0;
                lineBuffer[0] = 0;
            }

            // sprintf(lineBuffer + s,"%s%s%u;%u;%um ", color, full_color, (int)(g * 255), (int)(g * 255), (int)(b * 255));
            sprintf(lineBuffer + s, "%s%s;%um ", color, full_color256, c256);
        }
        sprintf(lineBuffer + strlen(lineBuffer), "%s%s\n", color, defaultColor);
    }
    printf("%s", lineBuffer);
}