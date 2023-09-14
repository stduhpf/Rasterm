#ifndef RASTERM
#include "rasterm.h"
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>

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
    char lineBuffer[4096] = "";
    for (int j = 0; j < HEIGHT; j++)
    {
        lineBuffer[0] = 0;
        for (int i = 0; i < WIDTH; i++)
        {
            int charColor = termColorBuffer[i][j][0];
            int charid = termColorBuffer[i][j][1];
            sprintf(lineBuffer + strlen(lineBuffer), "%s%s%c", color, colors[charColor], ascii[charid]);
        }
        printf("%s\n%s%s", lineBuffer, color, defaultColor);
    }
}

void print_grayscale(float buffer[WIDTH][HEIGHT][4])
{
    char lineBuffer[4096] = "";
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
            sprintf(lineBuffer + strlen(lineBuffer), "%c", ascii[charid]);
        }
        printf("%s\n%s%s", lineBuffer, color, defaultColor);
    }
}
