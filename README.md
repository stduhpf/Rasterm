# Rasterized Toy Renderer

A very simple software renderer using single-threaded CPU rasterization, and outputting result as ASCII art to stdout. Some 3D models are "transpiled" from wavefront object to C header files using code stolen from [olive.c](https://github.com/tsoding/olive.c/tree/master/tools), because I didn't want to lose time dealing with files at the start of this project.
This is just a toy project, don't expect good performance or usability.

I got the idea to do such a project from [olive.c](https://github.com/tsoding/olive.c). Check it out!

## Requirements:

Only a standard C compiler, and either Windows or a POSIX-compliant operating system.

## Output:

By default, it produces images like this:

```
                                        p
                                    $RPYF       Ax!m    dD&QKKZY
                                      p8NRKKKXH8NN$$8UQ5/
                                     XmHHmDW@&&QQQMM$88U
                                 9bAHRN@@@@@@@&&&QQMM$$mbh
                              tbX00WN%%&&&@@@@@&%%QQ000#HXO    b
                             POXDggNN%%%&&&@@@@@%%%WWW0gDRKd Vgq
                             POXDggNNN%Q&&&&&&&&&%%QNNMBBmmOMS
                 16BQQ8      PpKRRggNNQQQ&%%&&%%%%WWWgggRROh
                     #&0p    qVARRBBMMMQQW%%%QQQQWWN00$mKdx|
                      uQ@Q00Mw4A88$$$00WNNQQNNWWMMM#DDOkEo
                        HNN@@]hGGHH##gBBMMB000###AAAhaxC!
      999$$$$0000K        Tqw}PPpKbHHHDDHRRRAAAddEZZ{iH
    //999$$$$000KK=            jkk44hhhpqq6aaZZ1f[R
    S##GGGMMMQ8eee=               //vvCvvJ)}h2
     Sbbbbpppppeee
      5999p%%%kii
        999k55ir
          RRHH

```

## Usefull macros

### Graphics engine: (rasterm.h)

- `RASTERM_IMPLEMENTATION` : turns the header into a source file
- `ASPECT_RATIO=[float]` : changes the aspect ratio of the render (controls the width indirectly, TODO: move iot to renderer)
- `FAST_MATH`: use fast inverse square root algorithm (might not be actually faster on modern hardware)
- `PERSPECTIVE_UV` enables perspective-corrected uv mapping on 3D triangles (enabled by default in main.c)

### Renderer: (main.c)

- `HEIGHT=[int]` : change size of canvas (default value:64)
- `PX_ASPECT=[float]` : to adapt to the "aspect ratio" of the display's pixels (default 2.21 for terminal font) (controls the width indirectly)
- `WIDTH=[int]`: I advise not using this one, you should rather use `ASPECT_RATIO="(float)[int]/HEIGHT"`, or you will have to deal with the pixel aspect ratio too
- `FRAMES=[n]` : Draws n consecutive animated frames (recommended) (default: 1, ignored when using RENDER_TARGET=GDI)
- `RENDER_TARGET=[GRAYSCALE|COLOR|FULL_COLOR|GDI(WINDOWS ONLY)]`: Change output mode (default: GRAYSCALE) (when using gdi, please link with gdi32)
- `STEP_RENDER` : Refreshes the display after each triangle drawn on canvas (looks cool, but is very slow)
- `FPS` (GDI only): number of frames to be displayed per second. (defaults to 60)
- `TEXTURED`: display the teapot uv-mapped with `./texture1.png`.
- `SHADOWMAP_DEMO`: renders a shadowmap, and add shadows to the table 
- `SHADOW_RES=[int]`: resolution of the shadowmap (square) (defaults to 256)

### Display lib: (printImg.h)

- `DITHER` : enables dithering for full color terminal rendering

## Build procedure:

To compile with de default config, you only need to link with libmath:

```sh
[your c compiler] -o threed main.c -lm
./threed
```

If you want to change the settings, if your compiler supports it, you could define the config macros with compilation flags.
For example for a sequence of 500 dithered full color "high resolution" 16/9 images, you can try:

```sh
[your c compiler] -o threed main.c -lm -DHEIGHT=128 -DASPECT_RATIO="16./9." -DFRAMES=500 -DRENDER_TARGET=FULL_COLOR -DDITHER
./threed
```

To compile for GDI target on windows:

```sh
[your c compiler] -o window main.c -lm -lgdi32 -DHEIGHT=720 -DASPECT_RATIO="16./9." -DRENDER_TARGET=GDI -O3
./window
```

## Architecture:

The code is split in 3 parts:

- **The renderer**: located in `main.c`.
- **The graphics engine** : located in `rasterm.h`. It is the core of this project.
- **The terminal output library**: located in `printImg.h`. It is in charge of displaying the rendered images to the terminal.
- **The Wavefront .obj parser**: located in `parseObj.c`.

### rasterm.h

This is the most important file of this repo.
It is a STB-style library that contains the necessary tools for rasterizing and shading 3D triangles to a 2D buffer.
