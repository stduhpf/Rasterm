# Rasterized Toy Renderer

A very simple software renderer using single-threaded CPU rasterization, and outputting result as ASCII art to stdout. All the rendering and display logic is in `main.c`, the 3D models are "transpiled" from wavefront object to C header files using code stolen from [olive.c](https://github.com/tsoding/olive.c/tree/master/tools) because I didn't want to lose time dealing with files.
This is just a toy project, don't expect good performance or usability.

I got the idea to do such a project from [olive.c](https://github.com/tsoding/olive.c). Check it out! 

## Requirements:

Only a standard C compiler.

