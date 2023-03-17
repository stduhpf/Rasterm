# Rasterized Toy Renderer

A very simple software renderer using single-threaded CPU rasterization, and outputting result as ASCII art to stdout. All the rendering and display logic is in `main.c`, the 3D models are "transpiled" from wavefront object to C header files using code stolen from [olive.c](https://github.com/tsoding/olive.c/tree/master/tools), because I didn't want to lose time dealing with files.
This is just a toy project, don't expect good performance or usability.

I got the idea to do such a project from [olive.c](https://github.com/tsoding/olive.c). Check it out!

## Requirements:

Only a standard C compiler.

## Output:

It produces images like this:

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
