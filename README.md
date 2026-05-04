# How to Compile and Run with Different Options

The only two files to touch are main.cpp and Renderer.h (you'll need to recompile each time).

The file main.cpp contains a list of all the scenes (and some comments on potential issues the scene has), simply uncomment the scene you want to run. You'll need to have them downloaded and in the root folder of this repository.

You can modify a few things in Renderer.h:

* The Renderer class has a string and three static members.

  * Change the string to: "direct" for only direct lighting, "albedo" to get an albedo map, "normal" to get a normal map, and "color" or anything else to get path tracing.
  * Change the "is\_using\_instant\_radiosity" variable to run using instant radiosity, the above string must be set to "direct". You can also change how many lights are sampled (0 samples all of them) and how many light paths are traced to generate VPLs.
* By default, the code runs multi-threaded. If you want to run light tracing or just single-threaded, you'll need to comment out the multi-threaded code in "render()" and uncomment the single threaded code. You can also run a single threaded continuous rendering, which will make the image progressively better.
* You can increase the number of samples by changing the first line of the "render()" function (1 by default).
* Uncomment the "#define DENOISE" to denoise the ray traced image. You'll need OpenImageDenoise installed and properly linked at compilation.

