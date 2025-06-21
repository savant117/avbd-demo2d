# avbd-demo2d

This is a simple 2D implementation of Augmented Vertex Block Decent (AVBD).

For more details on the technique (including a pre-built web demo) see the project page: https://graphics.cs.utah.edu/research/projects/avbd/

This repository is not intended to be a super optimized implementation, but an easy to understand demonstration of how to implement the technique.

## Building

Checkout the code and submodules using:

```git clone --recurse-submodules https://github.com/savant117/avbd-demo2d```

Make sure you have cmake and a c++ compiler installed.

To build:

### Native

```
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

To run, launch Release/avbd_demo2d.

### Web

Install emscripten: https://emscripten.org/docs/getting_started/downloads.html

Install ninja

On Windows, ninja can be installed with:

```winget install Ninja-build.Ninja```

To build:

```
mkdir build-web
cd build-web
emcmake cmake ..
ninja
```

To run, open avbd_demo2d.html in your browser.