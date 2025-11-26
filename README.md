# Simple Ray Tracer in C++

This project implements a simple ray tracer in C++ that renders a scene featuring a black hole with a relativistic accretion disk, spheres, and other objects. It includes animation capabilities to simulate orbiting the black hole.

## Features

- Basic vector math library (`vec3.h`)
- Ray class (`ray.h`)
- Sphere intersection logic
- Black hole with event horizon and gravitational lensing
- Relativistic accretion disk with Doppler beaming effects
- PPM image output
- Animation mode: generates 120-frame sequence orbiting the black hole
- Video rendering support using FFmpeg

## Prerequisites

- CMake
- C++ Compiler (g++, clang++, etc.)
- FFmpeg (for video rendering)

## Building

To build the project:

```bash
mkdir -p build
cd build
cmake ..
make
```

## Running

### Single Image

To run the ray tracer and generate a single image:

```bash
./build/hello > image.ppm
```

The output image `image.ppm` can be viewed with any PPM viewer or converted to other formats.

### Animation

To generate an animation sequence:

```bash
./build/hello
```

This will create 120 PPM files in the `output/` directory: `render_0.ppm` to `render_119.ppm`.

### Rendering Video

To convert the animation frames into an MP4 video:

```bash
ffmpeg -i output/render_%d.ppm -c:v libx264 -pix_fmt yuv420p animation.mp4
```

This produces `animation.mp4`, a video of the camera orbiting the black hole at a safe distance, showcasing the relativistic effects of the accretion disk.
