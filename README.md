# Simple Ray Tracer in C++

This project implements a simple ray tracer in C++ that renders a scene with a sphere into a PPM image file.

## Features

- Basic vector math library (`vec3.h`)
- Ray class (`ray.h`)
- Sphere intersection logic
- PPM image output

## Prerequisites

- CMake
- C++ Compiler (g++, clang++, etc.)

## Building

To build the project:

```bash
mkdir -p build
cd build
cmake ..
make
```

## Running

To run the ray tracer and generate an image:

```bash
./build/hello > image.ppm
```

The output image `image.ppm` can be viewed with any PPM viewer or converted to other formats.
