#ifndef BLACKHOLE_H
#define BLACKHOLE_H

#include "vec3.h"

class black_hole {
    public:
        point3 center;
        double mass;
        double rs; // Schwarzschild radius

        black_hole(point3 c, double m) : center(c), mass(m) {
            // Assuming G=1, c=1
            // Schwarzschild radius Rs = 2GM/c^2
            rs = 2 * mass;
        }
};

#endif
