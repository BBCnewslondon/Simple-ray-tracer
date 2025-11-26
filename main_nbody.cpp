#include <iostream>
#include "nbody.h"

int main() {
    Universe solar_system;

    // 1. Define Bodies (using arbitrary units for simplicity)
    // Position, Velocity, Mass, Radius, Color
    Particle sun(point3(0,0,0), vec3(0,0,0), 1000.0, 2.0, color(1,1,0));
    
    // Earth starts at x=10, with velocity z=10 to orbit (v = sqrt(GM/r))
    // v = sqrt(1 * 1000 / 10) = 10.0
    Particle earth(point3(10,0,0), vec3(0,0,10), 1.0, 0.5, color(0,0,1));

    solar_system.add_particle(sun);
    solar_system.add_particle(earth);

    // 2. Run Simulation
    std::cout << "Time, Earth_X, Earth_Z, Radius\n";
    
    double dt = 0.01;
    for (int i = 0; i < 1000; ++i) {
        solar_system.step(dt);
        
        // Print CSV format to check orbits in Excel/Python
        if (i % 10 == 0) {
            auto p = solar_system.particles[1].position;
            std::cout << i * dt << ", " << p.x() << ", " << p.z() << ", " << p.length() << "\n";
        }
    }
}