#ifndef NBODY_H
#define NBODY_H

#include "vec3.h"
#include <vector>
#include <cmath>

struct Particle {
    point3 position;
    vec3 velocity;
    vec3 acceleration;
    double mass;
    double radius;      // For rendering later
    color albedo;       // Color of the planet

    Particle(point3 p, vec3 v, double m, double r, color c)
        : position(p), velocity(v), acceleration(point3(0,0,0)), mass(m), radius(r), albedo(c) {}
};

class Universe {
    public:
        std::vector<Particle> particles;
        double G = 1.0; // Gravitational Constant (set to 1 for simulation units)

        void add_particle(const Particle& p) {
            particles.push_back(p);
        }

        // The "Engine Room": Calculate forces between ALL pairs
        // Complexity: O(N^2)
        void compute_forces() {
            // 1. Reset accelerations
            for (auto& p : particles) {
                p.acceleration = vec3(0,0,0);
            }

            // 2. Accumulate Gravity
            for (size_t i = 0; i < particles.size(); ++i) {
                for (size_t j = i + 1; j < particles.size(); ++j) {
                    Particle& p1 = particles[i];
                    Particle& p2 = particles[j];

                    vec3 delta_pos = p2.position - p1.position;
                    double dist_sq = delta_pos.length_squared();
                    double dist = sqrt(dist_sq);

                    // Newton's Law: F = G * m1 * m2 / r^2
                    // F_vector = F * direction = (G m1 m2 / r^2) * (r_vec / r)
                    //          = G m1 m2 * r_vec / r^3
                    
                    // Avoid division by zero (softening parameter)
                    double softening = 1e-2; 
                    double f_mag = (G * p1.mass * p2.mass) / pow(dist + softening, 3);
                    
                    vec3 force = delta_pos * f_mag;

                    // F = ma -> a = F/m
                    p1.acceleration += force / p1.mass;
                    p2.acceleration -= force / p2.mass; // Newton's 3rd Law
                }
            }
        }

        // Symplectic Integrator (Velocity Verlet)
        // This is stable for orbits!
        void step(double dt) {
            // 1. First Half-Kick & Drift
            for (auto& p : particles) {
                p.velocity += p.acceleration * (0.5 * dt);
                p.position += p.velocity * dt;
            }

            // 2. Recompute Forces at new positions
            compute_forces();

            // 3. Second Half-Kick
            for (auto& p : particles) {
                p.velocity += p.acceleration * (0.5 * dt);
            }
        }
};

#endif