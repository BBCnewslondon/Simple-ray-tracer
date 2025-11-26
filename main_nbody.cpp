#include "vec3.h"
#include "color.h"
#include "sphere.h"
#include "hittable.h"
#include "material.h"
#include "lambertian.h"
#include "metal.h"
#include "dielectric.h"
#include "nbody.h" // <--- Include your physics engine

#include <iostream>
#include <vector>
#include <memory>
#include <omp.h>
#include <fstream>
#include <string>

// Helper to map Particle color to Material
std::shared_ptr<material> make_material(color c) {
    // If it's very bright (like the Sun), make it emit light (diffuse for now)
    if (c.x() > 0.9 && c.y() > 0.9) 
        return std::make_shared<lambertian>(c); // In real raytracing, use diffuse_light
    return std::make_shared<lambertian>(c);
}

color ray_color(const ray& r, const std::vector<std::shared_ptr<hittable>>& world, int depth) {
    hit_record rec;
    if (depth <= 0) return color(0,0,0);

    bool hit_anything = false;
    double closest_so_far = 1e10;

    // Find the closest object
    for (const auto& object : world) {
        if (object->hit(r, 0.001, closest_so_far, rec)) {
            hit_anything = true;
            closest_so_far = rec.t;
        }
    }

    if (hit_anything) {
        // Flat shading: return albedo directly
        auto lambert = std::dynamic_pointer_cast<lambertian>(rec.mat_ptr);
        if (lambert) return lambert->albedo;
        return color(0,0,0);
    }

    // Starfield Background
    vec3 unit_direction = unit_vector(r.direction());
    auto t = 0.5*(unit_direction.y() + 1.0);
    // Deep space black/blue gradient
    return (1.0-t)*color(0.05, 0.05, 0.1) + t*color(0.1, 0.1, 0.2);
}

int main() {
    // Image Dimensions
    const auto aspect_ratio = 16.0 / 9.0;
    const int image_width = 400;
    const int image_height = static_cast<int>(image_width / aspect_ratio);

    // 1. Setup Physics Universe
    Universe solar_system;
    
    // Sun (Mass 1000, Radius 2, Yellow)
    solar_system.add_particle(Particle(point3(0,0,0), vec3(0,0,0), 1000.0, 2.0, color(1, 0.9, 0.2)));
    
    // Earth (Dist 10, Vel 10, Mass 1, Radius 0.5, Blue)
    // Velocity for circular orbit: v = sqrt(GM/r) = sqrt(1000/10) = 10
    solar_system.add_particle(Particle(point3(10,0,0), vec3(0,0,10), 1.0, 0.5, color(0.2, 0.4, 1.0)));

    // Mars (Dist 15, Mass 0.1, Radius 0.3, Red)
    // v = sqrt(1000/15) = 8.16
    solar_system.add_particle(Particle(point3(15,0,0), vec3(0,0,8.16), 0.1, 0.3, color(0.8, 0.3, 0.1)));

    // Jupiter (Dist 30, Mass 50, Radius 1.5, Orange)
    // v = sqrt(1000/30) = 5.77
    solar_system.add_particle(Particle(point3(30,0,0), vec3(0,0,5.77), 50.0, 1.5, color(0.8, 0.6, 0.4)));

    // 2. Animation Loop
    int total_frames = 200;
    double dt = 0.05; // Simulation time step per frame

    for (int frame = 0; frame < total_frames; ++frame) {
        
        // A. Physics Step
        // Run multiple physics sub-steps per frame for stability
        int sub_steps = 10; 
        for(int s=0; s<sub_steps; s++) {
            solar_system.step(dt / sub_steps);
        }

        // B. Sync Physics -> Graphics
        // Rebuild the 'world' list every frame with new positions
        std::vector<std::shared_ptr<hittable>> world;
        for (const auto& p : solar_system.particles) {
            auto mat = make_material(p.albedo);
            world.push_back(std::make_shared<sphere>(p.position, p.radius, mat));
        }

        // C. Update Camera (Orbit around the system slightly to keep view dynamic)
        // Camera looks at Sun (0,0,0) from high up
        point3 lookfrom(0, 40, 20); 
        point3 lookat(0, 0, 0);
        vec3 vup(0, 1, 0);
        auto dist_to_focus = (lookfrom - lookat).length();
        auto aperture = 0.0; // Pinhole camera (sharp)

        // Camera Vectors
        auto w = unit_vector(lookfrom - lookat);
        auto u = unit_vector(cross(vup, w));
        auto v = cross(w, u);
        auto vfov = 40.0; // Wide angle to see the planets
        auto theta = degrees_to_radians(vfov);
        auto h = tan(theta/2);
        auto viewport_height = 2.0 * h * dist_to_focus;
        auto viewport_width = aspect_ratio * viewport_height;
        
        auto origin = lookfrom;
        auto horizontal = viewport_width * u;
        auto vertical = viewport_height * v;
        auto lower_left_corner = origin - horizontal/2 - vertical/2 - dist_to_focus*w;

        // D. Render Frame
        std::string filename = "output/nbody_" + std::to_string(frame) + ".ppm";
        std::ofstream out(filename);
        out << "P3\n" << image_width << " " << image_height << "\n255\n";

        std::vector<color> pixels(image_width * image_height);

        #pragma omp parallel for schedule(dynamic)
        for (int j = image_height-1; j >= 0; --j) {
            for (int i = 0; i < image_width; ++i) {
                auto u_coord = double(i) / (image_width-1);
                auto v_coord = double(j) / (image_height-1);
                ray r(origin, lower_left_corner + u_coord*horizontal + v_coord*vertical - origin);
                
                // Low samples for speed (increase for quality)
                color pixel_color = ray_color(r, world, 10); 
                pixels[j * image_width + i] = pixel_color;
            }
        }

        for (const auto& c : pixels) {
            write_color(out, c);
        }
        out.close();
        std::cerr << "Frame " << frame << " rendered.\n";
    }
}