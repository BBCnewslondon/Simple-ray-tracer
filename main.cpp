#include "vec3.h"
#include "color.h"
#include "sphere.h"
#include "hittable.h"
#include "material.h"
#include "lambertian.h"
#include "metal.h"
#include "dielectric.h"
#include "black_hole.h"

#include <iostream>
#include <limits>
#include <vector>
#include <memory>
#include <omp.h>
#include <algorithm>

color ray_color(const ray& r, const std::vector<std::shared_ptr<hittable>>& world, const black_hole& bh, int depth) {
    if (depth <= 0) return color(0,0,0);

    double dt = 0.05; 
    double max_dist = 100.0;
    double current_dist = 0.0;
    
    point3 curr_pos = r.origin();
    vec3 curr_dir = unit_vector(r.direction());

    // NEW: Accumulator for the glowing gas
    color glow(0,0,0); 

    while (current_dist < max_dist) {
        // Check for intersection in this step
        hit_record rec;
        bool hit_anything = false;
        double closest_so_far = dt; 
        
        ray step_ray(curr_pos, curr_dir);
        
        for (const auto& object : world) {
            if (object->hit(step_ray, 0.001, closest_so_far, rec)) {
                hit_anything = true;
                closest_so_far = rec.t;
            }
        }
        
        // If we hit a solid object (planet), return its color PLUS the glow we accumulated so far
        if (hit_anything) {
            ray scattered;
            color attenuation;
            if (rec.mat_ptr->scatter(step_ray, rec, attenuation, scattered)) {
                color result = glow + attenuation * ray_color(scattered, world, bh, depth-1);
                result[0] = std::min(1.0, std::max(0.0, result[0]));
                result[1] = std::min(1.0, std::max(0.0, result[1]));
                result[2] = std::min(1.0, std::max(0.0, result[2]));
                return result;
            }
            color result = glow;
            result[0] = std::min(1.0, std::max(0.0, result[0]));
            result[1] = std::min(1.0, std::max(0.0, result[1]));
            result[2] = std::min(1.0, std::max(0.0, result[2]));
            return result;
        }

        double dist_to_hole = (curr_pos - bh.center).length();

        // ---------------------------------------------------------
        // NEW: Accretion Disk Physics
        // ---------------------------------------------------------
        // Define the disk: A flat region in XZ plane (y ~ 0)
        // Inner Radius: 2.6 * Rs (ISCO - Innermost Stable Circular Orbit)
        // Outer Radius: 12.0 * Rs
        double disk_height = 0.5;
        double inner_radius = 2.6 * bh.rs;
        double outer_radius = 12.0 * bh.rs;

        // Check if ray is physically inside the disk volume
        if (std::abs(curr_pos.y() - bh.center.y()) < disk_height && 
            dist_to_hole > inner_radius && 
            dist_to_hole < outer_radius) {
            
            // Gas Density: Hotter and denser near the center (1/r^2 falloff)
            double density = 1.0 / (dist_to_hole * dist_to_hole);
            
            // Color: "Blackbody" Orange (High Red/Green, Low Blue)
            color disk_color(1.0, 0.5, 0.1); 

            // Accumulate light: Color * Density * StepSize
            glow += disk_color * density * dt * 20.0; // *20 is an intensity multiplier
        }
        // ---------------------------------------------------------

        if (dist_to_hole < bh.rs) {
            color result = glow;
            result[0] = std::min(1.0, std::max(0.0, result[0]));
            result[1] = std::min(1.0, std::max(0.0, result[1]));
            result[2] = std::min(1.0, std::max(0.0, result[2]));
            return result; // Return just the glow (the hole itself is black)
        }
        
        // Adaptive Step and Gravity Logic
        dt = std::max(0.01, dist_to_hole / 10.0); 
        curr_pos += curr_dir * dt;
        current_dist += dt;
        
        // Update direction (Gravity)
        vec3 to_bh = bh.center - curr_pos;
        double dist_sq = to_bh.length_squared();
        double dist = sqrt(dist_sq);
        
        if (dist > bh.rs) {
             vec3 force_dir = unit_vector(to_bh);
             // 2 * G * M / r^2, G=1
             double force_mag = (2.0 * bh.mass) / dist_sq;
             vec3 acceleration = force_dir * force_mag;
             curr_dir += acceleration * dt;
             curr_dir = unit_vector(curr_dir);
        }
    }

    // Background (Checkerboard)
    vec3 unit_direction = curr_dir;
    auto u = 0.5 + atan2(unit_direction.z(), unit_direction.x()) / (2*3.14159);
    auto v = 0.5 - asin(unit_direction.y()) / 3.14159;
    
    color bg_color = (sin(u * 50) * sin(v * 50) > 0) ? color(0,0,0) : color(1,1,1);
    
    color final_color = glow + bg_color;
    final_color[0] = std::min(1.0, std::max(0.0, final_color[0]));
    final_color[1] = std::min(1.0, std::max(0.0, final_color[1]));
    final_color[2] = std::min(1.0, std::max(0.0, final_color[2]));
    return final_color;
}

int main() {

    // Image
    const auto aspect_ratio = 16.0 / 9.0;
    const int image_width = 400;
    const int image_height = static_cast<int>(image_width / aspect_ratio);

    // Camera
    auto lookfrom = point3(0,0,0);
    auto lookat = point3(0,0,-1);
    auto vup = vec3(0,1,0);
    auto vfov = 90.0;
    auto aperture = 0.1;
    auto focus_dist = 1.0;

    auto theta = degrees_to_radians(vfov);
    auto h = tan(theta/2);
    auto viewport_height = 2.0 * h * focus_dist;
    auto viewport_width = aspect_ratio * viewport_height;

    auto w = unit_vector(lookfrom - lookat);
    auto u = unit_vector(cross(vup, w));
    auto v = cross(w, u);

    auto origin = lookfrom;
    auto horizontal = focus_dist * viewport_width * u;
    auto vertical = focus_dist * viewport_height * v;
    auto lower_left_corner = origin - horizontal/2 - vertical/2 - focus_dist * w;

    // World
    std::vector<std::shared_ptr<hittable>> world;

    black_hole bh(point3(0, 1, -2), 0.3);

    auto material_ground = std::make_shared<lambertian>(color(0.8, 0.8, 0.0));
    auto material_center = std::make_shared<lambertian>(color(0.1, 0.2, 0.5));
    auto material_left   = std::make_shared<dielectric>(1.5);
    auto material_right  = std::make_shared<metal>(color(0.8, 0.6, 0.2), 0.0);

    world.push_back(std::make_shared<sphere>(point3( 0.0, -100.5, -1.0), 100.0, material_ground));
    world.push_back(std::make_shared<sphere>(point3( 0.0,    0.0, -1.0),   0.5, material_center));
    world.push_back(std::make_shared<sphere>(point3(-1.0,    0.0, -1.0),   0.5, material_left));
    world.push_back(std::make_shared<sphere>(point3( 1.0,    0.0, -1.0),   0.5, material_right));

    // Render
    std::vector<color> pixels(image_width * image_height);

    std::cout << "P3\n" << image_width << " " << image_height << "\n255\n";

#pragma omp parallel for schedule(dynamic)
    for (int j = image_height-1; j >= 0; --j) {
        for (int i = 0; i < image_width; ++i) {
            auto rd = random_in_unit_disk() * (aperture / 2);
            auto offset = u * rd.x() + v * rd.y();
            auto dir = lower_left_corner + (double(i)/(image_width-1))*horizontal + (double(j)/(image_height-1))*vertical - origin - offset;
            ray r(origin + offset, dir);
            color pixel_color = ray_color(r, world, bh, 50);
            pixels[j * image_width + i] = pixel_color;
        }
    }

    for (const auto& c : pixels) {
        write_color(std::cout, c);
    }

    std::cerr << "\nDone.\n";
}