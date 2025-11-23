#include "vec3.h"
#include "color.h"
#include "sphere.h"
#include "hittable.h"
#include "material.h"
#include "lambertian.h"
#include "metal.h"
#include "dielectric.h"

#include <iostream>
#include <limits>
#include <vector>
#include <memory>

color ray_color(const ray& r, const std::vector<std::shared_ptr<hittable>>& world, int depth) {
    hit_record rec;

    // If we've exceeded the ray bounce limit, no more light is gathered.
    if (depth <= 0)
        return color(0,0,0);

    bool hit_anything = false;
    auto closest_so_far = 1e10;

    for (const auto& object : world) {
        if (object->hit(r, 0.001, closest_so_far, rec)) {
            hit_anything = true;
            closest_so_far = rec.t;
        }
    }

    if (hit_anything) {
        ray scattered;
        color attenuation;
        if (rec.mat_ptr->scatter(r, rec, attenuation, scattered))
            return attenuation * ray_color(scattered, world, depth-1);
        return color(0,0,0);
    }

    vec3 unit_direction = unit_vector(r.direction());
    auto t = 0.5*(unit_direction.y() + 1.0);
    return (1.0-t)*color(1.0, 1.0, 1.0) + t*color(0.5, 0.7, 1.0);
}

int main() {

    // Image
    const auto aspect_ratio = 16.0 / 9.0;
    const int image_width = 400;
    const int image_height = static_cast<int>(image_width / aspect_ratio);

    // Camera
    auto viewport_height = 2.0;
    auto viewport_width = aspect_ratio * viewport_height;
    auto focal_length = 1.0;

    auto origin = point3(0, 0, 0);
    auto horizontal = vec3(viewport_width, 0, 0);
    auto vertical = vec3(0, viewport_height, 0);
    auto lower_left_corner = origin - horizontal/2 - vertical/2 - vec3(0, 0, focal_length);

    // World
    std::vector<std::shared_ptr<hittable>> world;

    auto material_ground = std::make_shared<lambertian>(color(0.8, 0.8, 0.0));
    auto material_center = std::make_shared<lambertian>(color(0.1, 0.2, 0.5));
    auto material_left   = std::make_shared<dielectric>(1.5);
    auto material_right  = std::make_shared<metal>(color(0.8, 0.6, 0.2), 0.0);

    world.push_back(std::make_shared<sphere>(point3( 0.0, -100.5, -1.0), 100.0, material_ground.get()));
    world.push_back(std::make_shared<sphere>(point3( 0.0,    0.0, -1.0),   0.5, material_center.get()));
    world.push_back(std::make_shared<sphere>(point3(-1.0,    0.0, -1.0),   0.5, material_left.get()));
    world.push_back(std::make_shared<sphere>(point3( 1.0,    0.0, -1.0),   0.5, material_right.get()));

    // Render

    std::cout << "P3\n" << image_width << " " << image_height << "\n255\n";

    for (int j = image_height-1; j >= 0; --j) {
        std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
        for (int i = 0; i < image_width; ++i) {
            auto u = double(i) / (image_width-1);
            auto v = double(j) / (image_height-1);
            ray r(origin, lower_left_corner + u*horizontal + v*vertical - origin);
            color pixel_color = ray_color(r, world, 50);
            write_color(std::cout, pixel_color);
        }
    }

    std::cerr << "\nDone.\n";
}