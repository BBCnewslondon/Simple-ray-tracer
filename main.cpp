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
#include <omp.h>

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
            color pixel_color = ray_color(r, world, 50);
            pixels[j * image_width + i] = pixel_color;
        }
    }

    for (const auto& c : pixels) {
        write_color(std::cout, c);
    }

    std::cerr << "\nDone.\n";
}