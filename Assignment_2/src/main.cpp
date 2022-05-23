// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

bool intersectsWithSphere(Vector3d ray_origin, Vector3d ray_direction, Vector3d sphere_center, double sphere_radius) {
    // p = e + td
    // f(p) = |p-c|^2 - R^2
    // f(p) = |e+td - c|^2 - R^2
    // f(p) = <e+td-c, e+td-c> - R^2 = 0
    // f(p) = t^2<d,d> + 2t<d, e-c> + <e-c, e-c> - R^2
    // f(p) = t^2*a + t*2b + c = 0
    // t = -b +- sqrt(b^2 - 4ac) / 2a

    // if b^2 - 4ac < 0, no soln
    // if b^2 - 4ac = 0, 1 soln
    // if b^2 - 4ac > 0, 2 soln

    double a = ray_direction.dot(ray_direction);
    double b = ray_direction.dot(ray_origin - sphere_center) * 2;
    double c = (ray_origin - sphere_center).dot(ray_origin - sphere_center) - (sphere_radius * sphere_radius);

    // discriminant
    double disc = (b*b) - (4*a*c);

    if(disc < 0) {
        return false;
    }
    else {
        // check if either t is positive, if not, no soln
        double tplus = (-b + sqrt(disc)) / 2*a;
        double tminus = (-b - sqrt(disc)) / 2*a;

        return ((tplus > 0) || (tminus > 0));
    }
}

Vector3d sphereIntersectionPoint(Vector3d ray_origin, Vector3d ray_direction, Vector3d sphere_center, double sphere_radius) {
    double a = ray_direction.dot(ray_direction);
    double b = ray_direction.dot(ray_origin - sphere_center) * 2;
    double c = (ray_origin - sphere_center).dot(ray_origin - sphere_center) - (sphere_radius * sphere_radius);

    double disc = (b*b) - (4*a*c);
    double t;

    double tplus = (-b + sqrt(disc)) / 2*a;
    double tminus = (-b - sqrt(disc)) / 2*a;

    // use smallest positive t
    if(tplus > 0 && tminus < 0) {
        t = tplus;
    }
    else if(tminus > 0 && tplus < 0) {
        t = tminus;
    }
    else if(tplus > 0 && tminus > 0) {
        t = fmin(tplus, tminus);
    }

    Vector3d intersectionPoint = ray_origin + (t * ray_direction);
    return intersectionPoint;
}

void raytrace_sphere()
{
    std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

    const std::string filename("sphere_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the
    // unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction.normalized();

            // Modified
            const Vector3d sphere_center(0, 0, 0);
            const double sphere_radius = 0.9;

            if (intersectsWithSphere(ray_origin, ray_direction, sphere_center, sphere_radius))
            {
                // The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection = sphereIntersectionPoint(ray_origin, ray_direction, sphere_center, sphere_radius);

                // Compute normal at the intersection points
                Vector3d ray_normal = (ray_intersection - sphere_center).normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

bool intersectsWithPgram(Vector3d ray_origin, Vector3d ray_direction, Vector3d pgram_origin, Vector3d pgram_u, Vector3d pgram_v) {
    // e = ray_origin
    // d = ray_direction
    // o = pgram_origin
    // uvector = pgram_u
    // vvector = pgram_v
    //
    // e + td = o + u*uvector + v*vvector
    // td - u*uvector - v*vector = o - e

    // Set up matrices for solution vector
    Matrix3d A;
    A << -pgram_u, -pgram_v, ray_direction;

    Vector3d b;
    b << pgram_origin - ray_origin;

    Vector3d sol = A.colPivHouseholderQr().solve(b);

    float u = sol(0);
    float v = sol(1);
    float t = sol(2);

    return ((t > 0) && (u >= 0) && (v >= 0) && (u <= 1) && (v <= 1));
}

Vector3d pgramIntersectionPoint(Vector3d ray_origin, Vector3d ray_direction, Vector3d pgram_origin, Vector3d pgram_u, Vector3d pgram_v) {
    // Set up matrices for the solution vector
    Matrix3d A;
    A << -pgram_u, -pgram_v, ray_direction;

    Vector3d b;
    b << pgram_origin - ray_origin;

    // solve system of 3 linear eqns
    Vector3d sol = A.colPivHouseholderQr().solve(b);

    // sol(2) is t
    Vector3d intersectionPoint = ray_origin + (sol(2) * ray_direction);
    return intersectionPoint;
}

void raytrace_parallelogram()
{
    std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

    const std::string filename("plane_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction.normalized();

            // TODO: Check if the ray intersects with the parallelogram
            if(intersectsWithPgram(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v))
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection
                // point
                Vector3d ray_intersection = pgramIntersectionPoint(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v);

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = pgram_v.cross(pgram_u).normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_perspective()
{
    std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

    const std::string filename("plane_perspective.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // TODO: Prepare the ray (origin point and direction)
            const Vector3d ray_origin = camera_origin;
            const Vector3d ray_direction = (pixel_center - ray_origin).normalized();

            // TODO: Check if the ray intersects with the parallelogram
            if (intersectsWithPgram(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v))
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection
                // point
                Vector3d ray_intersection = pgramIntersectionPoint(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v);

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = pgram_v.cross(pgram_u).normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_shading()
{
    std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

    const std::string filename("shading.png");

    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd R = MatrixXd::Zero(800, 800);
    MatrixXd G = MatrixXd::Zero(800, 800);
    MatrixXd B = MatrixXd::Zero(800, 800);

    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / A.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / A.rows(), 0);

    //Sphere setup
    const Vector3d sphere_center(0, 0, 0);
    const double sphere_radius = 0.9;

    //material params
    const Vector3d diffuse_color(1, 0, 1);
    const double specular_exponent = 100;
    const Vector3d specular_color(0., 0, 1);

    // Single light source
    const Vector3d light_position(-1, 1, 1);
    double ambient = 0.1;

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = camera_origin;
            const Vector3d ray_direction = (pixel_center - ray_origin).normalized();

            // Intersect with the sphere
            if (intersectsWithSphere(ray_origin, ray_direction, sphere_center, sphere_radius))
            {
                // The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection = sphereIntersectionPoint(ray_origin, ray_direction, sphere_center, sphere_radius);

                // Compute normal at the intersection point
                Vector3d ray_normal = (ray_intersection - sphere_center).normalized();

                // TODO: Add shading parameter here
                const double light_intensity = 1;
                Vector3d light_direction = (light_position - ray_intersection).normalized();
                Vector3d view_direction = (camera_origin - ray_intersection).normalized();

                const double diffuse = light_intensity * fmax(0, (light_direction.dot(ray_normal)));

                Vector3d h = (view_direction + light_direction).normalized();
                const double specular = light_intensity * std::pow(fmax(0, (ray_normal.dot(h))), specular_exponent);

                // Simple diffuse model
                R(i, j) = ambient + (diffuse_color[0] * diffuse) + (specular_color[0] * specular);
                G(i, j) = ambient + (diffuse_color[1] * diffuse) + (specular_color[1] * specular);
                B(i, j) = ambient + (diffuse_color[2] * diffuse) + (specular_color[2] * specular);

                // Clamp to zero
                R(i, j) = std::max(R(i, j), 0.);
                G(i, j) = std::max(G(i, j), 0.);
                B(i, j) = std::max(B(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

void raytrace_multiple()
{
    std::cout << "Simple ray tracer, multiple objects with shading" << std::endl;

    const std::string filename("multiple.png");

    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd R = MatrixXd::Zero(800, 800);
    MatrixXd G = MatrixXd::Zero(800, 800);
    MatrixXd B = MatrixXd::Zero(800, 800);

    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / A.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / A.rows(), 0);

    //Sphere setup
    const Vector3d sphere_center(-0.5, 0.5, 0);
    const double sphere_radius = 0.2;

    // Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(0, -1, -0.5);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // sphere material params
    const Vector3d sphere_diffuse_color(0.1, 1, 0.1);
    const double sphere_specular_exponent = 100;
    const Vector3d sphere_specular_color(0., 0, 1);

    // pgram material params
    const Vector3d pgram_diffuse_color(0.1, 0.1, 1);
    const double pgram_specular_exponent = 100;
    const Vector3d pgram_specular_color(0., 0, 1);

    // Single light source
    const Vector3d light_position(-1, 1, 1);
    double ambient = 0.1;

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = camera_origin;
            const Vector3d ray_direction = (pixel_center - ray_origin).normalized();

            // Intersect with the sphere
            if (intersectsWithSphere(ray_origin, ray_direction, sphere_center, sphere_radius))
            {
                // The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection = sphereIntersectionPoint(ray_origin, ray_direction, sphere_center, sphere_radius);

                // Compute normal at the intersection point
                Vector3d ray_normal = (ray_intersection - sphere_center).normalized();

                // TODO: Add shading parameter here
                const double light_intensity = 1;
                Vector3d light_direction = (light_position - ray_intersection).normalized();
                Vector3d view_direction = (camera_origin - ray_intersection).normalized();

                const double diffuse = light_intensity * fmax(0, (light_direction.dot(ray_normal)));

                Vector3d h = (view_direction + light_direction).normalized();
                const double specular = light_intensity * std::pow(fmax(0, (ray_normal.dot(h))), sphere_specular_exponent);

                // Simple diffuse model
                R(i, j) = ambient + (sphere_diffuse_color[0] * diffuse) + (sphere_specular_color[0] * specular);
                G(i, j) = ambient + (sphere_diffuse_color[1] * diffuse) + (sphere_specular_color[1] * specular);
                B(i, j) = ambient + (sphere_diffuse_color[2] * diffuse) + (sphere_specular_color[2] * specular);

                // Clamp to zero
                R(i, j) = std::max(R(i, j), 0.);
                G(i, j) = std::max(G(i, j), 0.);
                B(i, j) = std::max(B(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }

            // Intersect with the parallelogram
            if (intersectsWithPgram(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v))
            {
                // The ray hit the parallelogram, compute the exact intersection
                // point
                Vector3d ray_intersection = pgramIntersectionPoint(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v);

                // Compute normal at the intersection point
                Vector3d ray_normal = pgram_v.cross(pgram_u).normalized();

                // Add shading parameter here
                const double light_intensity = 1;
                Vector3d light_direction = (light_position - ray_intersection).normalized();
                Vector3d view_direction = (camera_origin - ray_intersection).normalized();

                const double diffuse = light_intensity * fmax(0, (light_direction.dot(ray_normal)));

                Vector3d h = (view_direction + light_direction).normalized();
                const double specular = light_intensity * std::pow(fmax(0, (ray_normal.dot(h))), pgram_specular_exponent);

                if(intersectsWithSphere(ray_intersection, light_direction, sphere_center, sphere_radius)) {
                    R(i, j) = ambient;
                    G(i, j) = ambient;
                    B(i, j) = ambient;
                }
                else {
                    // Simple diffuse model
                    R(i, j) = ambient + (pgram_diffuse_color[0] * diffuse) + (pgram_specular_color[0] * specular);
                    G(i, j) = ambient + (pgram_diffuse_color[1] * diffuse) + (pgram_specular_color[1] * specular);
                    B(i, j) = ambient + (pgram_diffuse_color[2] * diffuse) + (pgram_specular_color[2] * specular);
                }

                // Clamp to zero
                R(i, j) = std::max(R(i, j), 0.);
                G(i, j) = std::max(G(i, j), 0.);
                B(i, j) = std::max(B(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

int main()
{
    //raytrace_sphere();
    //raytrace_parallelogram();
    //raytrace_perspective();
    raytrace_shading();
    //raytrace_multiple();

    return 0;
}
