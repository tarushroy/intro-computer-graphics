////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <queue>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Class to store tree
////////////////////////////////////////////////////////////////////////////////
class AABBTree
{
public:
    class Node
    {
    public:
        AlignedBox3d bbox;
        int parent;   // Index of the parent node (-1 for root)
        int left;     // Index of the left child (-1 for a leaf)
        int right;    // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)
    };

    std::vector<Node> nodes;
    int root;

    AABBTree() = default;                           // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
};

////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////
const std::string data_dir = DATA_DIR;
const std::string filename("raytrace.png");
const std::string mesh_filename(data_dir + "dragon.off");

//Camera settings
const double focal_length = 2;
const double field_of_view = 0.7854; //45 degrees
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 2);

// Bonus 3
const int max_bounce = 2;

// Triangle Mesh
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)
AABBTree bvh;

// Bonus 1
// Sphere and Parallelogram Objects
std::vector<Vector3d> sphere_centers;
std::vector<double> sphere_radii;
std::vector<Matrix3d> parallelograms;

//Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.0, 0.5, 0.0, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);

// Precomputed (or otherwise) gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;
//Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv, 3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i)
    {
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    //setup tree
    bvh = AABBTree(vertices, facets);

    // Bonus 1
    // Spheres
    sphere_centers.emplace_back(1, 0.2, -1);
    sphere_radii.emplace_back(1);

    // Parallelograms
    parallelograms.emplace_back();
    parallelograms.back() << -100, 100, -100,
        -1.25, 0, -1.2,
        -100, -100, 100;

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Code
////////////////////////////////////////////////////////////////////////////////

AlignedBox3d bbox_from_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c)
{
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}

int longest_axis(MatrixXd A) {
    // find longest axis of centroids
    AlignedBox3d temp_box;
    for(int i = 0; i < A.rows(); i++) {
        Vector3d row = A.row(i);
        temp_box.extend(row);
    }
    Vector3d diag = temp_box.diagonal();
    int axis;
    if(diag(0) >= diag(1)) {
        axis = 0;
        if(diag(0) >= diag(2)) {
            axis = 0;
        }
        else {
            axis = 2;
        }
    }
    else {
        axis = 1;
        if(diag(1) >= diag(2)) {
            axis = 1;
        }
        else {
            axis = 2;
        }
    }

    return axis;
}

std::vector<int> sort_centroids(MatrixXd &A, std::vector<int> indexes) {
    // list to store rows
    std::vector<Vector3d> vlist;

    // get longest axis of centroids
    int axis = longest_axis(A);

    // loop through rows in matrix A
    for(int i = 0; i < A.rows(); i++) {
        vlist.push_back(A.row(i));
    }

    // list of indexes in same order as the sort
    std::vector<int> order(vlist.size());
    std::iota(order.begin(), order.end(), 0);

    // index sort based on vlist
    std::sort(order.begin(), order.end(), [&vlist, axis](int i1, int i2){ return vlist[i1](axis) < vlist[i2](axis); });

    // sort list of rows
    std::sort(vlist.begin(), vlist.end(), [axis](const Vector3d v1, const Vector3d v2){ return v1(axis) < v2(axis); });

    // stitch list back together into matrix
    for(int i = 0; i < A.rows(); i++) {
        A.row(i) = vlist[i];
    }

    // new indexes
    std::vector<int> sorted_indexes(order.size());

    // get triangle indexes
    for(int i = 0; i < indexes.size(); i++) {
        sorted_indexes[i] = indexes[order[i]];
    }

    return sorted_indexes;
}

int build(MatrixXd centroids, std::vector<int> indexes, std::vector<AABBTree::Node> &nodes, int parent) {
    // base case when centroids has only 1 point
    if (centroids.rows() == 1) {
        AABBTree::Node new_leaf;
        new_leaf.triangle = indexes[0];
        new_leaf.left = -1;
        new_leaf.right = -1;
        new_leaf.parent = parent;

        // get vertices of triangles
        Vector3i vertex_indexes = facets.row(new_leaf.triangle);
        Vector3d a = vertices.row(vertex_indexes(0));
        Vector3d b = vertices.row(vertex_indexes(1));
        Vector3d c = vertices.row(vertex_indexes(2));
        new_leaf.bbox = bbox_from_triangle(a, b, c);

        // append node to list of nodes
        nodes.push_back(new_leaf);

        // return node id
        return nodes.size() - 1;
    }

    // recursive case

    // sort
    indexes = sort_centroids(centroids, indexes);

    // split
    int S2_size = centroids.rows() / 2;
    int S1_size = centroids.rows() - S2_size;
    MatrixXd S1(S1_size, centroids.cols());
    MatrixXd S2(S2_size, centroids.cols());
    S1.setZero();
    S2.setZero();
    std::vector<int> S1_indexes(S1_size, 0);
    std::vector<int> S2_indexes(S2_size, 0);

    for(int i = 0; i < centroids.rows(); i++) {
        if (i < S1_size) {
            S1.row(i) = centroids.row(i);
            S1_indexes[i] = indexes[i];
        }
        else {
            S2.row(i - S1_size) = centroids.row(i);
            S2_indexes[i - S1_size] = indexes[i];
        }
    }

    // Update the box of the current node by merging boxes of the root of T1 and T2
    AABBTree::Node node;

    // recursively build subtree T1 corresponding to S1
    int nL = build(S1, S1_indexes, nodes, nodes.size() - 2);
    node.left = nL;

    // recursively build subtree T2 corresponding to S2
    int nR = build(S2, S2_indexes, nodes, nodes.size() - 2);
    node.right = nR;

    node.bbox = nodes[nL].bbox.extend(nodes[nR].bbox);

    nodes.push_back(node);

    // return new root node R of subtree
    return nodes.size() - 1;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F)
{
    // Compute the centroids of all the triangles in the input mesh
    MatrixXd centroids(F.rows(), V.cols());
    centroids.setZero();
    for (int i = 0; i < F.rows(); ++i)
    {
        for (int k = 0; k < F.cols(); ++k)
        {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }
    
    // TODO

    // Split each set of primitives into 2 sets of roughly equal size,
    // based on sorting the centroids along one direction or another.

    // intial sort for triangle indexes
    // find longest axis of centroids
    int axis = longest_axis(centroids);

    // list to store rows
    std::vector<Vector3d> vlist;

    // loop through rows in matrix A
    for(int i = 0; i < centroids.rows(); i++) {
        vlist.push_back(centroids.row(i));
    }

    // list of indexes in same order as the sort
    std::vector<int> triangle_indexes(vlist.size());
    std::iota(triangle_indexes.begin(), triangle_indexes.end(), 0);

    // index sort based on vlist
    stable_sort(triangle_indexes.begin(), triangle_indexes.end(), [&vlist, axis](int i1, int i2){ return vlist[i1](axis) < vlist[i2](axis); });

    // sort list of rows
    std::sort(vlist.begin(), vlist.end(), [axis](const Vector3d v1, const Vector3d v2){ return v1(axis) < v2(axis); });

    // stitch list back together into matrix
    for(int i = 0; i < centroids.rows(); i++) {
        centroids.row(i) = vlist[i];
    }

    // recursively build tree
    root = build(centroids, triangle_indexes, nodes, 0);
}

////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////

// Bonus 1
double ray_sphere_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, int index, Vector3d &p, Vector3d &N)
{
    const Vector3d sphere_center = sphere_centers[index];
    const double sphere_radius = sphere_radii[index];

    double a = ray_direction.dot(ray_direction);
    double b = ray_direction.dot(ray_origin - sphere_center) * 2;
    double c = (ray_origin - sphere_center).dot(ray_origin - sphere_center) - (sphere_radius * sphere_radius);

    double disc = (b*b) - (4*a*c);

    if(disc < 0) {
        return -1;
    }

    double t = -1;

    double tplus = (-b + sqrt(disc)) / 2*a;
    double tminus = (-b - sqrt(disc)) / 2*a;

    if(tplus > 0 && tminus < 0) {
        t = tplus;
    }
    else if(tminus > 0 && tplus < 0) {
        t = tminus;
    }
    else if(tplus > 0 && tminus > 0) {
        t = fmin(tplus, tminus);
    }

    p = ray_origin + (t * ray_direction);
    N = (p - sphere_center).normalized();

    return t;
}

// Bonus 1
double ray_parallelogram_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, int index, Vector3d &p, Vector3d &N)
{
    const Vector3d pgram_origin = parallelograms[index].col(0);
    const Vector3d A = parallelograms[index].col(1);
    const Vector3d B = parallelograms[index].col(2);
    const Vector3d pgram_u = A - pgram_origin;
    const Vector3d pgram_v = B - pgram_origin;

    Matrix3d AM;
    AM << -pgram_u, -pgram_v, ray_direction;

    Vector3d b;
    b << pgram_origin - ray_origin;

    Vector3d sol = AM.colPivHouseholderQr().solve(b);
    
    double u = sol(0);
    double v = sol(1);
    double t = sol(2);

    if ((t > 0) && (u >= 0) && (v >= 0) && (u <= 1) && (v <= 1)) {
        p = ray_origin + (t * ray_direction);
        N = pgram_v.cross(pgram_u).normalized();

        return t;
    }

    return -1;
}

double ray_triangle_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &a, const Vector3d &b, const Vector3d &c, Vector3d &p, Vector3d &N)
{
    // TODO
    // Compute whether the ray intersects the given triangle.
    // If you have done the parallelogram case, this should be very similar to it.

    // setup
    Vector3d triangle_origin = a;
    Vector3d triangle_u = b-a;
    Vector3d triangle_v = c-a;

    Matrix3d A;
    A << -triangle_u, -triangle_v, ray_direction;

    Vector3d B;
    B << triangle_origin - ray_origin;

    // solve linear system
    Vector3d soln = A.colPivHouseholderQr().solve(B);
    double u = soln(0);
    double v = soln(1);
    double t = soln(2);

    // check intersection conditions
    if((t > 0) && (u >= 0) && (v >= 0) && (u + v <= 1)) {
        // update ray and normal
        p = ray_origin + (t * ray_direction);
        N = triangle_v.cross(triangle_u).normalized();

        return t;
    }

    return -1;
}

bool ray_box_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const AlignedBox3d &box)
{
    // TODO
    // Compute whether the ray intersects the given box.
    // we are not testing with the real surface here anyway.
    
    // get bounds of bounding box in vector
    Vector3d box_min = box.min();
    Vector3d box_max = box.max();

    // get bounding planes components
    double t0x = (box_min(0) - ray_origin(0)) / ray_direction(0);
    double t1x = (box_max(0) - ray_origin(0)) / ray_direction(0);
    double t0y = (box_min(1) - ray_origin(1)) / ray_direction(1);
    double t1y = (box_max(1) - ray_origin(1)) / ray_direction(1);
    double t0z = (box_min(2) - ray_origin(2)) / ray_direction(2);
    double t1z = (box_max(2) - ray_origin(2)) / ray_direction(2);

    // compare x and y
    double tmin = std::min(t0x, t1x);
    double tmax = std::max(t0x, t1x);
    
    double tymin = std::min(t0y, t1y);
    double tymax = std::max(t0y, t1y);

    if (tmin > tymax || tymin >  tmax) {
        return false;
    }

    tmin = std::max(tmin, tymin);
    tmax = std::min(tmax, tymax);

    // compare result with z
    double tzmin = std::min(t0z, t1z);
    double tzmax = std::max(t0z, t1z);

    if (tmin > tzmax || tzmin >  tmax) {
        return false;
    }

    tmin = std::max(tmin, tzmin);
    tmax = std::min(tmax, tzmax);

    return true;
}

//Finds the closest intersecting object returns its index
//In case of intersection it writes into p and N (intersection point and normals)
bool find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N)
{
    Vector3d tmp_p, tmp_N;

    // TODO
    // Method (1): Traverse every triangle and return the closest hit.

    /*

    // keep track of intersections
    bool hasIntersection = false;
    double closest_t = std::numeric_limits<double>::max();

    // loop through all triangles (facets)
    for(int i=0; i < facets.rows(); i++) {
        Vector3i row = facets.row(i);

        // get vertices of triangle
        Vector3d a = vertices.row(row(0));
        Vector3d b = vertices.row(row(1));
        Vector3d c = vertices.row(row(2));

        // check for intersection
        double t = ray_triangle_intersection(ray_origin, ray_direction, a, b, c, tmp_p, tmp_N);

        if (t >= 0) {
            if (t < closest_t) {
                // update variables
                hasIntersection = true;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
        }
    }

    return hasIntersection;

    */


    // Method (2): Traverse the BVH tree and test the intersection with a
    // triangles at the leaf nodes that intersects the input ray.

    // variables to keep track of closest intersection
    bool hasIntersection = false;
    double closest_t = std::numeric_limits<double>::max();

    // queue
    std::queue<int> queue;
    queue.push(bvh.root);

    while(!queue.empty()) {
        // get element
        int node_id = queue.front();
        queue.pop();
        AABBTree::Node node = bvh.nodes[node_id];

        // check ray box intersection
        if(ray_box_intersection(ray_origin, ray_direction, node.bbox)) {
            // check for leaf node
            if(node.left == -1 && node.right == -1) {
                // get triangle
                Vector3i row = facets.row(node.triangle);
                Vector3i vertex_indexes = row.transpose();
                Vector3d a = vertices.row(vertex_indexes(0));
                Vector3d b = vertices.row(vertex_indexes(1));
                Vector3d c = vertices.row(vertex_indexes(2));

                // check for ray triangle intersection
                double t = ray_triangle_intersection(ray_origin, ray_direction, a, b, c, tmp_p, tmp_N);

                // if intersection
                if (t >= 0) {
                    if (t < closest_t) {
                        // update variables
                        hasIntersection = true;
                        closest_t = t;
                        p = tmp_p;
                        N = tmp_N;
                    }
                }
            }
            else {
                queue.push(node.left);
                queue.push(node.right);
            }
        }
    }

    // Bonus 1
    // loop through all spheres
    for (int i = 0; i < sphere_centers.size(); ++i)
    {
        //returns t and writes on tmp_p and tmp_N
        const double t = ray_sphere_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);
        //We have intersection
        if (t >= 0)
        {
            //The point is before our current closest t
            if (t < closest_t)
            {
                hasIntersection = true;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
        }
    }

    // loop through all parallelograms
    for (int i = 0; i < parallelograms.size(); ++i)
    {
        //returns t and writes on tmp_p and tmp_N
        const double t = ray_parallelogram_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);
        //We have intersection
        if (t >= 0)
        {
            //The point is before our current closest t
            if (t < closest_t)
            {
                hasIntersection = true;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
        }
    }

    return hasIntersection;
}

////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////

// Bonus 2
// Checks if the light is visible
bool is_light_visible(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &light_position)
{
    // Determine if the light is visible here
    Vector3d tmp_p, tmp_N;
    return !find_nearest_object(ray_origin, ray_direction, tmp_p, tmp_N);
}

Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction, int max_bounce) // Bonus 3
{
    //Intersection point and normal, these are output of find_nearest_object
    Vector3d p, N;

    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (!nearest_object)
    {
        // Return a transparent color
        return Vector4d(0, 0, 0, 0);
    }

    if (max_bounce < 0) {
        return Vector4d(0, 0, 0, 0);
    }

    // Ambient light contribution
    const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();

    // Punctual lights contribution (direct lighting)
    Vector4d lights_color(0, 0, 0, 0);
    for (int i = 0; i < light_positions.size(); ++i)
    {
        const Vector3d &light_position = light_positions[i];
        const Vector4d &light_color = light_colors[i];

        const Vector3d Li = (light_position - p).normalized();

        // Bonus 2
        const Vector3d p_epsilon = p + (0.0001 * Li);
        if(!is_light_visible(p_epsilon, Li, light_position)) {
            continue;
        }

        // Diffuse contribution
        Vector4d diff_color = obj_diffuse_color;
        const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);

        // Specular contribution
        const Vector3d Hi = (Li - ray_direction).normalized();
        const Vector4d specular = obj_specular_color * std::pow(std::max(N.dot(Hi), 0.0), obj_specular_exponent);
        // Vector3d specular(0, 0, 0);

        // Attenuate lights according to the squared distance to the lights
        const Vector3d D = light_position - p;
        lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
    }

    // Bonus 3
    Vector4d refl_color = obj_reflection_color;

    // Compute the color of the reflected ray and add its contribution to the current point color.
    Vector3d r = (ray_direction - (2*(ray_direction.dot(N))*N)).normalized();
    Vector3d p_epsilon = p + (0.0001 * r);

    Vector4d reflection_color(0, 0, 0, 0);
    reflection_color = reflection_color + refl_color.cwiseProduct(shoot_ray(p_epsilon, r, max_bounce-1));

    // Rendering equation
    Vector4d C = ambient_color + lights_color + reflection_color;

    //Set alpha to 1
    C(3) = 1;

    return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
    std::cout << "Simple ray tracer." << std::endl;

    int w = 640;
    int h = 480;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    //TODO
    double image_y = tan(field_of_view / 2) * focal_length;
    double image_x = image_y * aspect_ratio;

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);

    for (unsigned i = 0; i < w; ++i)
    {
        for (unsigned j = 0; j < h; ++j)
        {
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective)
            {
                // Perspective camera
                ray_origin = camera_position;
                ray_direction = (pixel_center - camera_position).normalized();
            }
            else
            {
                // Orthographic camera
                ray_origin = pixel_center;
                ray_direction = Vector3d(0, 0, -1);
            }

            const Vector4d C = shoot_ray(ray_origin, ray_direction, max_bounce); // Bonus 3
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    //setup_scene();

    //raytrace_scene();

    // tests
    Matrix2d M;
    M << 2, 3,
         4, 5;

    Matrix2d Mt = M.transpose();

    std::cout << Mt << std::endl;

    return 0;
}
