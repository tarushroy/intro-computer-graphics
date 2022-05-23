// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "raster.h"

#include <gif.h>
#include <fstream>

#include <Eigen/Geometry>
// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;
using namespace Eigen;

//Image height
const int H = 480;

//Camera settings
const double near_plane = 1.5; //AKA focal length
const double far_plane = near_plane * 100;
const double field_of_view = 0.7854; //45 degrees
const double aspect_ratio = 1.5;
const bool is_perspective = false;
const Vector3d camera_position(0, 0, 3);
const Vector3d camera_gaze(0, 0, -1);
const Vector3d camera_top(0, 1, 0);

//Object
const std::string data_dir = DATA_DIR;
const std::string mesh_filename(data_dir + "bunny.off");
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)

//Material for the object
const Vector3d obj_diffuse_color(0.5, 0.5, 0.5);
const Vector3d obj_specular_color(0.2, 0.2, 0.2);
const double obj_specular_exponent = 256.0;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector3d> light_colors;
//Ambient light
const Vector3d ambient_light(0.3, 0.3, 0.3);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    if (!in.good())
    {
        std::cerr << "Invalid file " << mesh_filename << std::endl;
        exit(1);
    }
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

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16);
}

Vector3f get_barycenter() {
    std::vector<VertexAttributes> vertex_attributes;

    // loop through triangles
    for(int i = 0; i < facets.rows(); i++) {
        // get triangle
        Vector3i triangle = facets.row(i);

        // loop over vertices in triangle
        for(int j = 0; j < 3; j++) {
            // get vertex of triangle
            Vector3d v = vertices.row(triangle[j]);

            // add to vertex attributes
            vertex_attributes.push_back(VertexAttributes(v[0], v[1], v[2]));
        }
    }

    Vector4f barycenter(0, 0, 0, 0);
    for(int i = 0; i < vertex_attributes.size(); i++) {
        barycenter += vertex_attributes[i].position;
    }
    barycenter = barycenter / vertex_attributes.size();

    return Vector3f(barycenter[0], barycenter[1], barycenter[2]);
}

Vector3f get_barycenter_with_va(std::vector<VertexAttributes> vertex_attributes) {
    Vector4f barycenter(0, 0, 0, 0);
    for(int i = 0; i < vertex_attributes.size(); i++) {
        barycenter += vertex_attributes[i].position;
    }
    barycenter = barycenter / vertex_attributes.size();

    return Vector3f(barycenter[0], barycenter[1], barycenter[2]);
}

void build_uniform(UniformAttributes &uniform)
{
    //TODO: setup uniform

    //TODO: setup camera, compute w, u, v
    Vector3d w = -1 * (camera_gaze.normalized());
    Vector3d u = (camera_top.cross(w)).normalized();
    Vector3d v = w.cross(u);

    //TODO: compute the camera transformation
    Matrix4f camera_transformation;
    camera_transformation << u(0), v(0), w(0), camera_position(0),
                             u(1), v(1), w(1), camera_position(1),
                             u(2), v(2), w(2), camera_position(2),
                             0, 0, 0, 1;

    //TODO: setup projection matrix
    float n = -near_plane;
    float f = -far_plane;
    float t = tan(field_of_view / 2) * near_plane;
    float r = t * aspect_ratio;
    float l = -r;
    float b = -t;

    // final projection matrix
    Matrix4f P;

    // orthographic projection matrix
    Matrix4f M_orth;
    M_orth << (2/(r-l)), 0, 0, -((r+l)/(r-l)),
              0, (2/(t-b)), 0, -((t+b)/(t-b)),
              0, 0, (2/(n-f)), -((n+f)/(n-f)),
              0, 0, 0, 1;

    if (is_perspective)
    {
        //TODO setup prespective camera

        // transform to perspective projection
        Matrix4f M_per;
        M_per << n, 0, 0, 0,
                 0, n, 0, 0,
                 0, 0, n+f, -1*f*n,
                 0, 0, 1, 0;

        P = M_orth * M_per;
    }
    else {
        P = M_orth;
    }

    // compute final transformation matrix
    uniform.transform = P * camera_transformation.inverse();
}

void simple_render(Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        VertexAttributes out;

        // use transformation matrix
        out.position = uniform.transform * va.position;

        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: build the vertex attributes from vertices and facets

    // loop through triangles
    for(int i = 0; i < facets.rows(); i++) {
        // get triangle
        Vector3i triangle = facets.row(i);

        // loop over vertices in triangle
        for(int j = 0; j < 3; j++) {
            // get vertex of triangle
            Vector3d v = vertices.row(triangle[j]);

            // add to vertex attributes
            vertex_attributes.push_back(VertexAttributes(v[0], v[1], v[2]));
        }
    }

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

Matrix4d compute_rotation(const double alpha)
{
    //TODO: Compute the rotation matrix of angle alpha on the y axis around the object barycenter
    Matrix4d res;

    // get barycenter
    Vector3f barycenter = get_barycenter();

    // using 3d rotation matrix around y-axis
    Matrix4d rotation;
    rotation << cos(alpha), 0, sin(alpha), 0,
                0, 1, 0, 0,
                -sin(alpha), 0, cos(alpha), 0,
                0, 0, 0, 1;
    
    // translate object to origin
    Matrix4d translation_left;
    translation_left << 1, 0, 0, barycenter[0],
                        0, 1, 0, barycenter[1],
                        0, 0, 1, barycenter[2],
                        0, 0, 0, 1;
    
    // translate object back
    Matrix4d translation_right;
    translation_right << 1, 0, 0, -barycenter[0],
                         0, 1, 0, -barycenter[1],
                         0, 0, 1, -barycenter[2],
                         0, 0, 0, 1;

    res = translation_left * rotation * translation_right;
    return res;
}

void wireframe_render(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    Matrix4d trafo = compute_rotation(alpha);
    Matrix4f rot_matrix = trafo.cast<float>();
    uniform.rotation = rot_matrix;

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        VertexAttributes out;

        // transform position vectors
        // apply rotation
        out.position = uniform.transform * uniform.rotation * va.position;

        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    std::vector<VertexAttributes> vertex_attributes;

    //TODO: generate the vertex attributes for the edges and rasterize the lines
    //TODO: use the transformation matrix

    // loop through triangles (facets)
    for(int i = 0; i < facets.rows(); i++) {
        // get triangle
        Vector3i triangle = facets.row(i);

        // get vertices of triangle
        Vector3d a = vertices.row(triangle[0]);
        Vector3d b = vertices.row(triangle[1]);
        Vector3d c = vertices.row(triangle[2]);

        // add lines of triangle to vertex attributes

        // ab line
        vertex_attributes.push_back(VertexAttributes(a[0], a[1], a[2]));
        vertex_attributes.push_back(VertexAttributes(b[0], b[1], b[2]));

        // ac line
        vertex_attributes.push_back(VertexAttributes(a[0], a[1], a[2]));
        vertex_attributes.push_back(VertexAttributes(c[0], c[1], c[2]));

        // bc line
        vertex_attributes.push_back(VertexAttributes(b[0], b[1], b[2]));
        vertex_attributes.push_back(VertexAttributes(c[0], c[1], c[2]));
    }

    rasterize_lines(program, uniform, vertex_attributes, 0.5, frameBuffer);
}

void get_shading_program(Program &program)
{
    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        VertexAttributes out;
        
        //TODO: transform the position and the normal
        // rotate position
        out.position = uniform.rotation * va.position;
 
        // transform normal with rotation
        Vector4f N(va.normal[0], va.normal[1], va.normal[2], 0);
        N = uniform.rotation * N;
        out.normal =  Vector3d(N[0], N[1], N[2]);

        // converting position to 3d vector for computation
        Vector3d p(out.position[0], out.position[1], out.position[2]);

        //TODO: compute the correct lighting
        // in camera space
        Vector3d lights_color(0, 0, 0);

        // use material color for ambient light color
        Vector3d ambient_color = va.color.array() * ambient_light.array();

        // uncomment for rainbow disco bunny
        // ambient_color = Vector3d(abs(out.position[0]), abs(out.position[1]), abs(out.position[2])).array() * ambient_light.array();

        // light loop
        for (int i = 0; i< light_positions.size(); i++) {
            const Vector3d &light_position = light_positions[i];
            const Vector3d &light_color = light_colors[i];

            const Vector3d Li = (light_position - p).normalized();

            // Diffuse
            const Vector3d diffuse = obj_diffuse_color * std::max(Li.dot(out.normal), 0.0);

            // Specular
            Vector3d view_direction = (camera_position - p).normalized();
            Vector3d h = (Li + view_direction).normalized();
            const double specular_coeff = std::pow(fmax(0, (out.normal.dot(h))), obj_specular_exponent);
            const Vector3d specular = obj_specular_color * specular_coeff;

            // Attenuate lights according to the squared distance to the lights
            const Vector3d D = light_position - p;
            lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
        }

        // transformation after lighting computation
        out.position = uniform.transform * out.position;

        // final color
        out.color = ambient_color + lights_color;

        return out;
    };  

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: create the correct fragment
        FragmentAttributes out(va.color[0], va.color[1], va.color[2]);
        out.position = va.position;
        return out;
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: implement the depth check
        if(!is_perspective) {
            // orthographic
            if((camera_position[2] - fa.position[2]) < previous.depth) {
                FrameBufferAttributes out(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
                out.depth = camera_position[2] - fa.position[2];
                return out;
            }
            else {
                return previous;
            }
        }
        else {
            // perspective
            if(fa.position[2] < previous.depth) {
                FrameBufferAttributes out(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
                out.depth = fa.position[2];
                return out;
            }
            else {
                return previous;
            }
        }
    };
}

void flat_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);
    Eigen::Matrix4d trafo = compute_rotation(alpha);
    Matrix4f rot_matrix = trafo.cast<float>();
    uniform.rotation = rot_matrix;

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: compute the normals
    //TODO: set material colors

    // loop through triangles (facets)
    for(int i = 0; i < facets.rows(); i++) {
        // get triangle
        Vector3i triangle = facets.row(i);

        // get vertices of triangle
        Vector3d a = vertices.row(triangle[0]);
        Vector3d b = vertices.row(triangle[1]);
        Vector3d c = vertices.row(triangle[2]);

        // compute normal of triangle
        Vector3d N = ((b-a).cross(c-a)).normalized();

        // create vertices
        VertexAttributes va(a[0], a[1], a[2]);
        VertexAttributes vb(b[0], b[1], b[2]);
        VertexAttributes vc(c[0], c[1], c[2]);

        // add normals to vertices
        va.normal = N;
        vb.normal = N;
        vc.normal = N;

        // set material colors
        va.color << 1, 1, 1;
        vb.color << 1, 1, 1;
        vc.color << 1, 1, 1;

        // add to vertex attributes
        vertex_attributes.push_back(va);
        vertex_attributes.push_back(vb);
        vertex_attributes.push_back(vc);
    }

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

void pv_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);

    Eigen::Matrix4d trafo = compute_rotation(alpha);
    Matrix4f rot_matrix = trafo.cast<float>();
    uniform.rotation = rot_matrix;

    //TODO: compute the vertex normals as vertex normal average

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: create vertex attributes
    //TODO: set material colors

    MatrixXd normals(vertices.size(), 3);
    normals.setZero();

    // loop through triangles (facets)
    for(int i = 0; i < facets.rows(); i++) {
        // get triangle
        Vector3i triangle = facets.row(i);

        // get vertices of triangle
        Vector3d a = vertices.row(triangle[0]);
        Vector3d b = vertices.row(triangle[1]);
        Vector3d c = vertices.row(triangle[2]);

        // compute normal of triangle
        Vector3d N = ((b-a).cross(c-a)).normalized();

        // running sum of normals of vertex
        normals.row(triangle[0]) += N;
        normals.row(triangle[1]) += N;
        normals.row(triangle[2]) += N;
    }

    // normalize every row in normals
    for(int i = 0; i < normals.rows(); i++) {
        normals.row(i) = normals.row(i).normalized();
    }

    // loop through triangles again
    for(int i = 0; i < facets.rows(); i++) {
        // get triangle
        Vector3i triangle = facets.row(i);

        // loop over vertices of triangle
        for(int j = 0; j < 3; j++) {
            // get vertex of triangle
            Vector3d v = vertices.row(triangle[j]);

            // create vertex attribute with position
            VertexAttributes va(v[0], v[1], v[2]);

            // set material color
            va.color << 1, 1, 1;

            // read normal of vertex from normals
            Vector3d N = normals.row(triangle[j]);

            // add normal to vertex
            va.normal = N;

            // add to vertex attributes
            vertex_attributes.push_back(va);
        }
    }

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

int main(int argc, char *argv[])
{
    setup_scene();

    int W = H * aspect_ratio;
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(W, H);
    vector<uint8_t> image;

    simple_render(frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("simple.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    // reset
    frameBuffer.setConstant(FrameBufferAttributes());

    wireframe_render(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("wireframe.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    // reset
    frameBuffer.setConstant(FrameBufferAttributes());

    flat_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("flat_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    // reset
    frameBuffer.setConstant(FrameBufferAttributes());

    pv_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("pv_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    //TODO: add the animation
    const double alpha = 0.05;
    const int numFrames = 125;
    int delay = 10;
    GifWriter g;
    const char *fileName;

    // reset
    frameBuffer.setConstant(FrameBufferAttributes());

    // wireframe animation
    fileName = "wireframe_animation.gif";
    GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);

    for (int i = 0; i < numFrames; i++)
    {
        frameBuffer.setConstant(FrameBufferAttributes());

        // call wireframe_render
        wireframe_render(i * alpha, frameBuffer);
        
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }

    GifEnd(&g);
    
    // reset
    frameBuffer.setConstant(FrameBufferAttributes());

    // flat shading bunny animation
    fileName = "flat_shading_animation.gif";
    GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);

    for (int i = 0; i < numFrames; i++)
    {
        frameBuffer.setConstant(FrameBufferAttributes());

        // call render
        flat_shading(i * alpha, frameBuffer);
        
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }

    GifEnd(&g);

    // reset
    frameBuffer.setConstant(FrameBufferAttributes());
    
    // pv bunny animation
    fileName = "pv_shading_animation.gif";
    GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);

    for (int i = 0; i < numFrames; i++)
    {
        frameBuffer.setConstant(FrameBufferAttributes());

        // call render
        pv_shading(i * alpha, frameBuffer);
        
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }

    GifEnd(&g);

    return 0;
}
