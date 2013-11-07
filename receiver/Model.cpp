#include <stdexcept>
#include <iostream>
#include <fstream>
#include <GL/glew.h>
#include <oglplus/all.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Model.hpp"
#include "Data3d.hpp"

namespace {

class Program
{
    oglplus::Program p;
    Program();
public:
    ~Program();
    static Program& instance();
    void use(const bool b);
};

Program::Program()
try {
    oglplus::VertexShader vs;
    vs.Source(
                "#version 430 core\n"
                "layout(location = 0) in vec3 vertex;\n"
                "layout(location = 0) uniform float camera_focal_x;\n"
                "layout(location = 1) uniform float camera_focal_y;\n"
                "layout(location = 2) uniform float camera_centre_x;\n"
                "layout(location = 3) uniform float camera_centre_y;\n"
                "layout(location = 4) uniform mat4 mvp_matrix;\n"
                "layout(location = 8) uniform sampler2DRect camera_y;\n"
                "out vec2 tex_coord;\n"

                "void main()\n"
                "{\n"
                "    vec4 v = vec4(vertex, 1.0f);\n"
                "    ivec2 size = textureSize(camera_y);\n"
                "    gl_Position = mvp_matrix * v;\n"
                "    tex_coord.s = size[0] - (camera_focal_x * v.x / v.z + camera_centre_x);\n"
                "    tex_coord.t = camera_focal_y * v.y / v.z + camera_centre_y;\n"
                "}\n"
                ).Compile();
    oglplus::FragmentShader fs;
    fs.Source(
                "#version 430 core\n"
                "layout(location = 8) uniform sampler2DRect camera_y;\n"
                "layout(location = 9) uniform sampler2DRect camera_u;\n"
                "layout(location = 10) uniform sampler2DRect camera_v;\n"
                "in vec2 tex_coord;\n"
                "layout(location = 0) out vec4 frag_color;\n"

                "void main()\n"
                "{\n"
                "   const highp float y = texture(camera_y, tex_coord).r;\n"
                "   const highp float u = texture(camera_u, tex_coord / 2.0).r - 0.5;\n"
                "   const highp float v = texture(camera_v, tex_coord / 2.0).r - 0.5;\n"
                "   const highp float r = y + 1.402 * v;\n"
                "   const highp float g = y - 0.344 * u - 0.714 * v;\n"
                "   const highp float b = y + 1.772 * u;\n"
                "   frag_color = vec4(r, g, b, 1.0);\n"
                "}\n"
                ).Compile();
    p.AttachShader(vs).AttachShader(fs).Link().DetachShader(fs).DetachShader(vs);
} catch(const oglplus::ProgramBuildError& pbe) {
    std::cerr <<
                 "Program build error (in " <<
                 pbe.GLSymbol() << ", " <<
                 pbe.ClassName() << " '" <<
                 pbe.ObjectDescription() << "'): " <<
                 pbe.what() << std::endl <<
                 pbe.Log() << std::endl;
    pbe.Cleanup();
}

Program::~Program()
{
}

Program& Program::instance()
{
    static Program prog;
    return prog;
}

void Program::use(const bool b)
{
    if (b)
        p.Use();
    else
        p.UseNone();
}

}

Model::Model()
{
    glGenVertexArrays(1, vao);
    glGenBuffers(n_vbo, vbo);
    glGenTextures(n_tex, tex);
    Eigen::Matrix4f::Map(&model_matrix[0]).setIdentity();
    Eigen::Matrix4f::Map(&matrix[0]).setIdentity();
    glBindVertexArray(vao[0]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[0]);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
    for (int i = 0; i < n_tex; ++i) {
        glBindTexture(GL_TEXTURE_RECTANGLE, tex[i]);
        glTexParameterf(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }
    glBindTexture(GL_TEXTURE_RECTANGLE, 0);
}

Model::~Model()
{
    glDeleteBuffers(n_vbo, vbo);
    glDeleteTextures(n_tex, tex);
    glDeleteVertexArrays(1, vao);
}

void Model::init()
{
    const auto ret = ::glewInit();
    if (ret != GLEW_OK)
        throw std::runtime_error("Unable to initialize OpenGL extensions.");
}

void Model::draw() const
{
    if (n_elements == 0)
        return;
    GLint front_face;
    glGetIntegerv(GL_FRONT_FACE, &front_face);
    glFrontFace(GL_CW);
    glBindVertexArray(vao[0]);
    for (int i = 0; i < n_tex; ++i) {
        glActiveTexture(GL_TEXTURE0 + i);
        glBindTexture(GL_TEXTURE_RECTANGLE, tex[i]);
    }
    glPushMatrix();
    glMultMatrixf(model_matrix);
    glMultMatrixf(matrix);
    Program::instance().use(true);
    float mv[16], pr[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, mv);
    glGetFloatv(GL_PROJECTION_MATRIX, pr);
    Eigen::Map<Eigen::Matrix4f> view_matrix(mv), proj_matrix(pr);
    const Eigen::Matrix4f mvp_matrix = proj_matrix * view_matrix;
    glUniformMatrix4fv(4, 1, GL_FALSE, mvp_matrix.data());
    glDrawElements(GL_TRIANGLES, n_elements, GL_UNSIGNED_INT, 0);
    Program::instance().use(false);
    glPopMatrix();
    for (int i = 0; i < n_tex; ++i) {
        glActiveTexture(GL_TEXTURE0 + i);
        glBindTexture(GL_TEXTURE_RECTANGLE, 0);
    }
    glBindVertexArray(0);
    glFrontFace(front_face);
}

void Model::load(const Data3d& data)
{
    n_elements = data.tri.size();
    if (n_elements == 0)
        return;
    if (data.name != name) {
        name = data.name;
        std::ifstream calibration("calib_" + name + ".txt");
        if (calibration.is_open())
            for (int i = 0; i < 16; ++i)
                calibration >> matrix[i];
        else
            Eigen::Map<Eigen::Matrix4f>(matrix).setIdentity();
        //std::cout << name << " calibration:\n" << Eigen::Map<Eigen::Matrix4f>(matrix) << std::endl;
    }
    std::copy(data.modelview, data.modelview + 16, model_matrix);
    glBindVertexArray(vao[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned) * data.tri.size(), data.tri.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * data.ver.size(), data.ver.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    int width, height;
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glBindTexture(GL_TEXTURE_RECTANGLE, tex[0]);
    glGetTexLevelParameteriv(GL_TEXTURE_RECTANGLE, 0, GL_TEXTURE_WIDTH, &width);
    glGetTexLevelParameteriv(GL_TEXTURE_RECTANGLE, 0, GL_TEXTURE_HEIGHT, &height);
    if (width != data.width || height != data.height) {
        glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_R8, data.width, data.height, 0, GL_RED, GL_UNSIGNED_BYTE, data.y_img.data());
        glBindTexture(GL_TEXTURE_RECTANGLE, tex[1]);
        glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_R8, data.width / 2, data.height / 2, 0, GL_RED, GL_UNSIGNED_BYTE, data.u_img.data());
        glBindTexture(GL_TEXTURE_RECTANGLE, tex[2]);
        glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_R8, data.width / 2, data.height / 2, 0, GL_RED, GL_UNSIGNED_BYTE, data.v_img.data());
    }
    else {
        glTexSubImage2D(GL_TEXTURE_RECTANGLE, 0, 0, 0, data.width, data.height, GL_RED, GL_UNSIGNED_BYTE, data.y_img.data());
        glBindTexture(GL_TEXTURE_RECTANGLE, tex[1]);
        glTexSubImage2D(GL_TEXTURE_RECTANGLE, 0, 0, 0, data.width / 2, data.height / 2, GL_RED, GL_UNSIGNED_BYTE, data.u_img.data());
        glBindTexture(GL_TEXTURE_RECTANGLE, tex[2]);
        glTexSubImage2D(GL_TEXTURE_RECTANGLE, 0, 0, 0, data.width / 2, data.height / 2, GL_RED, GL_UNSIGNED_BYTE, data.v_img.data());
    }
    glBindTexture(GL_TEXTURE_RECTANGLE, 0);
    glBindVertexArray(0);
    Program::instance().use(true);
    glUniform1f(0, data.focal_x);
    glUniform1f(1, data.focal_y);
    glUniform1f(2, data.center_x);
    glUniform1f(3, data.center_y);
    glUniform1i(8, 0);
    glUniform1i(9, 1);
    glUniform1i(10, 2);
    Program::instance().use(false);
}

void Model::save_view() const
{
    std::ofstream calibration("calib_" + name + ".txt");
    for (int i = 0; i < 16; ++i)
        calibration << matrix[i] << ' ';
}

void Model::translate(const double x, const double y, const double z)
{
    auto m = Eigen::Map<Eigen::Matrix4f>(matrix);
    Eigen::Affine3f a;
    a.matrix() = m;
    a.translate(Eigen::Vector3f(x, y, z));
    m = a.matrix();
}

void Model::rotate(const double rad, const double x, const double y, const double z)
{
    auto m = Eigen::Map<Eigen::Matrix4f>(matrix);
    Eigen::Affine3f a;
    a.matrix() = m;
    a.rotate(Eigen::AngleAxisf(rad, Eigen::Vector3f(x, y, z)));
    m = a.matrix();
}

void Model::reset_position()
{
    Eigen::Map<Eigen::Matrix4f>(matrix).setIdentity();
}
