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

static
oglplus::Program build_program();

class Program
{
    const oglplus::Program p;
    Program();
public:
    ~Program();
    static Program& instance();
    void use(const bool b) const;
    const oglplus::VertexAttribArray vertex_attrib_array;
    oglplus::Uniform<float> camera_focal_x_uniform, camera_focal_y_uniform, camera_centre_x_uniform, camera_centre_y_uniform;
    oglplus::Uniform<float> K_uniform;
    oglplus::Uniform<oglplus::Vec3f> T_uniform;
    oglplus::Uniform<oglplus::Mat3f> R_uniform;
    oglplus::Uniform<oglplus::Mat4f> mvp_matrix_uniform;
};

Program::Program() try :
    p(build_program()),
    vertex_attrib_array(p, "vertex"),
    camera_focal_x_uniform(p, "camera_focal_x"),
    camera_focal_y_uniform(p, "camera_focal_y"),
    camera_centre_x_uniform(p, "camera_centre_x"),
    camera_centre_y_uniform(p, "camera_centre_y"),
    K_uniform(p, "K"),
    T_uniform(p, "T"),
    R_uniform(p, "R"),
    mvp_matrix_uniform(p, "mvp_matrix")
{
    p.Use();
    (p / "camera_y") = 0;
    (p / "camera_u") = 1;
    (p / "camera_v") = 2;
    p.UseNone();
} catch (const oglplus::ProgramBuildError& pbe) {
   std::cerr <<
                "Program build error (in " <<
                pbe.GLSymbol() << ", " <<
                pbe.ClassName() << " '" <<
                pbe.ObjectDescription() << "'): " <<
                pbe.what() << std::endl <<
                pbe.Log() << std::endl;
   pbe.Cleanup();
}

oglplus::Program build_program()
{
    oglplus::Program p;
    oglplus::VertexShader vs;
    vs.Source(
                "#version 140\n"
                "in vec3 vertex;\n"

                "uniform float camera_focal_x;\n"
                "uniform float camera_focal_y;\n"
                "uniform float camera_centre_x;\n"
                "uniform float camera_centre_y;\n"
                "uniform float K[5];\n"
                "uniform vec3 T;\n"
                "uniform mat3 R;\n"
                "uniform mat4 mvp_matrix;\n"

                "out vec2 tex_coord;\n"

                "void main()\n"
                "{\n"
                "    vec4 v = vec4(vertex, 1.0);\n"
                "    gl_Position = mvp_matrix * v;\n"
                "    vec3 vt = R * vertex + T;\n"

                "    float r2, r4, r6, a1, a2, a3, cdist;\n"

                "    vt.x /= vt.z;\n"
                "    vt.y /= vt.z;\n"

                "    r2 = vt.x * vt.x + vt.y * vt.y;\n"
                "    r4 = r2 * r2;\n"
                "    r6 = r4 * r2;\n"
                "    a1 = 2 * vt.x * vt.y;\n"
                "    a2 = r2 + 2 * vt.x * vt.x;\n"
                "    a3 = r2 + 2 * vt.y * vt.y;\n"
                "    cdist = 1 + K[0] * r2 + K[1] * r4 + K[4] * r6;\n"
                "    float xd = vt.x * cdist + K[2] * a1 + K[3] * a2;\n"
                "    float yd = vt.y * cdist + K[2] * a3 + K[3] * a1;\n"

                "    tex_coord.s = camera_focal_x * xd + camera_centre_x;\n"
                "    tex_coord.t = camera_focal_y * yd + camera_centre_y;\n"
                "}\n"
                ).Compile();
    oglplus::FragmentShader fs;
    fs.Source(
                "#version 140\n"
                "#extension GL_ARB_texture_rectangle : enable\n"
                "in vec2 tex_coord;\n"

                "uniform sampler2DRect camera_y;\n"
                "uniform sampler2DRect camera_u;\n"
                "uniform sampler2DRect camera_v;\n"

                "out vec4 frag_color;\n"

                "void main()\n"
                "{\n"
                "   float y = texture2DRect(camera_y, tex_coord).r;\n"
                "   float u = texture2DRect(camera_u, tex_coord / 2.0).r - 0.5;\n"
                "   float v = texture2DRect(camera_v, tex_coord / 2.0).r - 0.5;\n"
                "   float r = y + 1.402 * v;\n"
                "   float g = y - 0.344 * u - 0.714 * v;\n"
                "   float b = y + 1.772 * u;\n"
                "   frag_color = vec4(r, g, b, 1.0);\n"
                "}\n"
                ).Compile();
    p.AttachShader(vs).AttachShader(fs).Link().DetachShader(fs).DetachShader(vs);
    return p;
}

Program::~Program()
{
}

Program& Program::instance()
{
    static Program prog;
    return prog;
}

void Program::use(const bool b) const
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
    Program::instance().vertex_attrib_array.Enable();
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
    auto& prog = Program::instance();
    prog.use(true);
    prog.camera_focal_x_uniform.Set(focal_x);
    prog.camera_focal_y_uniform.Set(focal_y);
    prog.camera_centre_x_uniform.Set(center_x);
    prog.camera_centre_y_uniform.Set(center_y);
    prog.K_uniform.Set(5, k);
    prog.T_uniform.Set(oglplus::Vec3f(t));
    prog.R_uniform.Set(Transposed(oglplus::Mat3f(r)));
    float mv[16], pr[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, mv);
    glGetFloatv(GL_PROJECTION_MATRIX, pr);
    Eigen::Map<Eigen::Matrix4f> view_matrix(mv), proj_matrix(pr);
    const Eigen::Matrix4f mvp_matrix = (proj_matrix * view_matrix);
    prog.mvp_matrix_uniform.Set(Transposed(oglplus::Mat4f(mvp_matrix.data(), 16)));
    glDrawElements(GL_TRIANGLES, n_elements, GL_UNSIGNED_INT, 0);
    prog.use(false);
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
    const auto& prog = Program::instance();
    prog.vertex_attrib_array.Setup<oglplus::Vec3f>();
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
    focal_x = data.focal_x;
    focal_y = data.focal_y;
    center_x = data.center_x;
    center_y = data.center_y;
    std::copy(data.t, data.t + 3, t);
    std::copy(data.r, data.r + 9, r);
    std::copy(data.k, data.k + 5, k);
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
