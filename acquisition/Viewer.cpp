#include <iostream>
#include <stdexcept>
#include <Eigen/Core>
#include <QMessageBox>
#include <QFile>
#include <GL/glew.h>
#include "Viewer.hpp"

Viewer::Viewer(QWidget *parent) :
    QGLViewer(parent)
{
}

Viewer::~Viewer()
{
    glDeleteProgram(prog);
    glDeleteTextures(1, tex);
    glDeleteBuffers(2, vbo);
    glDeleteVertexArrays(1, vao);
}

void Viewer::load(QtModelDescriptor data)
{
    glBindVertexArray(vao[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned) * data.tri.size(), data.tri.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * data.ver.size(), data.ver.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex[0]);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, data.width, data.height, 0, GL_RGB, GL_UNSIGNED_BYTE, data.rgb.data());
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindVertexArray(0);
    glProgramUniform1i(prog, uniform_camera_width, data.width);
    glProgramUniform1i(prog, uniform_camera_height, data.height);
    glProgramUniform1f(prog, uniform_camera_focal_x, data.focal_x);
    glProgramUniform1f(prog, uniform_camera_focal_y, data.focal_y);
    glProgramUniform1f(prog, uniform_camera_centre_x, data.center_x);
    glProgramUniform1f(prog, uniform_camera_centre_y, data.center_y);
    n_elements = data.tri.size();
    updateGL();
}

void Viewer::init()
{
    QFile vertex_shader_file(":/vertex_shader.vert");
    if (vertex_shader_file.open(QIODevice::ReadOnly | QIODevice::Text) == false) {
        QMessageBox::warning(this, "Error Loading shader", "Unable to load the shader source code");
        throw std::runtime_error("Unable to load the vertex shader");
    }
    auto vertex_shader_buffer = vertex_shader_file.readAll();
    QFile fragment_shader_file(":/fragment_shader.frag");
    if (fragment_shader_file.open(QIODevice::ReadOnly | QIODevice::Text) == false) {
        QMessageBox::warning(this, "Error Loading shader", "Unable to load the fragment source code");
        throw std::runtime_error("Unable to load the fragment shader");
    }
    auto fragment_shader_buffer = fragment_shader_file.readAll();
    glewInit();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CW);
    glGenVertexArrays(1, vao);
    glGenBuffers(2, vbo);
    glGenTextures(1, tex);
    glBindVertexArray(vao[0]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[0]);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, tex[0]);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    setSceneCenter(qglviewer::Vec(0, 0, 4));
    setSceneRadius(5);
    camera()->fitSphere(qglviewer::Vec(0, 0, 4), 5);
    camera()->setFOVToFitScene();
    const GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    const char* strings[1] {vertex_shader_buffer.data()};
    int len[1] {vertex_shader_buffer.size()};
    glShaderSource(vertex_shader, 1, strings, len);
    glCompileShader(vertex_shader);
    GLint status;
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &status);
    if (status == GL_FALSE) {
        GLint info_log_length = 0;
        glGetShaderiv(vertex_shader, GL_INFO_LOG_LENGTH , &info_log_length);
        std::vector<GLchar> info_log(info_log_length);
        glGetShaderInfoLog(vertex_shader, info_log_length, &info_log_length, info_log.data());
        std::cerr << "Vertex Compilation log:\n" << info_log.data() << std::endl;
    }
    const GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    strings[0] = fragment_shader_buffer.data();
    len[0] = fragment_shader_buffer.size();
    glShaderSource(fragment_shader, 1, strings, len);
    glCompileShader(fragment_shader);
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &status);
    if (status == GL_FALSE) {
        GLint info_log_length = 0;
        glGetShaderiv(fragment_shader, GL_INFO_LOG_LENGTH , &info_log_length);
        std::vector<GLchar> info_log(info_log_length);
        glGetShaderInfoLog(fragment_shader, info_log_length, &info_log_length, info_log.data());
        std::cerr << "Fragment Compilation log:\n" << info_log.data() << std::endl;
    }
    prog = glCreateProgram();
    glAttachShader(prog, vertex_shader);
    glAttachShader(prog, fragment_shader);
    glLinkProgram(prog);
    glGetProgramiv(prog, GL_LINK_STATUS, &status);
    if (status == GL_FALSE) {
        GLint info_log_length = 0;
        glGetProgramiv(prog, GL_INFO_LOG_LENGTH , &info_log_length);
        std::vector<GLchar> compiler_log(info_log_length);
        glGetProgramInfoLog(prog, info_log_length, &info_log_length, compiler_log.data());
        std::cerr << "Linker log:\n" << compiler_log.data() << std::endl;
    }
    glDetachShader(prog, fragment_shader);
    glDetachShader(prog, vertex_shader);
    glDeleteShader(fragment_shader);
    glDeleteShader(vertex_shader);
    uniform_camera_width = glGetUniformLocation(prog, "camera_width");
    uniform_camera_height = glGetUniformLocation(prog, "camera_height");
    uniform_camera_focal_x = glGetUniformLocation(prog, "camera_focal_x");
    uniform_camera_focal_y = glGetUniformLocation(prog, "camera_focal_y");
    uniform_camera_centre_x = glGetUniformLocation(prog, "camera_centre_x");
    uniform_camera_centre_y = glGetUniformLocation(prog, "camera_centre_y");
    uniform_mvp_matrix = glGetUniformLocation(prog, "mvp_matrix");
    const auto uniform_texture = glGetUniformLocation(prog, "camera_texture");
    glProgramUniform1i(prog, uniform_texture, 0);
}

void Viewer::draw()
{
    if (n_elements == 0)
        return;
    Eigen::Matrix4d mvp_matrix;
    camera()->getModelViewProjectionMatrix(mvp_matrix.data());
    Eigen::Matrix4f m = mvp_matrix.cast<float>();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex[0]);
    glUseProgram(prog);
    glUniformMatrix4fv(uniform_mvp_matrix, 1, GL_FALSE, m.data());
    glBindVertexArray(vao[0]);
    glDrawElements(GL_TRIANGLES, n_elements, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
    glUseProgram(0);
    glBindTexture(GL_TEXTURE_2D, 0);
}


bool Viewer::wireframe() const
{
    GLint v;
    glGetIntegerv(GL_POLYGON_MODE, &v);
    return v == GL_LINE;
}

void Viewer::setWireframe(const bool v)
{
    glPolygonMode(GL_FRONT_AND_BACK, v ? GL_LINE : GL_FILL);
}
