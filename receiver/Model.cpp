#include <GL/glew.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Model.hpp"
#include "Data3d.hpp"

Model::Model()
{
    glGenVertexArrays(1, vao);
    glGenBuffers(n_vbo, vbo);
    glGenTextures(1, tex);
    Eigen::Matrix4f::Map(&model_matrix[0]).setIdentity();
    glBindVertexArray(vao[0]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[0]);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glBindTexture(GL_TEXTURE_2D, tex[0]);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindVertexArray(0);
}

Model::~Model()
{
    glDeleteBuffers(n_vbo, vbo);
    glDeleteTextures(1, tex);
    glDeleteVertexArrays(1, vao);
}

void Model::draw() const
{
    if (n_elements == 0)
	return;
    glBindVertexArray(vao[0]);
    glBindTexture(GL_TEXTURE_2D, tex[0]);
    glPushMatrix();
    glMultMatrixf(model_matrix);
    glDrawElements(GL_TRIANGLES, n_elements, GL_UNSIGNED_INT, 0);
    glPopMatrix();
    glBindVertexArray(0);
}

void Model::load(const Data3d& data)
{
    n_elements = data.tri.size();
    if (n_elements == 0)
	return;
    std::copy(data.modelview, data.modelview + 16, model_matrix);
    glBindVertexArray(vao[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned) * data.tri.size(), data.tri.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * data.ver.size(), data.ver.data(), GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * data.tex.size(), data.tex.data(), GL_STATIC_DRAW);
    glTexCoordPointer(2, GL_FLOAT, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[3]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * data.nor.size(), data.nor.data(), GL_STATIC_DRAW);
    glNormalPointer(GL_FLOAT, 0, 0);
    glBindTexture(GL_TEXTURE_2D, tex[0]);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, data.width, data.height, 0, GL_RGB, GL_UNSIGNED_BYTE, &data.bgr.front());
    glBindVertexArray(0);
}