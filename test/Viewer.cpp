#include <functional>
#include <GL/glew.h>
#include <QKeyEvent>
#include <QMessageBox>
#include <QFileDialog>
#include "Viewer.hpp"
#include "../receiver/xvr_receiver.h"

Viewer::Viewer(const int w, const int h, QWidget* parent):
    QGLViewer(parent),
    width(w), height(h)/*,
    depth_stream(new std::ofstream("kinect0.depth", std::ios::binary))*/
{
    setWindowTitle("Meshified Point Cloud Viewer");
}

Viewer::~Viewer()
{
    ::xvr_receiver_destroy();
}

void Viewer::init()
{
    setKeyDescription(Qt::Key_W, "Toggles wireframe");
    setKeyDescription(Qt::Key_T, "Toggles textures");
    glewInit();
    glDisable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CW);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat ambient[4] = {0.2f, 0.2f, 0.2f, 1.0f};
    GLfloat diffuse[4] = {0.6f, 0.6f, 0.6f, 1.0f};
    GLfloat specular[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat position[4] = {0.0f, 4.0f, 10.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glLightfv(GL_LIGHT0, GL_POSITION, position);
    setSceneCenter(qglviewer::Vec(0, 0, 4));
    setSceneRadius(5);
    camera()->fitSphere(qglviewer::Vec(0, 0, 4), 5);
    camera()->setFOVToFitScene();
    startAnimation();

    ::xvr_receiver_init();
    ::xvr_receiver_load_static("test2.ply");
    ::xvr_receiver_start();
}

void Viewer::draw()
{
    ::xvr_receiver_draw();
}

void Viewer::keyPressEvent(QKeyEvent* e)
{
    if (e->key() == Qt::Key_W) {
        GLint previous_polygon_mode[2];
        glGetIntegerv(GL_POLYGON_MODE, previous_polygon_mode);
        const int mode = previous_polygon_mode[0] == GL_LINE ? GL_FILL : GL_LINE;
        glPolygonMode(GL_FRONT_AND_BACK, mode);
        updateGL();
    } else if (e->key() == Qt::Key_T) {
        if (glIsEnabled(GL_TEXTURE_2D) == GL_TRUE) {
            glDisable(GL_TEXTURE_2D);
            glEnable(GL_LIGHTING);
        }
        else {
            glEnable(GL_TEXTURE_2D);
            glDisable(GL_LIGHTING);
        }
        updateGL();
    }
    else if (e->key() == Qt::Key_0)
        selected_source = "cam0";
    else if (e->key() == Qt::Key_1)
        selected_source = "cam1";
    else if (e->key() == Qt::Key_2)
        selected_source = "cam2";
    else if (e->key() == Qt::Key_3)
        selected_source = "cam3";
    else if (e->key() == Qt::Key_R)
        xvr_receiver_reset_position(selected_source.c_str());
    else if (e->key() == Qt::Key_S)
        xvr_receiver_save_view();
    else if (e->key() == Qt::Key_Up)
        xvr_receiver_translate(selected_source.c_str(), 0.0, 0.0, 0.01);
    else if (e->key() == Qt::Key_Down)
        xvr_receiver_translate(selected_source.c_str(), 0.0, 0.0, -0.01);
    else if (e->key() == Qt::Key_Left)
        xvr_receiver_translate(selected_source.c_str(), -0.01, 0.0, 0.0);
    else if (e->key() == Qt::Key_Right)
        xvr_receiver_translate(selected_source.c_str(), 0.01, 0.0, 0.0);
    else if (e->key() == Qt::Key_E)
        xvr_receiver_translate(selected_source.c_str(), 0.0, 0.01, 0.0);
    else if (e->key() == Qt::Key_D)
        xvr_receiver_translate(selected_source.c_str(), 0.0, -0.01, 0.0);
    else if (e->key() == Qt::Key_P) {
        const QString filename = QFileDialog::getOpenFileName(this, "Open a record file", ".", "Records (*.beaming)");
        if (filename.isEmpty() == false)
            ::xvr_receiver_start_play(filename.toStdString().c_str());
    }
    else if (e->key() == Qt::Key_L) {
        const QString filename = QFileDialog::getSaveFileName(this, "Save the record file as", ".", "Records (*.beaming)");
        if (filename.isEmpty() == false)
            ::xvr_receiver_start_record(filename.toStdString().c_str());
    }
    else if (e->key() == Qt::Key_O)
        ::xvr_receiver_start();
    else
        QGLViewer::keyPressEvent(e);
}
