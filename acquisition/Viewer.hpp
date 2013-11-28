#pragma once
#include <QGLViewer/qglviewer.h>
#include "QtModelDescriptor.h"

class Viewer : public QGLViewer
{
    Q_OBJECT
    Q_PROPERTY(bool wireframe READ wireframe WRITE setWireframe);
    unsigned vao[1], vbo[2], tex[1], prog;
    int uniform_camera_focal_x, uniform_camera_focal_y, uniform_camera_centre_x, uniform_camera_centre_y, uniform_K, uniform_T, uniform_R, uniform_vp_matrix, uniform_model_matrix;
    int n_elements = 0;

    void init();
    void draw();

public:
    explicit Viewer(QWidget* parent = 0);
    ~Viewer();
    
signals:
    
public slots:
    void setModelMatrix(std::vector<float> m);
    void load(QtModelDescriptor data);
    bool wireframe() const;
    void setWireframe(const bool);
};
