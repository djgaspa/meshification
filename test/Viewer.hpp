#pragma once
#include <QGLViewer/qglviewer.h>


class Viewer : public QGLViewer
{
    Q_OBJECT

    const int width, height;

    void init();
    void draw();
    void keyPressEvent(QKeyEvent* e);

public:
    Viewer(const int w, const int h, QWidget* parent = 0);
    ~Viewer();
};
