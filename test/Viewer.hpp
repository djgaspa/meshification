#pragma once
#include <string>
#include <QGLViewer/qglviewer.h>


class Viewer : public QGLViewer
{
    Q_OBJECT

    const int width, height;
    std::string selected_source;

    void init();
    void draw();
    void keyPressEvent(QKeyEvent* e);

public:
    Viewer(const int w, const int h, QWidget* parent = 0);
    ~Viewer();
};
