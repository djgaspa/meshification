#pragma once
#include <QVector>

struct QtModelDescriptor
{
    int width, height;
    float center_x, center_y, focal_x, focal_y;
    QVector<float> ver;
    QVector<unsigned> tri;
    QVector<char> rgb;
};

Q_DECLARE_METATYPE(QtModelDescriptor);
