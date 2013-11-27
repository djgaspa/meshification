/*
    Copyright (C) 2011-2013 Paolo Simone Gasparello <p.gasparello@sssup.it>

    This file is part of meshificator.

    meshificator is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    meshificator is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with meshificator.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include <memory>
#include <string>
#include <vector>
#include <QMetaType>
#include "QtModelDescriptor.h"

class QTimerEvent;
class Source;
class DepthMeshifier;
class Consumer;
class AsyncWorker;

using RgbBuffer = std::vector<char>;
Q_DECLARE_METATYPE(RgbBuffer)

class QtAcquisition : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool borderColorEnabled READ isBorderColorEnabled WRITE setBorderColorEnabled)
    Q_PROPERTY(bool draw2dEnabled READ isDraw2dEnabled WRITE setDraw2dEnabled)
    Q_PROPERTY(bool markerEnabled READ isMarkerEnabled WRITE setMarkerEnabled)
    Q_PROPERTY(double markerSize READ getMarkerSize WRITE setMarkerSize)
    Q_PROPERTY(int nearPlane READ nearPlane WRITE setNearPlane)
    Q_PROPERTY(int farPlane READ farPlane WRITE setFarPlane)
    Q_PROPERTY(int minArea READ minContourArea WRITE setMinArea)
    Q_PROPERTY(int minContourArea READ minContourArea WRITE setMinContourArea)
    Q_PROPERTY(int depthThreshold READ depthThreshold WRITE setDepthThreshold)
    Q_PROPERTY(int minThreshold READ minThreshold WRITE setMinThreshold)
    Q_PROPERTY(int maxThreshold READ maxThreshold WRITE setMaxThreshold)
    Q_PROPERTY(int approxDP READ approxDP WRITE setApproxDP)
    Q_PROPERTY(int dilateErode READ dilateErode WRITE setDilateErode)
    Q_PROPERTY(bool backgroundSubtractionEnabled READ isBackgroundSubtractionEnabled WRITE setBackgroundSubtractionEnabled)

    std::unique_ptr<Source> camera;
    const std::string calib_file;
    std::unique_ptr<DepthMeshifier> meshify;
    std::unique_ptr<Consumer> consume;
    std::unique_ptr<AsyncWorker> consumer_worker;
    std::vector<unsigned> tri;
    int width = 640, height = 480;
    float focal_x, focal_y, center_x, center_y;
    std::vector<float> k, t, r;
    int frame_id = 0;
    int timer_id;

    void timerEvent(QTimerEvent*) override;
    void process_frame();

public:
    QtAcquisition(const int cam_id, const std::string& name, const std::string& address, QObject *parent = 0);
    ~QtAcquisition();
    bool isBorderColorEnabled() const;
    bool isDraw2dEnabled() const;
    bool isMarkerEnabled() const;
    double getMarkerSize() const;
    int nearPlane() const;
    int farPlane() const;
    int minArea() const;
    int minContourArea() const;
    int depthThreshold() const;
    int minThreshold() const;
    int maxThreshold() const;
    int approxDP() const;
    int dilateErode() const;
    bool isBackgroundSubtractionEnabled() const;

signals:
    void draw(RgbBuffer rgb, int width, int height);
    void message(QString m);
    void update(QtModelDescriptor desc);
    
public slots:
    void setup();
    void setAddress(QString name, QString address);
    void setBorderColorEnabled(bool e);
    void setDraw2dEnabled(bool e);
    void setMarkerEnabled(bool e);
    void setMarkerSize(double s);
    void saveView();
    void setNearPlane(int d);
    void setFarPlane(int d);
    void setMinArea(int a);
    void setMinContourArea(int a);
    void setDepthThreshold(int t);
    void setMinThreshold(int t);
    void setMaxThreshold(int t);
    void setApproxDP(int a);
    void setDilateErode(int s);
    void setBackgroundSubtractionEnabled(bool e);
};
