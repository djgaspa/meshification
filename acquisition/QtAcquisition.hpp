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

    std::unique_ptr<Source> camera;
    std::unique_ptr<DepthMeshifier> meshify;
    std::unique_ptr<Consumer> consume;
    std::unique_ptr<AsyncWorker> consumer_worker;
    std::vector<unsigned> tri;
    int width = 640, height = 480;
    int frame_id = 0;
    int timer_id;

    void timerEvent(QTimerEvent*) override;
    void process_frame();

public:
    QtAcquisition(const int cam_id, const std::string& address, const std::string& calib, QObject *parent = 0);
    ~QtAcquisition();
    bool isBorderColorEnabled() const;
    bool isDraw2dEnabled() const;
    bool isMarkerEnabled() const;

signals:
    void draw(RgbBuffer rgb, int width, int height);
    void message(QString m);
    
public slots:
    void setup();
    void setAddress(QString address);
    void setBorderColorEnabled(bool e);
    void setDraw2dEnabled(bool e);
    void setMarkerEnabled(bool e);
    void saveView();
};
