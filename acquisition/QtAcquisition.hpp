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
