#pragma once
#include <vector>
#include <QMainWindow>
#include <QString>

class QThread;

namespace Ui {
class AcquisitionUI;
}

using RgbBuffer = std::vector<char>;

class AcquisitionUI : public QMainWindow
{
    Q_OBJECT
    QThread* thread;
    Ui::AcquisitionUI* ui;

private slots:
    void on_le_address_editingFinished();
    void on_pb_start_clicked();
    void draw(RgbBuffer buffer, int width, int height);
    void message(QString);
    void started();
    void stopped();

public:
    explicit AcquisitionUI(QWidget* parent = 0);
    ~AcquisitionUI();

signals:
    void addressChanged(QString);
};
