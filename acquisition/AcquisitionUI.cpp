#include <iostream>
#include <QThread>
#include "AcquisitionUI.hpp"
#include <ui_AcquisitionUI.h>
#include "QtAcquisition.hpp"

AcquisitionUI::AcquisitionUI(QWidget *parent) :
    QMainWindow(parent),
    thread(new QThread(this)),
    ui(new Ui::AcquisitionUI)
{
    ui->setupUi(this);
    const QString Octet = "(?:[0-1]?[0-9]?[0-9]|2[0-4][0-9]|25[0-5])";
    ui->le_address->setValidator(new QRegExpValidator(QRegExp("^" + Octet + "\\." + Octet + "\\." + Octet + "\\." + Octet + "$"), this));
    connect(thread, SIGNAL(started()), SLOT(started()));
    connect(thread, SIGNAL(finished()), SLOT(stopped()));
    thread->connect(ui->pb_start, SIGNAL(clicked()), SLOT(start()));
    thread->connect(ui->pb_stop, SIGNAL(clicked()), SLOT(quit()));
}

AcquisitionUI::~AcquisitionUI()
{
    thread->quit();
    thread->wait();
    delete ui;
}

void AcquisitionUI::on_le_address_editingFinished()
{
    if (ui->le_address->isModified() == false)
        return;
    ui->le_address->setModified(false);
    emit addressChanged(ui->le_address->text());
}

void AcquisitionUI::on_pb_start_clicked()
{
    auto t = new QtAcquisition(0, ui->le_address->text().toStdString(), "cam.yml");
    t->moveToThread(thread);
    connect(thread, SIGNAL(started()), t, SLOT(setup()));
    connect(thread, SIGNAL(finished()), t, SLOT(deleteLater()));
    ui->actionColor_Edges->setChecked(t->isBorderColorEnabled());
    ui->actionDraw2D->setChecked(t->isDraw2dEnabled());
    ui->actionMarker->setChecked(t->isMarkerEnabled());
    t->connect(ui->actionColor_Edges, SIGNAL(toggled(bool)), SLOT(setBorderColorEnabled(bool)));
    t->connect(ui->actionDraw2D, SIGNAL(toggled(bool)), SLOT(setDraw2dEnabled(bool)));
    t->connect(ui->actionMarker, SIGNAL(toggled(bool)), SLOT(setMarkerEnabled(bool)));
    t->connect(ui->actionSave_View, SIGNAL(triggered()), SLOT(saveView()));
    t->connect(this, SIGNAL(addressChanged(QString)), SLOT(setAddress(QString)));
    connect(t, SIGNAL(draw(RgbBuffer, int, int)), SLOT(draw(RgbBuffer, int, int)));
    connect(t, SIGNAL(message(QString)), SLOT(message(QString)));
}

void AcquisitionUI::draw(RgbBuffer buffer, int width, int height)
{
    const QImage img((const unsigned char*)buffer.data(), width, height, QImage::Format_RGB888);
    ui->lbl_frame->setPixmap(QPixmap::fromImage(img));
    ui->lbl_frame->resize(width, height);
}

void AcquisitionUI::message(QString m)
{
    ui->statusbar->showMessage(m);
}

void AcquisitionUI::started()
{
    ui->pb_start->setEnabled(false);
    ui->pb_stop->setEnabled(true);
}

void AcquisitionUI::stopped()
{
    ui->pb_start->setEnabled(true);
    ui->pb_stop->setEnabled(false);
}
