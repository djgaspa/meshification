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

#include <iostream>
#include <QThread>
#include <QSettings>
#include <QMessageBox>
#include "AcquisitionUI.hpp"
#include <ui_AcquisitionUI.h>
#include "QtAcquisition.hpp"
#include "Viewer.hpp"

AcquisitionUI::AcquisitionUI(QWidget *parent) :
    QMainWindow(parent),
    thread(new QThread(this)),
    settings(new QSettings(QApplication::applicationDirPath() + "/acquisition.ini", QSettings::IniFormat, this)),
    ui(new Ui::AcquisitionUI)
{
    qRegisterMetaType<QtModelDescriptor>("QtModelDescriptor");
    ui->setupUi(this);
    const QString Octet = "(?:[0-1]?[0-9]?[0-9]|2[0-4][0-9]|25[0-5])";
    ui->le_address->setValidator(new QRegExpValidator(QRegExp("^" + Octet + "\\." + Octet + "\\." + Octet + "\\." + Octet + "$"), this));
    connect(thread, SIGNAL(started()), SLOT(started()));
    connect(thread, SIGNAL(finished()), SLOT(stopped()));
    connect(ui->action_Wireframe, SIGNAL(toggled(bool)), ui->viewer, SLOT(setWireframe(bool)));
    thread->connect(ui->pb_stop, SIGNAL(clicked()), SLOT(quit()));
    const auto address = settings->value("receiver_address", "127.0.0.1").toString();
    const auto name = settings->value("name", "cam").toString();
    ui->le_name->setText(name);
    ui->le_address->setText(address);
    emit addressChanged(name, address);
    ui->sb_camera->setValue(settings->value("camera_id", 0).toInt());
}

AcquisitionUI::~AcquisitionUI()
{
    thread->quit();
    thread->wait();
    delete ui;
}

void AcquisitionUI::on_le_name_editingFinished()
{
    if (ui->le_name->isModified() == false)
        return;
    ui->le_name->setModified(false);
    const auto name = ui->le_name->text();
    settings->setValue("name", name);
    const auto address = ui->le_address->text();
    emit addressChanged(name, address);
}

void AcquisitionUI::on_le_address_editingFinished()
{
    if (ui->le_address->isModified() == false)
        return;
    ui->le_address->setModified(false);
    const auto address = ui->le_address->text();
    settings->setValue("receiver_address", address);
    const auto name = ui->le_name->text();
    emit addressChanged(name, address);
}

void AcquisitionUI::on_pb_start_clicked()
try {
    auto t = new QtAcquisition(ui->sb_camera->value(), ui->le_name->text().toStdString(), ui->le_address->text().toStdString());
    t->moveToThread(thread);
    connect(thread, SIGNAL(started()), t, SLOT(setup()));
    connect(thread, SIGNAL(finished()), t, SLOT(deleteLater()));
    thread->start();
    ui->actionColor_Edges->setChecked(t->isBorderColorEnabled());
    ui->actionDraw2D->setChecked(t->isDraw2dEnabled());
    ui->actionMarker->setChecked(t->isMarkerEnabled());
    t->connect(ui->actionColor_Edges, SIGNAL(toggled(bool)), SLOT(setBorderColorEnabled(bool)));
    t->connect(ui->actionDraw2D, SIGNAL(toggled(bool)), SLOT(setDraw2dEnabled(bool)));
    t->connect(ui->actionMarker, SIGNAL(toggled(bool)), SLOT(setMarkerEnabled(bool)));
    t->connect(ui->actionSave_View, SIGNAL(triggered()), SLOT(saveView()));
    t->connect(ui->actionBackground_Subtraction, SIGNAL(toggled(bool)), SLOT(setBackgroundSubtractionEnabled(bool)));
    t->connect(this, SIGNAL(addressChanged(QString, QString)), SLOT(setAddress(QString, QString)));
    ui->sb_near_plane->setValue(t->nearPlane());
    t->connect(ui->sb_near_plane, SIGNAL(valueChanged(int)), SLOT(setNearPlane(int)));
    ui->sb_far_plane->setValue(t->farPlane());
    t->connect(ui->sb_far_plane, SIGNAL(valueChanged(int)), SLOT(setFarPlane(int)));
    ui->sb_min_contour_area->setValue(t->minContourArea());
    t->connect(ui->sb_min_contour_area, SIGNAL(valueChanged(int)), SLOT(setMinContourArea(int)));
    ui->sb_depth_threshold->setValue(t->depthThreshold());
    t->connect(ui->sb_depth_threshold, SIGNAL(valueChanged(int)), SLOT(setDepthThreshold(int)));
    ui->sb_min_area->setValue(t->minArea());
    t->connect(ui->sb_min_area, SIGNAL(valueChanged(int)), SLOT(setMinArea(int)));
    ui->sb_min_threshold->setValue(t->minThreshold());
    t->connect(ui->sb_min_threshold, SIGNAL(valueChanged(int)), SLOT(setMinThreshold(int)));
    ui->sb_max_threshold->setValue(t->maxThreshold());
    t->connect(ui->sb_max_threshold, SIGNAL(valueChanged(int)), SLOT(setMaxThreshold(int)));
    ui->sb_approx_dp->setValue(t->approxDP());
    t->connect(ui->sb_approx_dp, SIGNAL(valueChanged(int)), SLOT(setApproxDP(int)));
    ui->sb_dilate_erode->setValue(t->dilateErode());
    t->connect(ui->sb_dilate_erode, SIGNAL(valueChanged(int)), SLOT(setDilateErode(int)));
    connect(t, SIGNAL(draw(RgbBuffer, int, int)), SLOT(draw(RgbBuffer, int, int)));
    connect(t, SIGNAL(update(QtModelDescriptor)), ui->viewer, SLOT(load(QtModelDescriptor)));
    connect(t, SIGNAL(message(QString)), SLOT(message(QString)));
    t->setMarkerSize(settings->value("marker_size", 0.288).toDouble());
} catch (const std::exception& e) {
    QMessageBox::warning(this, "Device error", e.what());
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
    ui->sb_camera->setEnabled(false);
    settings->setValue("camera_id", ui->sb_camera->value());
}

void AcquisitionUI::stopped()
{
    ui->pb_start->setEnabled(true);
    ui->pb_stop->setEnabled(false);
    ui->sb_camera->setEnabled(true);
}
