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
