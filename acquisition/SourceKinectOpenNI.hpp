#pragma once
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

#include <memory>
#include <string>
#include "Source.hpp"

class SourceKinectOpenNI : public Source
{
    struct Impl;
    std::unique_ptr<Impl> p;
    static const int width_ = 640, height_ = 480;
    std::string serial_number;

public:
    SourceKinectOpenNI(const int id = 0);
    ~SourceKinectOpenNI();
    void grab() override;
    void startImage() override;
    void startDepth() override;
    void startIr() override;
    void stopAll() override;
    void getImage(char* rgb) override;
    void getDepth(char* depth) override;
    void getIr(char* ir) override;
    int width() const override;
    int height() const override;
    std::string get_serial() const override;
};
