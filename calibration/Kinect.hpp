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

class Kinect
{
    struct Impl;
    std::unique_ptr<Impl> p;
    static const int width_ = 640, height_ = 480;

public:
    Kinect(const int id = 0);
    ~Kinect();
    void grab();
    int width() const;
    int height() const;
    const unsigned char *retrieveImage();
    const unsigned short *retrieveIr();
    const unsigned short* retrieveDepth();
    void stopAll();
    void startIr();
    void startImage();
    void startDepth();
};
