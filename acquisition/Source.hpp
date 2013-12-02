#pragma once
/*
    Copyright (C) 2011-2012 Paolo Simone Gasparello <p.gasparello@sssup.it>

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

#include <string>

class Source
{
public:
    virtual ~Source() {}
    virtual void grab() = 0;
    virtual void startImage() {}
    virtual void startDepth() {}
    virtual void startIr() {}
    virtual void stopAll() {}
    virtual void getImage(char* rgb) = 0;
    virtual void getDepth(char* depth) = 0;
    virtual void getIr(char* ir) = 0;
    virtual int width() const = 0;
    virtual int height() const = 0;
    virtual std::string get_serial() const {
        return "unknown";
    }
};
