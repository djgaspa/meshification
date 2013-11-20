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

#define linux 1
#include <sstream>
#include <stdexcept>
#include <XnCppWrapper.h>
#include <opencv2/opencv.hpp>
#include "Kinect.hpp"

static const std::string xml_config =
        "<OpenNI>"
            "<Licenses>"
                "<License vendor=\"PrimeSense\" key=\"0KOIk2JeIBYClPWVnMoRKn5cdY4=\"/>"
            "</Licenses>"
            "<Log writeToConsole=\"false\" writeToFile=\"false\">"
                "<LogLevel value=\"3\"/>"
                "<Masks>"
                    "<Mask name=\"ALL\" on=\"true\"/>"
                "</Masks>"
                "<Dumps>"
                "</Dumps>"
            "</Log>"
            "<ProductionNodes>"
                "<Node type=\"Image\" name=\"Image1\" stopOnError=\"false\">"
                    "<Configuration>"
                        "<MapOutputMode xRes=\"640\" yRes=\"480\" FPS=\"30\"/>"
                        "<Mirror on=\"false\"/>"
                    "</Configuration>"
                "</Node> "
                "<Node type=\"IR\" name=\"IR1\" stopOnError=\"false\">"
                    "<Configuration>"
                        "<MapOutputMode xRes=\"640\" yRes=\"480\" FPS=\"30\"/>"
                        "<Mirror on=\"false\"/>"
                    "</Configuration>"
                "</Node> "
                "<Node type=\"Depth\" name=\"Depth1\">"
                    "<Configuration>"
                        "<MapOutputMode xRes=\"640\" yRes=\"480\" FPS=\"30\"/>"
                        "<Mirror on=\"false\"/>"
                    "</Configuration>"
                "</Node>"
            "</ProductionNodes>"
        "</OpenNI>\n";

struct Kinect::Impl
{
    xn::Context ctx;
    xn::ImageGenerator image;
    xn::IRGenerator ir;
    xn::DepthGenerator depth;
};

static void check(const XnStatus ret)
{
    if (ret == XN_STATUS_OK)
        return;
    std::ostringstream err;
    err << "Kinect error " << ret << ". " << ::xnGetStatusName(ret) << " : " << ::xnGetStatusString(ret);
    throw std::runtime_error(err.str());
}

Kinect::Kinect(const int id) :
    p(new Impl)
{
    check(p->ctx.Init());
    check(p->ctx.RunXmlScript(xml_config.c_str()));
    check(p->image.Create(p->ctx));
    check(p->depth.Create(p->ctx));
    check(p->ir.Create(p->ctx));
    check(p->ir.StartGenerating());
}

Kinect::~Kinect()
{
    p->ctx.StopGeneratingAll();
    p->depth.Release();
    p->image.Release();
    p->ctx.Release();
}

void Kinect::grab()
{
    check(p->ctx.WaitAnyUpdateAll());
}

int Kinect::width() const
{
    return width_;
}

int Kinect::height() const
{
    return height_;
}

const unsigned char* Kinect::retrieveImage()
{
    return p->image.GetImageMap();
}

const unsigned short* Kinect::retrieveIr()
{
    return p->ir.GetIRMap();
}

const unsigned short *Kinect::retrieveDepth()
{
    return p->depth.GetDepthMap();
}

void Kinect::stopAll()
{
    check(p->ctx.StopGeneratingAll());
}

void Kinect::startIr()
{
    check(p->ir.StartGenerating());
}

void Kinect::startImage()
{
    check(p->image.StartGenerating());
}

void Kinect::startDepth()
{
    check(p->depth.StartGenerating());
}
