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
#include "SourceKinectOpenNI.hpp"

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
                "<Node type=\"Depth\" name=\"Depth1\">"
                        "<Configuration>"
                                "<MapOutputMode xRes=\"640\" yRes=\"480\" FPS=\"30\"/>"
                                "<Mirror on=\"false\"/>"
                        "</Configuration>"
                "</Node>"
            "</ProductionNodes>"
        "</OpenNI>\n";

struct SourceKinectOpenNI::Impl
{
    xn::Context ctx;
    xn::ProductionNode production_node;
    xn::ImageGenerator image;
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

SourceKinectOpenNI::SourceKinectOpenNI(const int id) :
    p(new Impl)
{
    check(p->ctx.Init());
    xn::NodeInfoList devicesList;
    check(p->ctx.EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, nullptr, devicesList, 0));
    xn::NodeInfoList::Iterator it = devicesList.Begin();
    for (int i = 0; i < id && it != devicesList.End(); ++i)
        it++;
    if (it == devicesList.End()) {
        std::ostringstream err;
        err << "CvCapture_OpenNI::CvCapture_OpenNI : Failed device with index " << id;
        throw std::runtime_error(err.str());
    }
    xn::NodeInfo deviceNode = *it;
    check(p->ctx.CreateProductionTree(deviceNode, p->production_node));
    check(p->ctx.RunXmlScript(xml_config.c_str()));
    check(p->image.Create(p->ctx));
    check(p->depth.Create(p->ctx));
    //check(p->depth.GetAlternativeViewPointCap().SetViewPoint(p->image));
    check(p->ctx.StartGeneratingAll());
}

SourceKinectOpenNI::~SourceKinectOpenNI()
{
    p->ctx.StopGeneratingAll();
    p->depth.Release();
    p->image.Release();
    p->ctx.Release();
}

void SourceKinectOpenNI::grab(char* rgb, char* depth)
{
    check(p->ctx.WaitAnyUpdateAll());
    const char* buffer_rgb = (const char*)p->image.GetImageMap();
    std::copy(buffer_rgb, buffer_rgb + width_ * height_ * 3, rgb);
    const char* buffer_depth = (const char*)p->depth.GetDepthMap();
    std::copy(buffer_depth, buffer_depth + width_ * height_ * 2, depth);
}

int SourceKinectOpenNI::width() const
{
    return width_;
}

int SourceKinectOpenNI::height() const
{
    return height_;
}
