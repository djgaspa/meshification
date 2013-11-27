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
#include <libusb-1.0/libusb.h>
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

static
std::string get_serial_number(const int ref_bus, const int ref_address)
{
    std::string ret = "unknown";
    libusb_context *context;
    int result = libusb_init(&context);
    if (result < 0)
        return "unknown";
    libusb_device **devices;
    const auto count = libusb_get_device_list(context, &devices);
    if (count < 0)
        goto error1;
    int devIdx;
    for (devIdx = 0; devIdx < count; ++devIdx) {
        libusb_device* device = devices[devIdx];
        const auto busId = libusb_get_bus_number(device);
        const auto address = libusb_get_device_address(device);
        if (busId == ref_bus && address == ref_address)
            break;
    }
    if (devIdx >= count)
        goto error0;
    libusb_device_descriptor descriptor;
    result = libusb_get_device_descriptor(devices[devIdx], &descriptor);
    if (result < 0)
        goto error0;
    libusb_device_handle* dev_handle;
    result = libusb_open(devices[devIdx], &dev_handle);
    if (result < 0)
        goto error0;
    {
        char buffer[1024];
        int len = libusb_get_string_descriptor_ascii(dev_handle, descriptor.iSerialNumber, (unsigned char*)buffer, 1024);
        libusb_close(dev_handle);
        ret = std::string(buffer, len);
    }
error0:
    libusb_free_device_list(devices, 1);
error1:
    libusb_exit(context);
    return ret;
}

static
std::pair<int, int> get_bus_address(const xn::NodeInfo& deviceNode)
{
    std::istringstream is(deviceNode.GetCreationInfo());
    is.ignore(1024, '@');
    std::string bus_string;
    std::getline(is, bus_string, '/');
    int device = 0;
    is >> device;
    return std::make_pair(std::atoi(bus_string.c_str()), device);
}

struct Kinect::Impl
{
    xn::Context ctx;
    xn::ImageGenerator image;
    xn::IRGenerator ir;
    xn::DepthGenerator depth;

    std::string serial_number;
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
    xn::NodeInfoList devicesList;
    check(p->ctx.EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, nullptr, devicesList, 0));
    xn::NodeInfoList::Iterator it = devicesList.Begin();
    for (int i = 0; i < id && it != devicesList.End(); ++i)
        it++;
    if (it == devicesList.End()) {
        std::ostringstream err;
        err << "OpenNI: Device with index " << id << " is not present";
        throw std::runtime_error(err.str());
    }
    xn::NodeInfo deviceNode = *it;
    const auto bus_address = get_bus_address(deviceNode);
    p->serial_number = ::get_serial_number(bus_address.first, bus_address.second);
    check(p->ctx.RunXmlScript(xml_config.c_str()));
    check(p->ctx.CreateProductionTree(deviceNode));
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

std::string Kinect::getSerial() const
{
    return p->serial_number;
}