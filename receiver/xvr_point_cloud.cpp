#include <list>
#include "xvr_point_cloud.h"
#include "StaticModel.hpp"

static std::list<StaticModel> static_models;

extern "C" {

void xvr_point_cloud_init()
{
}

void xvr_point_cloud_draw()
{
    for (const auto& m : static_models)
        m.draw();
}

void xvr_point_cloud_destroy()
{
    static_models.clear();
}

void xvr_point_cloud_load(const char* fname)
{
    static_models.emplace_back();
    static_models.back().load(fname);
}

void xvr_point_cloud_toggle_point_smooth()
{
    for (auto& m : static_models)
        m.tooglePointSmooth();
}

}
