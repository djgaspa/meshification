#include <memory>
#include <forward_list>
#include "xvr_receiver.h"
#include "Receiver.hpp"
#include "StaticModel.hpp"

static std::unique_ptr<Receiver> p;
static std::list<StaticModel> static_models;

extern "C" {

void xvr_receiver_init()
{
    Receiver::init();
    p.reset(new Receiver);
}

void xvr_receiver_start()
{
    p->start();
}

void xvr_receiver_start_record(const char* fname)
{
    p->start(fname);
}

void xvr_receiver_start_play(const char* fname)
{
    p->start_play(fname);
}

void xvr_receiver_stop()
{
    p->stop();
}

void xvr_receiver_draw_dynamic()
{
    p->draw();
}

void xvr_receiver_draw_static()
{
    for (const auto& m : static_models)
        m.draw();
}

void xvr_receiver_draw()
{
    xvr_receiver_draw_dynamic();
    xvr_receiver_draw_static();
}

void xvr_receiver_destroy()
{
    p.reset();
    static_models.clear();
}

void xvr_receiver_load_static(const char* fname)
{
    static_models.emplace_back();
    static_models.back().load(fname);
}

void xvr_receiver_translate(const char *name, const double x, const double y, const double z)
{
    p->translate(name, x, y, z);
}

void xvr_receiver_rotate(const char* name, const double rad, const double x, const double y, const double z)
{
    p->rotate(name, rad, x, y, z);
}

void xvr_receiver_reset_position(const char *name)
{
    p->reset_position(name);
}

void xvr_receiver_toggle_point_smooth()
{
    for (auto& m : static_models)
        m.tooglePointSmooth();
}

void xvr_receiver_save_view()
{
    p->save_view();
}

}
