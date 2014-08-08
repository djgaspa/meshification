#include <memory>
#include "xvr_receiver.h"
#include "Receiver.hpp"

static std::unique_ptr<Receiver> p;

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

void xvr_receiver_start_connect(const char* address)
{
    p->start_connect(address);
}

void xvr_receiver_stop()
{
    p->stop();
}

void xvr_receiver_draw()
{
    p->draw();
}

void xvr_receiver_destroy()
{
    p.reset();
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

void xvr_receiver_save_view()
{
    p->save_view();
}

}
