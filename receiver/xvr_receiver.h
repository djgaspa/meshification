#pragma once

extern "C" {
void xvr_receiver_init();
void xvr_receiver_draw();
void xvr_receiver_load_static(const char* fname);
void xvr_receiver_destroy();
void xvr_receiver_translate(const char* name, const double x, const double y, const double z);
void xvr_receiver_rotate(const char* name, const double rad, const double x, const double y, const double z);
void xvr_receiver_reset_position(const char* name);
void xvr_receiver_toggle_point_smooth();
void xvr_receiver_save_view();
}
