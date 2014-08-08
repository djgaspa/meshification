#pragma once

extern "C" {
void xvr_point_cloud_init();
void xvr_point_cloud_draw();
void xvr_point_cloud_destroy();
void xvr_point_cloud_load(const char* filename);
void xvr_point_cloud_toggle_point_smooth();
}
