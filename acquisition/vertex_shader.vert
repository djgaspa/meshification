#version 330 core
uniform int camera_width;
uniform int camera_height;
uniform float camera_focal_x;
uniform float camera_focal_y;
uniform float camera_centre_x;
uniform float camera_centre_y;
uniform mat4 mvp_matrix;
layout(location = 0) in vec3 vertex;
out vec2 tex_coord;

void main()
{
    vec4 v = vec4(vertex, 1.0f);
    gl_Position = mvp_matrix * v;
    tex_coord.s = 1.0f - (camera_focal_x * v.x / v.z + camera_centre_x) / camera_width;
    tex_coord.t = (camera_focal_y * v.y / v.z + camera_centre_y) / camera_height;
}
