#version 330 core
uniform float camera_focal_x;
uniform float camera_focal_y;
uniform float camera_centre_x;
uniform float camera_centre_y;
uniform mat4 mvp_matrix;
uniform sampler2DRect camera_texture;
layout(location = 0) in vec3 vertex;
out vec2 tex_coord;

void main()
{
    vec4 v = vec4(vertex, 1.0f);
    ivec2 size = textureSize(camera_texture);
    gl_Position = mvp_matrix * v;
    tex_coord.s = size[0] - (camera_focal_x * v.x / v.z + camera_centre_x);
    tex_coord.t = camera_focal_y * v.y / v.z + camera_centre_y;
}
