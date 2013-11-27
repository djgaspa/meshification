#version 330 core
uniform float camera_focal_x;
uniform float camera_focal_y;
uniform float camera_centre_x;
uniform float camera_centre_y;
uniform float K[5];
uniform vec3 T;
uniform mat3 R;
uniform mat4 mvp_matrix;
uniform sampler2DRect camera_texture;
layout(location = 0) in vec3 vertex;
out vec2 tex_coord;

void main()
{
    vec4 v = vec4(vertex, 1.0f);
    ivec2 size = textureSize(camera_texture);
    gl_Position = mvp_matrix * v;
    vec3 vt = R * vertex + T;

    float r2, r4, r6, a1, a2, a3, cdist;

    vt.x /= vt.z;
    vt.y /= vt.z;

    r2 = vt.x * vt.x + vt.y * vt.y;
    r4 = r2 * r2;
    r6 = r4 * r2;
    a1 = 2 * vt.x * vt.y;
    a2 = r2 + 2 * vt.x * vt.x;
    a3 = r2 + 2 * vt.y * vt.y;
    cdist = 1 + K[0] * r2 + K[1] * r4 + K[4] * r6;
    float xd = vt.x * cdist + K[2] * a1 + K[3] * a2;
    float yd = vt.y * cdist + K[2] * a3 + K[3] * a1;

    tex_coord.s = camera_focal_x * xd + camera_centre_x;
    tex_coord.t = camera_focal_y * yd + camera_centre_y;
}
