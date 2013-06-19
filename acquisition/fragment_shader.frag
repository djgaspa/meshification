#version 330 core
uniform sampler2DRect camera_texture;
in vec2 tex_coord;
layout(location = 0) out vec4 frag_color;

void main()
{
    frag_color = texture(camera_texture, tex_coord);
}
