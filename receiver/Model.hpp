#pragma once
#include <string>

class Data3d;

class Model
{
    static const int n_vbo = 2, n_tex = 3;
    unsigned vao[1], vbo[n_vbo], tex[n_tex];
    size_t n_elements = 0;
    float model_matrix[16], matrix[16], t[3], r[9], k[5];
    float focal_x, focal_y, center_x, center_y;
    std::string name;

    Model(const Model&);
    Model& operator=(const Model&);

public:
    Model();
    ~Model();
    static void init();
    void draw() const;
    void load(const Data3d& data);
    void save_view() const;
    void translate(const double x, const double y, const double z);
    void rotate(const double rad, const double x, const double y, const double z);
    void reset_position();
    std::string get_name() const {
        return name;
    }
};
