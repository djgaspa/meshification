#pragma once
#include <string>

class Data3d;

class Model
{
    static const int n_vbo = 2;
    unsigned vao[1], vbo[n_vbo], tex[1];
    size_t n_elements = 0;
    float model_matrix[16], matrix[16];
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
