#pragma once

#include <Eigen/Core>
#include "geometry.h"
#include "tgaimage.h"


struct Shader {
    virtual ~Shader(){};
    virtual Vec4f vertex(int iface, int nthvert) = 0;
    virtual bool fragment(Vec3f bar, TGAColor &color) = 0;
};


extern Matrix Projection;
extern Matrix Modelview;
extern Matrix Viewport;


void lookat(Vec3f camera, Vec3f center, Vec3f up);
void perspective(float distance);
void viewport(int width, int height);
void viewport(int x, int y, int w, int h);

void triangle(Vec4f *pts, Shader &shader, TGAImage &image, TGAImage &zbuffer);
void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color);
Vec3f perspectiveDivision(Vec4f v);
Vec3f matrix4fProductVec3f(Matrix transform, Vec3f v);