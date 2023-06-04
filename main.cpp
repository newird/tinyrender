#include <vector>
#include <cmath>
#include <iostream>
#include <limits>
#include <Eigen/Dense>
#include <algorithm>

#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include "closegl.h"


Model *model = NULL;
const int width  = 800;
const int height = 800;

Vec3f center(0, 0, 0);
Vec3f cameraPos(0.4,0.4,1);
Vec3f light_dir(1,1,2);
Vec3f up(0,1,0);

struct GourandShader : public Shader
{
    Vec3f varying_intensity;

    GourandShader()
    {
        varying_intensity.setZero();
    }

    virtual Vec4f vertex(int iface, int nthvert)
    {
        Vec3f normal = model->normal(iface, nthvert);
        Vec3f light = light_dir.normalized();
        varying_intensity[nthvert] = std::max(0.0f, normal.dot(light));
        Vec3f model_pos = model->vert(iface, nthvert);
        Vec4f local_pos;
        local_pos << model_pos[0], model_pos[1], model_pos[2], 1.0f;
        return Viewport * Projection * Modelview * local_pos;
    }

    virtual bool fragment(Vec3f bary, TGAColor &color)
    {
        float average = bary.dot(varying_intensity);
        color = TGAColor(255,255,255) * average;
        return false;
    }
};


struct texShader : public Shader
{
    Eigen::Matrix<float, 2, 3> varying_uv;
    Matrix uniform_M;
    Matrix uniform_MIT;
    Eigen::Matrix<float, 3, 3> model_pos;

    virtual Vec4f vertex(int iface, int nthvert)
    {
        varying_uv.col(nthvert) = model->uv(iface, nthvert);
        model_pos.col(nthvert) = model->vert(iface, nthvert);
        Vec4f local_pos;
        local_pos << model_pos(0, nthvert), model_pos(1, nthvert), model_pos(2, nthvert), 1.0f;
        return Viewport * Projection * Modelview * local_pos;
    }

    virtual bool fragment(Vec3f bary, TGAColor &color)
    {
        Vec2f uv = varying_uv * bary;
        Vec3f normal = model->normal(uv).normalized();
        Vec3f light = light_dir.normalized();
        Vec3f frag_pos = model_pos * bary;
        Vec3f eye_dir = (cameraPos - frag_pos).normalized();
        Vec3f half = (eye_dir + light).normalized();

        float spec = std::pow(std::max(0.f, half.dot(normal)), model->specular(uv));
        float diff = std::max(0.f, normal.dot(light));
        
        color = model->diffuse(uv);
        for(int i = 0; i < 3; i++) color[i] = std::min<float>(5 + color[i]*(diff + 0.6f * spec), 255);
        return false;
    }
};


int main(int argc, char** argv) 
{
    if (2==argc) 
    {
        model = new Model(argv[1]);
    } else 
    {
        model = new Model("G:\\project\\c++\\tinyrender\\resource\\obj\\african_head\\african_head.obj");
    }

    TGAImage image(width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::RGB);

    lookat(cameraPos, center, up);
    viewport(width, height);
    // viewport(width/8, height/8, width*3/4, height*3/4);
    perspective((cameraPos - center).norm());

    texShader shader;
    shader.uniform_M = Projection * Modelview;
    shader.uniform_MIT = (Projection * Modelview).inverse().transpose();

    for (int i=0; i<model->nfaces(); i++) 
    {
        Vec4f screen_coords[3];
        for(int j = 0; j < 3; j++)
        {
            screen_coords[j] = shader.vertex(i, j);
        }
        triangle(screen_coords, shader, image, zbuffer);
    }

    image.  flip_vertically(); // to place the origin in the bottom left corner of the image
    zbuffer.flip_vertically();
    image.  write_tga_file("output.tga");
    zbuffer.write_tga_file("zbuffer.tga");

    delete model;
    return 0;
}
