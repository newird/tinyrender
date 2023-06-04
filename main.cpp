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
    Eigen::Matrix<float, 3, 3> model_pos;
    Eigen::Matrix<float, 3 ,3> normals;

    virtual Vec4f vertex(int iface, int nthvert)
    {
        varying_uv.col(nthvert) = model->uv(iface, nthvert);
        model_pos.col(nthvert) = model->vert(iface, nthvert);
        normals.col(nthvert) = model->normal(iface, nthvert);
        Vec4f local_pos;
        local_pos << model_pos(0, nthvert), model_pos(1, nthvert), model_pos(2, nthvert), 1.0f;
        return Viewport * Projection * Modelview * local_pos;
    }

    virtual bool fragment(Vec3f bary, TGAColor &color)
    {
        Vec2f uv = varying_uv * bary;
        Vec3f normal = normals * bary;
        Vec3f light = light_dir.normalized();
        Vec3f frag_pos = model_pos * bary;
        Vec3f eye_dir = (cameraPos - frag_pos).normalized();
        Vec3f half = (eye_dir + light).normalized();
        //compute TBN
        Eigen::Matrix2f delta_uv;
        Vec2f uv0 = varying_uv.col(0), uv1 = varying_uv.col(1), uv2 = varying_uv.col(2);
        delta_uv << uv1[0] - uv0[0], uv1[1] - uv0[1],
                    uv2[0] - uv0[0], uv2[1] - uv0[1];
        Eigen::Matrix<float, 2, 3> E;
        Vec3f p0 = model_pos.col(0), p1 = model_pos.col(1), p2 = model_pos.col(2);
        E << p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2],
             p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2];
        Eigen::Matrix<float, 2, 3> TB = delta_uv.inverse() * E;
        Vec3f T = TB.row(0);
        //orthogonalization
        T = (T - T.dot(normal) * normal).normalized();
        Vec3f B = normal.cross(T).normalized();
        //TBN
        Eigen::Matrix3f TBN;
        TBN.col(0) = T;
        TBN.col(1) = B;
        TBN.col(2) = normal;
        //compute N
        Vec3f N = (TBN * model->normal(uv).normalized()).normalized();

        float spec = std::pow(std::max(0.f, half.dot(N)), model->specular(uv));
        float diff = std::max(0.f, N.dot(light));
        
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
