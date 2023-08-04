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
const int width  = 1200;
const int height = 1200;
TGAImage depthbuffer;

Vec3f center(0, 0, 0);
Vec3f cameraPos(0.2f,-0.2f,1.f);
Vec3f light_dir(1,1,1);
Vec3f up(0,1,0);

struct texShader : public Shader
{
    Eigen::Matrix<float, 2, 3> varying_uv;
    Eigen::Matrix<float, 3, 3> model_pos;
    Eigen::Matrix<float, 3 ,3> normals;
    // Matrix uniform_Mshadow;

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
        Vec3f normal = (normals * bary).normalized();
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


        // need orthogonalization?
        // Vec3f T = TB.row(0).normalized();
        // T = (T - T.dot(normal) * normal).normalized();
        // Vec3f B = normal.cross(T).normalized();
        
        //TBN
        Eigen::Matrix3f TBN;
        TBN.col(0) = TB.row(0).normalized();
        TBN.col(1) = TB.row(1).normalized();
        TBN.col(2) = normal;
        //compute N
        Vec3f N = (TBN * model->normal(uv).normalized()).normalized();
        //shadow
        // Vec3f s_pos = matrix4fProductVec3f(uniform_Mshadow, frag_pos);
        // float shadow = 1.0f;
        // if(!(s_pos[0] < 0.f || s_pos[0] > width || s_pos[1] < 0.f || s_pos[1] > width))
        //     shadow = .3+.7*(depthbuffer.get(s_pos[0], s_pos[1]).bgra[2] <=  s_pos[2]);

        float spec = std::pow(std::max(0.f, half.dot(N)), 50);
        float diff = std::max(.2f, N.dot(light));
        
        color = model->diffuse(uv);

        // for(int i = 0; i < 3; i++) color[i] = std::min<float>(5 + color[i]*shadow*(diff + 0.6f * spec), 255);
        for(int i = 0; i < 3; i++) color[i] = std::min<float>(color[i]*(diff), 255);
        return false;
    }
};




// struct depthShader : public Shader
// {
//     virtual Vec4f vertex(int iface, int nthvert)
//     {
//         Vec4f local_pos = model->vert(iface, nthvert).homogeneous();
//         return Viewport * Projection * Modelview * local_pos;
//     }

//     virtual bool fragment(Vec3f bary, TGAColor& color){ return false;}
// };


// struct floorShader : public Shader
// {
//     Eigen::Matrix<float, 2, 3> varying_uv;
//     Eigen::Matrix<float, 3, 3> model_pos;

//     virtual Vec4f vertex(int iface, int nthvert)
//     {
//         varying_uv.col(nthvert) = model->uv(iface, nthvert);
//         Vec4f local_pos;
//         local_pos << model_pos(0, nthvert), model_pos(1, nthvert), model_pos(2, nthvert), 1.0f;
//         return Viewport * Projection * Modelview * local_pos;
//     }

//     virtual bool fragment(Vec3f bary, TGAColor& color)
//     {
//         Vec2f uv = varying_uv * bary;
//         color = model->diffuse(uv);
//         return false;
//     }
// };


int main(int argc, char** argv) 
{
    if (2==argc) 
    {
        model = new Model(argv[1]);
    } else 
    {
        // model = new Model("G:\\project\\c++\\tinyrender\\resource\\obj\\african_head\\african_head.obj");
        model = new Model("G:\\project\\c++\\tinyrender\\resource\\obj\\diablo3_pose\\diablo3_pose.obj");
        // model = new Model("G:\\project\\c++\\tinyrender\\resource\\obj\\floor.obj");
    }
    TGAImage depthImage(width, height, TGAImage::RGB);
    TGAImage image(width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::RGB);
    depthbuffer = TGAImage(width, height, TGAImage::RGB);

    // depthShader dshader;
    // lookat(light_dir, center, up);
    // viewport(width, height);
    // perspective((light_dir - center).norm());
    // for (int i=0; i<model->nfaces(); i++) 
    // {
    //     Vec4f screen_coords[3];
    //     for(int j = 0; j < 3; j++)
    //     {
    //         screen_coords[j] = dshader.vertex(i, j);
    //     }
    //     triangle(screen_coords, dshader, depthImage, depthbuffer);
    // }

    // Matrix Mshadow = Viewport * Projection * Modelview;

    texShader shader;
    // shader.uniform_Mshadow = Mshadow;
    lookat(cameraPos, center, up);
    viewport(width, height);
    perspective((cameraPos - center).norm());
    // orthogonal();
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
    depthbuffer.flip_vertically();
    depthbuffer.write_tga_file("depthbuffer.tga");
    image.  write_tga_file("output.tga");
    zbuffer.write_tga_file("zbuffer.tga");

    delete model;
    return 0;
}
