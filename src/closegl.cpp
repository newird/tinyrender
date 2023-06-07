#include <Eigen/Dense>
#include <iostream>

#include "closegl.h"

Matrix Modelview;
Matrix Viewport;
Matrix Projection;

void lookat(Vec3f camera, Vec3f center, Vec3f up)
{
    
    Vec3f z = (camera - center).normalized();
    Vec3f x = up.cross(z).normalized();
    Vec3f y = z.cross(x).normalized();

    Matrix translate = Matrix::Identity();
    Matrix view = Matrix::Identity();
    for(int i = 0; i < 3; ++i)
    {
        view(0, i) = x(i);
        view(1, i) = y(i);
        view(2, i) = z(i);

        translate(i, 3) = -camera(i);
    }
    Modelview = view * translate;
}

void viewport(int width, int height)
{
    // Viewport << width / 2.0f, 0, 0, width / 2.0f,
    //             0, height / 2.0f, 0, height / 2.0f,
    //             0, 0, 1.f/10.f, 1.f,
    //             0, 0, 0, 1;


    Viewport << width / 2.0f, 0, 0, width / 2.0f,
                0, height / 2.0f, 0, height / 2.0f,
                0, 0, 255 / 2.0f, 255 / 2.0f,
                0, 0, 0, 1;
}


void viewport(int x, int y, int w, int h)
{
    Viewport << w/2.f,0,0,x+w/2.f,
                0,h/2.f,0,y+h/2.f,
                0,0,255.f/2.f,255.f/2.f,
                0,0,0,1;
}


void perspective(float distance)
{
    Matrix perspective = Matrix::Identity();
    perspective(3,2) = -1/distance;
    Projection = perspective;
}



Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P) {
    Vec3f s[2];
    for (int i=2; i--; ) {
        s[i][0] = C[i]-A[i];
        s[i][1] = B[i]-A[i];
        s[i][2] = A[i]-P[i];
    }
    Vec3f u = s[0].cross(s[1]);
    if (std::abs(u[2])>1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(1.f - (u[0] + u[1])/u[2], u[1]/u[2], u[0]/u[2]);
        //return Vec3f(1.f-(u[0]+u[1])/u[2], u[1]/u[2], u[0]/u[2]);
    return Vec3f(-1,1,1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}


inline Vec3f perspectiveDivision(Vec4f v)
{
    return Vec3f(v[0] / v[3],
                 v[1] / v[3],
                 v[2] / v[3]);
}

void triangle(Vec4f *pts, Shader &shader, TGAImage &image, TGAImage &zbuffer)
{
    float min_x = pts[0][0] / pts[0][3], max_x = pts[0][0] / pts[0][3];
    float min_y = pts[0][1] / pts[0][3], max_y = pts[0][1] / pts[0][3];
    for(int i = 0; i < 3; i++){
        min_x = min_x < pts[i][0] / pts[i][3] ? min_x : pts[i][0] / pts[i][3];
        max_x = max_x > pts[i][0] / pts[i][3] ? max_x : pts[i][0] / pts[i][3];
        min_y = min_y < pts[i][1] / pts[i][3] ? min_y : pts[i][1] / pts[i][3];
        max_y = max_y > pts[i][1] / pts[i][3] ? max_y : pts[i][1] / pts[i][3];
    }
    Vec3f p;
    TGAColor color(0);
    for(int i = min_x; i < max_x; i++)
    {
        for(int j = min_y; j < max_y; j++)
        {
            p[0] = i + 0.5f;
            p[1] = j + 0.5f;
            Vec3f bary = barycentric(perspectiveDivision(pts[0]), perspectiveDivision(pts[1]), perspectiveDivision(pts[2]), p);
            if(bary[0] >= 0 && bary[1] >= 0 && bary[2] >= 0)
            {
                p[2] = bary[0] * pts[0][2] + bary[1] * pts[1][2] + bary[2] * pts[2][2];
                float w = bary[0] * pts[0][3] + bary[1] * pts[1][3] + bary[2] * pts[2][3];
                float frag_depth = p[2] / w;
                // std::cout << frag_depth << std::endl;
                if(zbuffer.get(i, j).bgra[0] < frag_depth)
                {
                    bool discard = shader.fragment(bary, color);
                    if(!discard)
                    {
                        zbuffer.set(i, j, TGAColor(frag_depth, frag_depth, frag_depth));
                        image.set(i, j, color);
                    }
                }
            }
        }
    }
}


void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
    bool flip = false;
    if(std::abs(y1 - y0) > std::abs(x1 - x0)){
        flip = true;
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if(x1 < x0){
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int dx = x1 - x0;
    int dy = std::abs(y1 - y0);
    int acc = 0;
    int cur_y = y0;
    for(int cur_x = x0; cur_x <= x1; cur_x++){
        if(flip) image.set(cur_y, cur_x, color);
        else image.set(cur_x, cur_y, color);
        acc += dy;
        if(acc * 2 > dx){
            acc -= dx;
            cur_y += (y1>y0?1:-1);
        }
    }
}

Vec3f matrix4fProductVec3f(Matrix transform, Vec3f v)
{
    Vec4f v4;
    v4 << v[0], v[1], v[2], 1.0f;
    v4 = transform * v4;
    return perspectiveDivision(v4);
}