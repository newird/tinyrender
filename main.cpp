#include <vector>
#include <cmath>
#include <iostream>
#include <limits>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
Model *model = NULL;
const int width  = 800;
const int height = 800;
const int depth  = 255;
Vec3f center(0, 0, 0);
Vec3f cameraPos(1,1,1);
Vec3f light_dir(0,0,1);

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color);
Vec3f barycentric2D(Vec2i *pts, Vec2i p);
Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P);
void triangle(Vec3f *pts, float *zbuffer, TGAImage &image, TGAColor color);
TGAColor lightAttenuation(TGAColor color, float intensity);

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

Vec3f barycentric2D(Vec3f *pts, Vec3f p){
    float a = (float)(-(p.x - pts[1].x) * (pts[2].y - pts[1].y) + (p.y - pts[1].y) * (pts[2].x - pts[1].x)) / 
        (float)(-(pts[0].x - pts[1].x) * (pts[2].y - pts[1].y) + (pts[0].y - pts[1].y) * (pts[2].x - pts[1].x));
    float b = (float)(-(p.x - pts[2].x) * (pts[0].y - pts[2].y) + (p.y - pts[2].y) * (pts[0].x - pts[2].x))/
        (float)(-(pts[1].x - pts[2].x) * (pts[0].y - pts[2].y) + (pts[1].y - pts[2].y) * (pts[0].x - pts[2].x));
    return Vec3f(a, b, 1.0f - a - b);
}

Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P) {
    Vec3f s[2];
    for (int i=2; i--; ) {
        s[i][0] = C[i]-A[i];
        s[i][1] = B[i]-A[i];
        s[i][2] = A[i]-P[i];
    }
    Vec3f u = s[0] ^ s[1];
    if (std::abs(u[2])>1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z);
    return Vec3f(-1,1,1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}


Matrix lookat(Vec3f camera, Vec3f center, Vec3f up)
{
    Vec3f z = (camera - center).normalize();
    Vec3f x = (up ^ z).normalize();
    Vec3f y = (z ^ x).normalize();

    Matrix translate = Matrix::identity(4);
    Matrix view = Matrix::identity(4);
    for(int i = 0; i < 3; ++i)
    {
        view[0][i] = x[i];
        view[1][i] = y[i];
        view[2][i] = z[i];

        translate[i][3] = -camera[i];
    }
    return view * translate;
}

Matrix viewport(float width, float height)
{
    Matrix vport = Matrix::identity(4);
    vport[0][0] = width / 2.f;
    vport[1][1] = height / 2.f;
    vport[2][2] = depth / 2.f;

    vport[0][3] = width / 2.f;
    vport[1][3] = height / 2.f;
    vport[2][3] = depth / 2.f;

    return vport;
}

Matrix perspective(float c)
{
    Matrix perspective = Matrix::identity(4);
    perspective[3][2] = -1/c;
    return perspective;
}


Matrix v2m(Vec3f v)
{
    Matrix m(4, 1);
    m[0][0] = v.x;
    m[1][0] = v.y;
    m[2][0] = v.z;
    m[3][0] = 1.0f;
    return m;
}

Vec3f m2v(Matrix m)
{
    return Vec3f(m[0][0]/m[3][0], m[1][0]/m[3][0], m[2][0]/m[3][0]);
}


void triangle(Vec3f *pts, float *zbuffer, TGAImage &image, Vec2i *uv, float intensity){
    float min_x = pts[0].x, max_x = pts[0].x;
    float min_y = pts[0].y, max_y = pts[0].y;
    for(int i = 0; i < 3; i++){
        min_x = min_x < pts[i].x ? min_x : pts[i].x;
        max_x = max_x > pts[i].x ? max_x : pts[i].x;
        min_y = min_y < pts[i].y ? min_y : pts[i].y;
        max_y = max_y > pts[i].y ? max_y : pts[i].y;
    }
    Vec3f p;
    for(int i = min_x; i < max_x; i++){
        for(int j = min_y; j < max_y; j++){
            p.x = i + 0.5f;
            p.y = j + 0.5f;
            Vec3f bary = barycentric(pts[0], pts[1], pts[2], p);
            if(bary.x >= 0 && bary.y >= 0 && bary.z >= 0){
                p.z = bary.x * pts[0].z + bary.y * pts[1].z + bary.z * pts[2].z;
                if(zbuffer[i * width + j] < p.z){
                    float u = 0, v = 0;
                    for(int i = 0; i < 3; i++){
                        u += bary[i] * uv[i][0];
                        v += bary[i] * uv[i][1];
                    }
                    zbuffer[i * width + j] = p.z;
                    image.set(i, j, lightAttenuation(model->diffuse(Vec2i(int(u), int(v))), intensity));
                }
            }
        }
    }
}

TGAColor lightAttenuation(TGAColor color, float intensity){
    return TGAColor(color.r * intensity, color.g * intensity, color.b * intensity, color.a);
}

int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("G:\\project\\c++\\tinyrender\\resource\\african_head.obj");
    }

    TGAImage image(width, height, TGAImage::RGB);

    float *zbuffer = new float[width * height];
    for(int i = 0; i < width * height; i++){
        zbuffer[i] = -std::numeric_limits<float>::max();
    }

    Matrix modelView = lookat(cameraPos, center, Vec3f(0,1,0));
    Matrix persp = perspective((cameraPos - center).norm());
    Matrix vport = viewport(width, height);

    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec3f pts[3];
        Vec3f world_coords[3];
        for(int i = 0; i < 3; i++){
            Vec3f v = model->vert(face[i]);
            world_coords[i] = v;
            // pts[i] = Vec3f((v.x + 1.0f) * width / 2. , (v.y + 1.0f) * height / 2., v.z);
            pts[i] = m2v(vport * persp * modelView * v2m(v));
        }
        Vec3f normal = ((world_coords[1] - world_coords[0]) ^ (world_coords[2] - world_coords[1])).normalize();
        float intensity = normal * light_dir;
        Vec2i uvs[3];
        for(int ii = 0; ii < 3; ii++) uvs[ii] = model->uv(i, ii);
        if(intensity > 0)
            triangle(pts, zbuffer, image, uvs, intensity);
    }
    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    delete[] zbuffer;
    delete model;
    return 0;
}
