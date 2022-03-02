// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// 叉乘公式：x1*y2 - x2*y1
// 叉乘的结果是向量，这里返回的结果是 第三维的数值
float cross_product(float P1[2], float P2[2], float Q[2]) {
    return (P2[0]-P1[0])*(Q[1]-P1[1]) - (Q[0]-P1[0])*(P2[1]-P1[1]);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    float P[2] = {x,y};
    float X[2] = { _v[0][0], _v[0][1]};
    float Y[2] = { _v[1][0], _v[1][1]};
    float Z[2] = { _v[2][0], _v[2][1]};
    
    if (cross_product(X,Y, P)*cross_product(Y,Z,P) > 0 &&
        cross_product(Y,Z,P)*cross_product(Z,X,P) > 0 &&
        cross_product(Z,X,P)*cross_product(X,Y, P) > 0)
        return true;
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type, bool use_msaa)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation 因为前面的mvp会把物体缩放到 [-1,1]^3的立方体空间中；
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
        }        

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);
        rasterize_triangle(t, use_msaa);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, bool use_msaa) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    float x_range[2], y_range[2];
    // 获取要遍历的x、y的范围
    float x_array[3] = {v[0][0], v[1][0], v[2][0]};
    float y_array[3] = {v[0][1], v[1][1], v[2][1]};
    x_range[0] = *std::min_element(x_array, x_array+3);
    x_range[1] = *std::max_element(x_array, x_array+3);
    y_range[0] = *std::min_element(y_array, y_array+3);
    y_range[1] = *std::max_element(y_array, y_array+3);
    // std::cout << v[0] << "\n" << v[1] << "\n"<< v[2] << "\n";

    for (float x=floor(x_range[0]);x<=x_range[1];x++) {
        for (float y = floor(y_range[0]);y<=y_range[1];y++) {
            if (!use_msaa) {
                if (insideTriangle(x, y, t.v)) {
                    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    if (y+1 >= height || x+1 >=width)
                        std::cout << "error\n";
                    if (depth_buf[get_index(x, y)] > z_interpolated) {
                        depth_buf[get_index(x, y)] = z_interpolated;
                        set_pixel(Eigen::Vector3f{x, y, 0}, t.getColor());
                    }
                }
            } else {
                int msaa_cnt = 0;
                for (float interval_x : {0.25, 0.75}) {
                    for (float interval_y : {0.25, 0.75}) {
                        float multi_sample_x = x + interval_x, multi_sample_y = y + interval_y;
                        if (insideTriangle(multi_sample_x, multi_sample_y, t.v)) {
                            auto[alpha, beta, gamma] = computeBarycentric2D(multi_sample_x, multi_sample_y, t.v);
                            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;
                            int sample_point_index = get_index(x, y)*4 + msaa_cnt;
                            if (msaa_depth_buf[sample_point_index] > z_interpolated) {
                                msaa_depth_buf[sample_point_index] = z_interpolated;
                                msaa_color_buf[sample_point_index] = t.getColor();
                            }
                        }
                        msaa_cnt++;
                    }
                }
                // Vector3f的默认初始化是随机值，要小心！！！
                Eigen::Vector3f color={0,0,0};
                for ( int cnt=0; cnt<=3; cnt++) {
                    int sample_point_index = get_index(x, y)*4 + cnt;
                    color += msaa_color_buf[sample_point_index];
                }
                
                color = color/4;
                set_pixel(Eigen::Vector3f{x, y, 0}, color);
            }
        }
    }
    
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(msaa_depth_buf.begin(), msaa_depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(msaa_color_buf.begin(), msaa_color_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    msaa_depth_buf.resize(4 * w * h); // 乘以超采样的倍数
    msaa_color_buf.resize(4 * w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    // std::cout << height << " " << y<<"\n";
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    if (ind > frame_buf.size())
        std::cout << "22222\n";
    frame_buf[ind] = color;

}

// clang-format on