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

// added by user
static bool isSameSide(const Eigen::Vector3f& pt1, const Eigen::Vector3f& pt2, 
                       const Eigen::Vector3f& a, const Eigen::Vector3f& b)
{
    // reference: https://blackpawn.com/texts/pointinpoly/default.html
    Eigen::Vector3f cp1 = (b-a).cross(pt1-a);
    Eigen::Vector3f cp2 = (b-a).cross(pt2-a);
    if (cp1.dot(cp2)>0)
    {
        return true;
    }
    else
    {
        return false;
    }    
}

// function interface has been modified for msaa bonus
// static bool insideTriangle(int x, int y, const Vector3f* _v)
static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // reference: https://blackpawn.com/texts/pointinpoly/default.html
    Eigen::Vector3f pt;
    pt << (float)x, (float)y, 0.0f;
    bool isInside = isSameSide(pt, _v[0], _v[1], _v[2]) && isSameSide(pt, _v[1], _v[0], _v[2]) && isSameSide(pt, _v[2], _v[0], _v[1]);
    return isInside;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
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
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    
    // check triangle coordinates
    // for(auto idx : {0, 1, 2})
    // {
    //     std::clog << "pt" << idx << " | " << v[idx].x() 
    //                              << " | " << v[idx].y() 
    //                              << " | " << v[idx].z() 
    //                              << " | " << v[idx].w() << std::endl;
    // }
    
    // get bounding box
    auto ext_pt_x = std::minmax_element(v.begin(), v.end(),
                                       [](const Eigen::Vector4f& lhs, const Eigen::Vector4f & rhs){
                                            return lhs.x() < rhs.x();
                                       });
    auto ext_pt_y = std::minmax_element(v.begin(), v.end(),
                                       [](const Eigen::Vector4f& lhs, const Eigen::Vector4f & rhs){
                                            return lhs.y() < rhs.y();
                                       });
    // std::clog << "ext_pt_x: " << ext_pt_x.first->x() << " | " << ext_pt_x.second->x() << std::endl;
    // std::clog << "ext_pt_y: " << ext_pt_y.first->y() << " | " << ext_pt_y.second->y() << std::endl;

    int min_x = floor(ext_pt_x.first->x());
    int max_x = ceil(ext_pt_x.second->x());
    int min_y = floor(ext_pt_y.first->y());
    int max_y = ceil(ext_pt_y.second->y());

    Eigen::Vector3f v3f[3];
    for (int idx : {0, 1, 2})
    {
        v3f[idx] << v[idx].x(), v[idx].y(), 1.0f;
    }
    // std::transform(std::begin(v), std::end(v), v3f.begin(), [](auto& vec) { return Eigen::Vector3f(vec.x(), vec.y(), vec.z()); });
    
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // loop [min_x, max_x] x [min_y, max_y]
    for (int x=min_x;x<=max_x;++x)
    {
        for (int y=min_y;y<=max_y;++y)
        {
            // no msaa
            // float x_pos = (float)x + 0.5;
            // float y_pos = (float)y + 0.5;
            // auto[alpha, beta, gamma] = computeBarycentric2D(x_pos, y_pos, t.v);
            // float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            // z_interpolated *= w_reciprocal;
            // int buff_ind = x + y*width;
            // if (insideTriangle(x_pos, y_pos, v3f) && (-z_interpolated<depth_buf[buff_ind]))
            // {
            //     set_pixel(Eigen::Vector3f(x, y, 1.0f), t.getColor());
            //     depth_buf[buff_ind] = -z_interpolated;
            // }

            // // apply msaa
            int sup_px_cnt = 0;
            for (int x_sup: {0, 1})
            {
                for (int y_sup: {0, 1})
                {
                    float x_pos = (float)x + 0.25 + x_sup*0.5;
                    float y_pos = (float)y + 0.25 + y_sup*0.5;

                    auto[alpha, beta, gamma] = computeBarycentric2D(x_pos, y_pos, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    int buff_ind = (x*2 + x_sup) + (y*2 + y_sup)*width*2;
                    if (insideTriangle(x_pos, y_pos, v3f))
                    {
                        if ((-z_interpolated<depth_buf_msaa2x2[buff_ind]))
                        {
                            depth_buf_msaa2x2[buff_ind] = -z_interpolated;
                            ++sup_px_cnt;
                        }
                    }
                }
            }
            if ((sup_px_cnt>0))
            {
                float intensity = (float)sup_px_cnt / 4.0f;
                mix_pixel(Eigen::Vector3f(x, y, 1.0f), t.getColor()*intensity);
            }
        }

    }


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
        std::fill(depth_buf_msaa2x2.begin(), depth_buf_msaa2x2.end(), std::numeric_limits<float>::infinity());  // added by user for msaa bonus
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    depth_buf_msaa2x2.resize(w * h * 2 * 2);  // added by user for msaa bonus
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// added by user for msaa bonus
void rst::rasterizer::mix_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] += color;
}


// clang-format on