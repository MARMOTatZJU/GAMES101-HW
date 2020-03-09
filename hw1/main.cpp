#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
// additional including
#include <cmath>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;
    std::clog << "view" << std::endl << view << std::endl;  // added by user

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    // Done
    float rotation_rad = acos(-1) * (rotation_angle) / 180.0f;
    // std::clog << "rotation_rad=" << rotation_rad << std::endl;
    Eigen::Matrix3f rotation_model = Eigen::Matrix3f::Identity();
    Eigen::Vector3f z_axis = Eigen::Vector3f(0, 0, 1);
    Eigen::Matrix3f prod_z_axis;
    prod_z_axis << 0, -1, 0, 
                   1, 0, 0, 
                   0, 0, 0;
    rotation_model = cos(rotation_rad) * rotation_model + 
                     (1-cos(rotation_rad)) * z_axis * z_axis.transpose() + 
                     sin(rotation_rad) * prod_z_axis;
    
    model.block<3, 3>(0, 0) = rotation_model;

    std::clog << "model" << std::endl << model << std::endl; 

    return model;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    // Done
    float rotation_rad = acos(-1) * (angle) / 180.0f;
    // std::clog << "rotation_rad=" << rotation_rad << std::endl;
    Eigen::Matrix3f rotation_model = Eigen::Matrix3f::Identity();
    // Eigen::Vector3f n_axis = Eigen::Vector3f(0, 0, 1);
    Eigen::Matrix3f cross_prod_axis;
    cross_prod_axis << 0, -axis.z(), axis.y(),
                       axis.z(), 0, -axis.x(),
                       -axis.y(), axis.x(), 0;
    rotation_model = cos(rotation_rad) * rotation_model + 
                     (1-cos(rotation_rad)) * axis * axis.transpose() + 
                     sin(rotation_rad) * cross_prod_axis;
    model.block<3, 3>(0, 0) = rotation_model;

    std::clog << "model" << std::endl << model << std::endl; 

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float eye_fov_rad = eye_fov / 180.0f * acos(-1);
    float t = zNear * tan(eye_fov_rad/2.0f);
    float r = t * aspect_ratio;
    float b = -t;
    float l = -r;
    float n = -zNear;
    float f = -zFar;
    // frustum -> cubic
    Eigen::Matrix4f M_p2o;
    M_p2o << n, 0, 0, 0,
             0, n, 0, 0,
             0, 0, n+f, -n*f,
             0, 0, 1, 0;
    // orthographic projection
    Eigen::Matrix4f M_o_shift = Eigen::Matrix4f::Identity();
    M_o_shift(0, 3) = -(r+l)/2.0f;
    M_o_shift(1, 3) = -(t+b)/2.0f;
    M_o_shift(2, 3) = -(n+f)/2.0f;
    Eigen::Matrix4f M_o_scale = Eigen::Matrix4f::Identity();
    M_o_scale(0, 0) = 2.0f / (r-l);
    M_o_scale(1, 1) = 2.0f / (t-b);
    M_o_scale(2, 2) = 2.0f / (n-f);
    // squash all transformations
    projection = M_o_scale * M_o_shift * M_p2o * projection;
    std::clog << "projection" << std::endl << projection << std::endl;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};
    // added by user
    // std::vector<Eigen::Vector3f> pos{{3, 0, -2}, {0, 1, -2}, {-2, 0, -2}};
    // std::vector<Eigen::Vector3f> pos{{2, 0, 0}, {0, 2, 0}, {-2, 0, 0}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    // added by user
    // Eigen::Vector3f axis(0, 0, 1);
    Eigen::Vector3f axis(1, 0, 0);

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        // r.set_model(get_rotation(axis, angle)); // added by user
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        // r.set_model(get_rotation(axis, angle)); // added by user
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
        std::clog << "angle: " << angle << std::endl;
        // std::clog << get_projection_matrix(45, 1, 0.1, 50) * get_view_matrix(eye_pos) * get_rotation(axis, angle) << std::endl;

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
