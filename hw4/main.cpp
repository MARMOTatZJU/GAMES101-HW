#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
// added by user
#include <cmath>
#include <algorithm>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm

    // reach end, return
    int num_control_points = control_points.size();
    // std::clog << "t=" << t << std::endl;
    // std::clog << "num_control_points=" << num_control_points << std::endl;
    if (num_control_points == 1)
    {
        // return control_points[0];
        cv::Point2f pix_center_point(0.5, 0.5);
        pix_center_point += (cv::Point2f) control_points[0];
        return pix_center_point;  // coordinate of the pixel center
    }
    // recursively
    else
    {
        cv::Point2f left_bezier_point, right_bezier_point;
        std::vector<cv::Point2f> left_control_points(&(control_points[0]), &(control_points[num_control_points-1]));
        std::vector<cv::Point2f> right_control_points(&(control_points[1]), &(control_points[num_control_points]));
        left_bezier_point = recursive_bezier(left_control_points, t);
        right_bezier_point = recursive_bezier(right_control_points, t);        
        return left_bezier_point * (1-t) + right_bezier_point * t;
    }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    static float t_step = 0.005;
    static cv::Point2f point;
    for (float t = 0.0f;t<=1.0f; t+=t_step)
    {
        std::clog << "t=" << t << std::endl;
        point = recursive_bezier(control_points, t);

        // draw red at point w/o anti-aliasing
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255; 

        // draw red at point w/t anti-aliasing
        // int min_x = std::max(0, (int)floor(point.x));
        // int max_x = std::min(window.cols-1, (int)ceil(point.x));
        // int min_y = std::max(0, (int)floor(point.y));
        // int max_y = std::min(window.rows-1, (int)ceil(point.y));

        // static float pow_antialiasing = 0.5;

        // window.at<cv::Vec3b>(min_y, min_x)[1] = (uint) ( 255 * pow((1.0f - sqrt( (min_x-point.x)*(min_x-point.x) + (min_y-point.y)*(min_y-point.y))), pow_antialiasing) );
        // window.at<cv::Vec3b>(max_y, min_x)[1] = (uint) ( 255 * pow((1.0f - sqrt( (min_x-point.x)*(min_x-point.x) + (max_y-point.y)*(max_y-point.y))), pow_antialiasing) );
        // window.at<cv::Vec3b>(min_y, max_x)[1] = (uint) ( 255 * pow((1.0f - sqrt( (max_x-point.x)*(max_x-point.x) + (min_y-point.y)*(min_y-point.y))), pow_antialiasing) );
        // window.at<cv::Vec3b>(max_y, max_x)[1] = (uint) ( 255 * pow((1.0f - sqrt( (max_x-point.x)*(max_x-point.x) + (max_y-point.y)*(max_y-point.y))), pow_antialiasing) );
        
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
