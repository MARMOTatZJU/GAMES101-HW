#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    // matrix add i + j
    std::cout << i+j << std::endl;
    // matrix scalar multiply i * 2.0
    std::cout << i * 2.0f << std::endl;
    // matrix multiply i * j
    std::cout << i * j << std::endl;
    // matrix multiply vector i * v
    std::cout << v.transpose() * i << std::endl;

    std::cout << std::endl;
    std::cout << "********************* transformation test *********************" << std::endl;
    std::cout << std::endl;    

    Eigen::Vector3f point_P(2.0f, 1.0f, 1.0f);
    Eigen::Matrix3f T_rot, T_shift;
    
    float theta = 45.0f / 180.0f * std::acos(-1);
    T_rot <<
        std::cos(theta),    -std::sin(theta),   0,
        std::sin(theta),    std::cos(theta),    0,
        0,                  0,                  1;
    T_shift <<
        1,                  0,                  1,
        0,                  1,                  2,
        0,                  0,                  1;
    
    std::cout << theta << std::endl;
    std::cout << T_rot << std::endl;
    std::cout << T_shift << std::endl;

    Eigen::Vector3f point_P_transformed;
    point_P_transformed = T_shift * T_rot * point_P;
    std::cout << point_P_transformed << std::endl;

    std::cout << "********************* concatenation test *********************" << std::endl;
    Eigen::Matrix3f concat_a;
    concat_a << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    // Eigen::Vector3f concat_b = Eigen::Vector3f(2);
    // Eigen::Vector3f concat_c = Eigen::Vector3f(3).transpose();
    
    Eigen::Matrix4f concat_d = Eigen::Matrix4f::Zero();
    concat_d.block<3, 3>(0, 0) = concat_a;

    std::clog << concat_d << std::endl;

    // float tmp = -1.5;
    // std::clog << cos(500) << fmod(tmp, 3) << std::endl;
    std::clog << "seq: " << std::endl << point_P.segment(0, 2) << std::endl;


    // cast
    Eigen::Vector2f v2f;
    v2f = point_P;
    std::clog << "cast: " << std::endl << v2f << std::endl;


    return 0;
}