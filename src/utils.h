#pragma once
// #include <cmath>
#include <math.h> 
#include <iostream>
#include <list>
#include <map>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

namespace pr {
  
    // typedef Eigen::Vector3f Vec3f;
    typedef Eigen::Matrix<double, 3, 1> Vec3d;
    typedef Eigen::Matrix<double, 4, 1> Vec4d;
    typedef Eigen::Matrix<double, 3, 3> Matrix3d;
    typedef Eigen::Matrix<double, 4, 4> Matrix4d;

    struct Keypoint {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        int id;
        Vec3d direction_vector;

        Keypoint(int id, double direction_vector[3]);
        Keypoint(int id, Vec3d direction_vector);
    };

    struct Camera {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        int id;
        Vec3d gt_position;
        Vec3d gt_orientation;
        Vec3d position;
        Vec3d orientation;
        vector<Keypoint> keypoints;
        
        Camera();

        Camera(const Camera& cam);

        Camera(int id, double gt_position[3], double gt_orientation[3], double position[3], double orientation[3]);
    };

    class Sim3 {
        // private:
        //     /* data */
        public:
            double scale;
            Vec3d rotation; 
            Vec3d translation; 

            Sim3();

            Sim3(double scale, Vec3d rotation, Vec3d translation);

            Vec3d operator*(Vec3d v);

            Matrix4d as_matrix();

            Sim3 inverse();

            void box_plus(Vec3d translation, Vec3d rotation, double scale);
    };
    
    void quaternion_to_RPY(double (&orientation)[3]);
    Vec3d RPY_to_quaternion(Vec3d rpy);

    Matrix3d Rx(double rot_x);
    Matrix3d Rx_prime(double rot_x);
    Matrix3d Ry(double rot_x);
    Matrix3d Ry_prime(double rot_x);
    Matrix3d Rz(double rot_x);
    Matrix3d Rz_prime(double rot_x);
    Matrix3d v2tRPY(const Vec3d& v);
    Vec3d tRPY2v(const Matrix3d& rot);
    Matrix4d v2t(Eigen::Matrix<double, 6, 1> v);

    vector<Camera> load_data(string dataset_path);

    inline Matrix3d skew(const Vec3d& v){
        Matrix3d S;
        S << 0, -v[2], v[1],
            v[2], 0, -v[0],
            -v[1], v[0], 0;
    return S;
    }

    inline Vec3d skew2v(const Matrix3d& S){
        Vec3d v;
        v << S(2,1), S(0,2), S(1,0);
        return v;
    }

    double rmse(const map<int, Vec3d>& landmarks, const map<int, Vec3d>& gt_landmarks);

    void save_landmarks(const map<int, pr::Vec3d>& landmarks, string output_path);
    void save_camera_positions(const vector<Camera>& cameras, string output_path);
    map<int, pr::Vec3d> load_landmarks(string path);


}
