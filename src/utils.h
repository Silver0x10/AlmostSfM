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
    typedef Eigen::Matrix<float, 3, 1> Vec3f;
    typedef Eigen::Matrix<float, 4, 1> Vec4f;
    typedef Eigen::Matrix<float, 3, 3> Matrix3f;
    typedef Eigen::Matrix<float, 4, 4> Matrix4f;

    struct Keypoint {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        int id;
        Vec3f direction_vector;

        Keypoint(int id, float direction_vector[3]);
    };

    struct Camera {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        int id;
        Vec3f gt_position;
        Vec3f gt_orientation;
        Vec3f position;
        Vec3f orientation;
        vector<Keypoint> keypoints;
        
        Camera();

        Camera(const Camera& cam);

        Camera(int id, float gt_position[3], float gt_orientation[3], float position[3], float orientation[3]);
    };

    class Sim3 {
        // private:
        //     /* data */
        public:
            float scale;
            Vec3f rotation; 
            Vec3f translation; 

            Sim3();

            Sim3(float scale, Vec3f rotation, Vec3f translation);

            Vec3f operator*(Vec3f v);

            Matrix4f as_matrix();

            Sim3 inverse();

            void box_plus(Vec3f translation, float scale, Vec3f rotation);
    };
    
    void quaternion_to_RPY(float (&orientation)[3]);

    Matrix3f Rx(float rot_x);
    Matrix3f Rx_prime(float rot_x);
    Matrix3f Ry(float rot_x);
    Matrix3f Ry_prime(float rot_x);
    Matrix3f Rz(float rot_x);
    Matrix3f Rz_prime(float rot_x);
    Matrix3f v2tRPY(const Vec3f& v);
    Vec3f tRPY2v(const Matrix3f& rot);

    vector<Camera> load_data(string dataset_path);

    inline Matrix3f skew(const Vec3f& v){
        Matrix3f S;
        S << 0, -v[2], v[1],
            v[2], 0, -v[0],
            -v[1], v[0], 0;
    return S;
    }

    inline Vec3f skew2v(const Matrix3f& S){
        Vec3f v;
        v << S(2,1), S(0,2), S(1,0);
        return v;
    }

    float rmse(const map<int, Vec3f>& landmarks, const map<int, Vec3f>& gt_landmarks);

    void save_landmarks(const map<int, pr::Vec3f>& landmarks, string output_path);
    void save_camera_positions(const vector<Camera>& cameras, string output_path);
    map<int, pr::Vec3f> load_landmarks(string path);


}
