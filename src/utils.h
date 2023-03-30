#pragma once
#include <cmath>
#include <iostream>
#include <list>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

namespace pr {
  
    // typedef Eigen::Vector3f Vec3f;
    typedef Eigen::Matrix<float, 3, 1> Vec3f;
    typedef Eigen::Matrix<float, 3, 3> Matrix3f;

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
        
        Camera(int id, float gt_position[3], float gt_orientation[3], float position[3], float orientation[3]);
    };

    void quaternion_to_RPY(float (&orientation)[3]);

    Matrix3f v2tRPY(const Vec3f& v);

    vector<Camera> load_data(string dataset_path);

    inline Matrix3f skew(const Vec3f& v){
    Matrix3f S;
    S << 0, -v[2], v[1],
      v[2], 0, -v[0],
      -v[1], v[0], 0;
    return S;
  }

}
