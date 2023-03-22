#pragma once
#include <iostream>
#include <list>
#include <vector>
#include <Eigen/Core>

using namespace std;

namespace pr {
  
    // typedef Matrix<float, 3, 1> Vec3f;
    typedef Eigen::Vector3f Vec3f;

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

void quaternion_to_angle(float orientation[3]);


}
