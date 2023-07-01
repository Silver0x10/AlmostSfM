#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "utils.h"

namespace pr {
    // Eigen::Transform<float,3,Eigen::Affine> icp_3d(const map<int, Vec3f>& landmarks, const map<int, Vec3f>& gt_landmarks);
    // Eigen::Affine3f icp_3d(map<int, Vec3f>& landmarks, map<int, Vec3f> gt_landmarks);
    Sim3 icp_3d(map<int, Vec3f>& landmarks, map<int, Vec3f> gt_landmarks);
}
