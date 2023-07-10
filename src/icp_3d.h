#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "utils.h"

namespace pr {

    void error_and_jacobian_icp_3d(Sim3 state, Vec3f landmark, Vec3f gt_landmark, Vec3f& error, Eigen::MatrixXf& jacobians);

    Sim3 icp_3d(map<int, Vec3f>& landmarks, map<int, Vec3f> gt_landmarks);

}
