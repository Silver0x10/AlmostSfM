#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "utils.h"

namespace pr {

    void error_and_jacobian_sicp_3d(Sim3 state, Vec3d landmark, Vec3d gt_landmark, Vec3d& error, Eigen::MatrixXd& jacobian);

    Sim3 sicp_3d(map<int, Vec3d> landmarks, map<int, Vec3d> gt_landmarks, int max_iterations);
    Sim3 sicp_3d(map<int, Vec3d> landmarks, map<int, Vec3d> gt_landmarks, int max_iterations, Sim3 initial_guess);

}
