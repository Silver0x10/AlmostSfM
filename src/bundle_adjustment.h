#pragma once 
#include <iostream>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include "utils.h"
#include "relative_position_calculator.h"

namespace pr {

    void error_and_jacobian_BA_pose_landmark(const Camera& camera, const Vec3d& keypoint_dir_vector, const Vec3d& landmark, Vec3d& error, Eigen::MatrixXd& jacobians);

    void bundle_adjustment(vector<Camera>& cameras, map<int, Vec3d>& landmarks, int rounds);

}

