#pragma once 
#include <iostream>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include "utils.h"
#include "relative_position_calculator.h"
#include "init_translations.h"

namespace pr {

    void error_and_jacobian_position_landmark(const Camera& camera, const Vec3f& keypoint_dir_vector, const Vec3f& landmark, Vec3f& error, Eigen::MatrixXf& jacobians);

    void bundle_adjustment(vector<Camera>& cameras, map<int, Vec3f>& landmarks);

}

