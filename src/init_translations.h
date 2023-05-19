#pragma once
#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
// #include <Eigen/SVD>
#include "utils.h"
#include "relative_position_calculator.h"

namespace pr {

    void error_and_jacobian_translation(const Vec3f& t_ij, const Matrix3f& R_i, const Vec3f& t_i, const Vec3f& t_j, Vec3f& error, Eigen::MatrixXf& jacobians);

    void init_translations(vector<Camera>& cameras);

}
