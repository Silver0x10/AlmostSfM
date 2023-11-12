#pragma once
#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <random>
#include "utils.h"
#include "relative_position_calculator.h"

namespace pr {

    void error_and_jacobian_translation(const Vec3d& t_ij, const Matrix3d& R_i, const Vec3d& t_i, const Vec3d& t_j, Vec3d& error, Eigen::MatrixXd& jacobians);

    void init_translations(vector<Camera>& cameras);

}
