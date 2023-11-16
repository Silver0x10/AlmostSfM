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

    void error_and_jacobian_translation(const Vec3d& t_ij, const Matrix3d& R_i, Eigen::MatrixXd& jacobians);

    void init_translations(vector<Camera>& cameras);

}
