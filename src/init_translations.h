#pragma once
#include <Eigen/Core>
#include "utils.h"

namespace pr {

    void error_and_jacobian_translation(const Vec3f& t_ij, const Matrix3f& R_i, const Vec3f& t_i, const Vec3f& t_j, Vec3f& error, Eigen::MatrixXf& jacobian);

    void init_translations(vector<Camera>& cameras);

}
