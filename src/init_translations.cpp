#pragma once
#include "init_translations.h"


namespace pr {

    void error_and_jacobian_translation(const Vec3f& t_ij, const Matrix3f& rot_i, const Vec3f& t_i, const Vec3f& t_j, Vec3f& error, Eigen::MatrixXf& jacobian) {
    }

    void init_translations(vector<Camera>& cameras) {
        int system_size = pow(cameras.size(), 2);
        Vec3f state[system_size];
        for(auto t: state) t.setZero();
        
        Eigen::MatrixXf H(3*system_size, 3*system_size);
        Eigen::VectorXf b(3*system_size);
        
        // chi errror

        H.setZero();
        b.setZero();
        

    }


}