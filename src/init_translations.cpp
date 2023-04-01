// #pragma once
#include "init_translations.h"

namespace pr {

    void error_and_jacobian_translation(const Vec3f& t_ij, const Matrix3f& rot_i, const Vec3f& t_i, const Vec3f& t_j, Vec3f& error, Eigen::MatrixXf& jacobians) {
        error = skew(t_ij) * rot_i.transpose() * (t_i - t_j);

        Matrix3f jac_t_i = skew(t_ij) * rot_i.transpose();
        Matrix3f jac_t_j = -jac_t_i;
        auto delta_wrt_i = rot_i.transpose() * (t_i - t_j);
        Matrix3f jac_t_ij;
        jac_t_ij << 
            0,                   delta_wrt_i[2],    -delta_wrt_i[1], 
           -delta_wrt_i[2],      0,                  delta_wrt_i[0], 
            delta_wrt_i[1],     -delta_wrt_i[0],     0; 
        
        jacobians.block<3,3>(0, 0) = jac_t_i;
        jacobians.block<3,3>(0, 3) = jac_t_j;
        jacobians.block<3,3>(0, 6) = jac_t_ij;
    }


    void init_translations(vector<Camera>& cameras) {
        int num_cameras = cameras.size();
        int system_size = pow(num_cameras, 2);
        Vec3f state[system_size];
        for(auto& t: state) t.setRandom(); // TODO: maybe consider damping factors instead 
        
        Eigen::MatrixXf matrixH(3*system_size, 3*system_size);
        Eigen::VectorXf b(3*system_size);
        
        // chi errror

        matrixH.setZero();
        b.setZero();
        

        for(int i=0; i<num_cameras; ++i){
            for(int j=0; j<num_cameras; ++j){
                const auto& t_ij = state[i*num_cameras + j];
                const auto& rot_i = v2tRPY(cameras[i].orientation);
                const auto& t_i = state[i*num_cameras];
                const auto& t_j = state[j*num_cameras];
                
                Vec3f error;
                Eigen::MatrixXf jacobians(3, 9);
                error_and_jacobian_translation(t_ij, rot_i, t_i, t_j, error, jacobians);
                auto jac_t_i = jacobians.block<3,3>(0, 0) + Eigen::Matrix3f::Identity();
                auto jac_t_j = jacobians.block<3,3>(0, 3) +Eigen::Matrix3f::Identity();
                auto jac_t_ij = jacobians.block<3,3>(0, 6)+Eigen::Matrix3f::Identity();

                // H += jacobian.transpose()*jacobian;
                // b += jacobian.transpose()*error;

                cout << i << ' ' << j << endl;
            }
        }

        // auto dx = H.ldlt().solve(-b); // TODO: ripasso Cholesky decomposition; consider the null space of H



    }


}