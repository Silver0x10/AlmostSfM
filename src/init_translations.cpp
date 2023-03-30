// #pragma once
#include "init_translations.h"

namespace pr {

    void error_and_jacobian_translation(const Vec3f& t_ij, const Matrix3f& rot_i, const Vec3f& t_i, const Vec3f& t_j, Vec3f& error, Eigen::MatrixXf& jacobian, int num_cameras, int i, int j) {
        error = skew(t_ij) * rot_i.transpose() * (t_i - t_j);

        Matrix3f jac_t_i = skew(t_ij) * rot_i.transpose();
        Matrix3f jac_t_j = -jac_t_i;
        auto delta_wrt_i = rot_i.transpose() * (t_i - t_j);
        Matrix3f jac_t_ij;
        jac_t_ij << 
            0,                   delta_wrt_i[2],    -delta_wrt_i[1], 
           -delta_wrt_i[2],      0,                  delta_wrt_i[0], 
            delta_wrt_i[1],     -delta_wrt_i[0],     0; 
        
        jacobian.block<3,3>(0, 3*(i*num_cameras)) = jac_t_i;
        jacobian.block<3,3>(0, 3*(j*num_cameras)) = jac_t_j;
        jacobian.block<3,3>(0, 3*(i*num_cameras + j)) = jac_t_ij;
    }


    void init_translations(vector<Camera>& cameras) {
        int num_cameras = cameras.size();
        int system_size = pow(num_cameras, 2);
        Vec3f state[system_size];
        for(auto& t: state) t.setRandom(); // TODO: maybe consider damping factors instead 
        
        Eigen::MatrixXf H(3*system_size, 3*system_size);
        Eigen::VectorXf b(3*system_size);
        
        // chi errror

        H.setZero();
        b.setZero();
        

        for(int i=0; i<num_cameras; ++i){
            for(int j=0; j<num_cameras; ++j){
                const auto& t_ij = state[i*num_cameras + j];
                const auto& rot_i = v2tRPY(cameras[i].orientation);
                const auto& t_i = state[i*num_cameras];
                const auto& t_j = state[j*num_cameras];
                
                Vec3f error;
                Eigen::MatrixXf jacobian(3, 3*system_size); // TODO: use only the modified elments instead of the entire matrix
                error_and_jacobian_translation(t_ij, rot_i, t_i, t_j, error, jacobian, num_cameras, i, j);

                // H += jacobian.transpose()*jacobian;
                // b += jacobian.transpose()*error;

                cout << i << ' ' << j << endl;
            }
        }

        // auto dx = H.ldlt().solve(-b); // TODO: ripasso Cholesky decomposition; consider the null space of H



    }


}