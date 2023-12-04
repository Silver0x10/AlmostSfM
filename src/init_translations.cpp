// #pragma once
#include "init_translations.h"
#include <assert.h> 

namespace pr {

    void error_and_jacobian_translation(const Vec3d& t_ij, const Matrix3d& rot_i, Eigen::MatrixXd& jacobians) {
        // error = skew(t_ij) * (rot_i.transpose() * (t_j - t_i)); // always zero since t_i and t_j not initialized

        Matrix3d jac_t_j = skew(t_ij) * rot_i.transpose();
        Matrix3d jac_t_i = -jac_t_j;
        
        jacobians.block<3,3>(0, 0) = jac_t_i;
        jacobians.block<3,3>(0, 3) = jac_t_j;
    }


    void init_translations(vector<Camera>& cameras) {
        int system_size = 3*(cameras.size() - 1);
        Eigen::MatrixXd matrix_H(system_size, system_size);
        matrix_H.setZero();
        
        for(int i=0; i<(int)(cameras.size()); ++i){
            for(int j=i+1; j<(int)(cameras.size()); ++j){

                Vec3d t_ij = calculate_relative_position(cameras[i], cameras[j]);
                if(t_ij.norm() == 0) continue;

                const auto& rot_i = v2tRPY(cameras[i].orientation);
                
                Eigen::MatrixXd jacobians(3, 6);
                error_and_jacobian_translation(t_ij, rot_i, jacobians);
                Matrix3d jac_t_i = (Matrix3d)(jacobians.block<3,3>(0, 0));
                Matrix3d jac_t_j = (Matrix3d)(jacobians.block<3,3>(0, 3)); 
                
                // H += jacobian.transpose()*jacobian;
                if(i>0) matrix_H.block<3,3>(3*(i-1), 3*(i-1)) += jac_t_i.transpose()*jac_t_i;
                if(j>0) matrix_H.block<3,3>(3*(j-1), 3*(j-1)) += jac_t_j.transpose()*jac_t_j;
                if(i>0 and j>0) {
                    matrix_H.block<3,3>(3*(i-1), 3*(j-1)) += jac_t_i.transpose()*jac_t_j;
                    matrix_H.block<3,3>(3*(j-1), 3*(i-1)) += jac_t_j.transpose()*jac_t_i;
                }
            }
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd_H(matrix_H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        // matrix_H += 0.6 * Eigen::MatrixXd::Identity(3*system_size, 3*system_size);
        auto t_initialized = svd_H.matrixV().rightCols<1>();

        // cout << "last singular value: " << svd_H.singularValues()(svd_H.singularValues().size()-1) << endl;
        // Eigen::VectorXf Ht = matrix_H * t_initialized;
        // cout << "norm: " << Ht.norm() << endl;

        for(int i=1; i < (int)cameras.size(); i++) cameras[i].position = 10*t_initialized.block<3,1>(3*(i-1), 0);
    }
}