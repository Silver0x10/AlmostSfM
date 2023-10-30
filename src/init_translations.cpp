// #pragma once
#include "init_translations.h"


namespace pr {

    void error_and_jacobian_translation(const Vec3f& t_ij, const Matrix3f& rot_i, const Vec3f& t_i, const Vec3f& t_j, Vec3f& error, Eigen::MatrixXf& jacobians) {
        error = skew(t_ij) * (rot_i.transpose() * (t_j - t_i));

        Matrix3f jac_t_j = skew(t_ij) * rot_i.transpose();
        Matrix3f jac_t_i = -jac_t_j;
        
        jacobians.block<3,3>(0, 0) = jac_t_i;
        jacobians.block<3,3>(0, 3) = jac_t_j;
    }


    void init_translations(vector<Camera>& cameras) {
        cameras[0].position.setZero();
        int system_size = cameras.size() - 1;

        Vec3f state[system_size];
        for(auto& t: state) t.setZero(); 
        auto t_0 = Eigen::Vector3f::Zero();
        Eigen::MatrixXf matrix_H(3*system_size, 3*system_size);
        matrix_H.setZero();
        
        for(int i=0; i<(int)(cameras.size()); ++i){
            for(int j=0; j<(int)(cameras.size()); ++j){
                if(i == 0 and j == 0) continue;

                // TODO: Maybe it should be given as input to the system and not computed?
                const auto& t_ij = v2tRPY(cameras[i].orientation).transpose()*(cameras[j].gt_position - cameras[i].gt_position); // GT for checking correctness
                // const auto& t_ij = calculate_relative_position(cameras[i], cameras[j]); // doen't work :(

                const auto& rot_i = v2tRPY(cameras[i].orientation);
                const auto& t_i = (i!=0) ? state[i - 1] : t_0;
                const auto& t_j = (j!=0) ? state[j - 1] : t_0;
                
                Vec3f error;
                Eigen::MatrixXf jacobians(3, 6);
                error_and_jacobian_translation(t_ij, rot_i, t_i, t_j, error, jacobians);
                Matrix3f jac_t_i = (i!=0) ? (Matrix3f)(jacobians.block<3,3>(0, 0)) : Eigen::Matrix3f::Zero();
                Matrix3f jac_t_j = (j!=0) ? (Matrix3f)(jacobians.block<3,3>(0, 3)) : Eigen::Matrix3f::Zero(); 
                
                // H += jacobian.transpose()*jacobian;
                if(i>0) matrix_H.block<3,3>(3*(i-1), 3*(i-1)) += jac_t_i.transpose()*jac_t_i;
                if(j>0) matrix_H.block<3,3>(3*(j-1), 3*(j-1)) += jac_t_j.transpose()*jac_t_j;
                if(i>0 and j>0) {
                    matrix_H.block<3,3>(3*(i-1), 3*(j-1)) += jac_t_i.transpose()*jac_t_j;
                    matrix_H.block<3,3>(3*(j-1), 3*(i-1)) += jac_t_j.transpose()*jac_t_i;
                }
            }
        }

        Eigen::JacobiSVD<Eigen::MatrixXf> svd_H(matrix_H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        auto t_initialized = svd_H.matrixV().rightCols<1>();

        cout << "last singular value: " << svd_H.singularValues()(svd_H.singularValues().size()-1) << endl;
        Eigen::VectorXf Ht = matrix_H * t_initialized;
        cout << "norm: " << Ht.norm() << endl;


        for(int i=1; i < (int)cameras.size(); i++) cameras[i].position = t_initialized.block<3,1>(3*(i-1), 0);
    }
}