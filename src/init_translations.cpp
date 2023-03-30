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