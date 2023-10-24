#include "icp_3d.h"

namespace pr {

    // Lesson 23b_registration_on_a_manifold taken as reference

    void error_and_jacobian_icp_3d(Sim3 state, Vec3f landmark, Vec3f gt_landmark, Vec3f& error, Eigen::MatrixXf& jacobian) {

        Vec3f z_hat = state * gt_landmark;
        error = z_hat - landmark;

        // wrt the translation components:
        jacobian.block<3,3>(0, 0).setIdentity(); 
        // wrt the scaling component:
        jacobian.block<3,1>(0, 3) = z_hat; 
        // wrt the camera orientation:
        jacobian.block<3,3>(0, 4) = skew(-z_hat);
    }

    Sim3 icp_3d(map<int, Vec3f>& landmarks, map<int, Vec3f> gt_landmarks, int iterations) {
        Sim3 state = Sim3();

        Eigen::MatrixXf matrix_H(7, 7);
        Eigen::MatrixXf b(7, 1);

        for (int iteration = 0; iteration < iterations; iteration++) {
            cout << "ICP 3D iteration: " << iteration << "\t->\t";
            matrix_H.setZero();
            b.setZero();
            float chi_tot = 0.0;

            for(auto l: landmarks){
                Vec3f error;
                Eigen::MatrixXf jacobian(3, 7);
                jacobian.setZero();
                auto gt_l = gt_landmarks[l.first];

                error_and_jacobian_icp_3d(state, l.second, gt_l, error, jacobian);
                float chi = error.transpose() * error;

                if(chi >= 1e6) {
                    error *= sqrt(1e6/chi);
                    chi = 1e6;
                }
                chi_tot += chi;

                matrix_H += jacobian.transpose() * jacobian;
                b += jacobian.transpose()*error;
            }

            cout << "Error: " << chi_tot << endl;
            if(chi_tot < 0.01) break;
            
            matrix_H += 0.6 * Eigen::MatrixXf::Identity(7, 7);

            Eigen::MatrixXf dx(7, 1);
            dx.setZero();
            dx = matrix_H.fullPivLu().solve(-b);

            Vec3f d_translation = dx.block<3,1>(0, 0);
            float d_scale = dx(3, 0);
            Vec3f d_rotation = dx.block<3,1>(4,0);

            state.box_plus(d_translation, d_scale, d_rotation);
        }

        return state;
    }
}