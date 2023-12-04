#include "icp_3d.h"

namespace pr {

    // Lesson 23b_registration_on_a_manifold taken as reference

    // void error_and_jacobian_icp_3d(Matrix4d state, Vec3d landmark, Vec3d gt_landmark, Vec3d& error, Eigen::MatrixXd& jacobian) {

    //     Vec3d z_hat = state * landmark;
    //     error = z_hat - gt_landmark;

    //     // wrt the translation components:
    //     jacobian.block<3,3>(0, 0).setIdentity(); 
    //     // wrt the camera orientation:
    //     jacobian.block<3,3>(0, 3) = skew(-z_hat);
    //     // wrt the scaling component:
    //     jacobian.block<3,1>(0, 6) = z_hat; 
    // }

    void error_and_jacobian_icp_3d(Sim3 state, Vec3d landmark, Vec3d gt_landmark, Vec3d& error, Eigen::MatrixXd& jacobian) {

        Vec3d z_hat = state * gt_landmark; // scale * ( R*gt_landmark + t )
        error = z_hat - landmark;

        // wrt the translation components:
        jacobian.block<3,3>(0, 0).setIdentity(); 
        // wrt the camera orientation:
        jacobian.block<3,3>(0, 3) = skew(-z_hat);
        // wrt the scaling component:
        jacobian.block<3,1>(0, 6) = z_hat; 
    }

    Sim3 sicp_3d(map<int, Vec3d> landmarks, map<int, Vec3d> gt_landmarks, int max_iterations, Sim3 initial_guess) { // SICP
        Sim3 state = initial_guess;

        Eigen::MatrixXd matrix_H(7, 7);
        Eigen::MatrixXd b(7, 1);
        Vec3d error; 
        Eigen::MatrixXd jacobian(3, 7);

        for (int iteration = 0; iteration < max_iterations; iteration++) {
            cout << "ICP 3D iteration: " << iteration << "\t->\t";
            matrix_H.setZero();
            b.setZero();
            double chi_tot = 0.0;

            for(auto l: landmarks){
                error.setZero();
                jacobian.setZero();
                auto gt_l = gt_landmarks[l.first];

                // error_and_jacobian_icp_3d(state_matrix, l.second, gt_l, error, jacobian);
                error_and_jacobian_icp_3d(state, l.second, gt_l, error, jacobian);
                double chi = error.transpose() * error;

                chi_tot += chi;
                if(chi >= 1e6) continue;

                matrix_H += jacobian.transpose() * jacobian;
                b += jacobian.transpose()*error;
            }

            cout << "Error: " << chi_tot << endl;
            if(chi_tot < 0.01) break;
            
            matrix_H += 10.0 * Eigen::MatrixXd::Identity(7, 7);

            Eigen::MatrixXd dx(7, 1);
            dx.setZero();
            dx = matrix_H.fullPivLu().solve(-b);

            Vec3d d_translation = dx.block<3,1>(0,0);
            Vec3d d_rotation = dx.block<3,1>(3,0);
            double d_scale = dx(6, 0);

            state.box_plus(d_translation, d_rotation, d_scale);
        }

        return state;
    }

    Sim3 sicp_3d(map<int, Vec3d> landmarks, map<int, Vec3d> gt_landmarks, int max_iterations) {
        Sim3 initial_guess = Sim3();
        return sicp_3d(landmarks, gt_landmarks, max_iterations, initial_guess);
    }

}