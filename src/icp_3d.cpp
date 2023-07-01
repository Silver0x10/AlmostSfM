#include "icp_3d.h"

namespace pr {

    void error_and_jacobian_icp_3d(Sim3 state, Vec3f landmark, Vec3f gt_landmark, Vec3f& error, Eigen::MatrixXf& jacobians) {
        error = state * landmark - gt_landmark;

        jacobians.block<3,3>(0, 0).setIdentity(); // wrt the translation components

        jacobians.block<3,1>(0, 3) = v2tRPY(state.rotation) * landmark; // wrt the scaling component

        jacobians.block<3,1>(0, 4) = Rx_prime(state.rotation[0])*Ry(state.rotation[1])*Rz(state.rotation[2]) * landmark; // wrt the camera orientation wrt x
        jacobians.block<3,1>(0, 5) = Rx(state.rotation[0])*Ry_prime(state.rotation[1])*Rz(state.rotation[2]) * landmark; // wrt the camera orientation wrt y
        jacobians.block<3,1>(0, 6) = Rx(state.rotation[0])*Ry(state.rotation[1])*Rz_prime(state.rotation[2]) * landmark; // wrt the camera orientation wrt z
    }

    Sim3 icp_3d(map<int, Vec3f>& landmarks, map<int, Vec3f> gt_landmarks) {
        Sim3 state;

        Eigen::MatrixXf matrix_H(7, 7);
        Eigen::MatrixXf b(7, 1);

        for (int iteration = 0; iteration < 5; iteration++) {
            cout << "ICP 3D iteration: " << iteration << endl;
            matrix_H.setZero();
            b.setZero();
            float chi_tot = 0.0;

            for(auto l: landmarks){
                Vec3f error;
                Eigen::MatrixXf jacobian(3, 7);
                auto gt_l = gt_landmarks[l.first];

                error_and_jacobian_icp_3d(state, l.second, gt_l, error, jacobian);
                float chi = error.transpose() * error;
                chi_tot += chi;

                matrix_H += jacobian.transpose() * jacobian;
                b += jacobian.transpose()*error;
            }

            cout << "chi_tot error: " << chi_tot << endl;
            if(chi_tot < 1) break;
            
            Eigen::MatrixXf dx(7, 1);
            dx.setZero();
            dx = matrix_H.fullPivLu().solve(-b);

            Vec3f translation = dx.block<3,1>(0, 0);
            float scale = dx(3, 0);
            Vec3f rotation = dx.block<3,1>(4,0);

            // cout << "dx: " << dx.transpose() << endl;
            cout << endl << "tr: " << translation.transpose() << endl;
            cout << "s: " << scale << endl;
            cout << "rot: " << rotation.transpose() << endl<<endl;

            state.perturb(translation, scale, rotation);
        }

        return state;
    }
}