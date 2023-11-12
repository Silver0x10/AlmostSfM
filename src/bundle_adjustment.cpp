#include "bundle_adjustment.h"

namespace pr {

    void error_and_jacobian_BA_pose_landmark(const Camera& camera, const Vec3d& keypoint_dir_vector, const Vec3d& landmark, Vec3d& error, Eigen::MatrixXd& jacobians) {
        Vec3d dir_vector_hat = v2tRPY(camera.orientation).transpose()*(landmark - camera.position);
        // Vec3d dir_vector_hat = v2tRPY(camera.orientation) * landmark + camera.position;
        dir_vector_hat.normalize();
        error = skew(keypoint_dir_vector) * dir_vector_hat;

        auto alpha = camera.orientation[0];
        auto beta = camera.orientation[1];
        auto gamma = camera.orientation[2];
        jacobians.block<3,1>(0, 0) = skew(keypoint_dir_vector) * (Rx_prime(alpha)*Ry(beta)*Rz(gamma)).transpose() * (landmark - camera.position); // wrt the camera orientation wrt x
        jacobians.block<3,1>(0, 1) = skew(keypoint_dir_vector) * (Rx(alpha)*Ry_prime(beta)*Rz(gamma)).transpose() * (landmark - camera.position); // wrt the camera orientation wrt y
        jacobians.block<3,1>(0, 2) = skew(keypoint_dir_vector) * (Rx(alpha)*Ry(beta)*Rz_prime(gamma)).transpose() * (landmark - camera.position); // wrt the camera orientation wrt z

        jacobians.block<3,3>(0, 3) = - skew(keypoint_dir_vector) * v2tRPY(camera.orientation).transpose(); // wrt the camera position

        jacobians.block<3,3>(0, 6) = skew(keypoint_dir_vector) * v2tRPY(camera.orientation).transpose();  // wrt the landmark
    }    
    
    void bundle_adjustment(vector<Camera>& cameras, map<int, Vec3d>& landmarks, int rounds){
        int system_size = 2*cameras.size() + landmarks.size();
        
        // (Like) Data association
        vector<int> landmark_index2id; for(const auto& l: landmarks) landmark_index2id.push_back(l.first);
        map<int, int> landmark_id2index; for(int i=0; i<(int)landmark_index2id.size(); i++) landmark_id2index.insert({landmark_index2id[i], i});

        Eigen::MatrixXd matrix_H(3*system_size, 3*system_size);
        Eigen::MatrixXd b(3*system_size, 1);

        for (int iteration = 0; iteration < rounds; iteration++) {
            cout << "BA iteration: " << iteration << "\t->\t";
            matrix_H.setZero();
            b.setZero();
            double chi_tot = 0.0;
            Vec3d error; 
            Eigen::MatrixXd jacobians(3, 9);

            for(const auto& cam: cameras) {
                
                for(const auto& kp: cam.keypoints) {
                    error.setZero();
                    jacobians.setZero();
                    Vec3d l = landmarks[kp.id];
                    error_and_jacobian_BA_pose_landmark(cam, kp.direction_vector, l, error, jacobians);
                    double chi = error.transpose() * error;
                    chi_tot += chi;

                    Matrix3d jac_cam_orientation = (Matrix3d)(jacobians.block<3,3>(0, 0));
                    Matrix3d jac_cam_position = (Matrix3d)(jacobians.block<3,3>(0, 3));
                    Matrix3d jac_landmark = (Matrix3d)(jacobians.block<3,3>(0, 6));

                    int i_position = 3 * cam.id;
                    int i_orientation = 3*( cameras.size() + cam.id );
                    int j = 3*( 2*cameras.size() + landmark_id2index[kp.id] );

                    matrix_H.block<3,3>(i_position, i_position)    += jac_cam_position.transpose()*jac_cam_position;
                    matrix_H.block<3,3>(i_position, i_orientation) += jac_cam_position.transpose()*jac_cam_orientation;
                    matrix_H.block<3,3>(i_position, j)             += jac_cam_position.transpose()*jac_landmark;

                    matrix_H.block<3,3>(i_orientation, i_position)    += jac_cam_orientation.transpose()*jac_cam_position;
                    matrix_H.block<3,3>(i_orientation, i_orientation) += jac_cam_orientation.transpose()*jac_cam_orientation;
                    matrix_H.block<3,3>(i_orientation, j)             += jac_cam_orientation.transpose()*jac_landmark;
                    
                    matrix_H.block<3,3>(j, i_position)    += jac_landmark.transpose()*jac_cam_position;
                    matrix_H.block<3,3>(j, i_orientation) += jac_landmark.transpose()*jac_cam_orientation;
                    matrix_H.block<3,3>(j, j)             += jac_landmark.transpose()*jac_landmark;
                    
                    b.block<3,1>(i_position, 0) += jac_cam_position.transpose()*error;
                    b.block<3,1>(i_orientation, 0) += jac_cam_orientation.transpose()*error;
                    b.block<3,1>(j, 0) += jac_landmark.transpose()*error;
                }
                
            }

            cout << "chi_tot error: " << chi_tot << endl;
            if(chi_tot < 1) break;
            
            matrix_H += 0.6*Eigen::MatrixXd::Identity(3*system_size, 3*system_size);

            Eigen::MatrixXd dx(3*system_size, 1);
            dx.setZero();
            dx = matrix_H.fullPivLu().solve(-b);
            // dx = matrix_H.ldlt().solve(-b);

            // TODO: check updates
            // State perturbation (cameras):
            for(int i=0; i < (int)cameras.size(); i++) {
                // position update
                Vec3d dx_i_position = dx.block<3,1>(3*i, 0);
                cameras[i].position += dx_i_position;
                // orientation update
                Vec3d dx_i_orientation = dx.block<3,1>(3*(i + cameras.size()), 0);
                cameras[i].orientation =  tRPY2v(v2tRPY(dx_i_orientation) * v2tRPY(cameras[i].orientation));
                // cameras[i].orientation += dx_i_orientation;
                // for(int j=0; j<3; j++) { // wrap each orientation angle to [-pi; pi)
                //     cameras[i].orientation[j] = atan2(sin(cameras[i].orientation[j]), cos(cameras[i].orientation[j]));
                // }
            }
            // State perturbation (landmarks):
            for(int j=2*cameras.size(); j < system_size; j++) {
                Vec3d dx_j = dx.block<3,1>(3*j, 0);
                landmarks[landmark_index2id[j-2*cameras.size()]] += dx_j;
            }

        }
        
    }

}