#include "bundle_adjustment.h"

namespace pr {


    void error_and_jacobian_BA_pose_landmark_linear(const Matrix4d& camera_pose, const Vec3d& keypoint_dir_vector, const Vec3d& landmark, Vec3d& error, Eigen::MatrixXd& jacobians) {
        Vec3d landmark_camera_frame = camera_pose.block<3,3>(0,0) * landmark + camera_pose.block<3,1>(0,3);
        Vec3d dir_hat = landmark_camera_frame / landmark_camera_frame.norm();

        error = dir_hat - keypoint_dir_vector;

        Eigen::MatrixXd jacobian_norm = ( Eigen::MatrixXd::Identity(3,3) - landmark_camera_frame*landmark_camera_frame.transpose() / (landmark_camera_frame.transpose()*landmark_camera_frame).value() ) / landmark_camera_frame.norm();

        // wrt camera pose
        jacobians.block<3,3>(0, 0) = Eigen::MatrixXd::Identity(3,3); // wrt camera position
        jacobians.block<3,3>(0, 3) = -skew(landmark_camera_frame); // wrt camera orientation
        jacobians.block<3,6>(0, 0) = jacobian_norm * jacobians.block<3,6>(0, 0);

        // wrt landmark position
        jacobians.block<3,3>(0, 6) = camera_pose.block<3,3>(0,0);
        jacobians.block<3,3>(0, 6) = jacobian_norm * jacobians.block<3,3>(0, 6);
    }

    // Not working properly yet
    void error_and_jacobian_BA_pose_landmark(const Matrix4d& camera_pose, const Vec3d& keypoint_dir_vector, const Vec3d& landmark, Vec3d& error, Eigen::MatrixXd& jacobians) {
        Vec3d landmark_camera_frame = camera_pose.block<3,3>(0,0).transpose() * (landmark - camera_pose.block<3,1>(0,3));
        Vec3d dir_hat = landmark_camera_frame / landmark_camera_frame.norm();

        error = skew(keypoint_dir_vector) * dir_hat;

        Eigen::MatrixXd jacobian_norm = ( Eigen::MatrixXd::Identity(3,3) - landmark_camera_frame*landmark_camera_frame.transpose() / (landmark_camera_frame.transpose()*landmark_camera_frame).value() ) / landmark_camera_frame.norm();

        // // wrt camera pose
        // wrt camera position
        jacobians.block<3,3>(0, 0) = - skew(keypoint_dir_vector) * camera_pose.block<3,3>(0,0).transpose(); 

        // wrt camera orientation
        Vec3d camera_orientation = camera_pose.block<3,3>(0,0).eulerAngles(2,1,0).reverse();
        auto alpha = camera_orientation[0];
        auto beta  = camera_orientation[1];
        auto gamma = camera_orientation[2];
        jacobians.block<3,1>(0, 3) = skew(keypoint_dir_vector) * ( Rz(gamma)*Ry(beta)*Rx_prime(alpha) ).transpose() * (landmark - camera_pose.block<3,1>(0,3)); // wrt the camera orientation wrt x
        jacobians.block<3,1>(0, 4) = skew(keypoint_dir_vector) * ( Rz(gamma)*Ry_prime(beta)*Rx(alpha) ).transpose() * (landmark - camera_pose.block<3,1>(0,3)); // wrt the camera orientation wrt y
        jacobians.block<3,1>(0, 5) = skew(keypoint_dir_vector) * ( Rz_prime(gamma)*Ry(beta)*Rx(alpha) ).transpose() * (landmark - camera_pose.block<3,1>(0,3)); // wrt the camera orientation wrt z
        
        jacobians.block<3,6>(0, 0) = jacobian_norm * jacobians.block<3,6>(0, 0);

        // // wrt landmark position
        jacobians.block<3,3>(0, 0) = skew(keypoint_dir_vector) * camera_pose.block<3,3>(0,0).transpose(); 
        jacobians.block<3,3>(0, 6) = jacobian_norm * jacobians.block<3,3>(0, 6);
    }
    
    void bundle_adjustment(vector<Camera>& cameras, map<int, Vec3d>& landmarks, int rounds, bool linear_constraint){
        int system_size = 3*(2*cameras.size() + landmarks.size());
        double damping = 0.5;

        // (Like) Data association
        vector<int> landmark_index2id; for(const auto& l: landmarks) landmark_index2id.push_back(l.first);
        map<int, int> landmark_id2index; for(int i=0; i<(int)landmark_index2id.size(); i++) landmark_id2index.insert({landmark_index2id[i], i});

        Eigen::MatrixXd state_cameras(4, 4*cameras.size()); state_cameras.setZero();
        Eigen::MatrixXd state_landmarks(3, landmarks.size()); state_landmarks.setZero();
        
        for(int i=0; i<(int)cameras.size(); ++i) {
            if(linear_constraint) {
                state_cameras.block<3,3>(0, 4*i) = v2tRPY(cameras[i].orientation).transpose();
                state_cameras.block<3,1>(0, 4*i + 3) = -state_cameras.block<3,3>(0, 4*i)*cameras[i].position;
            } else {
                state_cameras.block<3,3>(0, 4*i) = v2tRPY(cameras[i].orientation);
                state_cameras.block<3,1>(0, 4*i + 3) = cameras[i].position;
            }
            state_cameras(3, 4*i + 3) = 1.0;
        }
        for(int i=0; i<(int)landmark_index2id.size(); ++i) 
            state_landmarks.block<3,1>(0, i) = landmarks[landmark_index2id[i]]; 

        Eigen::MatrixXd matrix_H(system_size, system_size);
        Eigen::MatrixXd b(system_size, 1);
        Vec3d error; 
        Eigen::MatrixXd jacobians(3, 9);

        for (int iteration = 0; iteration < rounds; iteration++) {
            cout << "BA iteration: " << iteration << "\t->\t";
            matrix_H.setZero(); 
            b.setZero();
            double chi_tot = 0.0;

            for(const auto& cam: cameras) {
                for(const auto& kp: cam.keypoints) {
                    error.setZero();
                    jacobians.setZero();
                    int landmark_index = landmark_id2index[kp.id];
                    Vec3d landmark = state_landmarks.block<3,1>(0, landmark_index);
                    Matrix4d cam_pose = state_cameras.block<4,4>(0, 4*cam.id);
                    
                    if(linear_constraint) error_and_jacobian_BA_pose_landmark_linear(cam_pose, kp.direction_vector, landmark, error, jacobians);
                    else error_and_jacobian_BA_pose_landmark(cam_pose, kp.direction_vector, landmark, error, jacobians);
                    
                    double chi = error.transpose() * error;
                    chi_tot += chi;
                    if(chi >= 2) continue;

                    Matrix3d jac_cam_position = (Matrix3d)(jacobians.block<3,3>(0, 0));
                    Matrix3d jac_cam_orientation = (Matrix3d)(jacobians.block<3,3>(0, 3));
                    Matrix3d jac_landmark = (Matrix3d)(jacobians.block<3,3>(0, 6));

                    int i_position = 6 * cam.id;
                    int i_orientation = i_position + 3;
                    int j = 3*( 2*cameras.size() + landmark_index);

                    matrix_H.block<3,3>(i_position, i_position)    += jac_cam_position.transpose()*jac_cam_position;
                    matrix_H.block<3,3>(i_position, i_orientation) += jac_cam_position.transpose()*jac_cam_orientation;
                    matrix_H.block<3,3>(i_position, j)             += jac_cam_position.transpose()*jac_landmark;

                    matrix_H.block<3,3>(i_orientation, i_position)    += jac_cam_orientation.transpose()*jac_cam_position;
                    matrix_H.block<3,3>(i_orientation, i_orientation) += jac_cam_orientation.transpose()*jac_cam_orientation;
                    matrix_H.block<3,3>(i_orientation, j)             += jac_cam_orientation.transpose()*jac_landmark;
                    
                    matrix_H.block<3,3>(j, i_position)    += jac_landmark.transpose()*jac_cam_position;
                    matrix_H.block<3,3>(j, i_orientation) += jac_landmark.transpose()*jac_cam_orientation;
                    matrix_H.block<3,3>(j, j)             += jac_landmark.transpose()*jac_landmark;
                    
                    b.block<3,1>(i_position, 0)    += jac_cam_position.transpose()*error;
                    b.block<3,1>(i_orientation, 0) += jac_cam_orientation.transpose()*error;
                    b.block<3,1>(j, 0)             += jac_landmark.transpose()*error;
                }
                
            }

            cout << "chi_tot error: " << chi_tot << endl;
            // if(chi_tot < 1) break;
            
            matrix_H += damping * Eigen::MatrixXd::Identity(system_size, system_size);

            // Fix the position of the first camera
            for(int i=0; i<6; i++) { 
                matrix_H.row(i) = Eigen::VectorXd::Zero(matrix_H.cols());
                matrix_H.col(i) = Eigen::VectorXd::Zero(matrix_H.rows());
            }
            b.block<6,1>(0, 0) = Eigen::VectorXd::Zero(6);

            Eigen::MatrixXd dx(system_size, 1); dx.setZero();
            dx = matrix_H.fullPivLu().solve(-b);

            // State perturbation (cameras):
            for(int i=0; i < (int)cameras.size(); i++) {
                int cam_state = 6*i;
                Eigen::MatrixXd dx_i = dx.block<6,1>(cam_state, 0);
                Matrix4d transform = v2t(dx_i);
                state_cameras.block<4,4>(0, 4*i) = transform * state_cameras.block<4,4>(0, 4*i);
            }
            // State perturbation (landmarks):
            for(int j=0; j < (int)landmarks.size(); j++) {
                int landmark_state = 6*cameras.size() + 3*j;
                Vec3d dx_j = dx.block<3,1>(landmark_state, 0);
                state_landmarks.block<3,1>(0, j) += dx_j;
            }
        }

        for(int i=0; i < (int)cameras.size(); i++) {
            auto pose_i = state_cameras.block<4,4>(0, 4*i);
            auto R = pose_i.block<3,3>(0,0);
            auto t = pose_i.block<3,1>(0,3);
            if(linear_constraint) {
                cameras[i].orientation = R.transpose().eulerAngles(2,1,0).reverse();
                cameras[i].position = -R.transpose()*t;
            } else {
                cameras[i].orientation = R.eulerAngles(2,1,0).reverse();
                cameras[i].position = t;
            }
        }
        for(int j=0; j < (int)landmarks.size(); j++) {
            landmarks[landmark_index2id[j]] = state_landmarks.block<3,1>(0, j);
        }
    }

}