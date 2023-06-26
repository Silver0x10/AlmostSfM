#include "bundle_adjustment.h"

namespace pr {

    void error_and_jacobian_camera_poses(const Vec3f& t_ij, const Vec3f& rot_angles_i, const Vec3f& t_i, const Vec3f& t_j, Vec3f& error, Eigen::MatrixXf& jacobians) {
        auto rot_i = v2tRPY(rot_angles_i);
        error = skew(t_ij) * rot_i.transpose() * (t_j - t_i);

        jacobians.block<3,3>(0, 0) = -skew(t_ij) * rot_i.transpose(); // wrt the camera i position

        jacobians.block<3,3>(0, 3) = skew(t_ij) * rot_i.transpose(); // wrt the camera j position

        jacobians.block<3,1>(0, 6) = skew(t_ij) * (Rx_prime(rot_angles_i[0])*Ry(rot_angles_i[1])*Rz(rot_angles_i[2])).transpose() * (t_i - t_j); // wrt the camera orientation wrt x
        jacobians.block<3,1>(0, 7) = skew(t_ij) * (Rx(rot_angles_i[0])*Ry_prime(rot_angles_i[1])*Rz(rot_angles_i[2])).transpose() * (t_i - t_j); // wrt the camera orientation wrt y
        jacobians.block<3,1>(0, 8) = skew(t_ij) * (Rx(rot_angles_i[0])*Ry(rot_angles_i[1])*Rz_prime(rot_angles_i[2])).transpose() * (t_i - t_j); // wrt the camera orientation wrt z
    }

    void error_and_jacobian_position_landmark(const Camera& camera, const Vec3f& keypoint_dir_vector, const Vec3f& landmark, Vec3f& error, Eigen::MatrixXf& jacobians) {
        auto dir_vector_hat = v2tRPY(camera.orientation).transpose()*(landmark - camera.position);
        // dir_vector_hat.normalize();
        error = skew(keypoint_dir_vector) * dir_vector_hat;
        
        auto alpha = camera.orientation[0];
        auto beta = camera.orientation[1];
        auto gamma = camera.orientation[2];
        jacobians.block<3,1>(0, 0) = skew(keypoint_dir_vector) * (Rx_prime(alpha)*Ry(beta)*Rz(gamma)).transpose() * (landmark - camera.position); // wrt the camera orientation wrt x
        jacobians.block<3,1>(0, 1) = skew(keypoint_dir_vector) * (Rx(alpha)*Ry_prime(beta)*Rz(gamma)).transpose() * (landmark - camera.position); // wrt the camera orientation wrt y
        jacobians.block<3,1>(0, 2) = skew(keypoint_dir_vector) * (Rx(alpha)*Ry(beta)*Rz_prime(gamma)).transpose() * (landmark - camera.position); // wrt the camera orientation wrt z

        jacobians.block<3,3>(0, 3) = -skew(keypoint_dir_vector) * v2tRPY(camera.orientation).transpose(); // wrt the camera position

        jacobians.block<3,3>(0, 6) = skew(keypoint_dir_vector) * v2tRPY(camera.orientation).transpose();  // wrt the landmark
    }
    
    void bundle_adjustment(vector<Camera>& cameras, map<int, Vec3f>& landmarks, int rounds){
        int system_size = 2*cameras.size() + landmarks.size();
        
        // (Like) Data association
        vector<int> landmark_index2id;
        for(const auto& l: landmarks) landmark_index2id.push_back(l.first);
        map<int, int> landmark_id2index;
        for(int i=0; i<(int)landmark_index2id.size(); i++) landmark_id2index.insert({landmark_index2id[i], i});
        // for(const auto& l: landmarks) { // just for checking if the association is correct
        //     int index = landmark_id2index[l.first];
        //     cout << l.first << "\t" << index << "\t" << landmark_index2id[index] << endl;
        // }

        Eigen::MatrixXf matrix_H(3*system_size, 3*system_size);
        Eigen::MatrixXf b(3*system_size, 1);

        for (int iteration = 0; iteration < rounds; iteration++) {
            cout << "BA iteration: " << iteration << endl;
            matrix_H.setZero();
            b.setZero();
            float chi_tot = 0.0;

            for(const auto& cam: cameras) {
                
                // Pose-Landmark
                for(const auto& kp: cam.keypoints) {
                    Vec3f error;
                    Eigen::MatrixXf jacobians(3, 9);
                    Vec3f l = landmarks[kp.id];
                    error_and_jacobian_position_landmark(cam, kp.direction_vector, l, error, jacobians);
                    float chi = error.transpose() * error;
                    chi_tot += chi;

                    Matrix3f jac_cam_orientation = (Matrix3f)(jacobians.block<3,3>(0, 0));
                    Matrix3f jac_cam_position = (Matrix3f)(jacobians.block<3,3>(0, 3));
                    Matrix3f jac_landmark = (Matrix3f)(jacobians.block<3,3>(0, 6));

                    int i_position = 3*cam.id;
                    int i_orientation = i_position + 3*cameras.size();
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
                
                // Pose-Pose
                for(const auto& cam2: cameras) {
                    if(cam2.id == cam.id) continue;
                    const auto& t_ij = v2tRPY(cam.gt_orientation).transpose()*(cam2.gt_position - cam.gt_position); // GT relative position for checking correctness
                    // const auto& t_ij = calculate_relative_position(cam, cam2);
                    const auto& rot_angles_i = cam.orientation;
                    const auto& t_i = cam.position;
                    const auto& t_j = cam2.position;
                    
                    Vec3f error;
                    Eigen::MatrixXf jacobians(3, 9);
                    error_and_jacobian_camera_poses(t_ij, rot_angles_i, t_i, t_j, error, jacobians);
                    float chi = error.transpose() * error;
                    // if(chi > 1) cout << chi << endl;
                    // if(chi > 1) continue;

                    chi_tot += chi;
                    Matrix3f jac_t_i = (Matrix3f)(jacobians.block<3,3>(0, 0));
                    Matrix3f jac_t_j = (Matrix3f)(jacobians.block<3,3>(0, 3)); 
                    Matrix3f jac_rot_i = (Matrix3f)(jacobians.block<3,3>(0, 6)); 
                    
                    int i = 3*cam.id;
                    int j = 3*cam2.id;
                    int k = 3*( cam.id + cameras.size() );

                    matrix_H.block<3,3>(i, i) += jac_t_i.transpose()*jac_t_i;
                    matrix_H.block<3,3>(i, j) += jac_t_i.transpose()*jac_t_j;
                    matrix_H.block<3,3>(i, k) += jac_t_i.transpose()*jac_rot_i;
                    
                    matrix_H.block<3,3>(j, i) += jac_t_j.transpose()*jac_t_i;
                    matrix_H.block<3,3>(j, j) += jac_t_j.transpose()*jac_t_j;
                    matrix_H.block<3,3>(j, k) += jac_t_j.transpose()*jac_rot_i;

                    matrix_H.block<3,3>(k, i) += jac_rot_i.transpose()*jac_t_i;
                    matrix_H.block<3,3>(k, j) += jac_rot_i.transpose()*jac_t_j;
                    matrix_H.block<3,3>(k, k) += jac_rot_i.transpose()*jac_rot_i;

                    b.block<3,1>(i, 0) += jac_t_i.transpose()*error;
                    b.block<3,1>(j, 0) += jac_t_j.transpose()*error;
                    b.block<3,1>(k, 0) += jac_rot_i.transpose()*error;
                }
                
            }

            cout << "chi_tot error: " << chi_tot << endl;
            if(chi_tot < 1) break;
            
            matrix_H += Eigen::MatrixXf::Identity(3*system_size, 3*system_size);

            Eigen::MatrixXf dx(3*system_size, 1);
            dx.setZero();
            dx = matrix_H.fullPivLu().solve(-b);
            // dx = matrix_H.ldlt().solve(-b);

            // State perturbation:
            for(int i=0; i < (int)cameras.size(); i++) {
                Vec3f dx_i_position = dx.block<3,1>(3*i, 0);
                cameras[i].position += dx_i_position;
                Vec3f dx_i_orientation = dx.block<3,1>(3*(i + cameras.size()), 0);
                // cout << i << " " << i + cameras.size()-1 << " " << cameras.size()-1 << " " << dx_i_orientation.transpose() << endl;
                cameras[i].orientation += dx_i_orientation;
                for(int j=0; j<3; j++) { // wrap each orientation angle to [-pi; pi)
                    float wrapped_angle = cameras[i].orientation[j];
                    wrapped_angle = (sin(wrapped_angle), cos(wrapped_angle));
                    cameras[i].orientation[j] = wrapped_angle;
                }
            }
            for(int j=2*cameras.size(); j < system_size; j++) {
                // cout << j-2*cameras.size() << "\t" << landmark_index2id[j-2*cameras.size()] << "\t" << landmark_id2index[landmark_index2id[j-2*cameras.size()]]<< endl;
                Vec3f dx_j = dx.block<3,1>(3*j, 0);
                landmarks[landmark_index2id[j-2*cameras.size()]] += dx_j;
            }

        }
        
    }

}