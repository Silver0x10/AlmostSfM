#include "bundle_adjustment.h"

namespace pr {

    void error_and_jacobian_camera_poses(const Vec3f& t_ij, const Matrix3f& rot_i, const Vec3f& t_i, const Vec3f& t_j, Vec3f& error, Eigen::MatrixXf& jacobians) {
        error = skew(t_ij) * rot_i.transpose() * (t_i - t_j);

        Matrix3f jac_t_i = skew(t_ij) * rot_i.transpose();
        Matrix3f jac_t_j = -jac_t_i;
        // Matrix3f jac_t_k = ;
        
        jacobians.block<3,3>(0, 0) = jac_t_i;
        jacobians.block<3,3>(0, 3) = jac_t_j;
        // jacobians.block<3,3>(0, 6) = jac_t_k;
    }

    void error_and_jacobian_position_landmark(const Camera& camera, const Vec3f& keypoint_dir_vector, const Vec3f& landmark, Vec3f& error, Eigen::MatrixXf& jacobians) {
        auto delta_position = landmark - camera.position;
        error = skew(keypoint_dir_vector) * delta_position;
        
        jacobians.block<3,3>(0, 0) = -skew(keypoint_dir_vector); // wrt the camera
        jacobians.block<3,3>(0, 3) = skew(keypoint_dir_vector);  // wrt the landmark
    }
    
    void bundle_adjustment(vector<Camera>& cameras, map<int, Vec3f>& landmarks){
        int system_size = cameras.size() + landmarks.size();
        
        // (Like) Data association
        vector<int> landmark_index2id;
        for(const auto& l: landmarks) landmark_index2id.push_back(l.first);
        map<int, int> landmark_id2index;
        for(int i=0; i<(int)landmark_index2id.size(); i++) landmark_id2index.insert({landmark_index2id[i], i});

        Eigen::MatrixXf matrix_H(3*system_size, 3*system_size);
        Eigen::MatrixXf b(3*system_size, 1);

        for (int iteration = 0; iteration < 10; iteration++) {
            cout << "BA iteration: " << iteration << endl;
            matrix_H.setZero();
            b.setZero();

            for(const auto& cam: cameras) {

                for(const auto& kp: cam.keypoints) {
                    Vec3f error;
                    Eigen::MatrixXf jacobians(3, 6);
                    auto l = landmarks[landmark_id2index[kp.id]];
                    error_and_jacobian_position_landmark(cam, kp.direction_vector, l, error, jacobians);
                    Matrix3f jac_cam = (Matrix3f)(jacobians.block<3,3>(0, 0));
                    Matrix3f jac_landmark = (Matrix3f)(jacobians.block<3,3>(0, 3));

                    int i = 3*cam.id;
                    int j = 3*(cameras.size()-1 + landmark_id2index[kp.id]);
                    matrix_H.block<3,3>(i, i) += jac_cam.transpose()*jac_cam;
                    matrix_H.block<3,3>(i, j) += jac_cam.transpose()*jac_landmark;
                    matrix_H.block<3,3>(j, i) += jac_landmark.transpose()*jac_cam;
                    matrix_H.block<3,3>(j, j) += jac_landmark.transpose()*jac_landmark; 
                    b.block<3,1>(i, 0) += jac_cam.transpose()*error;
                    b.block<3,1>(j, 0) += jac_landmark.transpose()*error;
                }

                for(const auto& cam2: cameras){
                    if(cam2.id == cam.id) continue;
                    // as in init translations
                    const auto& t_ij = calculate_relative_position(cam, cam2);
                    const auto& rot_i = v2tRPY(cam.orientation);
                    const auto& t_i = cam.position;
                    const auto& t_j = cam2.position;
                    
                    Vec3f error;
                    Eigen::MatrixXf jacobians(3, 6);
                    error_and_jacobian_camera_poses(t_ij, rot_i, t_i, t_j, error, jacobians);
                    Matrix3f jac_t_i = (Matrix3f)(jacobians.block<3,3>(0, 0));
                    Matrix3f jac_t_j = (Matrix3f)(jacobians.block<3,3>(0, 3)); 
                    
                    int i = 3*cam.id;
                    int j = 3*cam2.id;
                    matrix_H.block<3,3>(i, i) += jac_t_i.transpose()*jac_t_i;
                    matrix_H.block<3,3>(i, j) += jac_t_i.transpose()*jac_t_j;
                    matrix_H.block<3,3>(j, i) += jac_t_j.transpose()*jac_t_i;
                    matrix_H.block<3,3>(j, j) += jac_t_j.transpose()*jac_t_j;
                    b.block<3,1>(i, 0) += jac_t_i.transpose()*error;
                    b.block<3,1>(j, 0) += jac_t_j.transpose()*error;
                }

                matrix_H += Eigen::MatrixXf::Identity(3*system_size, 3*system_size);
                Eigen::MatrixXf dx(3*system_size, 1);
                dx.setZero();

                dx = matrix_H.fullPivLu().solve(b);
                // dx = matrix_H.ldlt().solve(b);
                // cout << dx << endl << endl;

                for(int i=0; i < (int)cameras.size(); i++) {
                    Vec3f dx_i = dx.block<3,1>(3*i, 0);
                    cameras[i].position += dx_i;
                }

                for(int i=cameras.size(); i < system_size; i++) {
                    Vec3f dx_i = dx.block<3,1>(3*i, 0);
                    landmarks[i]+= dx_i;
                }
                
            }
            



        }
        

        
    }

}