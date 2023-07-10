#include "triangulation.h"

namespace pr {

    // Reference: https://silo.tips/download/least-squares-intersection-of-lines

    map<int, Vec3f> triangulate(const Camera& cam_i, const Camera& cam_j) {
        map<int, Vec3f> cam_j_keypoints;
        for(const Keypoint& kp: cam_j.keypoints) cam_j_keypoints.insert({kp.id, kp.direction_vector});

        map<int, Vec3f> landmarks;
        for(const Keypoint& kp: cam_i.keypoints) {
            if(cam_j_keypoints.find(kp.id) == cam_j_keypoints.end()) continue;
            
            Matrix3f matrix_H = Matrix3f::Identity() - kp.direction_vector*kp.direction_vector.transpose();
            matrix_H += Matrix3f::Identity() - cam_j_keypoints[kp.id]*cam_j_keypoints[kp.id].transpose();
            Vec3f b = ( Matrix3f::Identity() - kp.direction_vector*kp.direction_vector.transpose() ) * cam_i.position;
            b += ( Matrix3f::Identity() - cam_j_keypoints[kp.id]*cam_j_keypoints[kp.id].transpose() ) * cam_j.position;

            Vec3f landmark_position = matrix_H.completeOrthogonalDecomposition().pseudoInverse() * b;

            landmarks.insert({kp.id, landmark_position});
        }

        return landmarks;
    }

    map<int, Vec3f> triangulate(const vector<Camera>& cameras){
        map<int, Vec3f> landmarks;
        map<int, Matrix3f> matrices_H;
        map<int, Vec3f> vectors_b;

        for(const Camera& cam: cameras){
            for(const Keypoint& kp: cam.keypoints){
                if(landmarks.find(kp.id) == landmarks.end()) {
                    Vec3f landmark_position;
                    landmark_position.setZero();
                    landmarks.insert({kp.id, landmark_position});

                    Matrix3f matrix_H = Eigen::Matrix3f::Identity() - kp.direction_vector*kp.direction_vector.transpose();
                    Vec3f b = ( Eigen::Matrix3f::Identity() - kp.direction_vector*kp.direction_vector.transpose() ) * cam.position;
                    vectors_b.insert({kp.id, b});
                    matrices_H.insert({kp.id, matrix_H});
                } 
                else {
                    matrices_H[kp.id] += Eigen::Matrix3f::Identity() - kp.direction_vector*kp.direction_vector.transpose();
                    vectors_b[kp.id] += ( Eigen::Matrix3f::Identity() - kp.direction_vector*kp.direction_vector.transpose() ) * cam.position;
                }
            }
        }

        for(auto& l: landmarks) {
            Matrix3f matrix_H = matrices_H[l.first];
            Vec3f b = vectors_b[l.first];
            l.second = matrix_H.completeOrthogonalDecomposition().pseudoInverse() * b;
        }

        return landmarks;
    }
    
}