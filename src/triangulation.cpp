#include "triangulation.h"

namespace pr {

    // Reference: https://silo.tips/download/least-squares-intersection-of-lines

    map<int, Vec3d> triangulate(const vector<Camera>& cameras){
        map<int, Vec3d> landmarks;
        map<int, Matrix3d> matrices_H;
        map<int, Vec3d> vectors_b;
        map<int, int> occurrences;

        for(const Camera& cam: cameras){
            for(const Keypoint& kp: cam.keypoints){

                auto dir = v2tRPY(cam.orientation) * kp.direction_vector; // express direction vector wrt "World" frame:

                if(landmarks.find(kp.id) == landmarks.end()) {
                    Vec3d landmark_position;
                    landmark_position.setZero();
                    landmarks.insert({kp.id, landmark_position});
                    occurrences.insert({kp.id, 1});

                    Matrix3d matrix_H = Eigen::Matrix3d::Identity() - dir*dir.transpose();
                    Vec3d b = ( Eigen::Matrix3d::Identity() - dir*dir.transpose() ) * cam.position;
                    vectors_b.insert({kp.id, b});
                    matrices_H.insert({kp.id, matrix_H});
                } 
                else {
                    matrices_H[kp.id] += Eigen::Matrix3d::Identity() - dir*dir.transpose();
                    vectors_b[kp.id] += ( Eigen::Matrix3d::Identity() - dir*dir.transpose() ) * cam.position;
                    occurrences[kp.id] += 1;
                }
            }
        }

        for(auto l_occ: occurrences)
            if(l_occ.second < 2) landmarks.erase(l_occ.first);
        
        for(auto& l: landmarks) {
            Matrix3d matrix_H = matrices_H[l.first];
            Vec3d b = vectors_b[l.first];
            l.second = matrix_H.fullPivLu().solve(b);
            // l.second = matrix_H.completeOrthogonalDecomposition().pseudoInverse() * b;
        }

        return landmarks;
    }
    
}