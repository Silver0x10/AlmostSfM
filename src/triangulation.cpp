#include "triangulation.h"

namespace pr {

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
            l.second = matrix_H.fullPivLu().solve(b);
        }

        // Save landmarks positions on file
        ofstream landmarks_stream("landmarks.txt");
        for(const auto& l: landmarks){
            // cout << "L: " << l.first << "\t" << l.second.transpose() << endl;
            landmarks_stream << "L: " << l.first << "\t" << l.second.transpose() << endl;
        }
        landmarks_stream.close();

        return landmarks;
    }
    
}