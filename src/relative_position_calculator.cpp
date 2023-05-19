// #pragma once
#include "relative_position_calculator.h"
// #include <iostream>
// #include <opencv2/core.hpp>
// #include <opencv2/calib3d.hpp>
namespace pr {

    Essential_Matrix eight_point_algorithm(vector<cv::Vec3f>& dir_vectors_i, vector<cv::Vec3f>& dir_vectors_j){

        cv::normalize(dir_vectors_i, dir_vectors_i);
        cv::normalize(dir_vectors_j, dir_vectors_j);

        cv::Mat A(dir_vectors_i.size(), 9, CV_32F);
        for (size_t i = 0; i < dir_vectors_i.size(); ++i) {
            A.at<float>(i, 0) = dir_vectors_j[i][0] * dir_vectors_i[i][0];
            A.at<float>(i, 1) = dir_vectors_j[i][0] * dir_vectors_i[i][1];
            A.at<float>(i, 2) = dir_vectors_j[i][0] * dir_vectors_i[i][2];
            A.at<float>(i, 3) = dir_vectors_j[i][1] * dir_vectors_i[i][0];
            A.at<float>(i, 4) = dir_vectors_j[i][1] * dir_vectors_i[i][1];
            A.at<float>(i, 5) = dir_vectors_j[i][1] * dir_vectors_i[i][2];
            A.at<float>(i, 6) = dir_vectors_j[i][2] * dir_vectors_i[i][0];
            A.at<float>(i, 7) = dir_vectors_j[i][2] * dir_vectors_i[i][1];
            A.at<float>(i, 8) = dir_vectors_j[i][2] * dir_vectors_i[i][2];
        }

        cv::Mat u, s, vt;
        cv::SVD::compute(A, s, u, vt); // SVD of A
        cv::Mat essential_matrix = vt.t().col(8); // E (up to scale) is the column of V corresponding to the least singular value
        essential_matrix = essential_matrix.clone().reshape(0,3);
        
        // enforce singularity
        cv::SVD::compute(essential_matrix, s, u, vt);
        s.at<float>(0, 2) = 0.0;

        essential_matrix = u * cv::Mat::diag(s) * vt;

        Essential_Matrix structE;
        cv::cv2eigen(essential_matrix, structE.e); // OR structE.e = Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(essential_matrix.ptr<float>(), essential_matrix.rows, essential_matrix.cols);
        cv::cv2eigen(u, structE.u);
        cv::cv2eigen(cv::Mat::diag(s), structE.s);
        cv::cv2eigen(vt, structE.vt);
        
        return structE;
    }

    Vec3f calculate_relative_position(const Camera& cam_i, const Camera& cam_j){
        vector<cv::Vec3f> correspondences_i;
        vector<cv::Vec3f> correspondences_j;

        int k = 0;
        cv::Vec3f dir_vector_temp;
        for(int i = 0; i < (int)(cam_i.keypoints.size()); i++){
            for(int j = k; j < (int)(cam_j.keypoints.size()); j++) 
                if(cam_j.keypoints[j].id == cam_i.keypoints[i].id) {
                    eigen2cv(cam_i.keypoints[i].direction_vector, dir_vector_temp);
                    correspondences_i.push_back(dir_vector_temp);

                    eigen2cv(cam_j.keypoints[j].direction_vector, dir_vector_temp);
                    correspondences_j.push_back(dir_vector_temp);

                    k = j+1;
                }
        }

        auto essential_matrix = eight_point_algorithm(correspondences_i, correspondences_j);
    
        Matrix3f matrixW;
        matrixW << 0, 1, 0,
                  -1, 0, 0,
                   0, 0, 1;
        // Matrix3f matrixR = essential_matrix.u * matrixW * essential_matrix.vt;
        Vec3f t_ij = skew2v(essential_matrix.vt.transpose() * essential_matrix.s * matrixW.transpose() * essential_matrix.vt);
        return t_ij;
    }

    
}

