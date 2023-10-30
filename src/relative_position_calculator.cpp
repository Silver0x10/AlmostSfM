
#include "relative_position_calculator.h"

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

    /* Counts how many landmarks are in the FOV of both cameras */
    int count_admissible_points(const Vec3f& t, const Matrix3f& matrixR, const map<int, Vec3f>& landmarks) {
        int admissible_points = 0;
        
        // auto r3 = matrixR.row(2);
        for(auto& l: landmarks) {
            // TODO: understand why it works without checking if landmarks are in front of cam_i
            bool in_front_of_the_camera_i =  true; 
            // bool in_front_of_the_camera_i =  l.second[2] > 0;

            bool in_front_of_the_camera_j =  (matrixR*l.second + t)[2] > 0;
            // bool in_front_of_the_camera_j =  ( r3 * (l.second + t) ) > 0;
            
            if(in_front_of_the_camera_i and in_front_of_the_camera_j) ++admissible_points;
        }

        return admissible_points;
    }


    Vec3f extract_t(const Camera& cam_i, const Camera& cam_j, const Essential_Matrix& e) {
        Matrix3f matrixW;
        matrixW << 0,-1, 0,
                   1, 0, 0,
                   0, 0, 1;

        vector<Matrix3f> solutions_for_R;
        Matrix3f matrixR_0 = e.u * matrixW * e.vt; 
        solutions_for_R.push_back(matrixR_0);
        Matrix3f matrixR_1 = e.u * matrixW.transpose() * e.vt; 
        solutions_for_R.push_back(matrixR_1);

        vector<Vec3f> solutions_for_t;
        // Vec3f t_0 = e.u.col(2); // should be correct, but it doesn't work :( ( TODO: understand why )
        Vec3f t_0 = skew2v(e.vt.transpose() * e.s * matrixW.transpose() * e.vt); 
        solutions_for_t.push_back(t_0);
        Vec3f t_1 = -t_0;
        solutions_for_t.push_back(t_1);
        
        map<int, Vec3f> landmarks = triangulate(cam_i, cam_j);
        // Express landmarks in cam_i frame coordinates:
        // for(auto& l: landmarks) l.second = v2tRPY(cam_i.orientation).transpose() * (l.second - cam_i.position);
        for(auto& l: landmarks) l.second = v2tRPY(cam_i.orientation) * l.second + cam_i.position;

        Vec3f t_final;
        Matrix3f matrixR_final;
        int max_admissible_points = 0;
        for(const Vec3f& sol_t: solutions_for_t) {
            for(const Matrix3f& sol_R: solutions_for_R) {
                int admissible_points = count_admissible_points(sol_t, sol_R, landmarks);
                // cout << admissible_points << "\t";
                if( admissible_points > max_admissible_points ) {
                    max_admissible_points = admissible_points;
                    t_final = sol_t;
                    matrixR_final = sol_R;
                }
            }
        }
        // cout << endl << endl;
        return t_final;
    }


    void find_correspondences(const Camera& cam_i, const Camera& cam_j, vector<cv::Vec3f>& correspondences_i, vector<cv::Vec3f>& correspondences_j) {
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
    }


    Vec3f calculate_relative_position(const Camera& cam_i, const Camera& cam_j){
        vector<cv::Vec3f> correspondences_i;
        vector<cv::Vec3f> correspondences_j;
        find_correspondences(cam_i, cam_j, correspondences_i, correspondences_j);

        auto essential_matrix = eight_point_algorithm(correspondences_i, correspondences_j);
    
        auto t_ij = extract_t(cam_i, cam_j, essential_matrix);
                
        return t_ij;
    }

}

