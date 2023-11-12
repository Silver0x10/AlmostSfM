
#include "relative_position_calculator.h"

namespace pr {

    Essential_Matrix eight_point_algorithm(vector<cv::Vec3d>& dir_vectors_i, vector<cv::Vec3d>& dir_vectors_j){

        // cv::normalize(dir_vectors_i, dir_vectors_i);
        // cv::normalize(dir_vectors_j, dir_vectors_j);

        cv::Mat A(dir_vectors_i.size(), 9, CV_64F);
        for (size_t i = 0; i < dir_vectors_i.size(); ++i) {
            A.at<double>(i, 0) = dir_vectors_j[i][0] * dir_vectors_i[i][0];
            A.at<double>(i, 1) = dir_vectors_j[i][0] * dir_vectors_i[i][1];
            A.at<double>(i, 2) = dir_vectors_j[i][0] * dir_vectors_i[i][2];
            A.at<double>(i, 3) = dir_vectors_j[i][1] * dir_vectors_i[i][0];
            A.at<double>(i, 4) = dir_vectors_j[i][1] * dir_vectors_i[i][1];
            A.at<double>(i, 5) = dir_vectors_j[i][1] * dir_vectors_i[i][2];
            A.at<double>(i, 6) = dir_vectors_j[i][2] * dir_vectors_i[i][0];
            A.at<double>(i, 7) = dir_vectors_j[i][2] * dir_vectors_i[i][1];
            A.at<double>(i, 8) = dir_vectors_j[i][2] * dir_vectors_i[i][2];
        }

        cv::Mat u, s, vt;
        cv::SVD::compute(A, s, u, vt); // SVD of A
        cv::Mat essential_matrix = vt.t().col(8); // E (up to scale) is the column of V corresponding to the least singular value
        essential_matrix = essential_matrix.clone().reshape(0,3);
        
        // enforce singularity
        cv::SVD::compute(essential_matrix, s, u, vt);
        s.at<double>(0, 2) = 0.0;

        essential_matrix = u * cv::Mat::diag(s) * vt;

        Essential_Matrix structE;
        cv::cv2eigen(essential_matrix, structE.e); // OR structE.e = Eigen::Map<Eigen::Matrix<float, 3, 3,  ::RowMajor>>(essential_matrix.ptr<float>(), essential_matrix.rows, essential_matrix.cols);
        cv::cv2eigen(u, structE.u);
        cv::cv2eigen(cv::Mat::diag(s), structE.s);
        cv::cv2eigen(vt, structE.vt);
        
        return structE;
    }

    /* Counts how many landmarks are in the FOV of both cameras */
    int landmarks_in_front_of_two_cameras(const Camera& cam0, const Camera& cam1, const map<int, Vec3d>& landmarks) {
        int admissible_points = 0;
        
        for(auto& l: landmarks) {
            // bool in_front_of_the_cam0 =  true; 
            Vec3d l_wrt_cam0 = v2tRPY(cam0.orientation).transpose() * (l.second - cam0.position);
            bool in_front_of_the_cam0 =  l_wrt_cam0[2] >= 0;

            // bool in_front_of_the_cam1 =  true;
            Vec3d l_wrt_cam1 = v2tRPY(cam1.orientation).transpose() * (l.second - cam1.position);
            bool in_front_of_the_cam1 =  l_wrt_cam1[2] >= 0;
            
            if(in_front_of_the_cam0 and in_front_of_the_cam1) ++admissible_points;
        }

        return admissible_points;
    }


    Vec3d extract_t(Camera cam_i, Camera cam_j, const Essential_Matrix& e) {
        Matrix3d matrixW;
        matrixW << 0,-1, 0,
                    1, 0, 0,
                    0, 0, 1;

        // vector<Matrix3d> solutions_for_R;
        // Matrix3d matrixR_0 = e.u * matrixW * e.vt; 
        // solutions_for_R.push_back(matrixR_0);
        // Matrix3d matrixR_1 = e.u * matrixW.transpose() * e.vt; 
        // solutions_for_R.push_back(matrixR_1);

        vector<Vec3d> solutions_for_t;
        // Vec3f t_0 = e.u.col(2); // should be ok as well
        Vec3d t_0 = skew2v(e.vt.transpose() * e.s * matrixW.transpose() * e.vt); 

        // return t_0;
        
        solutions_for_t.push_back(t_0);
        Vec3d t_1 = -t_0;
        solutions_for_t.push_back(t_1);

        Vec3d t_final;
        Matrix3d matrixR_final;
        int max_admissible_points = 0;
        for(const Vec3d& sol_t: solutions_for_t) {
            cam_i.position.setZero();
            cam_j.position = sol_t;
            // cam_j.position = cam_i.position + sol_t;
            
            vector<Camera> cameras;
            cameras.push_back(cam_i);
            cameras.push_back(cam_j);
            map<int, Vec3d> landmarks = triangulate(cameras);

            int admissible_points = landmarks_in_front_of_two_cameras(cam_i, cam_j, landmarks);
            // cout << admissible_points << "\tof " << landmarks.size() << "\t|\t\t" << sol_t.transpose() << endl;
            if( admissible_points > max_admissible_points ) {
                max_admissible_points = admissible_points;
                t_final = sol_t;
            }
        }
        // cout <<  endl;
        return t_final;
    }


    void find_correspondences(const Camera& cam_i, const Camera& cam_j, vector<cv::Vec3d>& correspondences_i, vector<cv::Vec3d>& correspondences_j) {
        int k = 0;
        cv::Vec3d dir_vector_temp;
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
        // cout << "correspondences: " << correspondences_i.size() <<  endl;
    }


    Vec3d calculate_relative_position(const Camera& cam_i, const Camera& cam_j){
        vector<cv::Vec3d> correspondences_i;
        vector<cv::Vec3d> correspondences_j;
        find_correspondences(cam_i, cam_j, correspondences_i, correspondences_j);
        
        auto essential_matrix = eight_point_algorithm(correspondences_i, correspondences_j);
        
        auto t_ij = extract_t(cam_i, cam_j, essential_matrix);
        // cv::Mat E, R1, R2;
        // cv::eigen2cv(essential_matrix.e, E);
        // cv::Vec3d t;
        // cv::decomposeEssentialMat(E, R1, R2, t);
        // Vec3d t_ij;
        // cv::cv2eigen(t, t_ij);
                
        return t_ij;
    }

}

