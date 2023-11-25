#pragma once

#include <Eigen/Core>
#include <vector>
#include <string>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>

#include "utils.h"
#include "triangulation.h"

namespace pr {

    /* Wraps an Essential matrix and its SVD decomposition */
    struct Essential_Matrix { 
        Matrix3d e;
        Matrix3d u, s, vt;
    };

    Essential_Matrix eight_point_algorithm(vector<cv::Vec3d>& dir_vectors_i, vector<cv::Vec3d>& dir_vectors_j);
    
    int landmarks_in_front_of_two_cameras(const Camera& cam0, const Camera& cam1, const map<int, Vec3d>& landmarks);

    Vec3d extract_t(Camera cam_i, Camera cam_j, const Essential_Matrix& e);

    void find_correspondences(const Camera& cam_i, const Camera& cam_j, map<int, cv::Vec3d>& correspondences_i, map<int, cv::Vec3d>& correspondences_j);

    Vec3d calculate_relative_position(const Camera& cam_i, const Camera& cam_j);
    
}
