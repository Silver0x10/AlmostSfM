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
        Matrix3f e;
        Matrix3f u, s, vt;
    };

    Essential_Matrix eight_point_algorithm(vector<cv::Vec3f>& dir_vectors_i, vector<cv::Vec3f>& dir_vectors_j);
    
    int count_admissible_points(const Vec3f& t, const Matrix3f& matrixR, const map<int, Vec3f>& landmarks);

    Vec3f extract_t(const Camera& cam_i, const Camera& cam_j, const Essential_Matrix& e);

    Vec3f calculate_relative_position(const Camera& cam_i, const Camera& cam_j);
    
}
