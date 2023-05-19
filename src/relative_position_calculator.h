#pragma once
#include <Eigen/Core>
#include "utils.h"
#include <vector>
#include <string>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>

namespace pr {

    /* Wraps an Essential matrix and its SVD decomposition */
    struct Essential_Matrix { 
        Matrix3f e;
        Matrix3f u, s, vt;
    };

    Essential_Matrix eight_point_algorithm(vector<cv::Vec3f>& dir_vectors_i, vector<cv::Vec3f>& dir_vectors_j);

    Vec3f calculate_relative_position(const Camera& cam_i, const Camera& cam_j);
    
}
