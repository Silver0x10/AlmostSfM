#pragma once 
#include <iostream>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "utils.h"

namespace pr {

    float rmse(const vector<Vec3f>& landmarks);

    float eval_map(const vector<Vec3f>& landmarks);

    void eval_camera_rotations(const vector<Camera>& cameras, string output_dir);

    void eval_camera_positions(const vector<Camera>& cameras, string output_dir);

    void evaluation(const vector<Camera>& cameras, const map<int, Vec3f>& landmarks, const map<int, Vec3f>& gt_landmarks, string output_dir); 

}

