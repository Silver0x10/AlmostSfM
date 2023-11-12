#pragma once 
#include <iostream>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "utils.h"

namespace pr {

    void eval_map(const map<int, Vec3d>& landmarks, map<int, Vec3d> gt_landmarks, Sim3 transform, string output_dir);

    void eval_camera_rotations(const vector<Camera>& cameras, string output_dir);

    void eval_camera_positions(const vector<Camera>& cameras, string output_dir);

    void evaluation(const vector<Camera>& cameras, const map<int, Vec3d>& landmarks, const map<int, Vec3d>& gt_landmarks, const Sim3& transform, string output_dir); 

}

