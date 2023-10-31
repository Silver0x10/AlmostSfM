#pragma once 
#include <iostream>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR> 

#include "utils.h"

namespace pr {

    map<int, Vec3f> triangulate(const vector<Camera>& cameras);

    // map<int, Vec3f> triangulate(const Camera& camera0, const Camera& camera1);

}

