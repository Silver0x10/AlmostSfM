#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "utils.h"

namespace pr {

    Sim3 icp_3d(map<int, Vec3f>& landmarks, map<int, Vec3f> gt_landmarks);

}
