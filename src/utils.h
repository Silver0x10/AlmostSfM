#pragma once
#include <iostream>
#include <list>
#include <vector>
#include <Eigen/Core>

using namespace std;

namespace pr {
  
  typedef vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vec3f;


  struct Camera {
    int id;
    Vec3f gt_position[3];
    Vec3f gt_orientation[3];
    Vec3f position[3];
    Vec3f orientation[3];

    // array<Keypoint> keypoints;
  };

  struct Keypoint {
    int id;
    Vec3f direction_vector[3];
  };



}
