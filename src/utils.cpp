#include "utils.h"

namespace pr {

    Camera::Camera(int id, float gt_position[3], float gt_orientation[3], float position[3], float orientation[3]){
        this->id = id;
        this->gt_position = Eigen::Map<Vec3f>(gt_position, 3, 1);
        this->gt_orientation = Eigen::Map<Vec3f>(gt_orientation, 3, 1);
        this->position = Eigen::Map<Vec3f>(position, 3, 1);
        this->orientation = Eigen::Map<Vec3f>(orientation, 3, 1);
    }
    

    Keypoint::Keypoint(int id, float direction_vector[3]) {
        this->id = id;
        this->direction_vector = Eigen::Map<Vec3f>(direction_vector, 3, 1);
    }

    /* Given the imaginery part of a unit quaternion, reconstructs the whole quaternion and transforms it 
        in an Euler angles representation (Roll-Pitch-Yaw) .
    */
    void quaternion_to_RPY(float (&orientation)[3]){
        float qw, qx = orientation[0], qy = orientation[1], qz = orientation[2];

        // Implementation adapted from: see README_SFM1B.md
        float norm = sqrt(pow(qx, 2) + pow(qy, 2) + pow(qz, 2));
        if(norm > 1) qw = 1, qx = qy = qz = 0;
        else qw = sqrt(1. - norm);
        // ------ end adapted implementation ------ 

        // Implementation adapted from: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // roll (x-axis rotation)
        float sinr_cosp = 2 * (qw * qx + qy * qz);
        float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        orientation[0] = atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        float sinp = sqrt(1 + 2 * (qw * qy - qx * qz));
        float cosp = sqrt(1 - 2 * (qw * qy - qx * qz));
        orientation[1] = 2 * atan2(sinp, cosp) - M_PI / 2;

        // yaw (z-axis rotation)
        float siny_cosp = 2 * (qw * qz + qx * qy);
        float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        orientation[2] = atan2(siny_cosp, cosy_cosp);
        // ------ end adapted implementation ------ 
    }


}