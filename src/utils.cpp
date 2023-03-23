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

    // Implementation adapted from: https://gitlab.com/grisetti/probabilistic_robotics_2022_23/-/blob/main/source/cpp/24_projective_icp/src/defs.h

    inline Matrix3f Rx(float rot_x){
        double c=cos(rot_x);
        double s=sin(rot_x);
        Matrix3f R;
        R << 1,  0, 0,
        0,  c,  -s,
        0,  s,  c;
        return R;
    }
    
    inline Matrix3f Ry(float rot_y){
        double c=cos(rot_y);
        double s=sin(rot_y);
        Matrix3f R;
        R << c,  0,  s,
        0 , 1,  0,
        -s,  0, c;
        return R;
    }

    inline Matrix3f Rz(float rot_z){
        double c=cos(rot_z);
        double s=sin(rot_z);
        Matrix3f R;
        R << c,  -s,  0,
        s,  c,  0,
        0,  0,  1;
        return R;
    }

    Matrix3f v2tRPY(const Vec3f& v){
        Matrix3f T = Rx(v[0])*Ry(v[1])*Rz(v[2]);
        return T;
    }

    // ------ end adapted implementation ------ 

}