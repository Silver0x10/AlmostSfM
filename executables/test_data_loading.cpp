#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <vector>
// #include <random>
// #include <cmath>
#include <fstream>
#include "utils.h"


using namespace std;
using namespace pr;

void quaternionToEulerAngles(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();
    pitch = atan2(-rotationMatrix(2, 0), sqrt(pow(rotationMatrix(2,1), 2)+pow(rotationMatrix(2,2), 2)));
    roll  = atan2(rotationMatrix(2, 1)/cos(pitch), rotationMatrix(2, 2)/cos(pitch));
    yaw   = atan2(rotationMatrix(1, 0)/cos(pitch), rotationMatrix(0, 0)/cos(pitch));
}

int main (int argc, char** argv) {
    cout << "Test Data Loading:" << endl;

    string dataset_path = "../../dataset_and_info/dataset.txt"; // argv[1];
    vector<Camera> cameras = load_data(dataset_path);
    std::ofstream output_file("../../check_dataset.txt");
    for(auto cam:cameras) {
        output_file << "KF:\t" << cam.id << "\t" << cam.gt_position.transpose() << "\t" << RPY_to_quaternion(cam.gt_orientation).transpose() << "\t" << cam.position.transpose() << "\t" << RPY_to_quaternion(cam.orientation).transpose() << endl;
        for(int i=0; i<(int)cam.keypoints.size(); i++){
            output_file << "F:\t" << i << "\t" << cam.keypoints[i].id << "\t" << cam.keypoints[i].direction_vector.transpose() << endl;
        }
    }
    output_file.close();

    double quat[4] = {0.706223,  0.706223, 0.0353406, 0};
    double norm = pow(quat[0], 2) + pow(quat[1], 2) + pow(quat[2], 2);
    if(norm > 1) quat[3] = 1, quat[0] = quat[1] = quat[2] = 0;
    else quat[3] = sqrt(1. - norm);
    Eigen::Quaterniond quat_original(quat[3], quat[0], quat[1], quat[2]);
    cout << "quat_original: \t" << quat_original.x() << "\t" << quat_original.y() << "\t" << quat_original.z() << "\t" << quat_original.w() << "\t" << endl << endl;
    
    double temp[3] = {quat[0],  quat[1], quat[2]};
    quaternion_to_RPY(temp);
    Vec3d rpy(temp[0],temp[1],temp[2]);
    cout << "rpy: \t" << rpy.transpose() << endl << endl;

    Eigen::Quaterniond quat_from_rpy;
    quat_from_rpy = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());
    cout << "quat from rpy: \t" << quat_from_rpy.x() << "\t" << quat_from_rpy.y() << "\t" << quat_from_rpy.z() << "\t" << quat_from_rpy.w() << "\t" << endl << endl;

    string check = quat_from_rpy.isApprox(quat_original) ? "OK" : "NOPE";
    cout << "same rotation: " << check << endl;

    auto R = v2tRPY(rpy);
    cout << "R: " << endl << R << endl << endl;
    auto R_quat = quat_original.toRotationMatrix();
    cout << "R_quat:" <<endl << R_quat << endl << endl;

    cout << "identity if all ok:" << endl << R*R_quat.transpose() << endl << endl;


    return 0;
}