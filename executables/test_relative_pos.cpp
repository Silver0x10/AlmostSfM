#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "utils.h"
#include <random>
#include <cmath>
#include "relative_position_calculator.h"


using namespace std;
using namespace pr;
using namespace Eigen;


int main (int argc, char** argv) {
    cout << "Test relative position estimation:" << endl;

    string dataset_path = argv[1]; // "../../dataset_and_info/dataset.txt";
    vector<Camera> cameras = load_data(dataset_path);
    string gt_landmark_positions = argv[2]; // "../../dataset_and_info/GT_landmarks.txt";
    map<int, pr::Vec3f> gt_landmarks = load_landmarks(gt_landmark_positions);

    for(auto& cam: cameras) {
        cam.position = cam.gt_position;
        cam.orientation = cam.gt_orientation;
        for(auto& kp: cam.keypoints) {
            kp.direction_vector = gt_landmarks[kp.id] - cam.position;
            kp.direction_vector.normalize();
        }
    }

    int id_cam0 = 7;
    int id_cam1 = 42;
    auto cam0 = cameras[id_cam0];
    auto cam1 = cameras[id_cam1];
    auto t_ij = calculate_relative_position(cam0, cam1);

    cout << "Cam " << id_cam0 << " position: \t" << cam0.gt_position.transpose() << endl;
    cout << "Cam " << id_cam1 << " position: \t" << cam1.gt_position.transpose() << endl;
    
    auto gt_t_ij =  cam1.gt_position - cam0.gt_position;
    cout << "GT Relative position: \t" << gt_t_ij.transpose() << endl;
    cout << "PR Relative position: \t" << t_ij.transpose() << endl;
    float scale = 0.0;
    for(int i=0; i<(int)gt_t_ij.size(); i++) scale += (gt_t_ij[i] / t_ij[i])*1/gt_t_ij.size();
    t_ij *= scale;
    cout << "PR after scaling: \t" << t_ij.transpose() << endl;
    cout << "Scale: \t" << scale << endl;

    vector<cv::Vec3f> corr0;
    vector<cv::Vec3f> corr1;
    find_correspondences(cam0, cam1, corr0, corr1);

    Essential_Matrix structE = eight_point_algorithm(corr0, corr1);
    cout << endl << "Essential matrix: " << endl << structE.e << endl;

    // Check epipolar constaint
    float check = 0;
    for(int i=0; i<(int)corr0.size(); i++) {
        Vec3f d0; cv2eigen(corr0[i], d0);
        Vec3f d1; cv2eigen(corr1[i], d1);
        // float err = d0.transpose() * structE.e * d1;
        float err = d1.transpose() * structE.e * d0;
        check += err*err;
    }
    check = sqrt(check / corr0.size());
    cout << endl << "RMSE epipolar constaint: \t" << check << endl;

    return 0;
}