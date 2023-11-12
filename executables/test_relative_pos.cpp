#include <iostream>
#include <string>
#include <vector>
// #include <Eigen/Dense>
#include "utils.h"
#include <random>
#include <cmath>
#include "relative_position_calculator.h"

using namespace std;
using namespace pr;

int main (int argc, char** argv) {
    cout << "Test relative position estimation:" << endl;

    string dataset_path = argv[1]; // "../../dataset_and_info/dataset.txt";
    // string dataset_path = "../../dataset_and_info/dataset.txt";
    vector<Camera> cameras = load_data(dataset_path);
    string gt_landmark_positions = argv[2]; // "../../dataset_and_info/GT_landmarks.txt";
    // string gt_landmark_positions = "../../dataset_and_info/GT_landmarks.txt";
    map<int, pr::Vec3f> gt_landmarks = load_landmarks(gt_landmark_positions);
    
    for(auto& cam: cameras) {
        cam.orientation = cam.gt_orientation;
        for(auto& kp: cam.keypoints) {
            kp.direction_vector = v2tRPY(cam.orientation).transpose() * (gt_landmarks[kp.id] - cam.gt_position); // gt dir vector in camera frame
            kp.direction_vector.normalize();
        }
    }

    int counter = 0;
    int counter_ok = 0;
    int counter_neg = 0;
    int counter_wrong = 0;

    // vector<int> cams_ids = {42, 16};
    // for(int i: cams_ids){
    //     for(int j: cams_ids){
    for(int i=0; i<(int)(cameras.size()); ++i){
        for(int j=0; j<(int)(cameras.size()); ++j){
            if(i == j) continue;
            auto cam_i = cameras[i];
            auto cam_j = cameras[j];
            auto t_ij = calculate_relative_position(cam_i, cam_j);

            auto gt_t_ij =  v2tRPY(cam_i.orientation).transpose()*(cam_j.gt_position - cam_i.gt_position);
            double scale = abs(gt_t_ij[0] / t_ij[0]);
            auto t_ij_scaled = t_ij * scale;
            
            cout << "Cam " << i << " position: \t" << cam_i.gt_position.transpose() << endl;
            cout << "Cam " << j << " position: \t" << cam_j.gt_position.transpose() << endl << endl;

            cout << "GT Relative position: \t" << gt_t_ij.transpose() << endl;
            cout << "PR Relative position: \t" << t_ij.transpose() << endl;
            cout << "PR after scaling: \t" << t_ij_scaled.transpose() << endl;
            cout << "Scale: \t" << scale << endl;

            double delta = 1e-03;
            if(abs( (gt_t_ij - t_ij_scaled).norm() ) <= delta) {
                counter_ok++;
                cout << "OK" << endl;
            } else if (abs( (gt_t_ij + t_ij_scaled).norm() ) <= delta) {
                counter_neg++;
                cout << "OPPOSITE" << endl;
            } else {
                counter_wrong++;
                cout << "NOPE SADLY" << endl;
            }

            vector<cv::Vec3f> corr0;
            vector<cv::Vec3f> corr1;
            find_correspondences(cam_i, cam_j, corr0, corr1);

            Essential_Matrix structE = eight_point_algorithm(corr0, corr1);
            cout << endl << "Essential matrix: " << endl << structE.e << endl;
            // Check epipolar constaint
            float check = 0;
            for(int i=0; i<(int)corr0.size(); i++) {
                Vec3f d0; cv2eigen(corr0[i], d0);
                Vec3f d1; cv2eigen(corr1[i], d1);
                float err = d1.transpose() * structE.e * d0;
                check += err*err;
            }
            check = sqrt(check / corr0.size());
            cout << endl << "RMSE epipolar constaint: \t" << check << endl;
            
            cout << endl << "########################" << endl;
            counter++;
        }
    }

    cout << endl << "CORRECT relative position estimations: \t" << counter_ok << "/" << counter << endl;
    cout << "OPPOSITE relative position estimations: \t" << counter_neg << "/" << counter << endl;
    // cout << "WRONG relative position estimations: \t" << counter_wrong << "/" << counter << endl;
    
    return 0;
}