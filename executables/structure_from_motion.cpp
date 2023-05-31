#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include "utils.h"
#include <opencv2/viz.hpp>
#include "init_translations.h"
#include "relative_position_calculator.h"
#include "triangulation.h"
#include "bundle_adjustment.h"

using namespace std;
using namespace pr;

void map_visualization(string landmarks_path,  string gt_landmarks_path){
    cv::viz::Viz3d window("Map visualization");
    cv::viz::Viz3d gt_window("GT Map visualization");

    // Estimated Landmarks visualization
    map<int, pr::Vec3f> landmarks = load_landmarks(landmarks_path);
    std::vector<cv::Point3d> landmarks_cv;
    for(const auto& l: landmarks) 
        landmarks_cv.push_back( cv::Point3d(l.second.x(), l.second.y(), l.second.z()) );
    cv::viz::WCloud landmarks_cloud(landmarks_cv, cv::viz::Color::yellow());
    landmarks_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 2 );
    window.showWidget("landmarks", landmarks_cloud);
    window.spin();

    // GT Landmarks visualization
    map<int, pr::Vec3f> gt_landmarks = load_landmarks(gt_landmarks_path);
    std::vector<cv::Point3d> gt_landmarks_cv;
    for(const auto& l: gt_landmarks) 
        gt_landmarks_cv.push_back( cv::Point3d(l.second.x(), l.second.y(), l.second.z()) );
    cv::viz::WCloud gt_landmarks_cloud(gt_landmarks_cv, cv::viz::Color::green());
    gt_landmarks_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 2 );
    gt_window.showWidget("gt_landmarks", gt_landmarks_cloud);
    gt_window.spin();

}

int main (int argc, char** argv) {
    string dataset_path = argv[1];
    string out_camera_positions = argv[2];
    string out_landmark_positions = argv[3];
    string gt_landmark_positions = argv[4];

    vector<Camera> cameras = load_data(dataset_path);
    cout << "Data loaded from: " << dataset_path << endl << endl;

    cout << "Initialization...";
    init_translations(cameras);
    cout << "\tDONE" << endl;
    
    cout << "Triangulation... ";
    auto landmarks = triangulate(cameras);
    cout << "\tDONE" << endl;
    
    cout << endl << "Bundle Adjustment... " << endl;
    bundle_adjustment(cameras, landmarks);
    cout << "DONE" << endl;

    save_camera_positions(cameras, out_camera_positions);
    cout << "Camera positions saved in: " << out_camera_positions << endl;

    save_landmarks(landmarks, out_landmark_positions);
    cout << "Landmark positions saved in: " << out_landmark_positions << endl;

    // // Evaluation
    // eval_translations(cameras);


    map_visualization(out_landmark_positions, gt_landmark_positions);

    return 0;

}