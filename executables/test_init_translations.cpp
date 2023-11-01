#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <opencv2/viz.hpp>
#include "utils.h"
#include "relative_position_calculator.h"
#include "icp_3d.h"
#include "init_translations.h"


using namespace std;
using namespace pr;

void visualize(const vector<Camera>& cameras){
    cv::viz::Viz3d window("Translations Initialization Test");

    // Estimated Cameras visualization (BLUE)
    std::vector<cv::Point3d> cameras_cv;
    for(const auto& c: cameras)
        cameras_cv.push_back( cv::Point3d( c.position.x(), c.position.y(), c.position.z()) );
    cv::viz::WCloud cameras_cloud(cameras_cv, cv::viz::Color::blue());
    cameras_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 5 );
    window.showWidget("cameras", cameras_cloud);

    // GT Cameras visualization (VIOLET)
    std::vector<cv::Point3d> gt_cameras_cv;
    for(const auto& c: cameras) 
        gt_cameras_cv.push_back( cv::Point3d( c.gt_position.x(), c.gt_position.y(), c.gt_position.z()) );
    cv::viz::WCloud gt_cameras_cloud(gt_cameras_cv, cv::viz::Color::violet());
    gt_cameras_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 5 );
    window.showWidget("gt_cameras", gt_cameras_cloud);

    window.spin();
}

int main (int argc, char** argv) {
    cout << "Test Translations Initialization:" << endl;

    string dataset_path = argv[1]; // "./dataset_and_info/dataset.txt";
    vector<Camera> cameras = load_data(dataset_path);
    string gt_landmark_positions = argv[2]; // "./dataset_and_info/GT_landmarks.txt";
    map<int, pr::Vec3f> gt_landmarks = load_landmarks(gt_landmark_positions);

    init_translations(cameras);
    
    cout << endl << "Doing ICP for camera positions..." << endl;
    map<int, Vec3f> cam_positions;
    map<int, Vec3f> cam_gt_positions;
    for(const auto& cam: cameras) {
        cam_positions.insert({cam.id, cam.position});
        cam_gt_positions.insert({cam.id, cam.gt_position});
    }
    auto transform = icp_3d(cam_positions, cam_gt_positions, 10);
    for(auto& cam: cameras) cam.gt_position = transform * cam.gt_position;

    visualize(cameras);

    return 0;
}