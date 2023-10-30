#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <opencv2/viz.hpp>
#include "utils.h"
#include "relative_position_calculator.h"
#include "init_translations.h"


using namespace std;
using namespace pr;
// using namespace Eigen;

void visualize(const vector<Camera>& cameras){
    cv::viz::Viz3d window("Translations Initialization Test");

    // Estimated Cameras visualization (BLUE)
    std::vector<cv::Point3d> cameras_cv;
    for(const auto& c: cameras){
        cameras_cv.push_back( cv::Point3d( c.position.x(), c.position.y(), c.position.z()) );
    }
    // cv::viz::WCloud cameras_cloud(cameras_cv, cv::viz::Color::red());
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

    string dataset_path = "../../dataset_and_info/dataset.txt";
    // string dataset_path = argv[1]; // "../../dataset_and_info/dataset.txt";
    vector<Camera> cameras = load_data(dataset_path);
    string gt_landmark_positions = "../../dataset_and_info/GT_landmarks.txt";
    // string gt_landmark_positions = argv[2]; // "../../dataset_and_info/GT_landmarks.txt";
    map<int, pr::Vec3f> gt_landmarks = load_landmarks(gt_landmark_positions);

    init_translations(cameras);
    Vec3f delta; delta << 0.1, 0.1, 0.1;
    for(int i=0; i < (int)cameras.size(); i++){
        cameras[i].position = cameras[i].position * 70 + cameras[0].gt_position - delta;
    }

    visualize(cameras);

    return 0;
}