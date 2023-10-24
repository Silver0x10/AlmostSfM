#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <random>
#include <cmath>
#include <opencv2/viz.hpp>
#include "utils.h"
#include "triangulation.h"
#include "icp_3d.h"

using namespace std;
using namespace pr;
using namespace Eigen;

void visualize(const vector<Camera>& cameras, const map<int, pr::Vec3f>& landmarks, const map<int, pr::Vec3f>& gt_landmarks){
    cv::viz::Viz3d window("Map visualization");

    // Estimated Landmarks visualization (RED)
    std::vector<cv::Point3d> landmarks_cv;
    for(const auto& l: landmarks) 
        landmarks_cv.push_back( cv::Point3d(l.second.x(), l.second.y(), l.second.z()) );
    cv::viz::WCloud landmarks_cloud(landmarks_cv, cv::viz::Color::red());
    landmarks_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 5 );
    window.showWidget("landmarks", landmarks_cloud);

    // GT Landmarks visualization (GREEN)
    std::vector<cv::Point3d> gt_landmarks_cv;
    for(const auto& l: gt_landmarks)
        gt_landmarks_cv.push_back( cv::Point3d(l.second.x(), l.second.y(), l.second.z()) );
    cv::viz::WCloud gt_landmarks_cloud(gt_landmarks_cv, cv::viz::Color::green());
    gt_landmarks_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 5 );
    window.showWidget("gt_landmarks", gt_landmarks_cloud);

    // GT Cameras visualization (VIOLET)
    std::vector<cv::Point3d> gt_cameras_cv;
    for(const auto& c: cameras) 
        gt_cameras_cv.push_back( cv::Point3d( c.gt_position.x(), c.gt_position.y(), c.gt_position.z()) );
    cv::viz::WCloud gt_cameras_cloud(gt_cameras_cv, cv::viz::Color::violet());
    gt_cameras_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 3 );
    window.showWidget("gt_cameras", gt_cameras_cloud);

    window.spin();
}

int main (int argc, char** argv) {
    cout << "Test Triangulation:" << endl;

    string dataset_path = "../../dataset_and_info/dataset.txt";
    vector<Camera> cameras = load_data(dataset_path);

    for(auto& cam: cameras) {
        cam.position = cam.gt_position;
        cam.orientation = cam.gt_orientation;
    }

    string gt_landmark_positions = "../../dataset_and_info/GT_landmarks.txt";
    map<int, pr::Vec3f> gt_landmarks = load_landmarks(gt_landmark_positions);

    map<int, pr::Vec3f> landmarks;
    landmarks = triangulate(cameras);
    
    // visualize(cameras, landmarks, gt_landmarks);
    cout << "l size:    " << landmarks.size() << endl;
    cout << "gt_l size: " << gt_landmarks.size() << endl;

    for(auto l: landmarks){
        cout << l.first << "\t" << gt_landmarks[l.first].transpose() << "\t|\t" << l.second.transpose() << endl;
    }

    return 0;

}