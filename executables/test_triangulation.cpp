#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <random>
#include <cmath>
#include <opencv2/viz.hpp>
#include "utils.h"
#include "triangulation.h"

using namespace std;
using namespace pr;
using namespace Eigen;

void visualize(const vector<Camera>& cameras, const vector<Camera>& cams, const map<int, pr::Vec3d>& landmarks, const map<int, pr::Vec3d>& gt_landmarks){
    cv::viz::Viz3d window("Triangulation Test");

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
        gt_cameras_cv.push_back( cv::Point3d( c.position.x(), c.position.y(), c.position.z()) );
    cv::viz::WCloud gt_cameras_cloud(gt_cameras_cv, cv::viz::Color::violet());
    gt_cameras_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 5 );
    window.showWidget("gt_cameras", gt_cameras_cloud);

    // Used Cameras visualization (Blue)
    std::vector<cv::Point3d> cameras_cv;
    for(const auto& c: cams) 
        cameras_cv.push_back( cv::Point3d( c.position.x(), c.position.y(), c.position.z()) );
    cv::viz::WCloud cameras_cloud(cameras_cv, cv::viz::Color::blue());
    cameras_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 10 );
    window.showWidget("cameras", cameras_cloud);

    window.spin();
}

int main (int argc, char** argv) {
    cout << "Triangulation Test" << endl;

    string dataset_path = "../../dataset_and_info/dataset.txt"; // argv[1];
    vector<Camera> cameras = load_data(dataset_path);
    
    string gt_landmark_positions = "../../dataset_and_info/GT_landmarks.txt"; // argv[2];
    map<int, pr::Vec3d> gt_landmarks = load_landmarks(gt_landmark_positions);

    for(auto& cam: cameras) {
        cout << "##############################################################################" << endl;
        cam.position = cam.gt_position;
        cam.orientation = cam.gt_orientation;
        cout << "id: " << cam.id << endl;
        cout << "pos: " << cam.position.transpose() << endl << endl;

        for(auto& kp: cam.keypoints) {
            cout << kp.id << ": \t" << (v2tRPY(cam.orientation).transpose() * gt_landmarks[kp.id]).transpose() << endl;
            cout << "dir: " << kp.direction_vector.transpose() << endl;
            kp.direction_vector = v2tRPY(cam.orientation).transpose() * (gt_landmarks[kp.id] - cam.position); // gt dir vector in camera frame
            kp.direction_vector.normalize();
            cout << "gtd: " << kp.direction_vector.transpose() << endl << endl;
        }
    }

    
    float error = rmse(landmarks, gt_landmarks);
    cout << "RMSE: " << error << endl;
    map<int, pr::Vec3d> landmarks = triangulate(cameras);
    // vector<Camera> cams;
    // cams.push_back(cameras[42]);
    // cams.push_back(cameras[7]);
    // cams.push_back(cameras[69]);
    // map<int, pr::Vec3d> landmarks = triangulate(cams);
    // map<int, pr::Vec3d> gt_lands;
    // for(auto l: landmarks) gt_lands.insert({l.first, gt_landmarks[l.first]});


    // double error = rmse(landmarks, gt_landmarks);
    // cout << "RMSE: " << error << endl;

    for(auto& l: landmarks) l.second = 0.985 * l.second; // just to make the visualization clearer
    visualize(cameras, cameras, landmarks, gt_landmarks);
    // visualize(cameras, cams, landmarks, gt_lands);

    return 0;
}