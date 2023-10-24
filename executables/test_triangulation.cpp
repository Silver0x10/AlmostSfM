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

void visualize(const map<int, pr::Vec3f>& landmarks, const map<int, pr::Vec3f>& gt_landmarks){
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

    window.spin();
}

int main (int argc, char** argv) {
    cout << "Triangulation Test" << endl;

    string dataset_path = argv[1];
    vector<Camera> cameras = load_data(dataset_path);
    
    string gt_landmark_positions = argv[2];
    map<int, pr::Vec3f> gt_landmarks = load_landmarks(gt_landmark_positions);

    for(auto& cam: cameras) {
        cam.position = cam.gt_position;
        cam.orientation = cam.gt_orientation;
        for(auto& kp: cam.keypoints) {
            kp.direction_vector = gt_landmarks[kp.id] - cam.position;
            kp.direction_vector.normalize();
        }
    }

    map<int, pr::Vec3f> landmarks = triangulate(cameras);
    
    float error = rmse(landmarks, gt_landmarks);
    cout << "RMSE: " << error << endl;

    for(auto& l: landmarks) l.second = 0.985 * l.second; // just to make the visualization clearer
    visualize(landmarks, gt_landmarks);

    return 0;
}