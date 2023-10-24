#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <opencv2/viz.hpp>
#include "utils.h"
#include "icp_3d.h"

using namespace std;
using namespace pr;
using namespace Eigen;

void visualize(const map<int, pr::Vec3f>& landmarks, const map<int, pr::Vec3f>& gt_landmarks){
    cv::viz::Viz3d window("ICP 3D Test");

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
    cout << "Test ICP 3D:" << endl;

    string dataset_path = argv[1]; // "../../dataset_and_info/dataset.txt";
    vector<Camera> cameras = load_data(dataset_path);


    string gt_landmark_positions = argv[2]; //"../../dataset_and_info/GT_landmarks.txt";
    map<int, pr::Vec3f> gt_landmarks = load_landmarks(gt_landmark_positions);

    float scale;
    Vec3f rotation; 
    Vec3f translation; 

    scale = log(0.5);
    rotation << -M_PI/2, M_PI/2, -M_PI;
    translation << 0.1, 0.2, 0.5;
    Sim3 transform = Sim3(scale, rotation, translation);

    map<int, pr::Vec3f> landmarks;
    for(auto const& gt_l: gt_landmarks){
        Vec3f transformed_gt_l = transform * gt_l.second;
        landmarks.insert({gt_l.first, transformed_gt_l});
    }

    int icp_iterations = 5;
    Sim3 estimated_transform = icp_3d(landmarks, gt_landmarks, icp_iterations);
    cout << endl << "s_hat: \t" << estimated_transform.scale << " \t\t\ts_gt: \t" << exp(scale) << endl;
    cout << "R_hat: \t" << estimated_transform.rotation.transpose() << "\tR_gt: \t" << rotation.transpose() << endl;
    cout << "t_hat: \t" << estimated_transform.translation.transpose() << "\tt_gt: \t" << translation.transpose() << endl;
    
    for(auto& gt_l: gt_landmarks) gt_l.second = estimated_transform * gt_l.second;
    visualize(landmarks, gt_landmarks);

    return 0;

}