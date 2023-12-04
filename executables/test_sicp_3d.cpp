#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <opencv2/viz.hpp>
#include "utils.h"
#include "sicp_3d.h"

using namespace std;
using namespace pr;
using namespace Eigen;

void visualize(const map<int, pr::Vec3d>& landmarks, const map<int, pr::Vec3d>& gt_landmarks){
    cv::viz::Viz3d window("SICP 3D Test");

    // Estimated Landmarks visualization (RED)
    std::vector<cv::Point3d> landmarks_cv;
    for(const auto& l: landmarks) 
        landmarks_cv.push_back( cv::Point3d(l.second.x(), l.second.y(), l.second.z()) );
    cv::viz::WCloud landmarks_cloud(landmarks_cv, cv::viz::Color::red());
    landmarks_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 7 );
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
    cout << "Test SICP 3D:" << endl;

    map<int, pr::Vec3d> gt_landmarks;
    for(int i=0; i<100; i++)
        gt_landmarks.insert({i, (Vec3d::Random() - Vec3d::Ones()*0.5)*10});

    double scale, inv_scaling;
    Vec3d rotation; 
    Vec3d translation; 
    
    inv_scaling = 0.5;
    scale = log(inv_scaling);
    rotation << -M_PI/2, M_PI/2, -M_PI;
    translation << 10, 0.2, 0.5;
    Sim3 transform = Sim3(scale, rotation, translation);

    map<int, pr::Vec3d> landmarks;
    for(auto const& gt_l: gt_landmarks){
        Vec3d transformed_gt_l = transform * gt_l.second;
        landmarks.insert({gt_l.first, transformed_gt_l});
    }

    // Sim3 initial_guess = Sim3();
    Sim3 initial_guess = transform;
    initial_guess.translation += Vec3d(-4,0.7,-0.7);
    initial_guess.translation += Vec3d::Ones()*0.4;
    initial_guess.scale += 0.1;

    int icp_max_iterations = 100;
    // Sim3 estimated_transform = sicp_3d(landmarks, gt_landmarks, icp_max_iterations);
    Sim3 estimated_transform = sicp_3d(landmarks, gt_landmarks, icp_max_iterations, initial_guess);
    cout << endl << "s_hat: \t" << estimated_transform.scale << " \t \t \t \ts_gt: \t" << exp(scale) << endl;
    cout << "R_hat: \t" << estimated_transform.rotation.transpose() << "\t \t \tR_gt: \t" << rotation.transpose() << endl;
    cout << "t_hat: \t" << estimated_transform.translation.transpose() << " \t \tt_gt: \t" << translation.transpose() << endl;

    for(auto& gt_l: gt_landmarks) gt_l.second = estimated_transform * gt_l.second;
    visualize(landmarks, gt_landmarks);

    return 0;

}