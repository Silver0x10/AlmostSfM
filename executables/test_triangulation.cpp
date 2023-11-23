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

void visualize(const vector<Camera>& cameras, const map<int, pr::Vec3d>& landmarks, const map<int, pr::Vec3d>& gt_landmarks){
    cv::viz::Viz3d window("Triangulation Test");

    // GT Landmarks visualization (GREEN)
    std::vector<cv::Point3d> gt_landmarks_cv;
    for(const auto& l: gt_landmarks)
        gt_landmarks_cv.push_back( cv::Point3d(l.second.x(), l.second.y(), l.second.z()) );
    cv::viz::WCloud gt_landmarks_cloud(gt_landmarks_cv, cv::viz::Color::green());
    gt_landmarks_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 10 );
    window.showWidget("gt_landmarks", gt_landmarks_cloud);

    // Estimated Landmarks visualization (RED)
    std::vector<cv::Point3d> landmarks_cv;
    for(const auto& l: landmarks)
        landmarks_cv.push_back( cv::Point3d(l.second.x(), l.second.y(), l.second.z()) );
    cv::viz::WCloud landmarks_cloud(landmarks_cv, cv::viz::Color::red());
    landmarks_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 7 );
    window.showWidget("landmarks", landmarks_cloud);

    // Cameras(BLUE) and direction vectors(BLUE something) visualization
    std::vector<cv::Point3d> gt_cameras_cv;
    for(const auto& c: cameras){
        gt_cameras_cv.push_back( cv::Point3d( c.position.x(), c.position.y(), c.position.z()) );
        for(const auto& kp: c.keypoints) {
            Vec3d dir_in_world = v2tRPY(c.orientation) * kp.direction_vector;
            cv::viz::WLine line(cv::Point3d(c.position[0], c.position[1], c.position[2]), cv::Point3d(c.position[0]+dir_in_world[0], c.position[1]+dir_in_world[1], c.position[2]+dir_in_world[2]));
            line.setColor(cv::viz::Color::celestial_blue());
            window.showWidget("line" + to_string(c.id) + to_string(kp.id), line);
        }
    }
    cv::viz::WCloud gt_cameras_cloud(gt_cameras_cv, cv::viz::Color::blue());
    gt_cameras_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 10 );
    window.showWidget("gt_cameras", gt_cameras_cloud);

    window.spin();
}

int main (int argc, char** argv) {
    cout << "Triangulation Test" << endl;

    vector<Camera> cameras;
    map<int, pr::Vec3d> gt_landmarks;
    map<int, pr::Vec3d> landmarks;
    string dataset_path = "../../dataset_and_info/dataset.txt"; // argv[1];
    string gt_landmark_positions = "../../dataset_and_info/GT_landmarks.txt"; // argv[2];

    if(argv[1] and strcmp(argv[1], "use_GT") == 0) {
        cameras = load_data(dataset_path);
        gt_landmarks = load_landmarks(gt_landmark_positions);
        for(auto& cam: cameras) {
            cam.position = cam.gt_position;
            cam.orientation = cam.gt_orientation;
            for(auto& kp: cam.keypoints) {
                kp.direction_vector = v2tRPY(cam.orientation).transpose() * (gt_landmarks[kp.id] - cam.position); // gt dir vector in camera frame
                kp.direction_vector.normalize();
            }
        }
        landmarks = triangulate(cameras);
    } else {
        gt_landmarks.insert({0, Vec3d(0.5, 0.5, 0.5)});
        
        Camera cam0;
        cam0.id = 0;
        cam0.orientation << 0, M_PI_4, M_PI_4;
        Vec3d dir0 = (v2tRPY(cam0.orientation).transpose() * (gt_landmarks[0] - cam0.position)).normalized(); 
        Keypoint kp0(0, dir0);
        cam0.keypoints.push_back(kp0);
        cameras.push_back(cam0);
        
        Camera cam1;
        cam1.id = 1;
        cam1.position << 1, 0, 0;
        cam1.orientation << 0, 3*M_PI_4, M_PI_4;
        Vec3d dir1 = (v2tRPY(cam1.orientation).transpose() * (gt_landmarks[0] - cam1.position)).normalized(); 
        Keypoint kp1(0, dir1);
        cam1.keypoints.push_back(kp1);
        cameras.push_back(cam1);
        
        Camera cam2;
        cam2.id = 2;
        cam2.position << 0, 0, 1;
        cam2.orientation << 0, -M_PI_4, M_PI_4;
        Vec3d dir2 = (v2tRPY(cam2.orientation).transpose() * (gt_landmarks[0] - cam2.position)).normalized(); 
        Keypoint kp2(0, dir2);
        cam2.keypoints.push_back(kp2);
        cameras.push_back(cam2);

        landmarks = triangulate(cameras);
    }
    
    cout << endl << "Triangulated landmark positions: " << endl;
    for(auto l: landmarks) cout << "id: " << l.first << "\npred: " << l.second.transpose() << "\ngt: " << gt_landmarks[l.first].transpose() << endl;

    double error = rmse(landmarks, gt_landmarks);
    cout << "\nRMSE: " << error << endl;

    visualize(cameras, landmarks, gt_landmarks);

    return 0;
}