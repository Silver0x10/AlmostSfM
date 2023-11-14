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
#include "evaluation.h"
#include "icp_3d.h"

using namespace std;
using namespace pr;

void visualize(const vector<Camera>& cameras, const map<int, pr::Vec3d>& landmarks, const map<int, pr::Vec3d>& gt_landmarks){
    cv::viz::Viz3d window("Map visualization");

    // Estimated Landmarks visualization (RED)
    std::vector<cv::Point3d> landmarks_cv;
    for(const auto& l: landmarks) 
        landmarks_cv.push_back( cv::Point3d(l.second.x(), l.second.y(), l.second.z()) );
    cv::viz::WCloud landmarks_cloud(landmarks_cv, cv::viz::Color::red());
    landmarks_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 5 );
    window.showWidget("landmarks", landmarks_cloud);

    // Estimated Cameras visualization (BLUE)
    std::vector<cv::Point3d> cameras_cv;
    for(const auto& c: cameras) 
        cameras_cv.push_back( cv::Point3d( c.position.x(), c.position.y(), c.position.z()) );
    cv::viz::WCloud cameras_cloud(cameras_cv, cv::viz::Color::blue());
    cameras_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 5 );
    window.showWidget("cameras", cameras_cloud);

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
    gt_cameras_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 5 );
    window.showWidget("gt_cameras", gt_cameras_cloud);

    window.spin();
}

int main (int argc, char** argv) {
    // string dataset_path = argv[1]; // "../../dataset_and_info/dataset.txt";
    string dataset_path = "../../dataset_and_info/dataset.txt";
    // string gt_landmark_positions = argv[2]; // "../../dataset_and_info/GT_landmarks.txt";
    string gt_landmark_positions = "../../dataset_and_info/GT_landmarks.txt";
    // string output_dir = argv[3]; // out_landmark_positions.substr(0, out_camera_positions.rfind('/'));
    string output_dir = "../../out/";
    int ba_rounds = 5;
    // int ba_rounds = stoi(argv[4]);
    string out_camera_positions = output_dir + "/cameras.txt";
    string out_landmark_positions = output_dir + "/landmarks.txt";

    vector<Camera> cameras = load_data(dataset_path);
    map<int, pr::Vec3d> gt_landmarks = load_landmarks(gt_landmark_positions);
    
    // For testing without noise
    for(auto& cam: cameras) {
        // cam.orientation = cam.gt_orientation;
        for(auto& kp: cam.keypoints) {
            kp.direction_vector = v2tRPY(cam.orientation).transpose() * (gt_landmarks[kp.id] - cam.gt_position); // gt dir vector in camera frame
            kp.direction_vector.normalize();
        }
        // vector<Keypoint> good_keypoints;
        // for(auto& kp: cam.keypoints) {
        //     Vec3d dir = v2tRPY(cam.orientation).transpose() * (gt_landmarks[kp.id] - cam.gt_position); // gt dir vector in camera frame
        //     dir.normalize();
        //     double error = (skew(dir) * kp.direction_vector).transpose() * (skew(dir) * kp.direction_vector);
        //     // cout << kp.id << ": " << error << endl;
        //     if(error < 0.9) good_keypoints.push_back(kp);
        // }
        // cam.keypoints = good_keypoints;
    }

    cout << "0) Initialization...";
    init_translations(cameras);
    cout << "\tDONE" << endl << endl;
    
    cout << "1) Triangulation... ";
    auto landmarks = triangulate(cameras);
    cout << "\tDONE" << endl;
    
    cout << endl << "2) Bundle Adjustment... (" << ba_rounds << " rounds)" << endl;
    bundle_adjustment(cameras, landmarks, ba_rounds);
    cout << "DONE" << endl;

    cout << endl << "3) Landmarks Registration... " << endl;
    double scaling = 10;
    map<int, pr::Vec3d> positions;
    map<int, pr::Vec3d> gt_positions;
    for(auto& cam: cameras) {
        cam.position *= scaling;
        cam.gt_position *= scaling;
        positions.insert({cam.id, cam.position});
        gt_positions.insert({cam.id, cam.gt_position});
    }
    for(auto& gt_l: gt_landmarks) {
        gt_l.second *= scaling;
        gt_positions.insert(gt_l);
    }
    for(auto& l: landmarks) {
        l.second *= scaling;
        positions.insert(l); 
    }
    Sim3 transform = sicp_3d(positions, gt_positions, cameras[0].gt_position, 10);
    // Sim3 transform = sicp_3d(landmarks, gt_landmarks, cameras[0].gt_position, 10);
    cout << "sim3: " << endl << transform.as_matrix() << endl << endl;
    cout << "DONE" << endl << endl;

    // Save cameras and landmarks data
    save_camera_positions(cameras, out_camera_positions);
    cout << "Camera positions saved in: " << out_camera_positions << endl;
    save_landmarks(landmarks, out_landmark_positions);
    cout << "Landmark positions saved in: " << out_landmark_positions << endl << endl;

    cout << "4) Evaluation... " << endl;
    evaluation(cameras, landmarks, gt_landmarks, transform, output_dir);
    cout << "DONE" << endl;

    for(auto& l: gt_landmarks) l.second = transform * l.second;
    for(auto& c: cameras) c.gt_position = transform * c.gt_position;
    visualize(cameras, landmarks, gt_landmarks);

    return 0;

}