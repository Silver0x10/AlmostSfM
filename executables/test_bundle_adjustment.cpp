#include <iostream>
#include <string>
#include <vector>
#include <opencv2/viz.hpp>
#include "utils.h"
#include "bundle_adjustment.h"
#include "triangulation.h"
#include "icp_3d.h"

using namespace std;
using namespace pr;

void visualize(const vector<Camera>& cameras, const map<int, pr::Vec3d>& landmarks, const map<int, pr::Vec3d>& gt_landmarks){
    cv::viz::Viz3d window("Bundle Adjustment Visualization");

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
    gt_landmarks_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 3 );
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
    cout << "Test BundleAdjustment" << endl;

    string dataset_path = "../../dataset_and_info/input_BA.txt";
    // string dataset_path = argv[1];
    vector<Camera> cameras = load_data(dataset_path);

    string gt_landmark_positions = "../../dataset_and_info/GT_landmarks.txt";
    // string gt_landmark_positions = argv[2];
    map<int, pr::Vec3d> gt_landmarks = load_landmarks(gt_landmark_positions);

    // test using gt data --> no changes
    for(auto& c: cameras) {
        c.position = c.gt_position;
        c.orientation = c.gt_orientation;
        for(auto& kp: c.keypoints) {
            kp.direction_vector = v2tRPY(c.orientation).transpose() * (gt_landmarks[kp.id] - c.gt_position); // gt dir vector in camera frame
            kp.direction_vector.normalize();
        }
    }

    auto landmarks = triangulate(cameras);
    // cout << "\tRMSE: " << rmse(landmarks, gt_landmarks) << endl << endl;
    map<int, pr::Vec3d> landmarks_before_BA; for(auto l: landmarks) landmarks_before_BA.insert(l);
    
    int ba_iterations = 5;
    bundle_adjustment(cameras, landmarks, ba_iterations);

    bool compare_to_GT = true;
    if (compare_to_GT) {
        // int icp_iterations = 5;
        // Sim3 estimated_transform = sicp_3d(landmarks, gt_landmarks, cameras[0].gt_position, icp_iterations);
        // cout << "\nSim3: \n" << estimated_transform.as_matrix() << endl; 
        // for(auto& gt_l: gt_landmarks) gt_l.second = estimated_transform * gt_l.second;
        // for(auto& cam: cameras) cam.gt_position = estimated_transform * cam.gt_position;
        visualize(cameras, landmarks, gt_landmarks);
    } else { // compare wrt original estimated values
        vector<Camera> cameras_before_BA = load_data(dataset_path);
        for(auto& cam: cameras) {
            cam.gt_position = cameras_before_BA[cam.id].position;
            cam.gt_orientation = cameras_before_BA[cam.id].orientation;
        }
        visualize(cameras, landmarks, landmarks_before_BA);
    }


    return 0;

}