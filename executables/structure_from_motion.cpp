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
#include "sicp_3d.h"

using namespace std;
using namespace pr;

void keyboardCallback(const cv::viz::KeyboardEvent& event, void* data) {
    if (event.action == cv::viz::KeyboardEvent::KEY_DOWN) {
        if (event.code == 'g') {
            cv::viz::Viz3d* window = static_cast<cv::viz::Viz3d*>(data);
            double current = window->getWidget("cameras").getRenderingProperty(cv::viz::OPACITY);
            window->getWidget("cameras").setRenderingProperty(cv::viz::OPACITY, 1-current);
        }
        if (event.code == 'G') {
            cv::viz::Viz3d* window = static_cast<cv::viz::Viz3d*>(data);
            double current = window->getWidget("gt_cameras").getRenderingProperty(cv::viz::OPACITY);
            window->getWidget("gt_cameras").setRenderingProperty(cv::viz::OPACITY, 1-current);
        }
        if (event.code == 'l') {
            cv::viz::Viz3d* window = static_cast<cv::viz::Viz3d*>(data);
            double current = window->getWidget("landmarks").getRenderingProperty(cv::viz::OPACITY);
            window->getWidget("landmarks").setRenderingProperty(cv::viz::OPACITY, 1-current);
        }
        if (event.code == 'L') {
            cv::viz::Viz3d* window = static_cast<cv::viz::Viz3d*>(data);
            double current = window->getWidget("gt_landmarks").getRenderingProperty(cv::viz::OPACITY);
            window->getWidget("gt_landmarks").setRenderingProperty(cv::viz::OPACITY, 1-current);
        }
        if (event.code == 'z') {
            cv::viz::Viz3d* window = static_cast<cv::viz::Viz3d*>(data);
            double current = window->getWidget("coordinate_system").getRenderingProperty(cv::viz::OPACITY);
            window->getWidget("coordinate_system").setRenderingProperty(cv::viz::OPACITY, 1-current);
        }
    }
}

void visualize(const vector<Camera>& cameras, const map<int, pr::Vec3d>& landmarks, const map<int, pr::Vec3d>& gt_landmarks){
    cv::viz::Viz3d window("Map visualization");

    // Estimated Landmarks visualization (RED)
    std::vector<cv::Point3d> landmarks_cv;
    for(const auto& l: landmarks) 
        landmarks_cv.push_back( cv::Point3d(l.second.x(), l.second.y(), l.second.z()) );
    cv::viz::WCloud landmarks_cloud(landmarks_cv, cv::viz::Color::red());
    landmarks_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 7 );
    window.showWidget("landmarks", landmarks_cloud);

    // Estimated Cameras visualization (BLUE)
    std::vector<cv::Point3d> cameras_cv;
    for(const auto& c: cameras) 
        cameras_cv.push_back( cv::Point3d( c.position.x(), c.position.y(), c.position.z()) );
    cv::viz::WCloud cameras_cloud(cameras_cv, cv::viz::Color::blue());
    cameras_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 7 );
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

    window.registerKeyboardCallback(keyboardCallback, &window);

    window.showWidget("coordinate_system", cv::viz::WCoordinateSystem(0.1));
    window.spin();
}

int main (int argc, char** argv) {
    string dataset_path = argv[1]; // "../../dataset_and_info/dataset.txt";
    string gt_landmark_positions = argv[2]; // "../../dataset_and_info/GT_landmarks.txt";
    string output_dir = argv[3]; // "../../out/"
    int ba_rounds = stoi(argv[4]); // 5
    int sicp_rounds = stoi(argv[5]); // 200
    string out_camera_positions = output_dir + "/cameras.txt";
    string out_landmark_positions = output_dir + "/landmarks.txt";

    vector<Camera> cameras = load_data(dataset_path);
    map<int, pr::Vec3d> gt_landmarks = load_landmarks(gt_landmark_positions);
    
    cout << "0) Initialization...";
    init_translations(cameras);
    cout << "\tDONE" << endl << endl;
    
    cout << "1) Triangulation... ";
    auto landmarks = triangulate(cameras);
    cout << "\tDONE" << endl;
    
    cout << endl << "2) Bundle Adjustment... (" << ba_rounds << " rounds)" << endl;
    bool ba_linear_constraint = true;
    bundle_adjustment(cameras, landmarks, ba_rounds, ba_linear_constraint);
    cout << "DONE" << endl;

    cout << endl << "3) Landmarks Registration... " << endl;
    Sim3 initial_guess = Sim3();
    initial_guess.translation = cameras[0].gt_position-cameras[0].position;
    initial_guess.rotation = tRPY2v(v2tRPY(cameras[0].gt_orientation).transpose()*v2tRPY(cameras[0].orientation));
    Sim3 transform = sicp_3d(landmarks, gt_landmarks, sicp_rounds, initial_guess);
    cout << "sim3: " << endl << transform.as_matrix() << endl << endl;
    cout << "DONE" << endl << endl;

    // Save cameras and landmarks data
    save_camera_poses(cameras, out_camera_positions);
    cout << "Camera poses saved in: " << out_camera_positions << endl;
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