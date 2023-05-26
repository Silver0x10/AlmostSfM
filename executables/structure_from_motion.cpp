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

using namespace std;
using namespace pr;

void save_landmarks(const map<int, pr::Vec3f>& landmarks, string output_path){
    ofstream landmarks_stream(output_path);
    for(const auto& l: landmarks){
        // cout << "L: " << l.first << "\t" << l.second.transpose() << endl;
        landmarks_stream << "L: " << l.first << "\t" << l.second.transpose() << endl;
    }
    landmarks_stream.close();
}

map<int, pr::Vec3f> load_landmarks(string path){
    map<int, pr::Vec3f> landmarks;

    ifstream data_stream(path);
    string line, line_type;
    int id;
    float x, y, z;
    for(string line; getline(data_stream, line); ) {
        istringstream line_stream(line);
        line_stream >> line_type >> id >> x >> y >> z;
        // cout << Vec3f(x,y,z) << endl << endl;
        landmarks.insert({id, Vec3f(x,y,z)});
    }

    return landmarks;
}

void map_visualization(string landmarks_path,  string gt_landmarks_path){
    cv::viz::Viz3d window("Map visualization");
    cv::viz::Viz3d gt_window("GT Map visualization");

    // Estimated Landmarks visualization
    map<int, pr::Vec3f> landmarks = load_landmarks(landmarks_path);
    std::vector<cv::Point3d> landmarks_cv;
    for(const auto& l: landmarks) 
        landmarks_cv.push_back( cv::Point3d(l.second.x(), l.second.y(), l.second.z()) );
    cv::viz::WCloud landmarks_cloud(landmarks_cv, cv::viz::Color::yellow());
    landmarks_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 1.5 );
    window.showWidget("landmarks", landmarks_cloud);
    window.spin();

    // GT Landmarks visualization
    map<int, pr::Vec3f> gt_landmarks = load_landmarks(gt_landmarks_path);
    std::vector<cv::Point3d> gt_landmarks_cv;
    for(const auto& l: gt_landmarks) 
        gt_landmarks_cv.push_back( cv::Point3d(l.second.x(), l.second.y(), l.second.z()) );
    cv::viz::WCloud gt_landmarks_cloud(gt_landmarks_cv, cv::viz::Color::green());
    gt_landmarks_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 1.5 );
    gt_window.showWidget("gt_landmarks", gt_landmarks_cloud);
    gt_window.spin();

}

int main (int argc, char** argv) {
    string dataset_path = argv[1];
    vector<Camera> cameras = load_data(dataset_path);
    cout << "Data loaded from: " << argv[1] << endl << endl;

    cout << "Initialization...";
    init_translations(cameras);
    cout << "\tDONE" << endl;
    
    cout << "Triangulation... ";
    auto landmarks = triangulate(cameras);
    cout << "\tDONE" << endl;
    
    cout << endl << "Bundle Adjustment... " << endl;
    bundle_adjustment(cameras, landmarks);
    cout << "DONE" << endl;

    save_landmarks(landmarks, argv[2]);
    cout << endl << "Landmark positions saved in: " << argv[2] << endl;

    // // Evaluation
    // eval_translations(cameras);

    map_visualization(argv[2], argv[3]);
    // load_landmarks(argv[2]);

    return 0;

}