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

void map_visualization(map<int, pr::Vec3f> landmarks){
    cv::viz::Viz3d window("Map visualization");
    std::vector<cv::Point3d> landmarks_cv;
    for(const auto& l: landmarks) 
        landmarks_cv.push_back( cv::Point3d(l.second.x(), l.second.y(), l.second.z()) );
    cv::viz::WCloud landmarks_cloud(landmarks_cv, cv::viz::Color::green());
    landmarks_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 1.5 );
    window.showWidget("landmarks", landmarks_cloud);
    window.spin();
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
    
    cout << "Bundle Adjustment... ";
    bundle_adjustment(cameras, landmarks);
    cout << "\tDONE" << endl;

    save_landmarks(landmarks, argv[2]);
    cout << endl << "Landmark positions saved in: " << argv[2] << endl;

    // // Evaluation
    // eval_translations(cameras);

    // map_visualization(landmarks);

    return 0;

}