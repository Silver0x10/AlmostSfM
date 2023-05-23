#include <iostream>
#include <string>
#include <vector>
#include <map>
#include "utils.h"
#include "init_translations.h"
#include "relative_position_calculator.h"
#include "triangulation.h"
#include <opencv2/viz.hpp>

using namespace std;
using namespace pr;


void map_visualization(map<int, pr::Vec3f> estimated_landmarks){
    cv::viz::Viz3d window("Map visualization");
    std::vector<cv::Point3d> landmarks_cv;
    for(const auto& l: estimated_landmarks) 
        landmarks_cv.push_back( cv::Point3d(l.second.x(), l.second.y(), l.second.z()) );
    cv::viz::WCloud landmarks_cloud(landmarks_cv, cv::viz::Color::green());
    landmarks_cloud.setRenderingProperty( cv::viz::POINT_SIZE, 1.5 );
    window.showWidget("landmarks", landmarks_cloud);
    window.spin();
}

int main (int argc, char** argv) {
    string dataset_path = "../../dataset_and_info/dataset.txt";
    vector<Camera> cameras = load_data(dataset_path);


    init_translations(cameras);
    // eval_translations(cameras);
    auto landmarks = triangulate(cameras);
    map_visualization(landmarks);
    

    return 0;

}