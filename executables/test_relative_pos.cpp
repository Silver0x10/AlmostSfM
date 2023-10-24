#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "utils.h"
#include <random>
#include <cmath>
#include "relative_position_calculator.h"


using namespace std;
using namespace pr;
using namespace Eigen;


int main (int argc, char** argv) {
    cout << "Test relative position estimation:" << endl;

    string dataset_path = "../../dataset_and_info/dataset.txt";
    vector<Camera> cameras = load_data(dataset_path);

    auto cam0 = cameras[0];
    auto cam1 = cameras[2];
    const auto& t_ij = calculate_relative_position(cam0, cam1);

    cout << "Cam1 position: " << cam0.gt_position.transpose() << endl;
    cout << "Cam2 position: " << cam1.gt_position.transpose() << endl;
    cout << "GT Relative position: " << (cam1.gt_position - cam0.gt_position).transpose() << endl;
    cout << "PR Relative position: " << t_ij.transpose() << endl;

    vector<cv::Vec3f> corr0;
    vector<cv::Vec3f> corr1;
    find_correspondences(cam0, cam1, corr0, corr1);

    Essential_Matrix structE = eight_point_algorithm(corr0, corr1);

    cout << endl << "essential: " << endl << structE.e << endl;

    Vec3f d0;
    cv2eigen(corr0[0], d0);
    Vec3f d1;
    cv2eigen(corr1[0], d1);
    auto check = d0.transpose() * structE.e * d1;
    cout << endl << "check epipolar constaint: " << check << endl;

    auto t = extract_t(cam0, cam1, structE);

    cout << endl << "relative position: " << t.transpose() << endl;

    return 0;

}