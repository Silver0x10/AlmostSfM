#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "utils.h"
using namespace std;
using namespace pr;

int main (int argc, char** argv) {
    ifstream data_stream("../dataset_and_info/dataset.txt");
    string line, line_type;

    vector<Camera> cameras;
    Camera* current_cam;

    for(string line; getline(data_stream, line); ) {
        istringstream line_stream(line);
        line_stream >> line_type;

        if(!line_type.compare("KF:")) { // Camera pose
            int id;
            float gt_position[3], gt_orientation[3], position[3], orientation[3];
            line_stream >> id;
            for(int i=0; i<3; ++i) line_stream >> gt_position[i];
            for(int i=0; i<3; ++i) line_stream >> gt_orientation[i];
            for(int i=0; i<3; ++i) line_stream >> position[i];
            for(int i=0; i<3; ++i) line_stream >> orientation[i];
            
            // quaternion_to_angle HERE

            Camera cam(id, gt_position, gt_orientation, position, orientation);
            cameras.push_back(cam);
            current_cam = &cameras.back();

            // TODO: Remove
            // cout << "KF " << id;
            // cout << "Camera id: " << cam.id << endl;
            // cout << " gt pos: ";
            // for(int i=0; i<3; ++i) cout << " " << cam.gt_position(i, 0);
            // cout << " gt or: ";
            // for(int i=0; i<3; ++i) cout << " " << cam.gt_orientation(i, 0);
            // cout << " pos: ";
            // for(int i=0; i<3; ++i) cout << " " << cam.position(i, 0);
            // cout << " or: ";
            // for(int i=0; i<3; ++i) cout << " " << cam.orientation(i, 0);
            // cout << endl;

        }
        else if(!line_type.compare("F:")) { // Keypoint
            int prog_id, id;
            line_stream >> prog_id >> id;
            
            float direction_vector[3];
            for(int i=0; i<3; ++i) line_stream >> direction_vector[i];

            Keypoint kp(id, direction_vector);
            current_cam->keypoints.push_back(kp);

            // cout << "F " << prog_id << " " << kp.id;
            // for(int i=0; i<3; ++i) cout << " " << kp.direction_vector(i, 0);
            // cout << endl;
        }
        // delete current_cam;


        // break;
    }

    // for(int i=0; i<cameras.size(); ++i ){
    //     auto cam = cameras[i];
    //     cout << cam.id << " kps: " << cam.keypoints.size() << endl;
    // }
    auto test = cameras[5].keypoints[5];
    cout << test.id << endl;
    cout << test.direction_vector << endl;

}