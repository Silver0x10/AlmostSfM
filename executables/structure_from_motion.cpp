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
            
            quaternion_to_RPY(gt_orientation);
            quaternion_to_RPY(orientation);

            Camera cam(id, gt_position, gt_orientation, position, orientation);
            cameras.push_back(cam);
            current_cam = &cameras.back();

        }
        else if(!line_type.compare("F:")) { // Keypoint
            int prog_id, id;
            line_stream >> prog_id >> id;
            
            float direction_vector[3];
            for(int i=0; i<3; ++i) line_stream >> direction_vector[i];

            Keypoint kp(id, direction_vector);
            current_cam->keypoints.push_back(kp);
        }
        
    }

    
    // auto test = cameras[5].keypoints[5];
    // cout << test.id << endl;
    // cout << test.direction_vector << endl;

}