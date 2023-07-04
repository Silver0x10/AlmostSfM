#include "utils.h"

namespace pr {

    Camera::Camera(int id, float gt_position[3], float gt_orientation[3], float position[3], float orientation[3]){
        this->id = id;
        this->gt_position = Eigen::Map<Vec3f>(gt_position, 3, 1);
        this->gt_orientation = Eigen::Map<Vec3f>(gt_orientation, 3, 1);
        this->position = Eigen::Map<Vec3f>(position, 3, 1);
        this->orientation = Eigen::Map<Vec3f>(orientation, 3, 1);
    }
    

    Keypoint::Keypoint(int id, float direction_vector[3]) {
        this->id = id;
        this->direction_vector = Eigen::Map<Vec3f>(direction_vector, 3, 1);
    }

    /* Given the imaginery part of a unit quaternion, reconstructs the whole quaternion and transforms it 
        in an Euler angles representation (Roll-Pitch-Yaw) .
    */
    void quaternion_to_RPY(float (&orientation)[3]){
        float qw, qx = orientation[0], qy = orientation[1], qz = orientation[2];

        // Implementation adapted from: see README_SFM1B.md
        float norm = sqrt(pow(qx, 2) + pow(qy, 2) + pow(qz, 2));
        if(norm > 1) qw = 1, qx = qy = qz = 0;
        else qw = sqrt(1. - norm);
        // ------ end adapted implementation ------ 

        // Implementation adapted from: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // roll (x-axis rotation)
        float sinr_cosp = 2 * (qw * qx + qy * qz);
        float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        orientation[0] = atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        float sinp = sqrt(1 + 2 * (qw * qy - qx * qz));
        float cosp = sqrt(1 - 2 * (qw * qy - qx * qz));
        orientation[1] = 2 * atan2(sinp, cosp) - M_PI / 2;

        // yaw (z-axis rotation)
        float siny_cosp = 2 * (qw * qz + qx * qy);
        float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        orientation[2] = atan2(siny_cosp, cosy_cosp);
        // ------ end adapted implementation ------ 

        for(auto& o: orientation) o = round(o * 100000.0) / 100000.0;
    }

    // Implementation adapted from: https://gitlab.com/grisetti/probabilistic_robotics_2022_23/-/blob/main/source/cpp/24_projective_icp/src/defs.h

    Matrix3f Rx(float rot_x){
        double c=cos(rot_x);
        double s=sin(rot_x);
        Matrix3f R;
        R << 1,  0, 0,
        0,  c,  -s,
        0,  s,  c;
        return R;
    }

    Matrix3f Rx_prime(float rot_x){
        double c=cos(rot_x);
        double s=sin(rot_x);
        Matrix3f R;
        R << 1,  0, 0,
        0,  -s,  -c,
        0,  c,  -s;
        return R;
    }
    
    Matrix3f Ry(float rot_y){
        double c=cos(rot_y);
        double s=sin(rot_y);
        Matrix3f R;
        R << c,  0,  s,
        0 , 1,  0,
        -s,  0, c;
        return R;
    }

    Matrix3f Ry_prime(float rot_y){
        double c=cos(rot_y);
        double s=sin(rot_y);
        Matrix3f R;
        R << -s,  0,  c,
        0 , 1,  0,
        -c,  0, -s;
        return R;
    }

    Matrix3f Rz(float rot_z){
        double c=cos(rot_z);
        double s=sin(rot_z);
        Matrix3f R;
        R << c,  -s,  0,
        s,  c,  0,
        0,  0,  1;
        return R;
    }

    Matrix3f Rz_prime(float rot_z){
        double c=cos(rot_z);
        double s=sin(rot_z);
        Matrix3f R;
        R << -s,  -c,  0,
        c,  -s,  0,
        0,  0,  1;
        return R;
    }

    Matrix3f v2tRPY(const Vec3f& v){
        Matrix3f T = Rx(v[0])*Ry(v[1])*Rz(v[2]);
        return T;
    }

    // ------ end adapted implementation ------ 

    Vec3f tRPY2v(const Matrix3f& rot) {
        float beta = atan2( -rot(2,0), sqrt(pow(rot(2,1), 2) + pow(rot(2,2), 2)) );
        float alpha = atan2( rot(2,1)/cos(beta) , rot(2,2)/cos(beta) );
        float gamma = atan2( rot(1,0)/cos(beta) , rot(0,0)/cos(beta) );
        Vec3f v;
        v << alpha, beta, gamma;
        return v;
    }

    vector<Camera> load_data(string dataset_path) {
        vector<Camera> cameras;
        
        ifstream data_stream(dataset_path);
        string line, line_type;

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
        return cameras;
    }

    void save_landmarks(const map<int, pr::Vec3f>& landmarks, string output_path){
        ofstream file_stream(output_path);
        for(const auto& l: landmarks) file_stream << "L: " << l.first << "\t" << l.second.transpose() << endl;
        file_stream.close();
    }

    void save_camera_positions(const vector<Camera>& cameras, string output_path){
        ofstream file_stream(output_path);
        for(const auto& cam: cameras) file_stream << "C: " << cam.id << "\t" << cam.position.transpose() << endl;
        file_stream.close();
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

    Sim3::Sim3() {
        this->scale = 1;
        this->rotation.setZero();
        this->translation.setZero();
    }

    Sim3::Sim3(float scale, Vec3f rotation, Vec3f translation) {
        this->scale = scale;
        this->rotation = rotation;
        this->translation = translation;
    }

    Vec3f Sim3::operator*(Vec3f v) {
        return this->scale * (v2tRPY(this->rotation) * v + this->translation);
    }

    void Sim3::perturb(Vec3f d_translation, float d_scale, Vec3f d_rotation) {
        // this->translation = v2tRPY(d_rotation)*this->translation + d_translation*this->scale;
        this->translation += d_translation;

        this->scale *= exp(d_scale);

        this->rotation = tRPY2v( v2tRPY(d_rotation) * v2tRPY(this->rotation) );
    }

}