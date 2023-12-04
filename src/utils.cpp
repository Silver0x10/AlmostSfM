#include "utils.h"

namespace pr {

    Camera::Camera(int id, double gt_position[3], double gt_orientation[3], double position[3], double orientation[3]){
        this->id = id;
        this->gt_position = Eigen::Map<Vec3d>(gt_position, 3, 1);
        this->gt_orientation = Eigen::Map<Vec3d>(gt_orientation, 3, 1);
        this->position = Eigen::Map<Vec3d>(position, 3, 1);
        this->orientation = Eigen::Map<Vec3d>(orientation, 3, 1);
    }

    Camera::Camera() {
        this->id = -1;
        this->gt_position = Vec3d::Zero();
        this->gt_orientation = Vec3d::Zero();
        this->position = Vec3d::Zero();
        this->orientation = Vec3d::Zero();
    }

    Camera::Camera(const Camera& cam){
        this->id = cam.id;
        this->gt_position = cam.gt_position;
        this->gt_orientation = cam.gt_orientation;
        this->position = cam.position;
        this->orientation = cam.orientation;
        this->keypoints = cam.keypoints;
    }
    

    Keypoint::Keypoint(int id, double direction_vector[3]) {
        this->id = id;
        this->direction_vector = Eigen::Map<Vec3d>(direction_vector, 3, 1);
    }

    Keypoint::Keypoint(int id, Vec3d direction_vector) {
        this->id = id;
        this->direction_vector = direction_vector;
    }

    /* Given the imaginery part of a unit quaternion, reconstructs the whole quaternion and transforms it 
        in an Euler angles representation (Roll-Pitch-Yaw) .
    */
    void quaternion_to_RPY(double (&orientation)[3]){
        double qw, qx = orientation[0], qy = orientation[1], qz = orientation[2];

        // Implementation adapted from: see README_SFM1B.md
        double norm = pow(qx, 2) + pow(qy, 2) + pow(qz, 2);
        if(norm > 1) qw = 1, qx = qy = qz = 0;
        else qw = sqrt(1. - norm);
        // ------ end adapted implementation ------ 

        // Implementation adapted from: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (qw * qx + qy * qz);
        double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        orientation[0] = atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = sqrt(1 + 2 * (qw * qy - qx * qz));
        double cosp = sqrt(1 - 2 * (qw * qy - qx * qz));
        orientation[1] = 2 * atan2(sinp, cosp) - M_PI / 2;

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        orientation[2] = atan2(siny_cosp, cosy_cosp);
        // ------ end adapted implementation ------ 

        // for(auto& o: orientation) o = round(o * 100000.0) / 100000.0;
    }

    Vec3d RPY_to_quaternion(Vec3d rpy) {
        Eigen::Quaterniond quat;
        quat = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());
        return Vec3d(quat.x(), quat.y(), quat.z());
    }

    // Implementation adapted from: https://gitlab.com/grisetti/probabilistic_robotics_2022_23/-/blob/main/source/cpp/24_projective_icp/src/defs.h

    Matrix3d Rx(double rot_x){
        double c=cos(rot_x);
        double s=sin(rot_x);
        Matrix3d R;
        R << 1,  0, 0,
        0,  c,  -s,
        0,  s,  c;
        return R;
    }

    Matrix3d Rx_prime(double rot_x){
        double c=cos(rot_x);
        double s=sin(rot_x);
        Matrix3d R;
        R << 0,  0, 0,
        0,  -s,  -c,
        0,  c,  -s;
        return R;
    }
    
    Matrix3d Ry(double rot_y){
        double c=cos(rot_y);
        double s=sin(rot_y);
        Matrix3d R;
        R << c,  0,  s,
        0 , 1,  0,
        -s,  0, c;
        return R;
    }

    Matrix3d Ry_prime(double rot_y){
        double c=cos(rot_y);
        double s=sin(rot_y);
        Matrix3d R;
        R << -s,  0,  c,
        0 , 0,  0,
        -c,  0, -s;
        return R;
    }

    Matrix3d Rz(double rot_z){
        double c=cos(rot_z);
        double s=sin(rot_z);
        Matrix3d R;
        R << c,  -s,  0,
        s,  c,  0,
        0,  0,  1;
        return R;
    }

    Matrix3d Rz_prime(double rot_z){
        double c=cos(rot_z);
        double s=sin(rot_z);
        Matrix3d R;
        R << -s,  -c,  0,
        c,  -s,  0,
        0,  0,  0;
        return R;
    }

    Matrix3d v2tRPY(const Vec3d& v){
        Matrix3d T = Rz(v[2])*Ry(v[1])*Rx(v[0]);
        // Matrix3d T = Rx(v[0])*Ry(v[1])*Rz(v[2]);
        return T;
    }

    // ------ end adapted implementation ------ 

    // TODO: check correctness
    Vec3d tRPY2v(const Matrix3d& rot) {
        // double alpha = atan2(-rot(1,2), rot(2,2));
        // double beta  = asin(rot(0,2));
        // double gamma = atan2(-rot(0,1), rot(0,0));
        double beta = atan2( -rot(2,0), sqrt(pow(rot(2,1), 2) + pow(rot(2,2), 2)) );
        double alpha = atan2( rot(2,1)/cos(beta) , rot(2,2)/cos(beta) );
        double gamma = atan2( rot(1,0)/cos(beta) , rot(0,0)/cos(beta) );
        Vec3d v;
        v << alpha, beta, gamma;
        return v;
    }

    Matrix4d v2t(Eigen::Matrix<double, 6, 1> v) {
        Eigen::MatrixXd transform = Eigen::MatrixXd::Identity(4,4);
        transform.block<3,3>(0,0) = v2tRPY(v.block<3,1>(3,0));
        transform.block<3,1>(0,3) = v.block<3,1>(0,0);
        return transform;
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
                double gt_position[3], gt_orientation[3], position[3], orientation[3];
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
                
                double direction_vector[3];
                for(int i=0; i<3; ++i) line_stream >> direction_vector[i];

                Keypoint kp(id, direction_vector);
                current_cam->keypoints.push_back(kp);
            }
        }
        return cameras;
    }

    void save_landmarks(const map<int, pr::Vec3d>& landmarks, string output_path){
        ofstream file_stream(output_path);
        for(const auto& l: landmarks) file_stream << "L: " << l.first << "\t" << l.second.transpose() << endl;
        file_stream.close();
    }

    void save_camera_positions(const vector<Camera>& cameras, string output_path){
        ofstream file_stream(output_path);
        for(const auto& cam: cameras) file_stream << "C: " << cam.id << "\t" << cam.position.transpose() << endl;
        file_stream.close();
    }

    map<int, pr::Vec3d> load_landmarks(string path){
        map<int, pr::Vec3d> landmarks;

        ifstream data_stream(path);
        string line, line_type;
        int id;
        double x, y, z;
        for(string line; getline(data_stream, line); ) {
            istringstream line_stream(line);
            line_stream >> line_type >> id >> x >> y >> z;
            // cout << Vec3f(x,y,z) << endl << endl;
            landmarks.insert({id, Vec3d(x,y,z)});
        }

        return landmarks;
    }

    Sim3::Sim3() {
        this->scale = 1;
        this->rotation.setZero();
        this->translation.setZero();
    }

    Sim3::Sim3(double scale, Vec3d rotation, Vec3d translation) {
        this->scale = exp(scale);
        this->rotation = rotation;
        this->translation = translation;
    }

    Matrix4d Sim3::as_matrix() {
        Matrix4d sim3 = Matrix4d::Zero();
        sim3.block<3,3>(0,0) = v2tRPY(this->rotation);
        sim3.block<3,1>(0,3) = this->translation;
        sim3(3,3) = this->scale;
        return sim3;
    }

    Vec3d Sim3::operator*(Vec3d v) {
        Matrix4d sim3 = this->as_matrix();
        Vec4d v_hom;
        v_hom.block<3,1>(0,0) = v;
        v_hom(3,1) = 1.0;
        auto res = sim3*v_hom;
        return res.block<3,1>(0,0) * res(3,0);
        // return this->scale * (v2tRPY(this->rotation) * v + this->translation);
    }

    void Sim3::box_plus(Vec3d d_translation, Vec3d d_rotation, double d_scale) {
        Sim3 d_x = Sim3(d_scale, d_rotation, d_translation);
        Matrix4d x_prime = d_x.as_matrix() * this->as_matrix();
        this->translation = x_prime.block<3,1>(0,3);
        this->scale = x_prime(3,3);
        // this->rotation = x_prime.block<3,3>(0,0).eulerAngles(2,1,0).reverse();
        this->rotation = tRPY2v(x_prime.block<3,3>(0,0));
    }

    Sim3 Sim3::inverse(){
        Matrix4d inverse = this->as_matrix().inverse();
        double scale = log(inverse(3,3));
        // Vec3d rotation = inverse.block<3,3>(0,0).eulerAngles(2,1,0).reverse();;
        Vec3d rotation = tRPY2v( inverse.block<3,3>(0,0) );
        Vec3d translation = inverse.block<3,1>(0,3);
        return Sim3(scale, rotation, translation);
    }

    double rmse(const map<int, Vec3d>& landmarks, const map<int, Vec3d>& gt_landmarks) {
        double total_squared_error = 0.;

        vector<int> ids; 
        for(const auto& l: landmarks) ids.push_back(l.first);
        
        for(int i: ids) {
            Vec3d delta_i = gt_landmarks.at(i) - landmarks.at(i);
            total_squared_error += (delta_i.transpose() * delta_i).value();
        }
        return sqrt(total_squared_error / ids.size());
    }

}