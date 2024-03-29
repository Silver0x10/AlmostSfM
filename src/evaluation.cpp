#include "evaluation.h"

namespace pr {

    void eval_map(const map<int, Vec3d>& landmarks, map<int, Vec3d> gt_landmarks, Sim3 transform, string output_dir) {
        ofstream file_stream(output_dir.append("/landmarks_errors.txt"));
        
        for(auto& gt_l: gt_landmarks) gt_l.second = transform * gt_l.second;
        
        double error = rmse(landmarks, gt_landmarks);
        file_stream << "RMSE\t" << error << endl;
        cout << "Landmarks RMSE: " << error << endl;

        file_stream << endl << "Sim(3):" << endl;
        file_stream << "\ttranslation:\t" << transform.translation.transpose() << endl;
        file_stream << "\tscale:\t" << transform.scale << endl;
        file_stream << "\trotation angles:\t" << transform.rotation.transpose() << endl;

        file_stream.close();
    }

    /*
    rotation:
        - compute the delta rotation between the two poses, R_delta = R(a,b,c)^T  * R(a',b',c')
        - compare it with delta of your solution in this way trace(eye(3) - R_delta^T * R_delta_gt)
    */
    void eval_camera_rotations(const vector<Camera>& cameras, string output_dir) {
        double total_error = 0;
        ofstream file_stream(output_dir.append("/camera_rotation_errors.txt"));
        file_stream << "id_camera_i\tid_camera_j\trotation_error" << endl;
        for(int i=0; i<(int)cameras.size(); i++)
            for(int j=0; j<(int)cameras.size(); j++)
                if(i != j) {
                    auto R_delta = v2tRPY(cameras[i].orientation).transpose() * v2tRPY(cameras[j].orientation);
                    auto R_delta_gt = v2tRPY(cameras[i].gt_orientation).transpose() * v2tRPY(cameras[j].gt_orientation);
                    auto error = (Matrix3d::Identity(3,3) - R_delta.transpose() * R_delta_gt).trace();
                    file_stream << i << "\t" << j << "\t" << error << endl;
                    total_error += error;
                }
        file_stream.close();
    }

    /*
    translation (solution up to scale):
    compute the ratio between estimated translation and gt translation and check if it is the same for all the pose pairs
        - compute the delta: R(a,b,c)^T * ([x';y';z']-[x;y;z])
        - compute the norm
        - do the same for the estimate
        - compute the ratio
    */
    void eval_camera_positions(const vector<Camera>& cameras, string output_dir) {
        ofstream file_stream(output_dir.append("/camera_position_errors.txt"));
        file_stream << "id_camera_i\tid_camera_j\tposition_error" << endl;
        for(int i=0; i<(int)cameras.size(); i++)
            for(int j=0; j<(int)cameras.size(); j++)
                if(i != j) {
                    Vec3d t_ij = v2tRPY(cameras[i].orientation).transpose() * ( cameras[j].position - cameras[i].position );
                    Vec3d t_ij_gt = v2tRPY(cameras[i].gt_orientation).transpose() * ( cameras[j].gt_position - cameras[i].gt_position );

                    auto ratio_ij = t_ij.squaredNorm() / t_ij_gt.squaredNorm();
                    file_stream << i << "\t" << j << "\t" << ratio_ij << endl;
                }
        file_stream.close();
    }

    void evaluation(const vector<Camera>& cameras, const map<int, Vec3d>& landmarks, const map<int, Vec3d>& gt_landmarks, const Sim3& transform, string output_dir) {
        eval_camera_rotations(cameras, output_dir);
        eval_camera_positions(cameras, output_dir);
        eval_map(landmarks, gt_landmarks, transform, output_dir);
    }


    
}