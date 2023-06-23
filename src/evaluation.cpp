#include "evaluation.h"

namespace pr {

    float rmse(const vector<Vec3f>& landmarks) {
        float error;

        return error;
    }

    float eval_map(const vector<Vec3f>& landmarks) {
        return 0;
    }

    /*
    rotation:
        - compute the delta rotation between the two poses, R_delta = R(a,b,c)^T  * R(a',b',c')
        - compare it with delta of your solution in this way trace(eye(3) - R_delta^T * R_delta_gt)
    */
    void eval_camera_rotations(const vector<Camera>& cameras, string output_dir) {
        float total_error = 0;
        ofstream file_stream(output_dir.append("/camera_rotation_errors.txt"));
        for(int i=0; i<(int)cameras.size(); i++)
            for(int j=0; j<(int)cameras.size(); j++)
                if(i != j) {
                    auto R_delta = v2tRPY(cameras[i].orientation).transpose() * v2tRPY(cameras[j].orientation);
                    auto R_delta_gt = v2tRPY(cameras[i].gt_orientation).transpose() * v2tRPY(cameras[j].gt_orientation);
                    auto error = (Matrix3f::Identity(3,3) - R_delta.transpose() * R_delta_gt).trace();
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
        for(int i=0; i<(int)cameras.size(); i++)
            for(int j=0; j<(int)cameras.size(); j++)
                if(i != j) {
                    Vec3f t_ij = v2tRPY(cameras[i].orientation).transpose() * ( cameras[j].position - cameras[i].position );
                    Vec3f t_ij_gt = v2tRPY(cameras[i].gt_orientation).transpose() * ( cameras[j].gt_position - cameras[i].gt_position );

                    auto ratio_ij = t_ij.squaredNorm() / t_ij_gt.squaredNorm();
                    file_stream << i << "\t" << j << "\t" << ratio_ij << endl;
                }
        file_stream.close();
    }

    void evaluation(const vector<Camera>& cameras, const map<int, Vec3f>& landmarks, const map<int, Vec3f>& gt_landmarks, string output_dir) {
        eval_camera_rotations(cameras, output_dir);
        eval_camera_positions(cameras, output_dir);
    }


    
}