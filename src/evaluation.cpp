#include "evaluation.h"

namespace pr {

    float rmse(const vector<Vec3f>& landmarks) {
        float error;

        return error;
    }

    float eval_map(const vector<Vec3f>& landmarks) {
        return 0;
    }

    float eval_camera_rotations(const vector<Camera>& cameras) {
        return 0;
    }

    /*
    translation (solution up to scale):
    compute the ratio between estimated translation and gt translation and check if it is the same for all the pose pairs
        - compute the delta: R(a,b,c)^T * ([x';y';z']-[x;y;z])
        - compute the norm
        - do the same for the estimate
        - compute the ratio
    */
    float eval_camera_positions(const vector<Camera>& cameras) {
        for(int i=0; i<(int)cameras.size(); i++)
            for(int j=0; j<(int)cameras.size(); j++)
                if(i != j) {
                    Vec3f t_ij = v2tRPY(cameras[i].orientation) * ( cameras[j].position - cameras[i].position );
                    t_ij.normalize();
                    Vec3f t_ij_gt = v2tRPY(cameras[i].gt_orientation) * ( cameras[j].gt_position - cameras[i].gt_position );
                    t_ij_gt.normalize();

                    Vec3f ratio_ij = t_ij.array() / t_ij_gt.array();
                    cout << i << "\t" << j << "\t" << ratio_ij(0) << endl ;
                }
    }

    void evaluation(const vector<Camera>& cameras, const map<int, Vec3f>& landmarks, const map<int, Vec3f>& gt_landmarks) {
        
    }


    
}