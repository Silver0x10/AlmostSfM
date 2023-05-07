// #pragma once
#include "init_translations.h"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <string>
#include <iostream>

namespace pr {

    void error_and_jacobian_translation(const Vec3f& t_ij, const Matrix3f& rot_i, const Vec3f& t_i, const Vec3f& t_j, Vec3f& error, Eigen::MatrixXf& jacobians) {
        error = skew(t_ij) * rot_i.transpose() * (t_i - t_j);

        Matrix3f jac_t_i = skew(t_ij) * rot_i.transpose();
        Matrix3f jac_t_j = -jac_t_i;
        auto delta_wrt_i = rot_i.transpose() * (t_i - t_j);
        Matrix3f jac_t_ij;
        jac_t_ij << 
            0,                   delta_wrt_i[2],    -delta_wrt_i[1], 
           -delta_wrt_i[2],      0,                  delta_wrt_i[0], 
            delta_wrt_i[1],     -delta_wrt_i[0],     0; 
        
        jacobians.block<3,3>(0, 0) = jac_t_i;
        jacobians.block<3,3>(0, 3) = jac_t_j;
        jacobians.block<3,3>(0, 6) = jac_t_ij;
    }


    void init_translations(vector<Camera>& cameras) {
        cout << "Init translations..." << endl;
        int num_cameras = cameras.size();
        int system_size = pow(num_cameras, 2);
        Vec3f state[system_size];
        for(auto& t: state) t.setRandom(); // TODO: maybe consider damping factors instead 
        
        Eigen::MatrixXf matrixH(3*system_size, 3*system_size);
        Eigen::VectorXf b(3*system_size);
        
        // chi errror

        cout << "init matrices" << endl;
        matrixH.setZero();
        b.setZero();
        cout << "end init matrices" << endl << endl;
        

        for(int i=0; i<num_cameras; ++i){
            for(int j=0; j<num_cameras; ++j){
                if(i==0 and j == 0) continue; // to force the first position

                const auto& t_ij = state[i*num_cameras + j];
                const auto& rot_i = v2tRPY(cameras[i].orientation);
                const auto& t_i = state[i*num_cameras];
                const auto& t_j = state[j*num_cameras];
                
                Vec3f error;
                Eigen::MatrixXf jacobians(3, 9);
                error_and_jacobian_translation(t_ij, rot_i, t_i, t_j, error, jacobians);
                auto jac_t_i = jacobians.block<3,3>(0, 0) + Eigen::Matrix3f::Identity();
                auto jac_t_j = jacobians.block<3,3>(0, 3) +Eigen::Matrix3f::Identity();
                auto jac_t_ij = jacobians.block<3,3>(0, 6)+Eigen::Matrix3f::Identity();

                // H += jacobian.transpose()*jacobian;
                matrixH.block<3,3>(3*(i*num_cameras), 3*(i*num_cameras))     += jac_t_i.transpose()*jac_t_i;
                matrixH.block<3,3>(3*(i*num_cameras), 3*(i*num_cameras + j)) += jac_t_i.transpose()*jac_t_ij;
                matrixH.block<3,3>(3*(i*num_cameras), 3*(j*num_cameras))     += jac_t_i.transpose()*jac_t_j;

                matrixH.block<3,3>(3*(i*num_cameras + j), 3*(i*num_cameras))     += jac_t_ij.transpose()*jac_t_i;
                matrixH.block<3,3>(3*(i*num_cameras + j), 3*(i*num_cameras + j)) += jac_t_ij.transpose()*jac_t_ij;
                matrixH.block<3,3>(3*(i*num_cameras + j), 3*(j*num_cameras))     += jac_t_ij.transpose()*jac_t_j;

                matrixH.block<3,3>(3*(j*num_cameras), 3*(i*num_cameras))     += jac_t_j.transpose()*jac_t_i;
                matrixH.block<3,3>(3*(j*num_cameras), 3*(i*num_cameras + j)) += jac_t_j.transpose()*jac_t_ij;
                matrixH.block<3,3>(3*(j*num_cameras), 3*(j*num_cameras))     += jac_t_j.transpose()*jac_t_j;
                
                // b += jacobian.transpose()*error;
                // b.segment<3>(3*(i*num_cameras))     += jac_t_i.transpose()*error;
                // b.segment<3>(3*(i*num_cameras + j)) += jac_t_ij.transpose()*error;
                // b.segment<3>(3*(j*num_cameras))     += jac_t_j.transpose()*error;

                // cout << i << ' ' << j << endl;
            }
        }

        // matrixH.block<3,3>(0,0) += Eigen::Matrix3f::Identity(); // to force the first position (maybe)
        // cout << matrixH.block<3,3>(0,0) << endl;
        // cout << matrixH.block<3,3>(0,0) << endl << endl;

        cout << "Solving..." << endl; 

        Eigen::FullPivLU<Eigen::MatrixXf> lu_decomp(matrixH);
        auto kernel = lu_decomp.kernel();
        std::cout << "The rank of A is " << lu_decomp.rank() << std::endl;
        std::cout << "Here is a matrix whose columns form a basis of the null-space of A:\n" << kernel << std::endl;

        Eigen::VectorXf dx = kernel.rightCols(1); // extracted from the null space basis

        cout << endl << dx << endl << dx.size() << endl;

        // // cout << sizeof(state)/sizeof(state[0])*3 << endl;
        // auto dx = matrixH.llt().solve(b); // TODO: ripasso Cholesky decomposition; consider the null space of H
        // cout << "dx size: " << dx.size() << endl << endl;

        // cout << dx << endl;


        // cout << "cols: " << matrixH.cols() << endl << "rows: " << matrixH.rows() << endl << "size: " << matrixH.size() << endl;
        
        cout << "Solved (maybe)" << endl; 

    }


}






#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

// Definizione della funzione obiettivo
struct ObjectiveFunction {
    const std::vector<Matrix3d>& rotations; // matrici di rotazione delle camere
    const std::vector<Vector3d>& translations; // vettori di traslazione delle camere
    const Matrix3d& refRotation; // matrice di rotazione della camera di riferimento
    const Vector3d& refTranslation; // vettore di traslazione della camera di riferimento

    ObjectiveFunction(const std::vector<Matrix3d>& rotations,
                      const std::vector<Vector3d>& translations,
                      const Matrix3d& refRotation,
                      const Vector3d& refTranslation)
        : rotations(rotations), translations(translations), refRotation(refRotation), refTranslation(refTranslation) {}

    // Calcolo della funzione obiettivo
    VectorXd operator()(const VectorXd& x) const {
        const int numCameras = translations.size();
        VectorXd residuals(3 * numCameras);

        for (int i = 0; i < numCameras; i++) {
            const Matrix3d& Ri = rotations[i];
            const Vector3d& ti = translations[i];

            const Vector3d& xi = x.segment<3>(3 * i);

            residuals.segment<3>(3 * i) = (Ri - refRotation) * xi + refTranslation - Ri * ti - xi;
        }

        return residuals;
    }

    // Calcolo del jacobiano della funzione obiettivo
    void computeJacobian(const VectorXd& x, MatrixXd& jacobian) const {
        const int numCameras = translations.size();
        jacobian.resize(3 * numCameras, 3 * numCameras);

        for (int i = 0; i < numCameras; i++) {
            const Matrix3d& Ri = rotations[i];

            jacobian.block<3, 3>(3 * i, 3 * i) = Ri - refRotation;
            jacobian.block<3, 3>(3 * i, 3 * i + 3) = -Matrix3d::Identity();
        }
    }
};

// Implementazione del metodo di Gauss-Newton
void gaussNewton(const std::vector<Matrix3d>& rotations,
                 const std::vector<Vector3d>& translations,
                 const Matrix3d& refRotation,
                 const Vector3d& refTranslation,
                 VectorXd& x) {
    const int numCameras = translations.size();
    const int numParams = 3 * numCameras;

    ObjectiveFunction objective(rotations, translations, refRotation, refTranslation);

    VectorXd residuals = objective(x);
    double error = residuals.squaredNorm();

    std::cout << "Initial error: " << error << std::endl;

    for (int iter = 0; iter < 10; iter++) { // Eseguo un massimo di 10 iterazioni
        MatrixXd jacobian;
        objective.computeJacobian(x, jacobian);

        VectorXd delta = -(jacobian.transpose() * jacobian).inverse() * jacobian.transpose() * residuals;

        x += delta;

        residuals = objective(x);
        double newError = residuals.squaredNorm();

        if (newError > error) { // Se l'errore è aumentato
            std::cout << "Error increased after iteration " << iter << std::endl;
            break;
        }

        std::cout << "Iteration " << iter << ", error = " << newError << std::endl;

        if (newError < 1e-12) { // Se l'errore è sufficientemente piccolo, esco dal ciclo
            break;
        }

        error = newError;
    }
}
