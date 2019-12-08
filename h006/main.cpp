//
// Created by zyx on 2019/12/8.
//
#include <Eigen/Eigen>
#include <vector>
#include <assert.h>
#include <iostream>
#include <random>
using namespace Eigen;
using namespace std;
Vector4f triangulate(std::vector<Eigen::Matrix<float, 3, 4>> KTs, std::vector<Eigen::Vector2f> mns){
    assert(KTs.size() >= 2);
    MatrixXf A(KTs.size() *2, 4);
    for(int i =0; i< KTs.size(); i++){
        Eigen::Matrix<float, 3, 4> &KT = KTs[i];
        float m = mns[i](0);
        float n = mns[i](1);
        for(int k=0; k<4; k++){
            A(i*2 + 0, k) = KT(0, k) - m * KT(2, k);
            A(i*2 + 1, k) = KT(1, k) - n * KT(2, k);
        }
    }

    JacobiSVD<Eigen::MatrixXf> svd(A, ComputeThinU | ComputeThinV );
    Matrix4f V = svd.matrixV();
    Vector4f svs = svd.singularValues();
    cout << svs(3) / svs(2);
    Vector4f r =  V.block<4, 1>(0, 3);
    r /= r(3);
    return r;
}

int main(){
    Matrix3f K;
    K << 600.0, 0,  320,
         0,     600, 240,
         0,     0,   1;
    Vector4f P_real = {1.0f, 1.0f, 40.0f, 1.0f};
    Matrix<float, 3, 4> Ts[6];
    Ts[0] << 1.0, 0,    0, 0.0,
             0.0, 1.0,  0, 0,
             0,   0,    1.0, 0;
    Ts[1] << 1.0, 0,    0, 1.0,
            0.0, 1.0,  0, 0,
            0,   0,    1.0, 0;
    Ts[2] << 1.0, 0,    0, 2.0,
            0.0, 1.0,  0, 0,
            0,   0,    1.0, 0;
    Ts[3] << 1.0, 0,    0, 3.0,
            0.0, 1.0,  0, 0,
            0,   0,    1.0, 0;

    Ts[4] << 1.0, 0,    0, 4.0,
            0.0, 1.0,  0, 0,
            0,   0,    1.0, 0;

    Ts[5] << 1.0, 0,    0, 5.0,
            0.0, 1.0,  0, 0,
            0,   0,    1.0, 0;

    for(int num_frame =2; num_frame <= 6; num_frame += 1){
        float sigma = 1.0f;
        std::vector<Eigen::Matrix<float, 3, 4>> KTs;
        std::vector<Eigen::Vector2f> mns;
        default_random_engine r_engine(time(0));
        normal_distribution<float> normalDistribution(0,sigma);
        cout << num_frame << ",";
        for(int i = 0; i < num_frame; i++){
            Vector3f mn = K * Ts[i] * P_real;
            mn /= mn(2);
            mn(0) += normalDistribution(r_engine);
            mn(1) += normalDistribution(r_engine);
            KTs.push_back(K * Ts[i]);
            mns.push_back(mn.block<2, 1>(0, 0));
        }
        cout << "," << triangulate(KTs, mns).transpose();
        cout << endl;
    }
}