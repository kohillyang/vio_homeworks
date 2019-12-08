//
// Created by zyx on 2019/12/8.
//
#include <Eigen/Eigen>
#include <vector>
#include <assert.h>
#include <iostream>
using namespace Eigen;
using namespace std;
Vector4f triangulate(std::vector<Eigen::Matrix<float, 3, 4>> KTs, std::vector<Eigen::Vector2f> mns){
    assert(KTs.size() > 2);
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
    Matrix<float, 3, 4> Ts[4];
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
    std::vector<Eigen::Matrix<float, 3, 4>> KTs;
    std::vector<Eigen::Vector2f> mns;
    for(int i = 0; i < sizeof(Ts) / sizeof(Matrix<float, 4, 3>); i++){
        Vector3f mn = K * Ts[i] * P_real;
        mn /= mn(2);
        KTs.push_back(K * Ts[i]);
        mns.push_back(mn.block<2, 1>(0, 0));
    }
    cout << triangulate(KTs, mns);
}