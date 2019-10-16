//
// Created by zyx on 2019/10/16.
//
#include <Eigen/Eigen>
#include <iostream>
// 请编程验证对于小量 ω =[0.01,0.02,0.03]T，
// 两种方法得到的结果非 常接近，实践当中可视为等同。
// 因此，在后文提到旋转时，我们并不刻 意区分旋转本身是 q 还是 R，也不区分其更新方式为上式的哪一种
using namespace Eigen;
using namespace std;
void orthonormalize(Eigen::Matrix3f& ColVecs)
{
	ColVecs.col(0).normalize();
	double temp;
	for(std::size_t k = 0; k != ColVecs.cols() - 1; ++k)
	{
		for(std::size_t j = 0; j != k + 1; ++j)
		{
			temp = ColVecs.col(j).transpose() * ColVecs.col(k + 1);
			ColVecs.col(k + 1) -= ColVecs.col(j) * temp;
		}
		ColVecs.col(k + 1).normalize();
	}
}
Matrix3f anti_symmetry_exp(const Vector3f &v){
	float theta = v.norm();
	Vector3f a = v / theta;
	Matrix3f a_x;
	a_x << 0,    -a[2],  a[1],
	     a[2],  0,    -a[0],
	    -a[1],  a[0],  0;
	float cos_theta = cos(theta);
	float sin_theta = sin(theta);
	Matrix3f I = Matrix3f::Identity();
	return cos_theta * I + (1 - cos_theta) * a * a.transpose() + sin_theta * a_x;
}
int main(){
	// Get an random initial pose;
	Matrix3f R_init = Matrix3f::Random() + 2.0f * Matrix3f::Ones();
	orthonormalize(R_init);
	Quaternionf q_init(R_init);

	Matrix3f w_x = anti_symmetry_exp({0.01f, 0.02f, 0.03f});
	Matrix3f R_result_by_rotation_matrix = R_init * w_x;

	Quaternionf delta_q(1, 0.5*0.01f, 0.5*0.02f, 0.5*0.03f);
	Quaternionf q_result = q_init * delta_q;
	q_result.normalize();

	Matrix3f R_result_by_quaternion = q_result.toRotationMatrix();
	cout << "Results by Roatation matrix: " << endl;
	cout << R_result_by_rotation_matrix << endl;
	cout << "Results by quaternion " << endl;
	cout << R_result_by_quaternion << endl;


	char buf[1000];
	float j = 10 + 5;
	sprintf(buf, "%s %s %f ssdbnibb %x", "122", "222", j, (int)j);
	cout << buf << endl;
}

