#include<iostream>
#include<fstream>
#include<eigen3/Eigen/Core>

#include "two_mass_model.h"

using namespace std;
using namespace Eigen;

typedef Matrix<double, 5, 5> Matrix5d;
typedef Matrix<double, 5, 1> Vector5d;
class ExtendedStateObserverForTwoMass
{
public:
	ExtendedStateObserverForTwoMass(Vector5d z0, double h = 0.01)
	{
		z_ = z0;
		r_ = 0;
		u_ = 0;
		h_ = h;
		t_ = 0;
		A_ = Matrix5d::Zero();
		A_.block(0, 1, 4, 4) = Matrix4d::Identity();
		//A_(4, 3) = -2;  //gray box
		b_ = 1;
		B_ << 0, 0, 0, b_, 0;
		C_ << 1, 0, 0, 0, 0;

		wc_ = 1;
		wo_ = 15;
		L_ << 5 * wo_, 10 * wo_ * wo_, 10 * wo_ * wo_ * wo_, 5 * wo_ * wo_ * wo_ * wo_, wo_* wo_* wo_* wo_* wo_;  //black box
		//double a2 = 2;
		//L_ << 5 * wo_, -a2 + 10 * wo_*wo_, -5 * a2*wo_ + 10 * wo_*wo_*wo_, a2*a2 - 10 * a2*wo_*wo_ + 5 * wo_*wo_*wo_*wo_, 5 * a2*a2*wo_ - 10 * a2*wo_*wo_*wo_ + wo_ * wo_*wo_*wo_*wo_;  //gray box
		//K_ << wc_, 4 * wc_*wc_, 6 * wc_*wc_*wc_, 4 * wc_*wc_*wc_*wc_, 1;
		K_ << 5 * wc_ * wc_ * wc_ * wc_, 10 * wc_ * wc_ * wc_, 10 * wc_ * wc_, 5 * wc_, 1;
	}
	~ExtendedStateObserverForTwoMass()
	{
		;
	}
	void setwc(double wc)
	{
		wc_ = wc;
		K_ = Vector5d::Zero();
		//K_ << wc_, 4 * wc_*wc_, 6 * wc_*wc_*wc_, 4 * wc_*wc_*wc_*wc_, 1;
		K_ << 5 * wc_ * wc_ * wc_ * wc_, 10 * wc_ * wc_ * wc_, 10 * wc_ * wc_, 5 * wc_, 1;
	}
	void setwo(double wo)
	{
		wo_ = wo;
		L_ = Vector5d::Zero();
		L_ << 5 * wo_, 10 * wo_ * wo_, 10 * wo_ * wo_ * wo_, 5 * wo_ * wo_ * wo_ * wo_, wo_* wo_* wo_* wo_* wo_;  //black box
		//double a2 = 2;
		//L_ << 5 * wo_, -a2 + 10 * wo_*wo_, -5 * a2*wo_ + 10 * wo_*wo_*wo_, a2*a2 - 10 * a2*wo_*wo_ + 5 * wo_*wo_*wo_*wo_, 5 * a2*a2*wo_ - 10 * a2*wo_*wo_*wo_ + wo_ * wo_*wo_*wo_*wo_;  /gray box
	}
	void setInput(double r)
	{
		r_ = r;
	}
	Vector5d update(double y)
	{
		Vector5d z1 = A_ * z_ + B_ * u_ + L_ * (y - C_.transpose() * z_);
		z_ = z_ + h_ * z1;
		t_ += h_;

		return z_;
	}
	double getControlValue()
	{
		double u0 = K_(0) * (r_ - z_(0)) - (K_(1) * z_(1) + K_(2) * z_(2) + K_(3) * z_(3) + K_(4) * z_(4));
		u_ = (-z_(4) + u0) / b_;
		if (u_ >= 1)
		{
			u_ = 1;
		}
		else if (u_ <= -1)
		{
			u_ = -1;
		}
		return u_;
	}

	Matrix5d A_;
	Vector5d B_;
	Vector5d C_;
	Vector5d L_;
	Vector5d K_;

	Vector5d z_;
	double r_;
	double u_;
	double h_;
	double t_;
	double b_;
	double wc_;
	double wo_;
};

// simulation for two-mass
int Simulate_TwoMass_ESO(ofstream& out, ofstream& obsout, double setpoint = 0)
{
	double r = setpoint;  //  set point

	Vector4d x0 = Vector4d::Zero();  //  initial state
	TwoMassModal twomass(x0, 0.001);
	SysModal::RungeKutta<Vector4d> rk(&twomass);

	ExtendedStateObserverForTwoMass eso(Vector5d::Zero());
	cout << "A= " << eso.A_ << endl;
	eso.setwc(2);  //  controller bandwidth
	eso.setwo(5);  //  observer bandwidth
	eso.setInput(r);

	int flag = 0;  //  controller sample frequency is 0.1 simulation frequency
	for (int k = 0; k < 20000; k++)
	{
		flag++;
		if (flag >= 10)
		{
			eso.update(twomass.readout());
			double u = eso.getControlValue();
			twomass.setInput(u);
			flag = 0;
		}
		if (k == 1000)
		{
			twomass.setDisturb(0, 1000);
		}
		else if (k == 1001)
		{
			twomass.setDisturb(0, 0);
		}
		Vector4d x = rk.update();
		cout << "t= " << rk.trans_->t_ << " , x= " << x.transpose() << "  u= " << eso.u_ << endl;
		out << rk.trans_->t_ << " " << rk.trans_->x_.transpose() << endl;
		obsout << eso.t_ << " " << eso.z_.transpose() << endl;
	}

	return 0;
}

#ifndef TEST_ALL
int main()
{
	ofstream out("twomass_eso.txt");
	ofstream obsout("twomass_eso_obsout.txt");
	Simulate_TwoMass_ESO(out, obsout);
	return 0;	
}
#endif