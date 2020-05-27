#include<iostream>
#include<fstream>
#include<eigen3/Eigen/Core>

#include "toy_model.h"

using namespace std;
using namespace Eigen;

class ExtendedStateObserver
{
public:
	ExtendedStateObserver(Vector3d z0, double h = 0.01)
	{
		z_ = z0;
		h_ = h;
		t_ = 0;
		beta1 = 100;
		beta2 = 300;
		beta3 = 1000;
	}
	~ExtendedStateObserver()
	{
		;
	}
	int setBeta(double  beta1, double beta2, double beta3)
	{
		this->beta1 = beta1;
		this->beta2 = beta2;
		this->beta3 = beta3;
		return 0;
	}
	Vector3d update(double y)
	{
		double e = z_(0) - y;
		z_(0) = z_(0) + h_ * (z_(1) - beta1 * e);
		//  ESO with different part of model known
		z_(1) = z_(1) + h_ * (z_(2) - beta2 * fal(e, 0.5, 0.01));  //nonlinear ESO
		//z_(1) = z_(1) + h_ * (z_(2) - beta2 * e);  //linear ESO
		//z_(1) = z_(1) + h_ * (z_(2) + nonlinearfun(t_, z_.head<2>()) - beta2 * fal(e, 0.5, 0.01));  //nonlinear part modal
		//z_(1) = z_(1) + h_ * (z_(2) + nonlinearfun_part1(t_, z_.head<2>()) - beta2 * fal(e, 0.5, 0.01));  //nonlinear part modal part1
		//z_(1) = z_(1) + h_ * (z_(2) + nonlinearfun_part2(t_, z_.head<2>()) - beta2 * fal(e, 0.5, 0.01));  //nonlinear part modal part2
		
		z_(2) = z_(2) + h_ * (-beta3 * fal(e, 0.25, 0.01));  //nonlinear ESO
		//z_(2) = z_(2) + h_ * (-beta3 * e);  //linear ESO

		t_ += h_;
		return z_;
	}
	double fal(double e, double a, double b)
	{
		double fun = 0;
		if (fabs(e) > b)
		{
			fun = pow(fabs(e), a) * mysign(e);
		}
		else
		{
			fun = e / pow(b, a);
		}
		return fun;
	}
	int mysign(double x)
	{
		if (x > 1e-15)
		{
			return 1;
		}
		else if (x < -1e-15)
		{
			return -1;
		}
		else
		{
			return 0;
		}
	}
	double nonlinearfun(double t, Vector2d x)
	{
		double fun = 0;
		fun = -(1 + 0.5 * cos(t)) * x(0) - (1 + sin(t / 3)) * x(1);
		//fun = -(1 + 0.5*cos(10*t))*x(0) - (1 + sin(5*t))*x(1);
		return fun;
	}
	double nonlinearfun_part1(double t, Vector2d x)
	{
		double fun = 0;
		fun = -(1 + 0.5 * cos(t)) * x(0);
		return fun;
	}
	double nonlinearfun_part2(double t, Vector2d x)
	{
		double fun = 0;
		fun = -(1 + sin(t / 3)) * x(1);
		return fun;
	}
	double disturbfun(double t)
	{
		double fun = 0;
		fun = mysign(sin(1.5 * t));
		return fun;
	}

	Vector3d z_;
	double h_;
	double t_;
	double beta1, beta2, beta3;
};

//  simulation for simple nonlinear model, observe disturbance without control
int Simulate_Toy_ESO(ofstream& out, ofstream& obsout, double setpoint = 1)
{
	double r = setpoint;  //  set point (currently unused)

	Vector2d x0(0.3, 0);  //  initial state
	TransModal trans(x0, 0.001);
	trans.setInput(Vector2d::Zero());
	SysModal::RungeKutta<Vector2d> rk(&trans);

	ExtendedStateObserver eso(Vector3d::Zero());
	eso.setBeta(100, 150, 3000);

	double y = 0, e = 0, u = 0;
	double f = 0;  //  disturbance function

	int flag = 0;  //  controller sample frequency is 0.1 simulation frequency
	for (int k = 0; k < 20000; k++)
	{
		flag++;
		if (flag >= 10)
		{
			eso.update(trans.readout());
			flag = 0;
		}
		Vector2d x = rk.update();
		cout << "t= " << rk.trans_->t_ << " , x= " << x.transpose() << endl;

		//  different disturbances, corresponding to the ones used in the toy model
		f = trans.nonlinearfun(rk.trans_->t_, rk.trans_->x_) + trans.disturbfun(rk.trans_->t_);
		//f = trans.nonlinearfun_part1(rk.trans_->t_, rk.trans_->x_) + trans.disturbfun(rk.trans_->t_);
		//f = trans.nonlinearfun_part2(rk.trans_->t_, rk.trans_->x_) + trans.disturbfun(rk.trans_->t_);
		//f = trans.disturbfun(rk.trans_->t_);
		//f = -rk.trans_->x_(0) - 0.25 * rk.trans_->x_(1) * (1.5 - rk.trans_->x_(0) * rk.trans_->x_(0)) + sin(0.5 * rk.trans_->t_);
		//f = eso.mysign(sin(0.5*rk.trans_->t_)) + 3 * cos(0.5*rk.trans_->t_);

		out << rk.trans_->t_ << " " << rk.trans_->x_.transpose() << endl;
		obsout << rk.trans_->t_ << " " << eso.z_.transpose() << " " << f << endl;
	}

	return 0;
}

#ifndef TEST_ALL
int main()
{
	ofstream out("toy_eso.txt");
	ofstream obsout("toy_eso_obs.txt");
	Simulate_Toy_ESO(out, obsout);
	return 0;
}
#endif