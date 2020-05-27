#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Core>

#include "toy_model.h"
#include "PIDController.h"

using namespace Eigen;
using namespace std;

class StateObserver
{
public:
	StateObserver(Vector2d z0, double h = 0.01)
	{
		z_ = z0;
		h_ = h;
	}
	~StateObserver()
	{
		;
	}
	Vector2d update(double y)
	{
		double e = z_(0) - y;
		z_(0) = z_(0) + h_ * (z_(1) - 100 * e);
		z_(1) = z_(1) - h_ * 200 * fal(e, 0.5, 0.01);  //nonlinear
		//z_(1) = z_(1) - h_ * 1000 * e;  //linear
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
	Vector2d z_;
	double h_;
};

//  simulation for simple nonlinear model
int Simulate_Toy_PID(ofstream& out, ofstream& obsout, double setpoint=1)
{
	double r = setpoint;  //  set point

	Vector2d x0(0.3,0);  //  initial state
	TransModal trans(x0, 0.001);
	trans.setInput(Vector2d::Zero());
	SysModal::RungeKutta<Vector2d> rk(&trans);
	
	double e = 0, u = 0;
	PIDController pid(1.8, 1, 1.5, 0.01);

	StateObserver obs(x0);  //  state observer for validation

	int flag = 0;  //  controller sample frequency is 0.1 simulation frequency
	for (int k = 0; k < 20000; k++)
	{
		flag++;
		if (flag >= 10)
		{
			e = r - trans.readout();
			u = pid.update(e);
			trans.setInput(Vector2d(0,u));
			obs.update(trans.readout());
			flag = 0;
		}
		Vector2d x = rk.update();
		cout << "t= " << rk.trans_->t_ << " , x= " << x.transpose() << endl;

		out << rk.trans_->t_ << " " << rk.trans_->x_.transpose() << endl;
		obsout << rk.trans_->t_ << " " << obs.z_.transpose() << endl;
	}

	return 0;
}

#ifndef TEST_ALL
int main()
{
	ofstream out("toy_pid.txt");
	ofstream obsout("toy_pid_obs.txt");
	Simulate_Toy_PID(out, obsout);
	return 0;
}
#endif