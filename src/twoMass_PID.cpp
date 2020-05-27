#include<iostream>
#include<fstream>
#include<eigen3/Eigen/Core>

#include "two_mass_model.h"
#include "PIDController.h"

using namespace std;
using namespace Eigen;

// simulation for two-mass
int Simulate_TwoMass_PID(ofstream& out, double setpoint=0)
{
	double r = setpoint;  //  set point

	Vector4d x0 = Vector4d::Zero();  //  initial state
	TwoMassModal twomass(x0, 0.001);
	SysModal::RungeKutta<Vector4d> rk(&twomass);

	PIDController pid(1.5, 0, 0, 0.01);
	double e = 0;

	int flag = 0;  //  controller sample frequency is 0.1 simulation frequency
	for (int k = 0; k < 20000; k++)
	{
		flag++;
		if (flag >= 10)
		{
			e = r - twomass.readout();
			double u = pid.update(e);
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
		cout << "t= " << rk.trans_->t_ << " , x= " << x.transpose() << "  u= " << twomass.u_ << endl;
		out << rk.trans_->t_ << " " << rk.trans_->x_.transpose() << endl;
	}

	return 0;
}

#ifndef TEST_ALL
int main()
{
	ofstream out("twomass_pid.txt");
	Simulate_TwoMass_PID(out);
	return 0;
}
#endif