#pragma once
#ifndef _PIDCONTROLLER_H
#define _PIDCONTROLLER_H

class PIDController
{
public:
	PIDController();
	PIDController(double P, double I, double D, double T = 0.01);
	~PIDController();
	int setPID(double P, double I, double D);
	double update(double e);

	double P_, I_, D_;
	double T_;
	double Kp_, Ki_, Kd_;
	double e_k, e_k_1, e_k_2;
	double u_k_1, u_delta, u_k;
	int initialized_;
};  //  class PIDController

#endif  //  _PIDCONTROLLER_H