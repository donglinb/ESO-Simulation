#include"PIDController.h"
PIDController::PIDController()
{
	P_ = 0.1;
	I_ = 0;
	D_ = 0;
	T_ = 0.01;
	Kp_ = P_;
	Ki_ = I_ * T_;
	Kd_ = D_ / T_;
	initialized_ = 0;
	e_k = e_k_1 = e_k_2 = 0;
	u_k_1= u_delta= u_k=0;
}
PIDController::PIDController(double P, double I, double D, double T)
{
	P_ = P;
	I_ = I;
	D_ = D;
	T_ = T;
	Kp_ = P_;
	Ki_ = I_ * T_;
	Kd_ = D_ / T_;
	initialized_ = 0;
	e_k = e_k_1 = e_k_2 = 0;
	u_k_1 = u_delta = u_k = 0;
}
PIDController::~PIDController()
{
	;
}
int PIDController::setPID(double P, double I, double D)
{
	P_ = P;
	I_ = I;
	D_ = D;
	Kp_ = P_;
	Ki_ = I_ * T_;
	Kd_ = D_ / T_;
	return 0;
}
double PIDController::update(double e)
{
	e_k_2 = e_k_1;
	e_k_1 = e_k;
	e_k = e;
	if (initialized_ <= 0)
	{
		u_delta = Kp_ * e_k;
		initialized_=1;
	}
	else if (initialized_ == 1)
	{
		u_delta = Kp_ * (e_k - e_k_1) + Ki_ * e_k;
		initialized_ = 2;
	}
	else
	{
		u_delta = Kp_ * (e_k - e_k_1) + Ki_ * e_k + Kd_ * (e_k - 2 * e_k_1 + e_k_2);
	}
	u_k = u_k_1 + u_delta;
	u_k_1 = u_k;
	return u_k;
}
