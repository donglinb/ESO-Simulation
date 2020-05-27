#pragma once
#ifndef _TOY_MODEL_H
#define _TOY_MODEL_H

#include<eigen3/Eigen/Core>
#include"SysModal.hpp"
using namespace Eigen;

//  simple nonlinear model
class TransModal :public SysModal::TransitionModal<Vector2d>
{
public:
	TransModal(Vector2d x0, double h = 0.1, double t0 = 0) :
		TransitionModal(x0, h, t0)
	{
		;
	}
	virtual Vector2d transition_func(double t, Vector2d x)
	{
		Vector2d x1 = Vector2d::Zero();
		x1(0) = x(1);
		// three different nonlinear functions
		x1(1) = nonlinearfun(t, x) + disturbfun(t);
		//x1(1) = -x(0) - 0.25 * x(1) * (1.5 - x(0) * x(0)) + sin(0.5 * t);
		//x1(1) = mysign(sin(0.5*t)) + 3 * cos(0.5*t);
		return x1;
	}
	double readout()
	{
		return x_(0);
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
};

#endif  //  _TOY_MODEL_H