#pragma once
#ifndef _TWO_MASS_MODEL_H
#define _TWO_MASS_MODEL_H

#include<eigen3/Eigen/Core>
#include"SysModal.hpp"
using namespace Eigen;

//  Two Mass Model
class TwoMassModal :public SysModal::TransitionModal<Vector4d>
{
public:
	TwoMassModal(Vector4d x0, double h = 0.1, double t0 = 0) :
		TransitionModal(x0, h, t0)
	{
		u_ = 0;
		c_ = 0;
		c1_ = 0, c2_ = 1;
		m1_ = 1, m2_ = 1;
		k_ = 1;
		w1_ = 0;
		w2_ = 0;
	}
	~TwoMassModal()
	{
		;
	}
	int setMass(double m1, double m2)
	{
		m1_ = m1;
		m2_ = m2;
		return 0;
	}
	int setSpring(double k)
	{
		k_ = k;
		return 0;
	}
	int setInput(double u)
	{
		u_ = u;
		return 0;
	}
	int setDisturb(double w1, double w2)
	{
		w1_ = w1;
		w2_ = w2;
		return 0;
	}
	virtual Vector4d transition_func(double t, Vector4d x)
	{
		Matrix4d A;
		Vector4d b1, b2;
		A << 0, 0, 1, 0,
			0, 0, 0, 1,
			-k_ / m1_, k_ / m1_, -c_ / m1_, c_ / m1_,
			k_ / m2_, -k_ / m2_, c_ / m2_, -c_ / m2_;
		b1 << 0, 0, 1 / m1_, 0;
		b2 << 0, 0, 0, 1 / m2_;
		Vector4d x1 = A * x + b1 * (u_ + w1_) + b2 * w2_;
		return x1;
	}
	double readout()
	{
		Vector4d c;
		c << c1_, c2_, 0, 0;
		return c.transpose() * x_;
	}

	double w1_, w2_;
	double u_;
	double m1_, m2_;
	double k_;
	double c_;
	double c1_, c2_;
};

#endif  //  _TWO_MASS_MODEL_H