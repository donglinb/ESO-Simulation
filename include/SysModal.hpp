#include"SysModal.h"
namespace SysModal
{
	//Runge Kutta
	template<typename T>
	T RungeKutta<T>::update()
	{
		T k1, k2, k3, k4;
		double t_ = trans_->t_;
		T x_ = trans_->x_;
		double h_ = trans_->h_;
		k1 = trans_->transition_func(t_, x_);
		k2 = trans_->transition_func(t_ + 0.5*h_, x_ + 0.5*h_*k1);
		k3 = trans_->transition_func(t_ + 0.5*h_, x_ + 0.5*h_*k2);
		k4 = trans_->transition_func(t_ + h_, x_ + h_ * k3);
		x_ = x_ + h_ * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
		t_ = t_ + h_;
		trans_->x_ = x_;
		trans_->t_ = t_;
		return x_;
	}
}