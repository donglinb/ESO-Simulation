#pragma once
#ifndef _SYSMODEL_H
#define _SYSMODEL_H

#include<eigen3/Eigen/Core>
namespace SysModal
{
	using namespace Eigen;
	
	template<typename T>
	class TransitionModal
	{
	public:
		TransitionModal(T x0, double h = 0.1, double t0 = 0)
		{
			x_ = x0;
			h_ = h;
			t_ = t0;
		}
		~TransitionModal()
		{
			;
		}
		int setStartPoint(double t0, T x0)
		{
			t_ = t0;
			x_ = x0;
			return 0;
		}
		int setStep(double h)
		{
			h_ = h;
			return 0;
		}
		int setInput(T u)
		{
			u_ = u;
			return 0;
		}
		virtual T transition_func(double t, T x)=0;

		T x_;
		T u_;
		double t_;
		double h_;
	};  //  class TransitionModal

	template<typename T>
	class RungeKutta
	{
	public:
		RungeKutta(TransitionModal<T> *trans)
		{
			trans_ = trans;
		}
		~RungeKutta()
		{
			;
		}
		T update();

		TransitionModal<T> *trans_;
	};  //  class RungeKutta

}   //  namespace SysModal

#endif  //  _SYSMODEL_H