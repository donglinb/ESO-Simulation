#include <iostream>
#include<fstream>
#include<eigen3/Eigen/Core>
#include"PIDController.h"
#include"SysModal.hpp"
using namespace Eigen;
using namespace std;

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
		//x1(1) = nonlinearfun(t, x) + disturbfun(t);
		x1(1) = -x(0) - 0.25*x(1)*(1.5-x(0)*x(0)) + sin(0.5*t);
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
		fun = -(1 + 0.5*cos(t))*x(0) - (1 + sin(t / 3))*x(1);
		//fun = -(1 + 0.5*cos(10*t))*x(0) - (1 + sin(5*t))*x(1);
		return fun;
	}
	double nonlinearfun_part1(double t, Vector2d x)
	{
		double fun = 0;
		fun = -(1 + 0.5*cos(t))*x(0);
		return fun;
	}
	double nonlinearfun_part2(double t, Vector2d x)
	{
		double fun = 0;
		fun = -(1 + sin(t / 3))*x(1);
		return fun;
	}
	double disturbfun(double t)
	{
		double fun = 0;
		fun = mysign(sin(1.5*t));
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
		A <<       0,            0,            1,             0,
			           0,            0,            0,             1,
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
		return c.transpose()*x_;
	}

	double w1_, w2_;
	double u_;
	double m1_, m2_;
	double k_;
	double c_;
	double c1_, c2_;
};

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
			fun = pow(fabs(e), a)*mysign(e);
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

class ExtendedStateObserver
{
public:
	ExtendedStateObserver(Vector3d z0,double h=0.01)
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
		z_(1) = z_(1) + h_ * (z_(2) - beta2 * fal(e, 0.5, 0.01));  //nonlinear
		//z_(1) = z_(1) + h_ * (z_(2) - beta2 * e);  //linear
		//z_(1) = z_(1) + h_ * (z_(2) + nonlinearfun(t_, z_.head<2>()) - beta2 * fal(e, 0.5, 0.01));  //nonlinear part modal
		//z_(1) = z_(1) + h_ * (z_(2) + nonlinearfun_part1(t_, z_.head<2>()) - beta2 * fal(e, 0.5, 0.01));  //nonlinear part modal part1
		//z_(1) = z_(1) + h_ * (z_(2) + nonlinearfun_part2(t_, z_.head<2>()) - beta2 * fal(e, 0.5, 0.01));  //nonlinear part modal part2
		z_(2) = z_(2) + h_ * (-beta3 * fal(e,0.25,0.01));  //nonlinear
		//z_(2) = z_(2) + h_ * (-beta3 * e);  //linear
		t_ += h_;
		return z_;
	}
	double fal(double e, double a, double b)
	{
		double fun = 0;
		if (fabs(e) > b)
		{
			fun = pow(fabs(e), a)*mysign(e);
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
		fun = -(1 + 0.5*cos(t))*x(0) - (1 + sin(t / 3))*x(1);
		//fun = -(1 + 0.5*cos(10*t))*x(0) - (1 + sin(5*t))*x(1);
		return fun;
	}
	double nonlinearfun_part1(double t, Vector2d x)
	{
		double fun = 0;
		fun = -(1 + 0.5*cos(t))*x(0);
		return fun;
	}
	double nonlinearfun_part2(double t, Vector2d x)
	{
		double fun = 0;
		fun= -(1 + sin(t / 3))*x(1);
		return fun;
	}
	double disturbfun(double t)
	{
		double fun = 0;
		fun = mysign(sin(1.5*t));
		return fun;
	}

	Vector3d z_;
	double h_;
	double t_;
	double beta1, beta2, beta3;
};

typedef Matrix<double, 5, 5> Matrix5d;
typedef Matrix<double, 5, 1> Vector5d;
class ExtendedStateObserverForTwoMass
{
public:
	ExtendedStateObserverForTwoMass(Vector5d z0, double h=0.01)
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
		L_ << 5 * wo_, 10 * wo_*wo_, 10 * wo_*wo_*wo_, 5 * wo_*wo_*wo_*wo_, wo_*wo_*wo_*wo_*wo_;  //black box
		//double a2 = 2;
		//L_ << 5 * wo_, -a2 + 10 * wo_*wo_, -5 * a2*wo_ + 10 * wo_*wo_*wo_, a2*a2 - 10 * a2*wo_*wo_ + 5 * wo_*wo_*wo_*wo_, 5 * a2*a2*wo_ - 10 * a2*wo_*wo_*wo_ + wo_ * wo_*wo_*wo_*wo_;  //gray box
		//K_ << wc_, 4 * wc_*wc_, 6 * wc_*wc_*wc_, 4 * wc_*wc_*wc_*wc_, 1;
		K_ << 5 * wc_*wc_*wc_*wc_, 10 * wc_*wc_*wc_, 10 * wc_*wc_, 5 * wc_, 1;
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
		K_ << 5 * wc_*wc_*wc_*wc_, 10 * wc_*wc_*wc_, 10 * wc_*wc_, 5 * wc_, 1;
	}
	void setwo(double wo)
	{
		wo_ = wo;
		L_ = Vector5d::Zero();
		L_ << 5 * wo_, 10 * wo_*wo_, 10 * wo_*wo_*wo_, 5 * wo_*wo_*wo_*wo_, wo_*wo_*wo_*wo_*wo_;  //black box
		//double a2 = 2;
		//L_ << 5 * wo_, -a2 + 10 * wo_*wo_, -5 * a2*wo_ + 10 * wo_*wo_*wo_, a2*a2 - 10 * a2*wo_*wo_ + 5 * wo_*wo_*wo_*wo_, 5 * a2*a2*wo_ - 10 * a2*wo_*wo_*wo_ + wo_ * wo_*wo_*wo_*wo_;  /gray box
	}
	void setInput(double r)
	{
		r_ = r;
	}
	Vector5d update(double y)
	{
		Vector5d z1 = A_ * z_ + B_ * u_ + L_*(y - C_.transpose()*z_);
		z_ = z_ + h_ * z1;
		t_ += h_;

		return z_;
	}
	double getControlValue()
	{
		double u0 = K_(0)*(r_ - z_(0)) - (K_(1)*z_(1) + K_(2)*z_(2) + K_(3)*z_(3) + K_(4)*z_(4));
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
int main()
{
	ofstream out("out.txt");
	ofstream obsout("obsout.txt");

	double r = 0;
	Vector4d x0 = Vector4d::Zero();
	TwoMassModal twomass(x0, 0.001);
	SysModal::RungeKutta<Vector4d> rk(&twomass);

	ExtendedStateObserverForTwoMass eso(Vector5d::Zero());
	cout << "A= " << eso.A_ << endl;
	//eso.setwc(2);
	eso.setwo(5);

	//PIDController pid(1.5, 0, 0, 0.01);
	//double e = 0;

	int flag = 0;
	for (int k = 0; k < 20000; k++)
	{
		flag++;
		if (flag >= 10)
		{
			eso.update(twomass.readout());
			double u = eso.getControlValue();
			//e = r - twomass.readout();
			//double u = pid.update(e);
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
		Vector4d x=rk.update();
		cout << "t= " << rk.trans_->t_ << " , x= " << x.transpose() << "  u= "<<eso.u_<<endl;
		out << rk.trans_->t_ << " " << rk.trans_->x_.transpose() << endl;
		obsout << eso.t_ << " " << eso.z_.transpose() << endl;
	}

	return 0;
}




//  simulation for simple nonlinear model
/*
int main()
{
	ofstream out("out.txt");
	ofstream obsout("obsout.txt");
	//double v0 = 1;
	//double y = 0, e = 0, u = 0;
 //   PIDController pid(1.8, 1, 1.5, 0.01);
	double f = 0;
	Vector2d x0;
	x0 << 0.3, 0;
	TransModal trans(x0,0.001);
	trans.setInput(Vector2d::Zero());
	SysModal::RungeKutta<Vector2d> rk(&trans);

	StateObserver obs(x0);
	ExtendedStateObserver eso(Vector3d::Zero());
	eso.setBeta(100, 150, 3000);

	int flag = 0;
	for (int k = 0; k < 20000; k++)
	{
		flag++;
		if (flag >= 10)
		{
			//y = trans.readout();
			//e = v0 - y;
			//u = pid.update(e);
			//Vector2d uv;
			//uv << 0, u;
			//trans.setInput(uv);
			obs.update(trans.readout());
			eso.update(trans.readout());
			flag = 0;
		}
		Vector2d x=rk.update();
		cout << "t= " << rk.trans_->t_ << " , x= " << x.transpose() << endl;

		//f = trans.nonlinearfun(rk.trans_->t_, rk.trans_->x_) + trans.disturbfun(rk.trans_->t_);
		//f = trans.nonlinearfun_part1(rk.trans_->t_, rk.trans_->x_) + trans.disturbfun(rk.trans_->t_);
		//f = trans.nonlinearfun_part2(rk.trans_->t_, rk.trans_->x_) + trans.disturbfun(rk.trans_->t_);
		//f = trans.disturbfun(rk.trans_->t_);
		f= -rk.trans_->x_(0) - 0.25*rk.trans_->x_(1)*(1.5 - rk.trans_->x_(0)*rk.trans_->x_(0)) + sin(0.5*rk.trans_->t_);
		//f = eso.mysign(sin(0.5*rk.trans_->t_)) + 3 * cos(0.5*rk.trans_->t_);
		
		out << rk.trans_->t_ << " " << rk.trans_->x_.transpose() << endl;
		obsout << rk.trans_->t_ << " " << obs.z_.transpose() << " " << eso.z_.transpose() << " " << f << endl;
	}

	return 0;
}
*/