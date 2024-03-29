#ifdef TEST_ALL

#include <iostream>
#include<fstream>

using namespace std;

extern int Simulate_Toy_ESO(ofstream& out, ofstream& obsout, double setpoint = 1);
extern int Simulate_Toy_PID(ofstream& out, ofstream& obsout, double setpoint = 1);
extern int Simulate_TwoMass_ESO(ofstream& out, ofstream& obsout, double setpoint = 0);
extern int Simulate_TwoMass_PID(ofstream& out, double setpoint = 0);

//  test all
int main()
{
	ofstream out, obsout;

	out.open("toy_eso.txt");
	obsout.open("toy_eso_obsout.txt");
	Simulate_Toy_ESO(out, obsout);
	out.close();
	obsout.close();

	out.open("toy_pid.txt");
	obsout.open("toy_pid_obsout.txt");
	Simulate_Toy_PID(out, obsout);
	out.close();
	obsout.close();

	out.open("twomass_eso.txt");
	obsout.open("twomass_eso_obsout.txt");
	Simulate_TwoMass_ESO(out, obsout);
	out.close();
	obsout.close();

	out.open("twomass_pid.txt");
	Simulate_TwoMass_PID(out);
	out.close();

	return 0;
}

#endif