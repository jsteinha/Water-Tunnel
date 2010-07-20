#include <math.h>

struct CP_State{
		double y;
		double yd;
		double theta;
		double thetad;
};

class simpleObserver
{

public:
	simpleObserver();
	simpleObserver(double dt, CP_State x0); // Constructor
	~simpleObserver(){} ; // Destructor
	
	static void update(double* meas, const double u);
	static void getEstimate(CP_State* output);

private:
	static void sysDyn(CP_State curState, CP_State* curState_dot, const double u);
	static void dyn(CP_State curState, CP_State* curState_dot, const double u);

	static CP_State x_hat;
	static double* meas;

	static unsigned const int STATE_DIM = 4;
	static double dt;
};