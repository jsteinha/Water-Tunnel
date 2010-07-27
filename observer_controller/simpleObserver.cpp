#include "simpleObserver.h"
#include <stdio.h>

//fixed gain state estimator
simpleObserver::simpleObserver(double dt_in, CP_State x0)
{
	dt=dt_in;

	//create initial state estimate
	x_hat.y = x0.y;
	x_hat.yd = x0.yd;
	x_hat.theta = x0.theta;
	x_hat.thetad = x0.thetad;
}

void simpleObserver::getEstimate(CP_State* output)
{
	output->y = x_hat.y;
	output->yd = x_hat.yd;
	output->theta = x_hat.theta;
	output->thetad = x_hat.thetad;
}

void simpleObserver::update (double* meas_in, const double u)
{
	meas=meas_in;
	CP_State x_hatd;
	
	dyn(x_hat, &x_hatd, u);

	x_hat.y = meas_in[0]; //just use measured positions because encoders are very good
	x_hat.theta = meas_in[1];

	x_hat.yd = x_hat.yd + dt*x_hatd.yd;
	x_hat.thetad = x_hat.thetad + dt*x_hatd.thetad;

	//printf("y: %f   yd: %f\n",x_hat.y, x_hat.yd);
}

void simpleObserver::dyn(CP_State curState, CP_State* curState_dot, const double u)
{
	const static double eps=.1;

	sysDyn(curState,curState_dot,u);
	
	curState_dot->y		= curState_dot->y - 2/eps*(curState.y-meas[0]);
	curState_dot->theta = curState_dot->theta - 2/eps*(curState.theta-meas[1]);
	curState_dot->yd	= curState_dot->yd - 1/pow(eps,2)*(curState.yd-meas[0]);
	curState_dot->thetad = curState_dot->thetad - 1/pow(eps,2)*(curState.thetad-meas[1]);
}

void simpleObserver::sysDyn(CP_State curState, CP_State* curState_dot, const double u)
{
	curState_dot->y		= curState.yd;
	curState_dot->yd	= u;
	curState_dot->theta = curState.thetad;
	curState_dot->thetad = 0;
}

double* simpleObserver::meas;
CP_State simpleObserver::x_hat;
double simpleObserver::dt;