#include "fixed_gain_state_est.h"

//fixed gain state estimator
FGSE::FGSE (void (*sysDyn_in)(const double*, double*, const double*), const unsigned int STATE_DIM_IN, const double dt_in, const double* x0)
{
	sysDyn=sysDyn_in;
	STATE_DIM=STATE_DIM_IN;
	dt=dt_in;

	T = gsl_odeiv_step_rkf45;
	s = gsl_odeiv_step_alloc (T, STATE_DIM_IN);

	odesys.function = odefunc;
	odesys.jacobian = NULL;
	odesys.dimension = STATE_DIM_IN;
	odesys.params = NULL;

	//create initial state estimate
	x_hat=new double[STATE_DIM];
	for(unsigned int i=0;i<STATE_DIM;i++)
		x_hat[i]=x0[i];
}

const double* FGSE::getEstimate()
{
	return x_hat;
}

int FGSE::odefunc (double t, const double y[], double f[], void *params)
{
	dyn(y,f,u);

	return GSL_SUCCESS;
}


void FGSE::update (double* meas_in, double* u_in)
{
	meas=meas_in;
	u = u_in;

	double* yerr = new double[STATE_DIM];

	gsl_odeiv_step_apply (s, 0.0, dt, x_hat, 
		yerr, NULL, NULL, &odesys);

	//RK4(x_hat, control, dyn, STATE_DIM, dt);
	x_hat[0] = meas_in[0]; //override output because enc give such good state est
	x_hat[1] = meas_in[1];
}

void FGSE::dyn(const double* x, double* xd, const double* u_in)
{
	const static double eps=.01;

	sysDyn(x,xd,u_in);
	
	for(unsigned int i=0;i<STATE_DIM;i++)
	{
		if(i<STATE_DIM/2)
			{
				xd[i] = xd[i] - 2/eps*(x[i]-meas[i]);
			}
			else
			{
				xd[i] = xd[i] - 1/pow(eps,2)*(x[i-2]-meas[i-2]);
			}
	}
}

void (*FGSE::sysDyn)(const double*, double*, const double*);

double* FGSE::x_hat;
double* FGSE::meas;
const gsl_odeiv_step_type* FGSE::T;
gsl_odeiv_step* FGSE::s;
gsl_odeiv_system FGSE::odesys;
double* FGSE::u;

unsigned int FGSE::STATE_DIM;
double FGSE::dt;