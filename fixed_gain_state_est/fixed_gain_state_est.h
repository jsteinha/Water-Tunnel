#include <gsl/gsl_blas.h>
#include <gsl/gsl_blas_types.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv.h>

#include <math.h>

class FGSE
{

public:
	FGSE(void (*dyn)(const double*, double*, const double*), const unsigned int STATE_DIM, const double dt, const double* x0) ; // Constructor
	~FGSE(){gsl_odeiv_step_free (s);}; // Destructor
	
	static void update(double* meas, double* u_in);
	static const double* getEstimate();

private:
	static const gsl_odeiv_step_type * T;
    static gsl_odeiv_step * s;
    static gsl_odeiv_system odesys;

	static int odefunc (double t, const double y[], double f[], void *params);
	static void (*sysDyn)(const double*, double*, const double*);
	static void dyn(const double*, double*, const double*);

	static double* x_hat;
	static double* meas;
	static double* u;

	static unsigned int STATE_DIM;
	static double dt;
};