#include <math.h>
#include <lcmt_servotubeCommand.h>
#include <lcmt_servotubeState.h>

class LaBcontroller
{

public:
	LaBcontroller(){};
	LaBcontroller(double (*costFunc_in)(const double*, const double*), 
				 double* initialGains, double dt_in, unsigned int STATE_DIM_IN) ; // Constructor
	~LaBcontroller(){}; // Destructor
	
	lcmt_servotubeCommand getBalancingCommand(CP_State state);
	double performUpdate();
	void setLearningParams(double baseline_in, double gamma_in, double eta_in, double* sigma_in);
	void setBaseline(double);
	void setGamma(double);
	void setEta(double);
	void setSigma(double*);
	void setCostChangeSaturation(double ccs_in);

private:
	unsigned int STATE_DIM; //set up for cartpole
	double dt;
	double* curGains;
	double* gainPerturbs;

	double (*costFunc)(const double* x, const double* u);
	void incrementCost(CP_State state, const double u);
	void perturbGains();

	double integratedCost;
	double costChangeSaturation;

	double baseline;
	double gamma;
	double eta;
	double* sigma;
};