#include "observer_controller.h"
#include <fstream>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <math.h>

//lcm
static lcmt_servotubeState_subscription_t * sub;
static lcm_t * lcm;
//end lcm

static const double PI = 3.14159265358979;

static double position = 0.0;
static double secondaryPosition= 0.0;
static int msgNum=0;
static int lastMsgNum=0;
static int thisMsgNum=0;
static double meas[2] = {0.0,0.0};

static double speedVkhz = .3;
static double dt = .001/speedVkhz;

static void hcp_dynamics(const double* q, double* q_dot, const double* u);
static double recenterAccelGen(double x,double xd);
static double LQRbalancingAccelGen(double t,double x,double td,double xd);
static lcmt_servotubeCommand positionTrackingController(double Yr, double Y, double Yd);
static lcmt_servotubeCommand velocityTrackingController(double Ydr);
static lcmt_servotubeCommand balancingController(double y, double theta, double yd, double thetad);
static lcmt_servotubeCommand IDBalancingController(double y, double theta, double yd, double thetad, double yr, double ydr);
static double costFunc(const double*, const double*);

static inline double mod_by_2PI(double th) { return fmod(fmod(th,2*PI)+3*PI,2*PI)-PI;; }

static int readInSysIDFile(char*& filename, double*& wave, int entriesPerDT);

int main(int argc, char** argv)
{
	bool sysIDOutputting = false;
	bool isLearning = false;
	bool keepRunning = true;
	bool finishExecution = false;

	int operatingMode;
	FILE* outputFile;
	FILE* learningRecord;
	char outputFilename[46] = "out_";
	int entriesPerDT = 1;

	LaBcontroller balanceLearner;
	const int itersPerUpdate = 3000;

	//sysid variables
	int sysIDlength = 0;
	double* wave = NULL;

	operatingMode = 1; //default to recentering operation
	if(argc > 1){
		sscanf(argv[1],"%d",&operatingMode);
	}

	switch(operatingMode)
	{
	case 0: //cease operation
		printf("Commanding termination...\n");
		break;

	case 1: //recenter operation
		printf("Commanding recentering...\n");
		break;

	case 2: //balancing
		printf("Executing balancing control...\n");
		break;

	case 3: //sysID operation
	case 4:
	case 5:
		char* filename;
		if(argc > 2)
			filename = argv[2]; //get filename from command line
		else
			filename = NULL; //prompt for file name in readInSysIDFile(...)

		if(operatingMode==5) //else entriesPerDT = 1 (set at declaration)
			entriesPerDT=2;

		sysIDlength = readInSysIDFile(filename, wave, entriesPerDT);

		strcat(outputFilename, filename);
		outputFile = fopen(outputFilename, "w");
		sysIDOutputting = true;

		if(sysIDlength<0) //file reading error
			keepRunning = false;
		break;

	case 6: //learning at balance
		isLearning = true;
		learningRecord = fopen("learningRecord.dat", "w");
		printf("Executing learning at balance control...\n");
		{
			double initialGains[]={-0.8, 0.277, -0.56, 0.42};
			balanceLearner = *(new LaBcontroller(costFunc,initialGains, dt, 4));
			//set learning params here if default is bad
		}
		break;

	case 99: //debug
		break;

	default:
		printf("Command not understood. Turning off.\n\n");
		return 0;
		break;
	}


	lcm = lcm_create ("udpm://");
	if (!lcm)
		return 1;

	sub = lcmt_servotubeState_subscribe (lcm, "wt_state", &my_handler, NULL);
	HANDLE lcm_watcher_thread;
	unsigned lcm_watcher_thread_ID;
	static HANDLE mutex = CreateMutex(NULL, false, "wt_state_mutex");
	ReleaseMutex(mutex);
	//lcm_watcher_thread =(HANDLE)_beginthreadex( NULL , 0,&lcm_watcher, lcm, 0, &lcm_watcher_thread_ID);

	unsigned int iter=0;

	lcmt_servotubeCommand myComm;
	CP_State state; state.y=0.0; state.theta=0.0; state.yd=0.0; state.thetad=0.0;

	double x0[] = {0.0, 0.0, 0.0, 0.0};
	FGSE observer(hcp_dynamics, 4, dt, x0);
	const double* state_est;

	double lastu = 0.0;
	double episodeCost = 0.0;

	while(keepRunning)//begin control loop
	{
		iter++;

		//keep handling until you get a new state measurement
		while(lastMsgNum==thisMsgNum)
		{
			lcm_handle(lcm);
			thisMsgNum = msgNum;
		}

		meas[0] = position;
		meas[1] = secondaryPosition;
		lastMsgNum=thisMsgNum;

		//observer
		observer.update(meas, &lastu);
		state_est = observer.getEstimate();
		state.y=state_est[0]; state.theta=state_est[1]; state.yd=state_est[2]; state.thetad=state_est[3];

		//controller
		switch(operatingMode)
		{
		case 0: //stop operation
			myComm.commandType=-1;
			myComm.commandValue=0;
			keepRunning = false;
			break;

		case 1: //recentering operation
			myComm.commandType=0;
			myComm.commandValue=0.0;
			keepRunning = false;
			break;

		case 2: //balancing operation
			//printf("theta: %f\n",mod_by_2PI(state.theta));
			myComm=balancingController(state.y, mod_by_2PI(state.theta), state.yd, state.thetad);
			break;

		case 3: //sysID position tracking
		case 4: //sysID velocity tracking
			if(operatingMode == 2)
				myComm = positionTrackingController(wave[iter-1], state.y, state.yd);
			else //operatingMode == 3
				myComm = velocityTrackingController(wave[iter-1]);

			fprintf(outputFile, "%f, %f, ",meas[0], meas[1]); 

			if(iter==sysIDlength/entriesPerDT) //turn off when reach end of tape
				keepRunning = false;
			break;

		case 5: //sysID closed-loop
			myComm = IDBalancingController(state.y, state.theta, 
				state.yd, state.thetad, wave[2*iter-2], wave[2*iter-1]);

			fprintf(outputFile, "%f, %f, ",meas[0], meas[1]); 

			if(iter==sysIDlength/entriesPerDT) //turn off when reach end of tape
				keepRunning = false;
			break;

		case 6: //learning at balance
			myComm = balanceLearner.getBalancingCommand(state);

			if(iter%itersPerUpdate == 0)
			{
				episodeCost = balanceLearner.performUpdate();
				printf("Cost: %f\n", episodeCost);
				fprintf(learningRecord, "%f, ",episodeCost);
			}
			break;

		case 99: //debug operation
			myComm.commandType=1;
			myComm.commandValue=-.01;
			break;


		default:
			keepRunning=false; //should never reach here
		}

		lastu = myComm.commandValue;
		send_message (lcm, &myComm);
	}

	if(sysIDOutputting)
		fclose(outputFile);

	if(isLearning)
		fclose(learningRecord);

	return 0;
}
////////////////////////////
//Helper functions
////////////////////////////
static double costFunc(const double* x, const double* u)
{
	double R= .01;
	double diagQ[] ={20, 5, 2, .5};

	int STATE_DIM = 4;

	double cost=R*pow((*u),2);

	for(int i=0; i<STATE_DIM; i++)
		cost +=diagQ[i]*pow(x[i],2);

	return cost;
}

static int readInSysIDFile(char*& filename, double*& wave, int entriesPerDT)
{
	char curVal[15];
	int whichWaveEl=0;
	int sysIDlength = -1;

	if(filename == NULL)
	{
		filename = new char[40];
		printf("Enter name of sysID waveform file: ");
		std::cin>>filename;
	}

	std::ifstream inf(filename);

	if (inf.fail())
	{
		printf("File not found. Closing...\n");
		return -1; // return error code for no file
	}
	else
	{
		inf.getline(curVal, 15);
		sysIDlength = entriesPerDT*strtol(curVal, NULL, 0);
		wave = new double[sysIDlength];

		while(!inf.getline(curVal, 15).eof())
		{
			if(whichWaveEl==sysIDlength)
			{
				whichWaveEl++; //increment so that error message appears
				break;
			}

			wave[whichWaveEl]=strtod(curVal, NULL);
			//printf("Hey: %f\n",wave[whichWaveEl]);
			whichWaveEl++;
		}

		if(whichWaveEl!=sysIDlength)
		{
			printf("Error, length of tape not specified correctly, shutting down...");
			return  -2; //return error code for wrong tape length
		}

		return sysIDlength;
	}
}

static lcmt_servotubeCommand balancingController(double y, double theta, double yd, double thetad)
{
	lcmt_servotubeCommand myComm;
	myComm.commandType = 1; //acceleration command

	double Kyp = -0.8, Kyd = -0.56; //p and d gains for y
	double Ktp = 0.277, Ktd = 0.42; //p and d gains for theta

	myComm.commandValue = -Kyp*y - Kyd*yd -Ktp*theta - Ktd*thetad ;

	return myComm;

}

static lcmt_servotubeCommand IDBalancingController(double y, double theta, double yd, 
												   double thetad, double yr, double ydr)
{
	lcmt_servotubeCommand myComm;
	myComm.commandType = 1; //acceleration command

	double Kyp = -0.8, Kyd = -0.56; //p and d gains for y
	double Ktp = 0.277, Ktd = 0.42; //p and d gains for theta

	myComm.commandValue = -Kyp*(y-yr) - Kyd*(yd-ydr) -Ktp*theta - Ktd*thetad ;

	return myComm;
}

static lcmt_servotubeCommand positionTrackingController(double Yr, double Y, double Yd)
{
	lcmt_servotubeCommand myComm;
	double Kp = 15, Kd = 5;

	myComm.commandType = 1; //acceleration command
	myComm.commandValue = -Kp*(Y - Yr) - Kd*Yd;

	return myComm;
}

static lcmt_servotubeCommand velocityTrackingController(double Ydr)
{
	lcmt_servotubeCommand myComm;

	myComm.commandType = 2; //velocity command
	myComm.commandValue =Ydr;

	return myComm;
}

static double LQRbalancingAccelGen(double tbar,double x,double td,double xd)
{
	//const double gains[]={268.326692168944,-22.3606797749996,79.7868227159699,-40.9696662360675};
	const double gains[] = {283.8022,  -22.3607,   92.3769,  -41.8218}; //using physical params

	double accel=-1*(
		gains[0]*tbar
		+gains[1]*x
		+gains[2]*td
		+gains[3]*xd
		);

	return accel;
}

static void hcp_dynamics(const double* q, double* q_dot, const double* u){
 	q_dot[0]=q[2];//xd
	q_dot[1]=q[3];//td
	q_dot[2]=*u;//xdd

	double a=17.4;
	double b=9.69;
	double c=4.61;
	double d=0.900;

	//tdd at upright    theta     yd       thetad     u
	q_dot[3]=            d*q[1] +b*q[2]   -c*q[3]  +a*(*u);
}

static double recenterAccelGen(double x,double xd)
{
	return -.5*x-.3*xd;
}

//LCM Code
unsigned __stdcall lcm_watcher(void *param)
{
	sub = lcmt_servotubeState_subscribe (lcm, "wt_state", &my_handler, NULL);
	while(1)
	{
		lcm_handle((lcm_t*) param);
	}

	_endthreadex(0);
	return 0;
}

static void my_handler (const lcm_recv_buf_t *rbuf, const char * channel, 
						const lcmt_servotubeState * msg, void* user)
{
	static HANDLE mutex = CreateMutex(NULL, false, "wt_state_mutex");

	if(WaitForSingleObject(mutex, 10000)==WAIT_TIMEOUT)
	{
		printf("MUTEX TIMEOUT ERROR 2\n");
	}
	else
	{
		position= msg->position;
		secondaryPosition= msg->positionSecondary;
		msgNum++;
	}
	ReleaseMutex(mutex);
}

static void send_message (lcm_t * lcm, lcmt_servotubeCommand* my_data)
{	
	//printf("Sent: %f\n",my_data->commandValue);
	lcmt_servotubeCommand_publish (lcm, "wt_command", my_data);
}