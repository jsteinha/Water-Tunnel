#include "observer_controller.h"
#include <fstream>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <string>
#include "simpleObserver.h"

//lcm
static lcmt_servotubeState_subscription_t * sub;
static lcm_t * lcm;
//end lcm

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
lcmt_servotubeCommand positionTrackingController(double Yr, double Y, double Yd);
lcmt_servotubeCommand velocityTrackingController(double Ydr);
lcmt_servotubeCommand balancingController(double y, double theta, double yd, double thetad);

int readInSysIDFile(char*& filename, double*& wave);

int main(int argc, char** argv)
{
	bool isOutputting = false;
	bool keepRunning = true;
	bool finishExecution = false;
	int operatingMode; //1 normal, 2 sysID
	FILE* outputFile;
	char outputFilename[46] = "out_";

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

	case 2: //sysID operation
	case 3:
		char* filename;
		if(argc > 2)
			filename = argv[2]; //get filename from command line
		else
			filename = NULL; //prompt for file name in readInSysIDFile(...)

		sysIDlength = readInSysIDFile(filename, wave);

		strcat(outputFilename, filename);
		outputFile = fopen(outputFilename, "w");
		isOutputting = true;

		if(sysIDlength<0) //file reading error
			keepRunning = false;

		//for(int tmp = 0; tmp<100; tmp++)
		//	printf("Hey: %f\n",wave[tmp]);
		break;

	case 4: //balancing
		printf("Executing balancing control...\n");
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

	int iter=0;

	lcmt_servotubeCommand myComm;
	CP_State state; state.y=0.0; state.theta=0.0; state.yd=0.0; state.thetad=0.0;

	double x0[] = {0.0, 0.0, 0.0, 0.0};
	FGSE observer(hcp_dynamics, 4, dt, x0);
	const double* state_est;

	double lastu = 0.0;

	//system("pause");
	while(keepRunning)//begin control loop
	{
		iter++;

		//keep handling until you get a new state measurement
		//(may want to change this so that observer/controller
		//run on their own clock)
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

		case 2: //sysID position tracking
		case 3: //sysID velocity tracking
			if(operatingMode == 2)
				myComm = positionTrackingController(wave[iter-1], state.y, state.yd);
			else //operatingMode == 3
				myComm = velocityTrackingController(wave[iter-1]);

			fprintf(outputFile, "%f, %f, ",meas[0], meas[1]); 

			if(iter==sysIDlength) //turn off when reach end of tape
				keepRunning = false;
			break;

		case 4: //balancing operation
			myComm=balancingController(state.y, state.theta, state.yd, state.thetad);
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

	if(isOutputting)
		fclose(outputFile);

	return 0;
}
////////////////////////////
//Helper functions
////////////////////////////
int readInSysIDFile(char*& filename, double*& wave)
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
		sysIDlength = strtol(curVal, NULL, 0);
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

lcmt_servotubeCommand balancingController(double y, double theta, double yd, double thetad)
{
	lcmt_servotubeCommand myComm;
	myComm.commandType = 1; //acceleration command

	double Kyp = .5, Kyd = .2; //p and d gains for y
	double Ktp = -.5, Ktd = 0; //p and d gains for theta

	myComm.commandValue = -Kyp*y - Kyd*yd -Ktp*theta - Ktd*thetad ;

	return myComm;
}

lcmt_servotubeCommand positionTrackingController(double Yr, double Y, double Yd)
{
	lcmt_servotubeCommand myComm;
	double Kp = 15, Kd = 5;

	myComm.commandType = 1; //acceleration command
	myComm.commandValue = -Kp*(Y - Yr) - Kd*Yd;

	return myComm;
}

lcmt_servotubeCommand velocityTrackingController(double Ydr)
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

	//tdd
	q_dot[3]=0; //dont know yet!
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