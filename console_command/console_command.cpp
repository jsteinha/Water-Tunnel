#include "console_command.h"
#include <fstream>
#include <time.h>
#include <stdlib.h>
#include <iostream>

//lcm
static lcmt_servotubeCommand_subscription_t * sub;
static lcm_t * lcm;
static int commandType = 1;
static double commandVal= 0.0;
static int final_iter = 0;
//end lcm

enum operationStateTypes{ executeLCMCommand, recenter, idle, finishAndTerminate};

clock_t lastCommandClock;
double allowableCommandWait = .1;

static double dt = .001/ServoTube::speedVkhz;
static const double omega=4.7385;
static double Etilde; //unitless energy error

static operationStateTypes parseLCMCommand(int curCommType, double curCommVal, ServoTube* CartPole);
static double recenterAccelGen(double x, double xd);
static bool nearEnoughToZero(ServoTube* CartPole);

ServoTube* CartPole;
KvaserCAN* kvaserCANbus;
const char *canDevice = "kvaser0"; // Identifies the CAN device, if necessary

int main(int argc, char** argv)
{
	lcm = lcm_create ("udpm://");
	if (!lcm)
		return 1;

	lastCommandClock=clock();
	clock_t previousClock;
	clock_t curClock;
	clock_t firstClock;

	HANDLE lcm_watcher_thread;
	unsigned lcm_watcher_thread_ID;
	static HANDLE mutex = CreateMutex(NULL, true, "wt_command_mutex");
	ReleaseMutex(mutex);
	lcm_watcher_thread =(HANDLE)_beginthreadex( NULL , 0,&lcm_watcher, lcm, 0, &lcm_watcher_thread_ID);

	//take user input for menu
	char userinput;

	if(argc>1)
		userinput = argv[1][0];
	else
	{
		printf("STARTUP MENU \n\n");

		printf("Select one of the options below: \n");
		printf("[1] Begin operation\n");

		printf("Enter command: ");
		std::cin>>userinput;
		printf("\n\n");

		switch(userinput)
		{
		case '1':
			//normal operation
			break;

		default:
			printf("Command not understood. Turning off.\n\n");
			return 0;
			break;
		}
	}

	//setup can connection
	kvaserCANbus = new KvaserCAN( canDevice );
	CartPole = new ServoTube(kvaserCANbus);

	//wait till connection established to set initial clocks (curClock will be initialized in loop)
	firstClock = clock();
	previousClock = firstClock;

	double deltaT=0.0;

	double measurements[2]; //store encoder measurements to send to state estimator

	int iter = 0;
	double finishingWaitTime=3;
	bool keepRunning = true;
	operationStateTypes operatingState = executeLCMCommand;

	double curCommVal = 0.0;
	int curCommType = 1;

	lcmt_servotubeState myState;

	while(keepRunning)//begin control loop
	{
		iter++;

		//get positions from amp
		CartPole->update();
		measurements[0]=CartPole->get_motor_pos();
		measurements[1]=CartPole->get_load_enc_pos();
		//printf("Pos: %f   %f\n",measurements[0],measurements[1]);
		
		//set current clock and send measurements
		curClock = clock();
		myState.position = measurements[0];
		myState.positionSecondary = measurements[1];
		send_message (lcm, &myState);

		//read in command if not finishing
		if(operatingState != finishAndTerminate && operatingState != recenter)
		{
			if(WaitForSingleObject(mutex, 10000)==WAIT_TIMEOUT)
				printf("MUTEX TIMEOUT ERROR 4\n");
			else
			{
				bool gotRecentCommand = (1.0*(curClock-lastCommandClock))/CLOCKS_PER_SEC <= allowableCommandWait;

				if( !gotRecentCommand && operatingState != idle && iter!= 1)
				{
					operatingState = idle;
					printf("No command received for %f seconds. Idling...\n",allowableCommandWait);
				}

				if( ((1.0*(curClock-lastCommandClock))/CLOCKS_PER_SEC < allowableCommandWait) )
					operatingState = executeLCMCommand;

				curCommVal = commandVal;
				curCommType = commandType;
			}
			ReleaseMutex(mutex);
		}

		switch(operatingState)
		{
		case executeLCMCommand:
			operatingState = parseLCMCommand(curCommType,curCommVal,CartPole);
			break;

		case recenter:
			if(nearEnoughToZero(CartPole))
			{
				CartPole->set_vel_command(0.0);
				operatingState = idle;
			}
			else
				CartPole->accelerate(recenterAccelGen(measurements[0], CartPole->get_motor_vel()));
			break;

		case idle:
			if(abs(CartPole->get_motor_vel())<.02)
				CartPole->set_vel_command(0.0);
			else
				CartPole->accelerate(-15*CartPole->get_motor_vel());
			break;

		case finishAndTerminate:
			if(nearEnoughToZero(CartPole))
			{
				CartPole->set_vel_command(0.0);
				keepRunning = false;
			}
			else
				CartPole->accelerate(recenterAccelGen(measurements[0], CartPole->get_motor_vel()));
			break;

		default:
			printf("Shouldn't get here!\n");
			CartPole->accelerate(recenterAccelGen(measurements[0], CartPole->get_motor_vel()));
			operatingState = finishAndTerminate;
			break;
		}

		if(iter%500==0)
		{
			deltaT=(1.0*(curClock-previousClock))/CLOCKS_PER_SEC;
			previousClock=curClock;
			//printf("dt: %f\n\n",deltaT/500.0);
		}
	}

	CartPole->set_vel_command(0.0); //zero velocity on exit
	return 0;
}
////////////////////////////
//Helper functions
////////////////////////////
static bool nearEnoughToZero(ServoTube* CartPole)
{
	const double posThreshold = .0025; //m
	const double velThreshold = .025; //m/s
	return abs(CartPole->get_motor_vel())<velThreshold && abs(CartPole->get_motor_pos())<posThreshold;
}

static operationStateTypes parseLCMCommand(int curCommType, double curCommVal, ServoTube* CartPole)
{
	switch(curCommType)
	{
	case 0:
		printf("Recentering...\n");
		return recenter;
		break;

	case 1:
		CartPole->accelerate(curCommVal);
		return executeLCMCommand;
		break;

	case 2:
		CartPole->set_vel_command(curCommVal);
		return executeLCMCommand;
		break;

	case -1:
		printf("Stopping operation on terminate signal...\n");
		return finishAndTerminate;
		break;

	default:
		printf("LCM command not understood. Stopping operation...\n");
		return finishAndTerminate;
		break;
	}
}

static double recenterAccelGen(double x, double xd)
{
	return -15*x - 15*xd;
}

//LCM Code
unsigned __stdcall lcm_watcher(void *param)
{
	sub = lcmt_servotubeCommand_subscribe (lcm, "wt_command", &my_handler, NULL);
	while(1)
	{
		//printf("Handling\n");
		lcm_handle((lcm_t*) param);
	}

	_endthreadex(0);
	return 0;
}

static void my_handler (const lcm_recv_buf_t *rbuf, const char * channel, 
        const lcmt_servotubeCommand * msg, void* user)
{
	static HANDLE mutex = CreateMutex(NULL, false, "wt_command_mutex");

	if(WaitForSingleObject(mutex, 10000)==WAIT_TIMEOUT)
	{
		printf("MUTEX TIMEOUT ERROR 3\n");
	}
	else
	{
		commandType= msg->commandType;
		commandVal= msg->commandValue;
		lastCommandClock = clock();
		//printf("comVal: %f\n",commandVal);
	}
	ReleaseMutex(mutex);
}

static void send_message (lcm_t * lcm, lcmt_servotubeState* my_data)
{	
	lcmt_servotubeState_publish (lcm, "wt_state", my_data);
}