#include "servotube_driver.h"
#include <fstream>
#include <time.h>
#include <stdlib.h>
#include <iostream>
#using <system.dll>

//lcm
static lcmt_hcp_u_subscription_t * sub;
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

using namespace System;
using namespace System::Timers;

static clock_t previousClock;
static clock_t curClock;
static clock_t firstClock;

static HANDLE lcm_watcher_thread;
static unsigned lcm_watcher_thread_ID;
static HANDLE mutex = CreateMutex(NULL, true, "wt_command_mutex");

static double deltaT=0.0;

static double measurements[2]; //store encoder measurements to send to state estimator

static int iter = 0;
static double finishingWaitTime=3;
static bool keepRunning = true;
static operationStateTypes operatingState;

static double curCommVal = 0.0;
static int curCommType = 1;

lcmt_hcp_y myState;
public ref class Timer1
{
private: 
	static System::Timers::Timer^ aTimer;

public:
	static void BeginTimerExecution()
	{

		// Create a new Timer with Interval set to 10 seconds.
		aTimer = gcnew System::Timers::Timer( 10000 );

		// Hook up the Elapsed event for the timer.
		aTimer->Elapsed += gcnew ElapsedEventHandler( Timer1::OnTimedEvent );

		aTimer->Interval = 1;
		aTimer->Enabled = true;
	}


private:
	// Specify what you want to happen when the Elapsed event is 
	// raised.
	static void OnTimedEvent( Object^ source, ElapsedEventArgs^ e )
	{
		iter++;

		//get positions from amp
		CartPole->update();
		measurements[0]=CartPole->get_motor_pos();
		measurements[1]=CartPole->get_load_enc_pos();
		//printf("Pos: %f   %f\n",measurements[0],measurements[1]);

		//set current clock and send measurements
		curClock = clock();
		myState.y = measurements[0];
		myState.theta = measurements[1];
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
	}

};


int main(int argc, char** argv)
{
	lcm = lcm_create ("udpm://");
	if (!lcm)
		return 1;

	lastCommandClock=clock();

	mutex = CreateMutex(NULL, true, "wt_command_mutex");
	ReleaseMutex(mutex);
	lcm_watcher_thread =(HANDLE)_beginthreadex( NULL , 0,&lcm_watcher, lcm, 0, &lcm_watcher_thread_ID);

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

	operatingState = executeLCMCommand;

	Timer1::BeginTimerExecution();
	while(keepRunning)//begin control loop
	{
		/*
		iter++;

		//get positions from amp
		CartPole->update();
		measurements[0]=CartPole->get_motor_pos();
		measurements[1]=CartPole->get_load_enc_pos();
		//printf("Pos: %f   %f\n",measurements[0],measurements[1]);

		//set current clock and send measurements
		curClock = clock();
		myState.y = measurements[0];
		myState.theta = measurements[1];
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
		}*/
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
	sub = lcmt_hcp_u_subscribe (lcm, "hcp_u", &my_handler, NULL);
	while(1)
	{
		//printf("Handling\n");
		lcm_handle((lcm_t*) param);
	}

	_endthreadex(0);
	return 0;
}

static void my_handler (const lcm_recv_buf_t *rbuf, const char * channel, 
						const lcmt_hcp_u * msg, void* user)
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

static void send_message (lcm_t * lcm, lcmt_hcp_y* my_data)
{	
	lcmt_hcp_y_publish (lcm, "hcp_y", my_data);
}