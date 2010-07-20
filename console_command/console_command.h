#include <stdio.h>
#include "servotube.h"
#include <math.h>

//lcm stuff
#include <lcm/lcm.h>
#include <getopt.h>
#include <windows.h>
#include <process.h>
#include <lcmt_servotubeCommand.h>
#include <lcmt_servotubeState.h>

unsigned __stdcall lcm_watcher(void *param);
static void send_message (lcm_t*, lcmt_servotubeState*);
static void my_handler (const lcm_recv_buf_t *rbuf, const char * channel, const lcmt_servotubeCommand * msg, void * user);

static const unsigned int traj_InterpBins = 111; //make odd to have value at su_baseThetaDot
static const unsigned int traj_centralBins = 51;
static const double trajectory_pos[traj_InterpBins] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static const double trajectory_neg[traj_InterpBins] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};