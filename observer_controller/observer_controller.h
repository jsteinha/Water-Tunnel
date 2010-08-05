#include <stdio.h>
#include <math.h>
#include <learning_at_balance.h>

//lcm stuff
#include <lcm/lcm.h>
#include <getopt.h>
#include <windows.h>
#include <process.h>
#include <lcmt_servotubeCommand.h>
#include <lcmt_servotubeState.h>
#include <fixed_gain_state_est.h>

//cp state

unsigned __stdcall lcm_watcher(void *param);
static void send_message (lcm_t*, lcmt_servotubeCommand*);
static void my_handler (const lcm_recv_buf_t *rbuf, const char * channel, const lcmt_servotubeState * msg, void * user);