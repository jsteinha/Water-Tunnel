#include <stdio.h>
#include "servotube.h"
#include <math.h>

//lcm stuff
#include <lcm/lcm.h>
#include <getopt.h>
#include <windows.h>
#include <process.h>
#include <lcmt_hcp_u.h>
#include <lcmt_hcp_y.h>

unsigned __stdcall lcm_watcher(void *param);
static void send_message (lcm_t*, lcmt_hcp_y*);
static void my_handler (const lcm_recv_buf_t *rbuf, const char * channel, const lcmt_hcp_u * msg, void * user);