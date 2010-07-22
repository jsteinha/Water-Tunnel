#include "servotube.h"

//filter made for 1.488kHz operation, so include multiplier to correct and get to right frequency
const double ServoTube::vel_filter[buf_length]={0.649013933906495,15.2461761064155,15.1733466572846,15.3172488195119,9.60962219383544,0,-9.60962219383544,-15.3172488195119,-15.1733466572846,-15.2461761064155,-0.649013933906495};

const double ServoTube::speedVkhz = .3;
const double ServoTube::VEL_SAT_VALUE=.2;
const double ServoTube::POS_SAT_VALUE=.056;
const double ServoTube::COUNTS_TO_METERS=0.0256/4096;
const double ServoTube::COUNTS_TO_RAD=0.00125663706;
int ServoTube::cur_mot_count = 0;

static void showerr( const Error *err, const char *str )
{
	if( err )
	{
		printf( "Error %s: %s\n", str, err->toString() );
		exit(1);
	}
}

//Begin ServoTube object functions
void ServoTube::update()
{
	int32 enc_ticks;
	int32 mot_ticks;
	amp.sdo.Upld32(0x2242,0x0,enc_ticks);
	amp.sdo.Upld32(0x6063,0x0,mot_ticks);

	//cur_mot_count = mot_ticks;
	/*
	uint32 tmp;
	CanFrame tmpFrame;
	tmpFrame.type=CAN_FRAME_DATA;
	
	(*ldEnc).Request();
	
	do
	{
		(*can).Recv(tmpFrame,-1);
	}
	while(tmpFrame.id!=LoadEncoderPDOid);
	
	tmp=(uint32)tmpFrame.data[3];tmp=tmp*0x100;
	tmp=tmp+(uint32)tmpFrame.data[2];tmp=tmp*0x100;
	tmp=tmp+(uint32)tmpFrame.data[1];tmp=tmp*0x100;
	tmp=tmp+(uint32)tmpFrame.data[0];

	enc_ticks=(int32) tmp;
	*/

	//increment buffer index, and set next value to load position in radians
	//printf("Enc Ticks: %d\n\n",enc_ticks);

	++buf_index;
	buf_index=buf_index%buf_length;
	load_enc_buf[buf_index]=((double)enc_ticks)*COUNTS_TO_RAD;
	motor_pos_buf[buf_index]=((double)mot_ticks)*COUNTS_TO_METERS;
}



void ServoTube::goto_trap(double pos)
{
	trap.pos = pos/COUNTS_TO_METERS;
	
	if(trap.pos>POS_SAT_VALUE)
		trap.pos=POS_SAT_VALUE;
	else if(trap.pos<-POS_SAT_VALUE)
		trap.pos=-POS_SAT_VALUE;

	err=amp.DoMove( trap );
	amp.WaitMoveDone( 30000 );
}

void ServoTube::move_rel_trap(double move_amount)
{
	trap.pos = trap.pos+move_amount;
	if(trap.pos>POS_SAT_VALUE)
		trap.pos=POS_SAT_VALUE;
	else if(trap.pos<-POS_SAT_VALUE)
		trap.pos=-POS_SAT_VALUE;

	err=amp.DoMove( trap );
}

void ServoTube::set_amp_mode(AMP_MODE inputAmpMode)
{
	if(curAmpMode!=inputAmpMode){
		err = amp.SetAmpMode( inputAmpMode );
		curAmpMode=inputAmpMode;
	}
}

void ServoTube::reset_vel_command()
{
	vel_com=0.0;
}

void ServoTube::accelerate(double accelRate)
{
	double accel_cap=100;
	if(accelRate>accel_cap)
		accelRate=accel_cap;

	if(accelRate<-accel_cap)
		accelRate=-accel_cap;

	//printf("vel_com: %f\n", vel_com);
	vel_com=vel_com+accelRate*.001/speedVkhz;
	//printf("vel_com: %f\n", vel_com);
	this->set_vel_command();
}

void ServoTube::set_vel_command(double vel_com_ovrd)
{
	vel_com=vel_com_ovrd;
	set_vel_command();
}

double ServoTube::get_vel_command()
{
	return vel_com;
}

void ServoTube::set_vel_command()
{	
	static bool alreadyDisplayed = false;
	//printf("Vel com: %f\n\n",vel_com);
	//printf("Motor Pos: %d\n",motor_pos);
	if(abs(this->get_motor_pos())>POS_SAT_VALUE)
	{
		if(!alreadyDisplayed)
		{
			printf("Hit software position stop...\n");
			alreadyDisplayed = true;
		}
		vel_com=0.0;
	}
	
	#if useDebugFile
		fprintf(debugFile, "%d, %f, ",cur_mot_count,vel_com/COUNTS_TO_METERS);
	#endif

	if(alreadyDisplayed && abs(this->get_motor_pos())<POS_SAT_VALUE)
	{
		alreadyDisplayed = false;
		printf("Clear of software position stop...\n");
	}

	if(vel_com>VEL_SAT_VALUE)
		vel_com=VEL_SAT_VALUE;
	else if(vel_com<-VEL_SAT_VALUE)
		vel_com=-VEL_SAT_VALUE;
	amp.SetVelocityProgrammed(vel_com/COUNTS_TO_METERS);	
	amp.SetAmpMode(AMPMODE_PROG_VEL);
	//printf("VelCom: %f\n",vel_com);
	//printf("MotPos: %f\n",get_motor_pos());
}

double ServoTube::get_motor_pos()
{
	return motor_pos_buf[buf_index];
}

double ServoTube::get_motor_vel()
{
	double tmp=0;
	for(int i=0;i<buf_length;i++)
	{
		tmp+=speedVkhz*vel_filter[i]*motor_pos_buf[(buf_index-i+buf_length)%buf_length];
	}

	return tmp;
}

double ServoTube::get_load_enc_pos()
{
	return load_enc_buf[buf_index];
}

double ServoTube::get_load_enc_vel()
{
	double tmp=0;
	for(int i=0;i<buf_length;i++)
	{
		tmp+=speedVkhz*vel_filter[i]*load_enc_buf[(buf_index-i+buf_length)%buf_length];
	}

	return tmp;
}

ServoTube::ServoTube()
{
}

ServoTube::ServoTube(KvaserCAN *kvaserCANbus)
{
	#if useDebugFile
		debugFile= fopen("servotubeDB.dat", "w");
	#endif

	curAmpMode=AMPMODE_CAN_PROFILE;
	LoadEncoderPDOid=0x00000580+canNodeID;

	vel_com=0.0;

	//ldEnc=new TPDO(canOpen, (uint32) LoadEncoderPDOid);

	//initialize buffer to 0
	buf_index=0;
	for(int i=0;i<buf_length;i++)
	{
		load_enc_buf[i]=0.0;
		motor_pos_buf[i]=0.0;
	}

	can = kvaserCANbus;

	// The libraries define one global object of type
	// CopleyMotionLibraries named cml.
	//
	// This object has a couple handy member functions
	// including this one which enables the generation of
	// a log file for debugging
	cml.SetDebugLevel( LOG_ERRORS );

	can->SetBaud( canBPS );

	// Open the CANopen network object
	const Error *err = canOpen.Open( *can );
	showerr( err, "Opening CANopen network" );

	// Initialize the ServoTube using default settings
	printf( "Doing init\n" );
	err = amp.Init( canOpen, canNodeID );
	showerr( err, "Initting amp" );

	/*
	//map a PDO to transfer the load encoder
	amp.sdo.Dnld32(0x1807,0x1, 0x80000000); //disable channel
	amp.sdo.Dnld8(0x1A07,0x0, (uint8)0); //say sending no objects
	amp.sdo.Dnld32(0x1A07,0x1, 0x22420020); //say transmit load encoder position
	amp.sdo.Dnld8(0x1A07,0x0, (uint8)1); //say sending one object
	amp.sdo.Dnld8(0x1807,0x2, (uint8) 1); //say transmit on every transmitFreq SYNCH cycles						   
	amp.sdo.Dnld32(0x1807,0x1, LoadEncoderPDOid); //name channel to transmit
	*/

	// Home the motor.
	hcfg.method  = CHM_NONE;
	hcfg.velFast = 100000;
	hcfg.velSlow = 50000;
	hcfg.accel   = 90000;
	hcfg.offset  = 0;

	err = amp.GoHome( hcfg );
	showerr( err, "Going home" );

	printf( "Waiting for home to finish...\n" );
	err = amp.WaitMoveDone( 20000 ); 
	showerr( err, "waiting on home" );

	// Setup the move speeds.  For simplicity I'm just using the same
	// vel, acc & decel for all moves.
	printf( "Setting up moves...\n" );

	trap.vel = 8000000;
	trap.acc = 500000;
	trap.dec = 500000;

	scurve.vel = 800000;
	scurve.acc = 50000;
	scurve.jrk = 8000;
}