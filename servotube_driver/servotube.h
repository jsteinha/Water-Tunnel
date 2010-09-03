#define useDebugFile 0

#include <cstdio>
#include <cstdlib>
#include "CML.h"
#include "can/can_kvaser.h"
#include <math.h>
#include <windows.h>
// If a namespace has been defined in CML_Settings.h, this
// macros starts using it. 
CML_NAMESPACE_USE();

/* local functions */
static void showerr( const Error *err, const char *str );

//ServoTube object
class ServoTube
{
	const Error *err;

public:
	ServoTube();
	ServoTube(KvaserCAN *kvaserCANbus) ; // Constructor
	
	#if useDebugFile
	~ServoTube(){fclose(debugFile);} ; // Destructor
	#else
	~ServoTube(){}; 
	#endif

	//FILE* debugFile;
	void update();
	void reset_vel_command();
	void accelerate(double accelerationRate);
	void goto_trap(double pos);
	void move_rel_trap(double move_amount);
	void set_amp_mode(AMP_MODE inputAmpMode);
	void set_vel_command();
	double get_vel_command();
	void set_vel_command(double vel_com_ovrd);
	double get_load_enc_pos();
	double get_load_enc_vel();
	double get_motor_pos();
	double get_motor_vel();
	
	static const double VEL_SAT_VALUE;
	static const double POS_SAT_VALUE;
	static const double speedVkhz;

private:
	static int cur_mot_count;
	//buffer and velocity filter for encoder
	static const int buf_length=11;
	//velocity filter from matlab; 11 tap with low pass behavior
	static const double vel_filter[buf_length];
	static const double COUNTS_TO_METERS;
	static const double COUNTS_TO_RAD;

	double load_enc_buf[buf_length];
	double motor_pos_buf[buf_length];

	double vel_com;

	int buf_index;

	CanOpen canOpen;
	Amp amp;
	HomeConfig hcfg;
	ProfileConfigTrap trap;
	ProfileConfigScurve scurve;
	KvaserCAN *can;
	uint32 LoadEncoderPDOid;
	AMP_MODE curAmpMode;

	//TPDO *ldEnc;

	/* local data */
	static const int32 canBPS=1000000;             // CAN network bit rate
	static const int16 canNodeID = 1;                // CANopen node ID
};