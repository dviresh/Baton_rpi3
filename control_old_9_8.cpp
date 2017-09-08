// Standard C headers
#include <math.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <iostream>
#include <pthread.h>
#include <ctime>
#include <time.h>
#include <string.h>
// Project specific headers
#include "Navio/MS5611.h"
#include "Navio/MPU9250.h"
#include "Navio/LSM9DS1.h"
#include "Navio/Util.h"
#include "AHRS.hpp"
#include "Navio/PWM.h"
#include "Navio/RCInput.h"
#include "Eigen/Dense"
#include "Navio/Ublox.h"


using namespace std;
using namespace Eigen;

/*****************************************************************************************************************************************************************************************
*
*
*                                                               ***** CONSTANTS AND VARIABELS DECLARATION *****
*
*
********************************************************************************************************************************************************************************************/

//---------------------------------------------------------------------------------------------------------------------
//------ Functions
//--------------------------------------------------------------------------------------------------------------------
int kbhit (void);
void Output (float, float, float, float);
//void Output(float,float,float);               // Outputs the computed milliseconds to Navio pins
float ManualAlphaMs (int);	// Converts Alpha angle to ms for -- Manual Controller
float ManualBetaMs (int);	// Converts  Beta angle to ms for -- Manual Controller
float AutoAlphaMs (float);	//  Converts  Alpha angle to ms for -- Auto Controller
float AutoBetaMs (float);	//   Converts  Beta  angle to ms for -- Auto Controller
float PeriodToAlt (int);
MatrixXd RotX (float);
MatrixXd RotY (float);
MatrixXd RotZ (float);
MatrixXd Jaco (float, float, float);
float AutoThrottle (float);
char *datetime (void);


//----------------------------------------------------------------------------------------------------------------------------
//----- Matrix and Vector Objects
//----------------------------------------------------------------------------------------------------------------------------
//--- Body Fixed Frame(BFF) Variables

VectorXd acceleration_bff (3);	// Acceleration
VectorXd omega_bff (3);		// Angular Velocities

//--- Inertial Frame(IF) Variables

VectorXd acceleration_if (3);	//Acceleration

VectorXd velocity_kif (3);	// Kth velocity in the inertial frame
VectorXd position_kif (3);	// Kth position in the inertial frame

VectorXd omega_kif (3);
VectorXd angle_kif (3);

VectorXd velocity_k1if (3);	//***************************** -- K+1th velocity in the inertial frame
VectorXd position_k1if (3);	//***************************** -- K+1th position in the inertial frame


VectorXd velocity_des_kif (3);
VectorXd position_des_kif (3);
VectorXd omega_des_kif (3);
VectorXd angle_des_kif (3);

//std::vector<double> pos_data; //*********** GPS vector *********************

Ublox gps;			//************* GPS Object **************


MatrixXd rotx (3, 3);		// Roll
MatrixXd roty (3, 3);		// Pitch
MatrixXd rotz (3, 3);		// Yaw
MatrixXd rot_bi (3, 3);		// Complete rotation matrix from BFF to IF
MatrixXd jacobian (3, 3);	// Jacobian Matrix
MatrixXd InertialMatrix (3, 3);	// Inertial Matrix
//--------------------------------------------------------------------------------------------------------------------------------
//--------- Constants
//--------------------------------------------------------------------------------------------------------------------------------
#define G_SI 9.80665
#define PI   3.14159
#define NUM_IMU_SAMPLES 3
#define NUM_IMU_CALIBS  100
#define mass 0.7		// kg

//-------------------------------------------------------------------------------------------------------------------------------
//---- Constants for Controller
//-------------------------------------------------------------------------------------------------------------------------------
// Auto

// Manual
float MBeta = 0.0001 / 1177;
float MAlpha = 0.000465 / 1171;
float CBeta = 0.0007;
#define PWM_OUTPUT_1 0		// 1
#define PWM_OUTPUT_2 1		// 2
#define PWM_OUTPUT_3 2		// 3
#define PWM_OUTPUT_4 3		// 4

#define MS_ZERO_ANGLE_1 1.45	// beta
#define MS_ZERO_ANGLE_2 1.45	//  alpha

#define PERIOD_CONTROL_LOOP 2500	// microseconds = 2.5 ms
#define PERIOD_GPS_LOOP     5000	// microseconds = 5 ms
#define PERIOD_INPUT_LOOP   10000	// microseconds = 10 ms
#define PERIOD_OUTPUT_LOOP  100000	// microseconds = 100 ms
//-------------------------------------------------------------------------------------------------------------------------------
//---- Sensor, input, and actuator objects
//-------------------------------------------------------------------------------------------------------------------------------
InertialSensor *imuMpu;
InertialSensor *imuLsm;
// NOTE: The IMU on this system only has angular rate, linear acceleration, and magnetometer


AHRS ahrs;
//Mahony AHRS
// NOTE: This doesn't use the magnetometer to get rid of bias drift. Plus, the magnetometer probably wouldn't work well inside our building.


PWM pwm1;
PWM pwm2;
PWM pwm3;
PWM pwm4;
RCInput rcin;

pthread_mutex_t gpsMutex;
pthread_mutex_t rcInputMutex;
pthread_mutex_t controllerStateMutex;
pthread_mutex_t outputMutex;
//MS5611 barometer;


//---------------------------------------------------------------------------------------------------------------
//-------- GPS Data
//---------------------------------------------------------------------------------------------------------------
double g_x;
double g_y;
double g_z1;
double g_z_0;
double ref_long;
double ref_lat;
double r = 6371;
float ms4;
//float z;
float g_t_0;
float g_az;
std::vector < double >
  pos_data;
int g_status;
float g_cmd_alpha;
float g_cmd_beta;
int count_gps = 1;
//------------------------------------------------------------------------------------------------------------
//------ Sensor reading data
//------------------------------------------------------------------------------------------------------------

float axMpu, ayMpu, azMpu;
float gxMpu, gyMpu, gzMpu;

float axLsm, ayLsm, azLsm;
float gxLsm, gyLsm, gzLsm;

float mx, my, mz;
float v_x = 0, v_y = 0;

float p, p_ini[10], p1, p0 = 0, dh;

//Variables
float t, t_ini[10], t0;
float g = G_SI, M = 0.02896, R = 8.314, a = 6.5;

int count = 0;

//-------------------------------------------------------------------------------------------------------------
//------ Orientation data
//-------------------------------------------------------------------------------------------------------------
float roll, pitch, yaw;


//---------------------------------------------------------------------------------------------------------
//------ RC controller inputs (mutex protected)
//---------------------------------------------------------------------------------------------------------
int g_AlphaControlRad;
int g_BetaControlRad;
int g_ThrustControlRadPerSec;
int g_ResetValuePeriod;
int g_switchSE;
int g_switchSF;
float g_zd;
float g_ma = 0.04;		//0.012 ;
float g_ca = 1.43;		//1.27;//1.51;
float g_mb = 0.02;		//0.02;
float g_cb = 1.46;		//1.21;//1.36;
float g_k1 = -1.0;		// thrust -- P gain
float g_k2 = -0.1;		// thrust -- D gain
float g_k3 = 0.6;		// alpha  -- P gain
float g_k4 = 0.07;		// alpha   -- D gain
float g_k5 = 0.6;		// beta    -- P gain
float g_k6 = 0.07;		//  beta    -- D gain
float g_k7 = -1.0;		// alpha
float g_k8 = -8.0;		// alpha
float g_k9 = -1.0;		// beta
float g_k10 = -4.0;		// beta
float g_zp = 0.0;
//----------------------------------------------------------------------------------------------------------
//----- Controller state variables (mutex protected)
//----------------------------------------------------------------------------------------------------------
float g_state[12];		// this contains [x, y, z, phi, theta, psi, dx, dy, dz, dphi, dtheta, dpsi]
float g_inputs[3];		// this contains [rotor_velocity, alpha, beta]

//----------------------------------------------------------------------------------------------------------
//---- Elased Time Outputs (mutex protected)
//----------------------------------------------------------------------------------------------------------
int elapsedOutput_t1;
int elapsedOutput_t2;
int elapsedOutput_t3;

//process timing variables
clock_t t1_i;
clock_t t1_f;
float t1_v;
clock_t t2_i;
clock_t t2_f;
float t2_v;
clock_t t3_i;
clock_t t3_f;
//clock_t t1_i;
float t3_v;

//Timing data
float offset[3];
struct timeval tv;
float dt, maxdt;
float mindt = 0.01;
unsigned long previoustime, currenttime;
float dtsumm = 0;
int isFirst = 1;

//Network data
int sockfd;
struct sockaddr_in servaddr = { 0 };

char sendline[80];

InertialSensor *
create_inertial_sensor (char *sensor_name)
{
  InertialSensor *imu;

  if (!strcmp (sensor_name, "mpu"))
    {
      printf ("Selected: MPU9250\n");
      imu = new MPU9250 ();
    }
  else if (!strcmp (sensor_name, "lsm"))
    {
      printf ("Selected: LSM9DS1\n");
      imu = new LSM9DS1 ();
    }
  else
    {
      return NULL;
    }

  return imu;
}

void
print_help ()
{
  printf ("Possible parameters:\nSensor selection: -i [sensor name]\n");
  printf ("Sensors names: mpu is MPU9250, lsm is LSM9DS1\nFor help: -h\n");
  printf ("If you want to visualize IMU data on another machine,\n");
  printf ("add IP address and port number (by default 7000):\n");
  printf ("-i [sensor name] ipaddress portnumber\n");

}

/***************************************************************************************************************************************
*
*
*                                       ------ Initial setup ----- 
*
*
******************************************************************************************************************************************/

void
imuSetup ()
{

  float t, p;
  float g = G_SI, p_0 = 1013.25;
  //-----------------------MPU initialization-- -- --------------------------

  imuMpu->initialize ();
  //imuLsm->initialize();

  //-------------------------------------------------------------------------

  float l_ax = 0.0, l_ay = 0.0, l_az = 0.0;
  float l_gx = 0.0, l_gy = 0.0, l_gz = 0.0;

  for (int i = 0; i < NUM_IMU_CALIBS; i++)
    {
      imuMpu->update ();
      //imuMpu->read_gyroscope(&gx, &gy, &gz);
      imuMpu->read_gyroscope (&gxMpu, &gyMpu, &gzMpu);

      // imuLsm->update();
      // imuLsm->read_gyroscope(&gxLsm, &gyLsm, &gzLsm);



      gxMpu *= 180 / PI;
      gyMpu *= 180 / PI;
      gzMpu *= 180 / PI;

      gxLsm *= 180 / PI;
      gyLsm *= 180 / PI;
      gzLsm *= 180 / PI;


      offset[0] += (-gxMpu * 0.0175 - gxLsm * 0.0175);
      offset[1] += (-gyMpu * 0.0175 - gyLsm * 0.0175);
      offset[2] += (-gzMpu * 0.0175 - gzLsm * 0.0175);
      usleep (10000);
    }
  offset[0] /= (2.0 * (float) NUM_IMU_CALIBS);
  offset[1] /= (2.0 * (float) NUM_IMU_CALIBS);
  offset[2] /= (2.0 * (float) NUM_IMU_CALIBS);

  printf ("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
  ahrs.setGyroOffset (offset[0], offset[1], offset[2]);


}

// Set up the servo motor objects for controlling the rotor velocity and the two linear actuators
int
servoSetup ()
{

  if (check_apm ())
    {
      return 1;
    }

  // Initialize the RC controller input object
  rcin.init ();


  //barometer.initialize();

  // Initialize the three PWM outputs

  if (!pwm1.init (PWM_OUTPUT_1))	// alpha linear actuator
    {
      fprintf (stderr, "Output Enable not set for PWM1. Are you root?\n");
      return 0;
    }
  if (!pwm2.init (PWM_OUTPUT_2))	// beta linear actuator
    {
      fprintf (stderr, "Output Enable not set for PWM2. Are you root?\n");
      return 0;
    }
  if (!pwm3.init (PWM_OUTPUT_3))	// rotor velocity
    {
      fprintf (stderr, "Output Enable not set for PWM3. Are you root?\n");
      return 0;
    }
  if (!pwm3.init (PWM_OUTPUT_4))	// rotor velocity
    {
      fprintf (stderr, "Output Enable not set for PWM3. Are you root?\n");
      return 0;
    }

  // Enable each of the PWM and set the initial period
  pwm1.enable (PWM_OUTPUT_1);
  pwm1.set_period (PWM_OUTPUT_1, 50);

  pwm2.enable (PWM_OUTPUT_2);
  pwm2.set_period (PWM_OUTPUT_2, 50);

  pwm3.enable (PWM_OUTPUT_3);
  pwm3.set_period (PWM_OUTPUT_3, 50);

  pwm3.enable (PWM_OUTPUT_4);
  pwm3.set_period (PWM_OUTPUT_4, 50);

}

/*******************************************************************************************************************************************************************************************
*
*
*                                                                     ------  THREADS ------
*
*
*
********************************************************************************************************************************************************************************************/

//***************************************************************************************** -- CONTROL THREAD -- ****************************************************************************


void *
controlThread (void *)
{
  static int loopCount = 0;
  struct timeval tv;
  unsigned long startTime, endTime, elapsed;
  float dt = (float) PERIOD_CONTROL_LOOP / 1000000.0;

// Inertial Frame Parameters Initializaion
// Velocities

  velocity_kif (0) = 0.0;
  velocity_kif (1) = 0.0;
  velocity_kif (2) = 0.0;

// Positions

  position_kif (0) = 0.0;
  position_kif (1) = 0.0;
  position_kif (2) = 0.0;

  while (1)
    {
      // Get the time at the start of the loop
      gettimeofday (&tv, NULL);
      startTime = 1000000 * tv.tv_sec + tv.tv_usec;

      // Read in the most recent RC controller inputs in a threadsafe manner
      int l_periodAlphaAngle;
      int l_periodBetaAngle;
      int l_periodRotorSpeed;
      int l_resetValue;
      int l_switchSE;
      int l_switchSF;
      float l_z;
      float l_x;
      float l_y;
      float l_zd;
      float l_zp;

      pthread_mutex_lock (&rcInputMutex);
      // l_z = g_z;
      l_periodAlphaAngle = g_AlphaControlRad;
      l_periodBetaAngle = g_BetaControlRad;
      l_periodRotorSpeed = g_ThrustControlRadPerSec;
      l_resetValue = g_ResetValuePeriod;
      l_z = g_z1;
      l_zd = g_zd;
      l_zp = g_zp;
      l_switchSE = g_switchSE;
      l_switchSF = g_switchSF;
      l_x = g_x;
      l_y = g_y;
      pthread_mutex_unlock (&rcInputMutex);


      // Read raw measurements from the MPU and update AHRS-

      //Accel + gyro.
      float l_ax = 0.0, l_ay = 0.0, l_az = 0.0;
      float l_gx = 0.0, l_gy = 0.0, l_gz = 0.0;
      for (int i = 0; i < NUM_IMU_SAMPLES; i++)
	{
	  imuMpu->update ();
	  imuMpu->read_accelerometer (&axMpu, &ayMpu, &azMpu);
	  imuMpu->read_gyroscope (&gxMpu, &gyMpu, &gzMpu);

	  imuLsm->update ();
	  imuLsm->read_accelerometer (&axLsm, &ayLsm, &azLsm);
	  imuLsm->read_gyroscope (&gxLsm, &gyLsm, &gzLsm);

	  l_ax += (axMpu + axLsm);
	  l_ay += (ayMpu + ayLsm);
	  l_az += (azMpu + azLsm);

	  l_gx += (gxMpu + gxLsm);
	  l_gy += (gyMpu + gyLsm);
	  l_gz += (gzMpu + gzLsm);

	}
      l_ax /= 2.0 * (float) NUM_IMU_SAMPLES;
      l_ay /= 2.0 * (float) NUM_IMU_SAMPLES;
      l_az /= 2.0 * (float) NUM_IMU_SAMPLES;

      l_gx /= 2.0 * (float) NUM_IMU_SAMPLES;
      l_gy /= 2.0 * (float) NUM_IMU_SAMPLES;
      l_gz /= 2.0 * (float) NUM_IMU_SAMPLES;


      l_ax /= G_SI;
      l_ay /= G_SI;
      l_az /= G_SI;
      l_gx *= 180 / PI;
      l_gy *= 180 / PI;
      l_gz *= 180 / PI;

      ahrs.updateIMU (l_ax, l_ay, l_az, l_gx * 0.0175, l_gy * 0.0175,
		      l_gz * 0.0175, dt);

      //------------------------Read Euler angles-- -- --------------------------
      ahrs.getEuler (&roll, &pitch, &yaw);

      roll *= (M_PI / 180.0);
      pitch *= (M_PI / 180.0);
      yaw *= (M_PI / 180.0);

      //-------------------Discard the time of the first cycle-- -- -------------
      if (!isFirst)
	{
	  if (dt > maxdt)
	    maxdt = dt;
	  if (dt < mindt)
	    mindt = dt;
	}
      isFirst = 0;


//----------------------- Body Fixed Frame Parameters -----------------------------------------------------------------------------------------

      /*  Accelerometer readings are the in the Body fixed frame, these will be trasformed to Intertial frame using the rotational matrices.
         These values would be used to get the intertial frame -- velocities and positions. */

      acceleration_bff (0) = l_ax;
      acceleration_bff (1) = l_ay;
      acceleration_bff (2) = l_az;

      omega_bff (0) = M_PI / 180.0 * l_gx;
      omega_bff (1) = M_PI / 180.0 * l_gy;
      omega_bff (2) = M_PI / 180.0 * l_gz;






//---------------------------- Rotational Matrices --------------------------------------------------------------------------------------------

      rotx << RotX (roll);
      roty << RotY (pitch);
      rotz << RotZ (yaw);

//Complete Rotation Matrix From BFF to IF

      rot_bi = rotz * roty * rotx;


// Jacobian Matrix -- used for converting bff anugular velocities to iff angular velocities

      jacobian << Jaco (roll, pitch, yaw);


/*  Note: Addiional Code

//-------------------- Intertial Frame Parameters ------------------------------------------------------


	acceleration_if = rot_bi * acceleration_bff;
	acceleration_if (2) = acceleration_if (2) - 1.0;
	acceleration_if *= G_SI;

	velocity_k1if = velocity_kif + acceleration_if * dt;
	position_k1if = position_kif + velocity_k1if * dt;



//--------------- Refresh Button: Pushing the "E" Lever on RC Transmitter ------------------------------

// up non zero -- down zero


	 //Note: Pushing the lever down will set the : Posioition and velocity values to zero. And on pushing it back up, new computed values of position and velocity will be used.
	 // the right values and give us good results.

	if (l_resetValue < 960)
	{
		velocity_k1if << 0, 0, 0;
		position_k1if << 0, 0, 0;
	}

	velocity_kif = velocity_k1if;
	position_kif = position_k1if;

*/

      angle_kif (0) = roll;
      angle_kif (1) = pitch;
      angle_kif (2) = yaw;

      omega_kif = jacobian * omega_bff;


//---------------------------- Auto Controller -------------------------------------------

      // WARNING: l_gz is in degrees and l_ax is normalized to 1.0


      // Settiing linear position and velocity gains to zero temporarily because estimates are bad.
      g_k7 = 1;
      g_k8 = 0;			//0;
      g_k9 = 1;
      g_k10 = 0;		//`0;


//--------------------------------------------------------------------------
//------ Roll - Pitch Controller  
//--------------------------------------------------------------------------

      position_des_kif (0) = (float) (l_periodBetaAngle - 1494) / 400.0 * 0.1;
      position_des_kif (1) =
	(float) (l_periodAlphaAngle - 1492) / 400.0 * 0.1;

      if (l_switchSE > 1490)
	{
	  position_kif (0) = 0;
	  position_kif (1) = 0;
	}
      else
	{
	  position_kif (0) = l_x;
	  position_kif (1) = l_y;
	}

      // float cmd_thrust = mass * G_SI + k1 * (position_kif(2) - position_des_kif(2)) + k2 * (velocity_kif(2) - velocity_des_kif(2));
      float cmd_alpha =
	g_k3 * (angle_kif (1) - angle_des_kif (1)) +
	g_k4 * (omega_kif (1) - omega_des_kif (1)) +
	g_k7 * (position_kif (0) - position_des_kif (0)) +
	g_k8 * (velocity_kif (0) - velocity_des_kif (0));
      float cmd_beta =
	g_k5 * (angle_kif (0) - angle_des_kif (0)) +
	g_k6 * (omega_kif (0) - omega_des_kif (0)) +
	g_k9 * (position_kif (1) - position_des_kif (1)) +
	g_k10 * (velocity_kif (1) - velocity_des_kif (1));

      g_cmd_alpha = cmd_alpha;
      g_cmd_beta = cmd_beta;


      float ms_2 = AutoBetaMs (cmd_beta);
      float ms_1 = AutoAlphaMs (-cmd_alpha);




      if (++loopCount % 8 == 0)
	{


//------------------------------------- Thrust Computation ---------------------------------

// Auto Throttle 
	  if (l_switchSE < 1500)
	    {

	      float t_0 = 1.8;	// Combined threshold thrust

	      float thrust =
		t_0 + g_k1 * (l_z - g_zd) + g_k2 * (l_z * dt - l_zp) / dt;
	      l_zp = l_z;

	      float ms_3 = AutoThrottle (thrust);
	      float ms_4 = ms_3;

	      Output (ms_1, ms_2, ms_3, ms_3);
	    }

// Manual throttle

	  else
	    {

	      float m = 0.001202;
	      float c = -0.3174;

	      float ms_3 = m * l_periodRotorSpeed + c;
	      float ms_4 = m * l_periodAlphaAngle + c;

	      Output (ms_1, ms_2, ms_3, ms_3);
	    }

	}



      pthread_mutex_lock (&controllerStateMutex);	//   Mutex On

//---------------------------------- Storing States of Baton -------------------------------

      g_state[0] = position_kif (0);
      g_state[1] = position_kif (1);
      g_state[2] = position_kif (2);
      g_state[3] = angle_kif (0);
      g_state[4] = angle_kif (1);
      g_state[5] = angle_kif (2);
      g_state[6] = velocity_kif (0);
      g_state[7] = velocity_kif (1);
      g_state[8] = velocity_kif (2);
      g_state[9] = omega_kif (0);
      g_state[10] = omega_kif (1);
      g_state[11] = omega_kif (2);

//-------------------------- Storing the inputs from RC Trasmitter ------------------------

      g_inputs[0] = l_periodAlphaAngle;
      g_inputs[1] = l_periodBetaAngle;
      g_inputs[2] = l_periodRotorSpeed;
      pthread_mutex_unlock (&controllerStateMutex);	// Mutex Off

      // Get the time after the execution of the loop and sleep the appropriate number of microseconds
      gettimeofday (&tv, NULL);
      endTime = 1000000 * tv.tv_sec + tv.tv_usec;

      elapsed = endTime - startTime;


      if (elapsed < PERIOD_CONTROL_LOOP)
	{
	  elapsedOutput_t1 = PERIOD_CONTROL_LOOP - elapsed;
	  usleep (elapsedOutput_t1);
	}

    }

}

//-------------------------------------------------------------------------------------------
//                                GPS Thread
//------------------------------------------------------------------------------------------
void *
gpsThread (void *)
{
  float l_z;
  char c;
  float l_x;
  float l_y;
  struct timeval tv;
  unsigned long startTime, endTime, elapsed;

  if (gps.testConnection ())
    {
      printf ("Ublox test OK\n");

      // gps.decodeMessages();
      // You can use this function to decode all messages, incoming from the GPS receiver. The function starts an infinite loop.
      // In this example we can only decode NAV_STATUS and NAV-POSLLH messages, the others are simply ignored.
      // You can add new message types in function decodeMessage() of class UBXParser(see ublox.h)


      // Here, however we use a different approach. Instead of trying to extract info from every message(as done in decodeMessages()),
      // this function waits for a message of a specified type and gets you just the information you need
      // In this example we decode NAV_STATUS and NAV-POSLLH messages, adding new types, however is quite easy
      float ref_z = 0;
      ref_lat = 0;
      ref_long = 0;
      int flag = 1;
      while (true)
	{
	  // Get the time at the start of the loop
	  gettimeofday (&tv, NULL);
	  startTime = 1000000 * tv.tv_sec + tv.tv_usec;

	  if (gps.decodeSingleMessage (Ublox::NAV_POSLLH, pos_data) == 1)
	    {
	      //ref_z = 0;
	      while (count_gps < 10000)
		{
		  float ref_lat1 = pos_data[1] / 10000000;
		  float ref_long1 = pos_data[2] / 10000000;
		  float ref_z1 = pos_data[4] / 1000;
		  ref_lat = ref_lat + ref_lat1;
		  ref_long = ref_long + ref_long1;
		  ref_z = ref_z + ref_z1;
		  count_gps++;
		}
	      if (flag == 1)
		{
		  ref_lat = ref_lat / 10000;
		  ref_long = ref_long / 10000;
		  ref_z = ref_z / 10000;
		  flag = 0;
		}
	      double lat = pos_data[1] / 10000000;
	      double Long = pos_data[2] / 10000000;

	      l_x = r * (ref_long - Long) * (M_PI / 180) * cos (Long);
	      l_y = r * (ref_lat - lat) * (M_PI / 180);
	      float l_za = pos_data[4] / 1000;
	      //printf("%f\t%f\n",ref_z,l_za);
	      l_z = l_za - ref_z;
	      g_z1 = l_z;
	      g_x = l_x;
	      g_y = l_y;
	      //printf("%f\t%f\n",l_z,g_z1);
	      // after desired message is successfully decoded, we can use the information stored in pos_data vector
	      // right here, or we can do something with it from inside decodeSingleMessage() function(see ublox.h).
	      // the way, data is stored in pos_data vector is specified in decodeMessage() function of class UBXParser(see ublox.h)
//                printf("x:%lf\ty:%lf\n",x*1000,y*1000);
//                printf("GPS Millisecond Time of Week: %.0lf s\n", pos_data[0]/1000);
//                printf("Longitude: %lf\n", pos_data[1]/10000000);
//                printf("Latitude: %lf\n", pos_data[2]/10000000);
//                printf("Height above Ellipsoid: %.3lf m\n", pos_data[3]/1000);
//                printf("Height above mean sea level: %.3lf m\n", pos_data[4]/1000);
//                printf("Horizontal Accuracy Estateimate: %.3lf m\n", pos_data[5]/1000);
//                printf("Vertical Accuracy Estateimate: %.3lf m\n", pos_data[6]/1000);
	    }
	  else
	    {
	      // printf("Message not captured\n");
	      // use this to see, how often you get the right messages
	      // to increase the frequency you can turn off the undesired messages or tweak ublox settings
	      // to increase internal receiver frequency
	    }
	  if (gps.decodeSingleMessage (Ublox::NAV_STATUS, pos_data) == 1)
	    {
	      printf ("Current GPS status:\n");
	      printf ("gpsFixOk: %d\n", ((int) pos_data[1] & 0x01));

	      printf ("gps Fix status: ");
	      switch ((int) pos_data[0])
		{
		case 0x00:
		  printf ("no fix\n");
		  break;

		case 0x01:
		  printf ("dead reckoning only\n");
		  break;

		case 0x02:
		  printf ("2D-fix\n");
		  break;

		case 0x03:
		  printf ("3D-fix\n");
		  break;
		case 0x04:
		  printf ("GPS + dead reckoning combined\n");
		  break;

		case 0x05:
		  printf ("Time only fix\n");
		  break;

		default:
		  printf ("Reserved value. Current state unknown\n");
		  break;

		}

	      printf ("\n");

	    }
	  else
	    {
	      // printf("Status Message not captured\n");
	    }


	  //usleep(200);
	}
      pthread_mutex_lock (&gpsMutex);
      g_z1 = l_z;
      //printf("%f\n",g_z);
      pthread_mutex_unlock (&gpsMutex);
      //printf("%f\t%f\n",l_z,g_z);

      // Get the time after the execution of the loop and sleep the appropriate number of microseconds
      gettimeofday (&tv, NULL);
      endTime = 1000000 * tv.tv_sec + tv.tv_usec;

      elapsed = endTime - startTime;

      if (elapsed < PERIOD_GPS_LOOP)
	{
	  //        elapsedOutput_t2=elapsed;

	  elapsedOutput_t2 = PERIOD_GPS_LOOP - elapsed;
	  usleep (elapsedOutput_t2);

	}
      else
	{
	  printf ("Ublox test not passed\nAbort program!\n");

	}

//    return 0;

    }
}


//--------------------------------------------------------------------------------------------
//                                Inputs Thread
//-------------------------------------------------------------------------------------------

void *
inputThread (void *)
{
  char c;
  struct timeval tv;
  unsigned long startTime, endTime, elapsed;
  while (1)
    {
// Get the time at the start of the loop
      gettimeofday (&tv, NULL);
      startTime = 1000000 * tv.tv_sec + tv.tv_usec;

// Input from rc controller

      int l_periodAlphaAngle = rcin.read (2);
      int l_periodBetaAngle = rcin.read (1);
      int l_periodRotorSpeed = rcin.read (0);
      int l_switchSE = rcin.read (4);
      int l_switchSF = rcin.read (5);

      pthread_mutex_lock (&rcInputMutex);
      float l_ca = g_ca;
      float l_cb = g_cb;
      pthread_mutex_unlock (&rcInputMutex);

      float l_zd = PeriodToAlt (l_periodRotorSpeed);

      if (kbhit () != 0)
	{
	  scanf (" %c", &c);
	}


      if (c != '0')
	{


	  switch (c)
	    {
	    case 'q':
	      l_ca = l_ca + 0.01;
	      c = '0';
	      break;
	    case 'a':
	      l_ca = l_ca - 0.01;
	      c = '0';
	      break;
	    case 'w':
	      l_cb = l_cb + 0.01;
	      c = '0';
	      break;

	    case 's':
	      l_cb = l_cb - 0.01;
	      c = '0';
	      break;

	    }

	}
      else
	{
	  l_ca = l_ca;
	  l_cb = l_cb;


	}



      pthread_mutex_lock (&rcInputMutex);	// Mutex on

// Storing local vaules in global variables
      g_ca = l_ca;
      g_cb = l_cb;
      g_AlphaControlRad = l_periodAlphaAngle;
      g_BetaControlRad = l_periodBetaAngle;
      g_ThrustControlRadPerSec = l_periodRotorSpeed;
      //g_ResetValuePeriod = l_resetValue;
      g_switchSE = l_switchSE;
      g_switchSF = l_switchSF;
      g_zd = l_zd;
      pthread_mutex_unlock (&rcInputMutex);	// Mutex off

// Get the time after the execution of the loop and sleep the appropriate number of microseconds
      gettimeofday (&tv, NULL);
      endTime = 1000000 * tv.tv_sec + tv.tv_usec;

      elapsed = endTime - startTime;

      if (elapsed < PERIOD_INPUT_LOOP)
	{
	  //        elapsedOutput_t2=elapsed;

	  elapsedOutput_t2 = PERIOD_INPUT_LOOP - elapsed;
	  usleep (elapsedOutput_t2);
	}

    }

}

//--------------------------------------------------------------------------------------
//                                   OUTPUT THREAD
//--------------------------------------------------------------------------------------

void *
outputThread (void *)
{
  struct timeval tv;
  unsigned long startTime, endTime, elapsed;
  char *date_time;		//Collect system date and time in this character array
//date_tfp, "This is a string which is written to a file\n");
  date_time = datetime ();
  FILE *fp;
  char filename[100];
//filename = "yo.txt";
  sprintf (filename, "log/%s", date_time);

  while (1)
    {
      // Get the time at the start of the loop
      gettimeofday (&tv, NULL);
      startTime = 1000000 * tv.tv_sec + tv.tv_usec;
      int l_periodAlphaAngle;
      int l_periodBetaAngle;
      int l_periodRotorSpeed;
      float l_state[12];
      float l_inputs[3];
      float l_cmd_alpha;
      float l_cmd_beta;
      float l_switchSF;
      int elapsedt1;
      int elapsedt2;
      int elapsedt3;
      float l_ca;
      float l_cb;

      // Copy the RC controller inputs into a local variable (mutex protected)
      pthread_mutex_lock (&rcInputMutex);	// Mutex on
      l_state[0] = g_state[0];
      l_state[1] = g_state[1];
      l_state[2] = g_state[2];
      l_state[3] = g_state[3];
      l_state[4] = g_state[4];
      l_state[5] = g_state[5];
      l_state[6] = g_state[6];
      l_state[7] = g_state[7];
      l_state[8] = g_state[8];
      l_state[9] = g_state[9];
      l_state[10] = g_state[10];
      l_state[11] = g_state[11];
      l_cmd_beta = g_cmd_beta;
      l_cmd_alpha = g_cmd_alpha;
      l_ca = g_ca;
      l_cb = g_cb;
      l_periodAlphaAngle = g_AlphaControlRad;
      l_periodBetaAngle = g_BetaControlRad;
      l_periodRotorSpeed = g_ThrustControlRadPerSec;
      l_switchSF = g_switchSF;
      pthread_mutex_unlock (&rcInputMutex);	// Mutex off



//-------------------------------- log file contents ------------------------------------------------------------

      if (l_switchSF > 2000)
	{

	  fp = fopen (filename, "a+");
	  fprintf (fp,
		   "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
		   l_state[0], l_state[1], l_state[2], l_state[3],
		   l_state[4], l_state[5], l_state[6], l_state[7],
		   l_state[8], l_state[9], l_state[10], l_state[11],
		   l_cmd_alpha, l_cmd_beta, g_k1, g_k2, g_k3, g_k4, g_k5,
		   g_k6);
	  fclose (fp);
	}

//------------------------------------ Outputs -----------------------------------------------------------------------

// Monitor Outputs
      printf ("Ca:%f\tCb:%f\n", l_ca, l_cb);

// Output Inputs from RC - Controller 
      // printf("RC thrust: %d\tRC alpha: %d\tRC beta: %d\n", l_period0, l_period1, l_period2);

// Outputting the Baton State Variables
//printf("pitch input:%d \t roll input:%d \t thrust  input:%d\n",g_AlphaControlRad,g_BetaControlRad,g_ThrustControlRadPerSec);

//printf("%lu,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f\n",startTime,l_state[0],l_state[1],l_state[2],l_state[3],l_state[4],l_state[5],l_state[6],l_state[7],l_state[8],l_state[9],l_state[10],l_state[11]);


      // Get the time after the execution of the loop and sleep the appropriate number of microseconds
      gettimeofday (&tv, NULL);
      endTime = 1000000 * tv.tv_sec + tv.tv_usec;

      elapsed = endTime - startTime;

      elapsedOutput_t3 = PERIOD_OUTPUT_LOOP - elapsed;

      pthread_mutex_lock (&outputMutex);
      elapsedt1 = elapsedOutput_t1;
      elapsedt2 = elapsedOutput_t2;
      elapsedt3 = elapsedOutput_t3;
      pthread_mutex_unlock (&outputMutex);


      if (elapsed < PERIOD_OUTPUT_LOOP)
	{
	  usleep (elapsedOutput_t3);
	}

    }
}

/*__________________________________________________________________________________________________________________________________________________________________________________________
*
*                                                             ***** Functions *****
*__________________________________________________________________________________________________________________________________________________________________________________________*/

float
PeriodToAlt (int period)
{

  float m = 0.00195;
  float c = -1.9149;

  float alt = m * period + c;

  return (alt);

}

char *
datetime ()
{
  char *array = (char *) malloc (sizeof (char) * 25);
  time_t result;
  result = time (NULL);
  struct tm *tm_info;
  tm_info = localtime (&result);
  strftime (array, 26, "%Y-%m-%d-%H:%M:%S", tm_info);
  return array;
}


int
kbhit (void)
{
  struct timeval tv;
  fd_set read_fd;

  /* Do not wait at all, not even a microsecond */
  tv.tv_sec = 0;
  tv.tv_usec = 0;

  /* Must be done first to initialize read_fd */
  FD_ZERO (&read_fd);

  /* Makes select() ask if input is ready:
   * 0 is the file descriptor for stdin    */
  FD_SET (0, &read_fd);

  /* The first parameter is the number of the
   * largest file descriptor to check + 1. */
  if (select (1, &read_fd, NULL, /*No writes */ NULL, /*No exceptions */ &tv)
      == -1)
    return 0;			/* An error occured */

  /*  read_fd now holds a bit map of files that are
   * readable. We test the entry for the standard
   * input (file 0). */

  if (FD_ISSET (0, &read_fd))
    /* Character pending on stdin */
    return 1;

  /* no characters were pending */
  return 0;
}



MatrixXd
RotX (float roll_x)
{


  MatrixXd l_rotx (3, 3);

  // Roll

  l_rotx (0, 0) = 1.0;
  l_rotx (0, 1) = 0.0;
  l_rotx (0, 2) = 0.0;
  l_rotx (1, 0) = 0.0;
  l_rotx (1, 1) = cos (roll_x);
  l_rotx (1, 2) = -sin (roll_x);
  l_rotx (2, 0) = 0.0;
  l_rotx (2, 1) = sin (roll_x);
  l_rotx (2, 2) = cos (roll_x);

  return (l_rotx);
}

MatrixXd
RotY (float pitch_y)
{

  MatrixXd l_roty (3, 3);

  // Pitch

  l_roty (0, 0) = cos (pitch_y);
  l_roty (0, 1) = 0.0;
  l_roty (0, 2) = sin (pitch_y);
  l_roty (1, 0) = 0.0;
  l_roty (1, 1) = 1.0;
  l_roty (1, 2) = 0.0;
  l_roty (2, 0) = -sin (pitch_y);
  l_roty (2, 1) = 0.0;
  l_roty (2, 2) = cos (pitch_y);
  return (l_roty);
}

MatrixXd
RotZ (float yaw_z)
{

  MatrixXd l_rotz (3, 3);

  l_rotz (0, 0) = cos (yaw_z);
  l_rotz (0, 1) = -sin (yaw_z);
  l_rotz (0, 2) = 0.0;
  l_rotz (1, 0) = sin (yaw_z);
  l_rotz (1, 1) = cos (yaw_z);
  l_rotz (1, 2) = 0.0;
  l_rotz (2, 0) = 0.0;
  l_rotz (2, 1) = 0.0;
  l_rotz (2, 2) = 1.0;
  return (l_rotz);

}

MatrixXd
Jaco (float roll_x, float pitch_y, float yaw_z)
{

  MatrixXd l_jacobian (3, 3);

  jacobian (0, 0) = 1.0;
  jacobian (0, 1) = sin (roll) * tan (pitch);
  jacobian (0, 2) = cos (roll) * tan (pitch);
  jacobian (1, 0) = 0;
  jacobian (1, 1) = cos (roll);
  jacobian (1, 2) = -sin (roll);
  jacobian (2, 0) = 0;
  jacobian (2, 1) = sin (roll) / cos (pitch);
  jacobian (2, 2) = cos (roll) / cos (pitch);

  return (l_jacobian);

}

float
ManualBetaMs (int period_beta)
{
  float mb = 0.000851926;	//0.00090909;//-0.0398;
  float cb = 0.21622808;	//0.1636382;//59.647;

  float mb_ms = 0.021;		//58;//143.2;//-0.0163;
  float cb_ms = -20.021;	//-88.6;//-213.8;//1.378;

  float beta = mb * period_beta + cb;
  float beta_ms = mb_ms * beta + cb_ms;

  return (beta_ms);

}

float
ManualAlphaMs (int period_alpha)
{
  //float alpha_d=alpha*180/M_PI;
  float ma = 58;		//0.000902527;//-0.03125;
  float ca = -88.6;		//0.174187726;//47.25;

  float ma_ms = 122;		//-0.0254;
  float ca_ms = -181.4;		//1.467;

  float alpha = ma * period_alpha + ca;
  float alpha_ms = ma_ms * alpha + ca_ms;

  return (alpha_ms);

}

float
AutoBetaMs (float beta)
{
  float beta_d = beta * 180 / M_PI;
  // printf("beta_d:%f\n",beta_d);
  float mb_ms = 0;		//-0.02;//0.01972789;//0.0232;//0.01553;//0.021667;//0.012;//0.016;             //0.013;//0.0214;
  float cb_ms = 1.517006807;	//1.59;//1.46;//1.43;//1.42;//1.46;

  float b_ms = g_mb * beta_d + g_cb;
//  cout<<b_ms<<endl;
  return (b_ms);
}

float
AutoAlphaMs (float alpha)
{
  float alpha_d = alpha * 180 / M_PI;
  // printf("alpha_d:%f\n",alpha_d);
  float ma_ms = 0;		//- 0.01151;//-0.013;//0.013427;//0.0139392;//0.0121875;//0.0133;//0.0102564;//0.015;//0.0083;//0.0125;//0.0117;         // 0.015;
  float ca_ms = 1.2996;		//1.3381815;//1.3262855;//0.734864;//1.391999;//1.417;//1.42;//1.38;//1.45;

  float a_ms = g_ma * alpha_d + g_ca;
  return (a_ms);

}

float
AutoThrottle (float thrustTop)
{

  float k = 2.6733;
  float pwm_desired = sqrt (thrustTop / k) + 1;

  return (pwm_desired);

}

void
Output (float ms1, float ms2, float ms3, float ms4)
{
  // ms1 = 1.45;
  //  ms2 = 1.45;
  // printf("ms1:%f\t ms2:%f\t , ms3:%f\n",ms1,ms2,ms3);

  if (ms1 > 2.0)
    ms1 = 2.0;
  else if (ms1 < 1.0)
    ms1 = 1.0;

  if (ms2 > 2.0)
    ms2 = 2.0;
  else if (ms2 < 1.0)
    ms2 = 1.0;

  if (ms3 > 2.0)
    ms3 = 2.0;
  else if (ms3 < 1.0)
    ms3 = 1.0;
//printf("ms1:%f\tms2:%f\tms3:%f\tms4:%f\n",ms1,ms2,ms3,ms4);
  pwm1.set_duty_cycle (PWM_OUTPUT_1, ms1);
  pwm2.set_duty_cycle (PWM_OUTPUT_2, ms2);
  pwm3.set_duty_cycle (PWM_OUTPUT_3, ms3);
  pwm4.set_duty_cycle (PWM_OUTPUT_4, ms3);
  //printf("ms_1:%f\tms2:%f\n",ms1,ms2);

}

/*__________________________________________________________________________________________________________________________________________________________________________________________
*
*                                                                ***** MAIN *****t
*__________________________________________________________________________________________________________________________________________________________________________________________*/
int
main (int argc, char *argv[])
{

// Declaring Threads
  pthread_t thread1;
  pthread_t thread2;
  pthread_t thread3;
  pthread_t thread4;

//Initializing the nutex
  pthread_mutex_init (&rcInputMutex, NULL);

  pthread_mutex_init (&controllerStateMutex, NULL);
  pthread_mutex_init (&outputMutex, NULL);



  //pthread_t thread1;
  int parameter;
  char *sensor_name;

  if (check_apm ())
    {
      return 1;
    }


  imuMpu = create_inertial_sensor ("mpu");
  imuLsm = create_inertial_sensor ("lsm");

  if (!imuMpu)
    {
      printf ("Wrong sensor name. Select: mpu or lsm\n");
      return EXIT_FAILURE;
    }
  if (!imuLsm)
    {
      printf ("Wrong sensor name.\n");
      return EXIT_FAILURE;
    }

  if (!imuMpu->probe ())
    {
      printf ("Sensor not enable\n");
      return EXIT_FAILURE;
    }
  if (!imuLsm->probe ())
    {
      printf ("Sensor not enable\n");
      return EXIT_FAILURE;
    }


  //---------------------------Network setup-- -- ---------------------------

  sockfd = socket (AF_INET, SOCK_DGRAM, 0);
  servaddr.sin_family = AF_INET;

  if (argc == 5)
    {
      servaddr.sin_addr.s_addr = inet_addr (argv[3]);
      servaddr.sin_port = htons (atoi (argv[4]));
    }
  else
    {
      servaddr.sin_addr.s_addr = inet_addr ("127.0.0.1");
      servaddr.sin_port = htons (7000);
    }

  //-------------------- IMU setup and main loop ----------------------------

  imuSetup ();
  servoSetup ();

  //---------------------- Calling the threads -------------------------------

  pthread_create (&thread1, NULL, controlThread, NULL);
  pthread_detach (thread1);
  pthread_create (&thread2, NULL, gpsThread, NULL);
  pthread_detach (thread2);
  pthread_create (&thread3, NULL, inputThread, NULL);
  pthread_detach (thread3);
  pthread_create (&thread4, NULL, outputThread, NULL);
  pthread_exit (&thread1);
  pthread_exit (&thread2);
  pthread_exit (&thread3);

}
