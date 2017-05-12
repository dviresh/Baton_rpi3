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
//#include <ncurses.h>
//#include <curses.h>
#include <sys/select.h>
#include <termios.h>
#include <fcntl.h>
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

using namespace std;
using namespace Eigen;

//--------------- Calibration Offsets -----------------------------

float g_ms_1 = 1.36;		//1.42;             // alpha
float g_ms_2 = 1.43;		// beta



/*__________________________________________________________________________________________________________________________________________________________________________________________
*
*
* 							***** Controller Variables and Functions Declaration *****
*
*__________________________________________________________________________________________________________________________________________________________________________________________*/


//----------------------------------------------------------------------------------------------------------------------
//                               Functions
//----------------------------------------------------------------------------------------------------------------------
void set_mode (int);
int get_key ();
//char getch(void);
void Output (float, float);	// Outputs the computed milliseconds to Navio pins
int kbhit (void);		// Function to detect the keypress from keboard
//int getch();
float ComputeAlphaSlope (float);
float ComputeBetaSlope (float);
float AutoAlphaMs (float);	//  Converts  Alpha angle to ms for -- Auto Controller
float AutoBetaMs (float);	//   Converts  Beta  angle to ms for -- Auto Controller
MatrixXd RotX (float);		//  Rotation Matrix along X
MatrixXd RotY (float);		//  Rotation Matirx about Y
MatrixXd RotZ (float);		//   Rotation Matix about z
MatrixXd Jaco (float, float, float);	// Jacobian matrix - Body Fixed Frame to Inertial Frame
//---------------------------------------------------------------------------------------------------------------------
//                             Matrix and Vector Objects
//----------------------------------------------------------------------------------------------------------------------

//Body Fixed Frame(BFF) Variables

VectorXd acceleration_bff (3);	// Acceleration(BFF)
VectorXd omega_bff (3);		// Angular Velocity(BFF)

// Inertial Frame(IF) Variables

VectorXd acceleration_if (3);	// Acceleration(IF)
VectorXd velocity_kif (3);	// Kth velocity (IF)
VectorXd position_kif (3);	// Kth position (IF)

VectorXd omega_kif (3);		// Angular Velocity (IF)
VectorXd angle_kif (3);		// Euler Angels (IF)

VectorXd velocity_k1if (3);	// K+1th Velocity (IF)
VectorXd position_k1if (3);	// K+1th Position (IF)

// Desired Variables 

VectorXd velocity_des_kif (3);	// Desired Velocity(IF)
VectorXd position_des_kif (3);	// Desired Position (IF)
VectorXd omega_des_kif (3);	// Desired Angular Velocity(IF)
VectorXd angle_des_kif (3);	// Desired Euler Angels (IF)

// Matrix Objects

MatrixXd rotx (3, 3);		// Roll
MatrixXd roty (3, 3);		// Pitch
MatrixXd rotz (3, 3);		// Yaw
MatrixXd rot_bi (3, 3);		// Complete rotation matrix from BFF to IF
MatrixXd jacobian (3, 3);	// Jacobian Matrix
MatrixXd InertialMatrix (3, 3);	// Inertial Matrix


/*_________________________________________________________________________________________________________________________________________________________________________________________


							 *****  Constants and Sensor Class Objects Declaration *****

___________________________________________________________________________________________________________________________________________________________________________________________*/

#define G_SI 9.80665
#define PI   3.14159
#define NUM_IMU_SAMPLES 3
#define NUM_IMU_CALIBS  100
#define mass 0.7		// kg


// ---  Constants for Controller ---

float ca_ms;
float cb_ms;

float ma_ms;
float mb_ms;

//----- Global pulse variables

//float g_ms_1 = 1.42;          // alpha
//float g_ms_2 = 1.43;          // beta


// Output Pin Definitions

#define PWM_OUTPUT_1 0		// 1
#define PWM_OUTPUT_2 1		// 2
#define PWM_OUTPUT_3 2		// 3
#define PWM_OUTPUT_4 3		// 4
// Thread Control Rates

#define PERIOD_CONTROL_LOOP 1000	// microseconds = 1 ms
#define PERIOD_CALIBRATION_LOOP   10000	// microseconds = 10 ms
#define PERIOD_OUTPUT_LOOP  100000	// microseconds = 100 ms


/*__________________________________________________________________________________________________________________________________________________________________________________________


                                                           *****  Sensor, input, and actuator objects *****

___________________________________________________________________________________________________________________________________________________________________________________________*/

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

pthread_mutex_t rcInputMutex;
pthread_mutex_t controllerStateMutex;
pthread_mutex_t outputMutex;
//MS5611 barometer;


/*__________________________________________________________________________________________________________________________________________________________________________________________


 							          ***** Sensor reading data *****

___________________________________________________________________________________________________________________________________________________________________________________________*/

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


//Orientation data

float roll, pitch, yaw;

char inc = 'z';
/*__________________________________________________________________________________________________________________________________________________________________________________________


								*****  Mutex Protected Variabels *****

___________________________________________________________________________________________________________________________________________________________________________________________*/

// Controllerr Inputs

int g_AlphaControlRad;
int g_BetaControlRad;
int g_ThrustControlRadPerSec;
int g_ResetValuePeriod;

// Elpased Time Outputs

int elapsedOutput_t1;
int elapsedOutput_t2;
int elapsedOutput_t3;


// Controller state variables (mutex protected)

float g_state[12];		// this contains [x, y, z, phi, theta, psi, dx, dy, dz, dphi, dtheta, dpsi]
float g_inputs[3];		// this contains [rotor_velocity, alpha, beta]


/*__________________________________________________________________________________________________________________________________________________________________________________________


								       ***** Timing data *****

___________________________________________________________________________________________________________________________________________________________________________________________*/



float offset[3];
struct timeval tv;
float dt, maxdt;
float mindt = 0.01;
unsigned long previoustime, currenttime;
float dtsumm = 0;
int isFirst = 1;

/*__________________________________________________________________________________________________________________________________________________________________________________________


									***** Network data *****

___________________________________________________________________________________________________________________________________________________________________________________________*/

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


/*__________________________________________________________________________________________________________________________________________________________________________________________


								***** Baton Initialization Functions ***** 

___________________________________________________________________________________________________________________________________________________________________________________________*/


//----------------------------------------------------------------------------------------------------------
//        IMU Initializatoin 
//----------------------------------------------------------------------------------------------------------

void
imuSetup ()
{

  float t, p;
  float g = G_SI, p_0 = 1013.25;


  imuMpu->initialize ();
  imuLsm->initialize ();

  float l_ax = 0.0, l_ay = 0.0, l_az = 0.0;
  float l_gx = 0.0, l_gy = 0.0, l_gz = 0.0;
  for (int i = 0; i < NUM_IMU_CALIBS; i++)
    {
      imuMpu->update ();	// MPU IMU board
      imuMpu->read_gyroscope (&gxMpu, &gyMpu, &gzMpu);

      imuLsm->update ();	// LSM IMU Board
      imuLsm->read_gyroscope (&gxLsm, &gyLsm, &gzLsm);

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

//----------------------------------------------------------------------------------------------
//                       Serovs Initialization
//---------------------------------------------------------------------------------------------
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

  if (!pwm4.init (PWM_OUTPUT_4))	// rotor velocity
    {
      fprintf (stderr, "Output Enable not set for PWM3. Are you root?\n");
      return 0;
    }


  // Enable each of the PWM and set the initial period
  pwm1.enable (PWM_OUTPUT_1);
  pwm1.set_period (PWM_OUTPUT_1, 200);

  pwm2.enable (PWM_OUTPUT_2);
  pwm2.set_period (PWM_OUTPUT_2, 200);

  pwm3.enable (PWM_OUTPUT_3);
  pwm3.set_period (PWM_OUTPUT_3, 200);

  pwm3.enable (PWM_OUTPUT_4);
  pwm3.set_period (PWM_OUTPUT_4, 200);
}

/*__________________________________________________________________________________________________________________________________________________________________________________________


                                                                     *****  THREADS *****

___________________________________________________________________________________________________________________________________________________________________________________________*/

//-----------------------------------------------------------------------------------------------------------
//                                              CONTROL THREAD
//----------------------------------------------------------------------------------------------------------
void *
controlThread (void *)
{
  struct timeval tv;
  unsigned long startTime, endTime, elapsed;

  float dt = (float) PERIOD_CONTROL_LOOP / 1000000.0;



  float l_ms_1;
  float l_ms_2;
  char c = '0';



  while (1)
    {
      // Get the time at the start of the loop
      gettimeofday (&tv, NULL);
      startTime = 1000000 * tv.tv_sec + tv.tv_usec;

      // Read in the most recent RC controller inputs in a threadsafe manner
      // FIXME: change l_period0 to something like l_periodRotorSpeed or l_period1 to l_periodTip
      int l_periodAlphaAngle;
      int l_periodBetaAngle;
      int l_periodRotorSpeed;
      int l_resetValue;

      pthread_mutex_lock (&rcInputMutex);
      l_periodAlphaAngle = g_AlphaControlRad;
      l_periodBetaAngle = g_BetaControlRad;
      l_periodRotorSpeed = g_ThrustControlRadPerSec;
      l_resetValue = g_ResetValuePeriod;
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

      //  roll *= (M_PI / 180.0);
      //  pitch *= (M_PI / 180.0);
      //  yaw *= (M_PI / 180.0);

      //-------------------Discard the time of the first cycle-- -- -------------
      if (!isFirst)
	{
	  if (dt > maxdt)
	    maxdt = dt;
	  if (dt < mindt)
	    mindt = dt;
	}
      isFirst = 0;

      //------------- Output Calibration Parameters  ---------

	 pthread_mutex_lock (&rcInputMutex);
	l_ms_1 = g_ms_1;
	l_ms_2 = g_ms_2;
	pthread_mutex_unlock (&rcInputMutex);     

      Output (l_ms_1, l_ms_2);
      printf ("1:%f \t 2:%f \n", g_ms_1, g_ms_2);

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

void *
calibrationThread (void *)
{

  struct timeval tv;
  unsigned long startTime, endTime, elapsed;

  float dt = (float) PERIOD_CALIBRATION_LOOP / 1000000.0;

  int c;
//  char c;
  float l_ms_1;
  float l_ms_2;

  while (1)
    {

// Get the time at the start of the loop
      gettimeofday (&tv, NULL);
      startTime = 1000000 * tv.tv_sec + tv.tv_usec;

      pthread_mutex_lock (&rcInputMutex);
      l_ms_1 = g_ms_1;
      l_ms_2 = g_ms_2;

      pthread_mutex_unlock (&rcInputMutex);


      switch (c = getchar ())
	{
	case 'q':
	  l_ms_1 = l_ms_1 + 0.01;
	  c = '0';
	  break;
	case 'a':
	  l_ms_1 = l_ms_1 - 0.01;
	  c = '0';
	  break;
	case 'w':
	  l_ms_2 = l_ms_2 + 0.01;
	  c = '0';
	  break;
	case 's':
	  l_ms_2 = l_ms_2 - 0.01;
	  c = '0';
	  break;


	}



      pthread_mutex_lock (&rcInputMutex);

      g_ms_1 = l_ms_1;
      g_ms_2 = l_ms_2;

      pthread_mutex_unlock (&rcInputMutex);


      // Get the time after the execution of the loop and sleep the appropriate number of microseconds
      gettimeofday (&tv, NULL);
      endTime = 1000000 * tv.tv_sec + tv.tv_usec;

      elapsed = endTime - startTime;

      if (elapsed < PERIOD_CALIBRATION_LOOP)
	{
	  elapsedOutput_t2 = PERIOD_CALIBRATION_LOOP - elapsed;
	  usleep (elapsedOutput_t2);
	}
    }

}

void *
outputThread (void *)
{

  struct timeval tv;
  unsigned long startTime, endTime, elapsed;

  float dt = (float) PERIOD_OUTPUT_LOOP / 1000000.0;

  while (1)
    {

      float l_ms_1;
      float l_ms_2;
      float l_roll;
      float l_pitch;


      pthread_mutex_lock (&rcInputMutex);
      l_ms_1 = g_ms_1;
      l_ms_2 = g_ms_2;
      l_roll = roll;
      l_pitch = pitch;
      pthread_mutex_unlock (&rcInputMutex);

//      printf ("\t Offsets ->\t alpha:%f beta:%f \t AHRS -> \t roll:%f  pitch:%f \n", l_ms_1,
//            l_ms_2, l_roll, l_pitch);


// Get the time at the start of the loop
      gettimeofday (&tv, NULL);
      startTime = 1000000 * tv.tv_sec + tv.tv_usec;


      // Get the time after the execution of the loop and sleep the appropriate number of microseconds
      gettimeofday (&tv, NULL);
      endTime = 1000000 * tv.tv_sec + tv.tv_usec;

      elapsed = endTime - startTime;

      if (elapsed < PERIOD_OUTPUT_LOOP)
	{
	  elapsedOutput_t3 = PERIOD_OUTPUT_LOOP - elapsed;
	  usleep (elapsedOutput_t3);
	}
    }

}

/*_________________________________________________________________________________________________________________________________________________________________________________________
*
* 					                      ***** Functions *****
*__________________________________________________________________________________________________________________________________________________________________________________________*/
//2functions

void
Output (float ms1, float ms2)
{
  // ms1 = 1.45;
  //  ms2 = 1.45;
  //printf ("ms1:%f\t ms2:%f\t \n", ms1, ms2);
/*
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
*/
  pwm1.set_duty_cycle (PWM_OUTPUT_1, ms1);
  pwm2.set_duty_cycle (PWM_OUTPUT_2, ms2);
  // pwm3.set_duty_cycle (PWM_OUTPUT_3, ms3);
  // pwm4.set_duty_cycle (PWM_OUTPUT_4, ms3);
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
  if (argc < 2)
    {
      printf ("Enter parameter\n");
      print_help ();

      return EXIT_FAILURE;
    }
  //prevent the error message
  opterr = 0;

  while ((parameter = getopt (argc, argv, "i:h")) != -1)
    {
      switch (parameter)
	{
	case 'i':

	  sensor_name = optarg;
	  break;
	case 'h':
	  print_help ();
	  return EXIT_FAILURE;
	case '?':
	  printf ("Wrong parameter.\n");
	  print_help ();
	  return EXIT_FAILURE;
	}
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
  //initscr();

  // keypad(stdscr, TRUE);
  // nonl();
  // cbreak();
  // noecho();

  //refresh();
  //---------------------- Calling the threads -------------------------------

  pthread_create (&thread1, NULL, controlThread, NULL);
  pthread_detach (thread1);
  pthread_create (&thread2, NULL, calibrationThread, NULL);
  pthread_detach (thread2);
  pthread_create (&thread3, NULL, outputThread, NULL);
  pthread_exit (&thread1);
  pthread_exit (&thread2);
  pthread_exit (&thread3);

}
