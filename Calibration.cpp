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
#include <ncurses.h>
#include <curses.h>
#include <sys/select.h>
#include <termios.h>

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

/*__________________________________________________________________________________________________________________________________________________________________________________________
*
*
* 							***** Controller Variables and Functions Declaration *****
*
*__________________________________________________________________________________________________________________________________________________________________________________________*/


//----------------------------------------------------------------------------------------------------------------------
//                               Functions
//----------------------------------------------------------------------------------------------------------------------

//char getch(void);
void Output (float, float, float);	// Outputs the computed milliseconds to Navio pins
int kbhit (void);			// Function to detect the keypress from keboard
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

// Output Pin Definitions

#define PWM_OUTPUT_1 0		// 1
#define PWM_OUTPUT_2 1		// 2
#define PWM_OUTPUT_3 2		// 3
#define PWM_OUTPUT_4 3		// 4
// Thread Control Rates

#define PERIOD_CONTROL_LOOP 2500	// microseconds = 1 ms
#define PERIOD_INPUT_LOOP   10000	// microseconds = 10 ms
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

// Inertial Frame Parameters Initializaion

// Velocities

  velocity_kif (0) = 0.0;
  velocity_kif (1) = 0.0;
  velocity_kif (2) = 0.0;

// Positions

  position_kif (0) = 0.0;
  position_kif (1) = 0.0;
  position_kif (2) = 0.0;

  char cal_a;
  char cal_b;
  char d;

  float l_ca_ms = 1.45;
  float l_ma_ms;
  float l_cb_ms = 1.42;
  float l_mb_ms;


  float a;
  float b;
  char c = '0';
  
  
  cout<<"calibration on? y/n "<<endl;
  cin>>d;

  while (1)
    {
      if(d=='n'){
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

      //-------------Console and network output with a lowered rate-- -- --------


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


// Jacobian Matrix 

      jacobian << Jaco (roll, pitch, yaw);


//-------------------- Intertial Frame Parameters ------------------------------------------------------


      acceleration_if = rot_bi * acceleration_bff;
      acceleration_if (2) = acceleration_if (2) - 1.0;
      acceleration_if *= G_SI;

      velocity_k1if = velocity_kif + acceleration_if * dt;
      position_k1if = position_kif + velocity_k1if * dt;


//--------------- Refresh Button: Pushing the "E" Lever on RC Transmitter ------------------------------   

/*
   Note: Pushing the lever down will set the : Posioition and velocity values to zero. And on pushing it back up, new computed values of position and velocity will be used.
         This needs to be done because, we get both position and velocity terms upon intergration. All the sensors of baton take some time to spit to the right values upon initialization.           So the intially velcity and position terms will be an intergration of noise. Once we sensors begin to give constant values, we refresh velocity and position, so that they integrate
         the right values and give us good results. 
*/

      if (l_resetValue < 960)
	{
	  velocity_k1if << 0, 0, 0;
	  position_k1if << 0, 0, 0;
	}

      velocity_kif = velocity_k1if;
      position_kif = position_k1if;

      angle_kif (0) = roll;
      angle_kif (1) = pitch;
      angle_kif (2) = yaw;

      omega_kif = jacobian * omega_bff;


//---------------------------- Auto Controller -------------------------------------------

      // WARNING: l_gz is in degrees and l_ax is normalized to 1.0

/*  float k1 = -1, k2 = -0.1; // Thrust gains
    float k3 = -1, k4 = -1, k7 =-1, k8=-8; // alpha gains        // gain values from matlab simulations
    float k5 = 1, k6 = -1,k9 = -1, k10 = -4; // beta gains
*/
      float k1 = -1, k2 = -0.1;	// Thrust gai1   
      float k3 = -0.95, k4 = -0.05, k7 = -1, k8 = -8;	// alpha gains     // Adjusted gain values after flight tests
      float k5 = -0.95, k6 = -0.05, k9 = -1, k10 = -4;	// beta gains

      k3 = -1;
      k5 = -1;
      k4 = 0;
      k6 = 0;

      // Settiing linear position and velocity gains to zero temporarily because estimates are bad.
      k7 = -1;
      k8 = 0;
      k9 = -1;
      k10 = 0;

      float x_des = (float) (l_periodBetaAngle - 1501) / 500.0 * 0.1;
      float y_des = (float) (l_periodAlphaAngle - 1498) / 500.0 * 0.1;

      position_kif (0) = 0;
      position_kif (1) = 0;
      velocity_kif << 0, 0, 0;

      position_des_kif << x_des, y_des, 0;
      velocity_des_kif << 0, 0, 0;
      angle_des_kif << 0, 0, 0;
      omega_des_kif << 0, 0, 0;

//------------------------------------ Auto Controller Equations ----------------------------

      // float cmd_thrust = mass * G_SI + k1 * (position_kif(2) - position_des_kif(2)) + k2 * (velocity_kif(2) - velocity_des_kif(2));
      float cmd_alpha =
	k3 * (angle_kif (1) - angle_des_kif (1)) + k4 * (omega_kif (1) -
							 omega_des_kif (1)) +
	k7 * (position_kif (0) - position_des_kif (0)) +
	k8 * (velocity_kif (0) - velocity_des_kif (0));
      float cmd_beta =
	k5 * (angle_kif (0) - angle_des_kif (0)) + k6 * (omega_kif (0) -
							 omega_des_kif (0)) +
	k9 * (position_kif (1) - position_des_kif (1)) +
	k10 * (velocity_kif (1) - velocity_des_kif (1));

          float ms_2 = AutoBetaMs (cmd_beta);
           float ms_1 = AutoAlphaMs (cmd_alpha);


//------------------------------------- Thrust Computation ---------------------------------

      float m =  0.001202;      // 0.001;
      float c = -0.3126;        //-0.15;
     //cout<<l_periodRotorSpeed<<endl;
      float thrust = m * l_periodRotorSpeed + c;

//-------------------------------------- Output To Pins -----------------------------------

      Output (ms_1, ms_2, thrust);




}


//------------------------------ Calibration -------------------------------------------------- 

else if(d == 'y'){
      if (kbhit () != 0)
	{
	  c = getchar ();
	}
      /* If a key has been pressed */
      if (c == 'q')
	{
	  l_ca_ms = l_ca_ms + 0.01;
          //Output (l_ca_ms, l_cb_ms,0);
	  pwm1.set_duty_cycle (PWM_OUTPUT_1, l_ca_ms);
	  pwm2.set_duty_cycle (PWM_OUTPUT_2, l_cb_ms );

	 c = '0';		// and put it back to \0
	 
	}
      else if (c == 'a')
	{

	  l_ca_ms = l_ca_ms - 0.01;
        //  Output (l_ca_ms,l_cb_ms,0);
	 pwm1.set_duty_cycle (PWM_OUTPUT_1, l_ca_ms);
	  pwm2.set_duty_cycle (PWM_OUTPUT_2, l_cb_ms);

	  c = '0';

	}

      else if (c == 'w')
	{

	  l_cb_ms = l_cb_ms + 0.01;
        //  Output (l_ca_ms,l_cb_ms,0);
	 pwm1.set_duty_cycle (PWM_OUTPUT_1, l_ca_ms);
	  pwm2.set_duty_cycle (PWM_OUTPUT_2, l_cb_ms);

	  c = '0';

	}

      else if (c == 's')
	{

	  l_cb_ms = l_cb_ms - 0.01;
        //  Output (l_ca_ms,l_cb_ms,0);
	 pwm1.set_duty_cycle (PWM_OUTPUT_1, l_ca_ms);
	  pwm2.set_duty_cycle (PWM_OUTPUT_2, l_cb_ms);

	  c = '0';

	}
	else if(c == 'd'){
        float ma = ComputeAlphaSlope(l_ca_ms);
        float mb = ComputeBetaSlope (l_cb_ms);
        printf("new intercept :: alpha:%f \t beta: %f \n",l_ca_ms,l_cb_ms);
	printf("new slope :: alpha:%f \t beta:%f\n",ma,mb);
  	break;
	}

       pwm1.set_duty_cycle (PWM_OUTPUT_1, l_ca_ms);
          pwm2.set_duty_cycle (PWM_OUTPUT_2, l_cb_ms);

      


    }

}


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


/*__________________________________ _______________________________________________________________________________________________________________________________________________________
*
* 					                      ***** Functions *****
*__________________________________________________________________________________________________________________________________________________________________________________________*/
//2functions


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
setSlope (float c)
{

  float m = (1.3 - c) / -12;
  return (m);

}



float
AutoBetaMs (float beta)
{
  float beta_d = beta * 180 / M_PI;
  // printf("beta_d:%f\n",beta_d);
  float mb_ms = 0.02;//0;		//0.016;              //0.013;//0.0214;
  float cb_ms = 1.42;

  float b_ms = mb_ms * beta_d + cb_ms;
  return (b_ms);
}

float
AutoAlphaMs (float alpha)
{
  // float ca_ms = 1.45;
//  float ma_ms = 0;//0.0125 ; 


  float alpha_d = alpha * 180 / M_PI;
  cout<<alpha_d;
   ma_ms = 0.015;//0.0125;//0.0117;           // 0.015;
   ca_ms = 1.45;//1.44;

  float a_ms = ma_ms * alpha_d + ca_ms;
  return (a_ms);

}

float ComputeAlphaSlope(float ca){
        float angle= 80-90;
	float ma = (1.3-ca)/angle;
	return(ma);
}

float ComputeBetaSlope(float cb){
	float angle = 84-90;
        float mb = (1.3-cb)/angle;
        return(mb);
}

void
Output (float ms1, float ms2, float ms3)
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
  pwm3.set_duty_cycle (PWM_OUTPUT_3, ms3);
  pwm4.set_duty_cycle (PWM_OUTPUT_4, ms3);
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

  //---------------------- Calling the threads -------------------------------

  pthread_create (&thread1, NULL, controlThread, NULL);
  //pthread_detach (thread1);
  //pthread_create (&thread2, NULL, inputThread, NULL);
  //pthread_detach (thread2);
  //pthread_create (&thread3, NULL, outputThread, NULL);
  pthread_exit (&thread1);
  //pthread_exit (&thread2);
  //pthread_exit (&thread3);

}
