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

//2main
///////////////////////////////////////
// Functions
//////////////////////////////////////`

void Output(float,float,float);        // Outputs the computed milliseconds to Navio pins
float ManualAlphaMs(int);                     //  Converts Alpha angle to ms for -- Manual Controller
float ManualBetaMs(int);                      // Converts  Beta angle to ms for -- Manual Controller
float AutoAlphaMs(float);                    //  Converts  Alpha angle to ms for -- Auto Controller
float AutoBetaMs(float);                    //   Converts  Beta  angle to ms for -- Auto Controller

////////////////////////////////////////
// Matrix Objects
///////////////////////////////////////

// Body Fixed Frame(BFF) Variables

VectorXd acceleration_bff(3);  //*********************** -- Body Fixed Frame Acceleration

VectorXd omega_bff(3);

// Inertial Frame(IF) Variables

VectorXd acceleration_if(3);   //********************** -- Inertial Frame Acceleration

VectorXd velocity_kif(3);  //***************************** -- Kth velocity in the inertial frame
VectorXd position_kif(3);  //***************************** -- Kth position in the inertial frame

VectorXd omega_kif(3);
VectorXd angle_kif(3);

VectorXd velocity_k1if(3); //***************************** -- K+1th velocity in the inertial frame
VectorXd position_k1if(3); //***************************** -- K+1th position in the inertial frame


VectorXd velocity_des_kif(3);
VectorXd position_des_kif(3);
VectorXd omega_des_kif(3);
VectorXd angle_des_kif(3);


MatrixXd rotx(3,3); // Roll
MatrixXd roty(3,3); // Pitch
MatrixXd rotz(3,3); // Yaw
MatrixXd rot_bi(3,3); // Complete rotation matrix from BFF to IF
MatrixXd jacobian(3,3); // Jacobian Matrix
MatrixXd InertialMatrix(3,3); // Inertial Matrix
////////////////////////////////////
// Constants
////////////////////////////////////
#define G_SI 9.80665
#define PI   3.14159
#define NUM_IMU_SAMPLES 3
#define NUM_IMU_CALIBS  100
#define mass 0.7 // kg

///////////////////////////////////
// Constants for Controller
/////////////////////////////////
// Auto

// Manual
float MBeta= 0.0001/1177;
float MAlpha= 0.000465/1171;
float CBeta= 0.0007;
float CAlpha= 0.0003;

#define PWM_OUTPUT_1 0 // 1
#define PWM_OUTPUT_2 1 // 2
#define PWM_OUTPUT_3 2 // 3

#define MS_ZERO_ANGLE_1 1.45   // beta
#define MS_ZERO_ANGLE_2 1.45   //  alpha

#define PERIOD_CONTROL_LOOP 2500   // microseconds = 1 ms
#define PERIOD_INPUT_LOOP   10000  // microseconds = 10 ms
#define PERIOD_OUTPUT_LOOP  100000 // microseconds = 100 ms


////////////////////////////////////
// Sensor, input, and actuator objects
////////////////////////////////////
InertialSensor * imuMpu;
InertialSensor * imuLsm;
// NOTE: The IMU on this system only has angular rate, linear acceleration, and magnetometer


AHRS        ahrs;
//Mahony AHRS
// NOTE: This doesn't use the magnetometer to get rid of bias drift. Plus, the magnetometer probably wouldn't work well inside our building.


PWM         pwm1;
PWM         pwm2;
PWM         pwm3;
RCInput     rcin;

pthread_mutex_t rcInputMutex;
pthread_mutex_t controllerStateMutex;
pthread_mutex_t outputMutex;
//MS5611 barometer;


////////////////////////////////////
//Sensor reading data
////////////////////////////////////
float       axMpu   , ayMpu, azMpu;
float       gxMpu   , gyMpu, gzMpu;

float       axLsm   , ayLsm, azLsm;
float       gxLsm   , gyLsm, gzLsm;

float       mx   , my, mz;
float       v_x = 0, v_y = 0;

float       p    , p_ini[10], p1, p0 = 0, dh;

//Variables
float       t    , t_ini[10], t0;
float       g = G_SI, M = 0.02896, R = 8.314, a = 6.5;

int                 count = 0;

////////////////////////////////////////////////////////////////////////
//Orientation data
////////////////////////////////////////////////////////////////////////
float       roll, pitch, yaw;
//float     rotx[3][3],roty[3][3],rotz[3][3];

////////////////////////////////////////////////////////////////////////
// RC controller inputs (mutex protected)
/////////////////////////////////////////////////////////////////////////
int                 g_AlphaControlRad;
int                 g_BetaControlRad;
int                 g_ThrustControlRadPerSec;
int                 g_ResetValuePeriod;

////////////////////////////////////////////////////////////////////////
// Controller state variables (mutex protected)
////////////////////////////////////////////////////////////////////////
float g_state[12];   // this contains [x, y, z, phi, theta, psi, dx, dy, dz, dphi, dtheta, dpsi]
float g_inputs[3];  // this contains [rotor_velocity, alpha, beta]

//////////////////////////////////////////////////////////////////////
// Elased Time Outputs (mutex protected)
/////////////////////////////////////////////////////////////////////
int elapsedOutput_t1;
int elapsedOutput_t2;
int elapsedOutput_t3;

//process timing variables
clock_t t1_i;
clock_t     t1_f;
float       t1_v;
clock_t     t2_i;
clock_t     t2_f;
float       t2_v;
clock_t     t3_i;
clock_t     t3_f;
//clock_t t1_i;
float       t3_v;

//Timing data

float       offset[3];
struct timeval  tv;
float       dt   , maxdt;
float       mindt = 0.01;
unsigned long   previoustime, currenttime;
float       dtsumm = 0;
int                 isFirst = 1;

//Network data
int                 sockfd;
struct sockaddr_in servaddr = {0};
char        sendline[80];

InertialSensor *create_inertial_sensor(char *sensor_name)
{
  InertialSensor *imu;

  if (!strcmp(sensor_name, "mpu"))
  {
    printf("Selected: MPU9250\n");
    imu = new MPU9250();
  }
  else if (!strcmp(sensor_name, "lsm"))
  {
    printf("Selected: LSM9DS1\n");
    imu = new LSM9DS1();
  }
  else
  {
    return NULL;
  }

  return imu;
}

void        print_help()
{
  printf("Possible parameters:\nSensor selection: -i [sensor name]\n");
  printf("Sensors names: mpu is MPU9250, lsm is LSM9DS1\nFor help: -h\n");
  printf("If you want to visualize IMU data on another machine,\n");
  printf("add IP address and port number (by default 7000):\n");
  printf("-i [sensor name] ipaddress portnumber\n");

}

//=============================Initial setup == == == == == == == == == == == == == == == == =

void        imuSetup()
{

  float       t  , p;
  float       g = G_SI, p_0 = 1013.25;
  //-----------------------MPU initialization-- -- --------------------------

  imuMpu->initialize();
  imuLsm->initialize();

  //-------------------------------------------------------------------------
  float l_ax = 0.0, l_ay = 0.0, l_az = 0.0;
  float l_gx = 0.0, l_gy = 0.0, l_gz = 0.0;
  for(int i = 0; i < NUM_IMU_CALIBS; i++)
  {
    imuMpu->update();
    imuMpu->read_gyroscope(&gxMpu, &gyMpu, &gzMpu);

    imuLsm->update();
    imuLsm->read_gyroscope(&gxLsm, &gyLsm, &gzLsm);

    gxMpu *= 180 / PI;
    gyMpu *= 180 / PI;
    gzMpu *= 180 / PI;

    gxLsm *= 180 / PI;
    gyLsm *= 180 / PI;
    gzLsm *= 180 / PI;
 

    offset[0] += (-gxMpu*0.0175 -gxLsm*0.0175);
    offset[1] += (-gyMpu*0.0175 -gyLsm*0.0175);
    offset[2] += (-gzMpu*0.0175 -gzLsm*0.0175);
    usleep(10000);
  }
  offset[0] /= (2.0*(float)NUM_IMU_CALIBS);
  offset[1] /= (2.0*(float)NUM_IMU_CALIBS);
  offset[2] /= (2.0*(float)NUM_IMU_CALIBS);

  printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
  ahrs.setGyroOffset(offset[0], offset[1], offset[2]);

  
}

// Set up the servo motor objects for controlling the rotor velocity and the two linear actuators
int servoSetup()
{

  if (check_apm())
  {
    return 1;
  }

  // Initialize the RC controller input object
  rcin.init();


  //barometer.initialize();

  // Initialize the three PWM outputs

  if (!pwm1.init(PWM_OUTPUT_1))                                           // alpha linear actuator
  {
    fprintf(stderr, "Output Enable not set for PWM1. Are you root?\n");
    return 0;
  }
  if (!pwm2.init(PWM_OUTPUT_2))                                           // beta linear actuator
  {
    fprintf(stderr, "Output Enable not set for PWM2. Are you root?\n");
    return 0;
  }
  if (!pwm3.init(PWM_OUTPUT_3))                                          // rotor velocity
  {
    fprintf(stderr, "Output Enable not set for PWM3. Are you root?\n");
    return 0;
  }

  // Enable each of the PWM and set the initial period
  pwm1.enable(PWM_OUTPUT_1);
  pwm1.set_period(PWM_OUTPUT_1, 200);

  pwm2.enable(PWM_OUTPUT_2);
  pwm2.set_period(PWM_OUTPUT_2, 200);

  pwm3.enable(PWM_OUTPUT_3);
  pwm3.set_period(PWM_OUTPUT_3, 200);
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

 void* controlThread(void *)
{
  struct timeval  tv;
  unsigned long   startTime, endTime, elapsed;

  float dt = (float)PERIOD_CONTROL_LOOP/1000000.0;

//********************************************************************************** -- Initializing the Inertial Frame parameters

// Velocities

  velocity_kif(0)=0.0;
  velocity_kif(1)=0.0;
  velocity_kif(2)=0.0;

// Positions

   position_kif(0)=0.0;
   position_kif(1)=0.0;
   position_kif(2)=0.0;

  while (1)
  {
    // Get the time at the start of the loop
    gettimeofday(&tv, NULL);
    startTime = 1000000 * tv.tv_sec + tv.tv_usec;

    // Read in the most recent RC controller inputs in a threadsafe manner
    // FIXME: change l_period0 to something like l_periodRotorSpeed or l_period1 to l_periodTip
    int l_periodAlphaAngle ;
    int l_periodBetaAngle ;
    int l_periodRotorSpeed ;
    int l_resetValue;
    
    pthread_mutex_lock(&rcInputMutex);
    l_periodAlphaAngle = g_AlphaControlRad;
    l_periodBetaAngle = g_BetaControlRad;
    l_periodRotorSpeed = g_ThrustControlRadPerSec;
    l_resetValue = g_ResetValuePeriod;
    pthread_mutex_unlock(&rcInputMutex);


    //--------Read raw measurements from the MPU and update AHRS-- -- ----------
    //Accel + gyro.
    float l_ax = 0.0, l_ay = 0.0, l_az = 0.0;
    float l_gx = 0.0, l_gy = 0.0, l_gz = 0.0;
    for (int i = 0 ; i < NUM_IMU_SAMPLES ; i++)
    {
   	imuMpu->update();
    	imuMpu->read_accelerometer(&axMpu, &ayMpu, &azMpu);
    	imuMpu->read_gyroscope(&gxMpu, &gyMpu, &gzMpu);

	imuLsm->update();
        imuLsm->read_accelerometer(&axLsm, &ayLsm, &azLsm);
        imuLsm->read_gyroscope(&gxLsm, &gyLsm, &gzLsm);

	l_ax += (axMpu+axLsm);
	l_ay += (ayMpu+ayLsm);
	l_az += (azMpu+azLsm);

	l_gx += (gxMpu+gxLsm);
        l_gy += (gyMpu+gyLsm);
        l_gz += (gzMpu+gzLsm);

    }
    l_ax /= 2.0*(float)NUM_IMU_SAMPLES;
    l_ay /= 2.0*(float)NUM_IMU_SAMPLES;
    l_az /= 2.0*(float)NUM_IMU_SAMPLES;

    l_gx /= 2.0*(float)NUM_IMU_SAMPLES;
    l_gy /= 2.0*(float)NUM_IMU_SAMPLES;
    l_gz /= 2.0*(float)NUM_IMU_SAMPLES;


    l_ax /= G_SI;
    l_ay /= G_SI;
    l_az /= G_SI;
    l_gx *= 180 / PI;
    l_gy *= 180 / PI;
    l_gz *= 180 / PI;

    ahrs.updateIMU(l_ax, l_ay, l_az, l_gx * 0.0175, l_gy * 0.0175, l_gz * 0.0175, dt);

    //------------------------Read Euler angles-- -- --------------------------
    ahrs.getEuler(&roll, &pitch, &yaw);

    roll  *= (M_PI/180.0);
    pitch *= (M_PI/180.0);
    yaw   *= (M_PI/180.0);

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


//******************************* -- Body Fixed Frame Parameters -- ************************************************************************************************************************

   /*  Accelerometer readings are the in the Body fixed frame, these will be trasformed to Intertial frame using the rotational matrices.

       These values would be used to get the intertial frame -- velocities and positions. */

   acceleration_bff(0) = l_ax;
   acceleration_bff(1) = l_ay;
   acceleration_bff(2) = l_az;

   omega_bff(0) = l_gx;
   omega_bff(1) = l_gy;
   omega_bff(2) = l_gz; 
//*********************************************************************** -- Rotational Matrices -- ****************************************************************************************

    // Roll

    rotx(0,0)=1.0;
    rotx(0,1)=0.0;
    rotx(0,2)=0.0;
    rotx(1,0)=0.0;
    rotx(1,1)=cos(roll);
    rotx(1,2)=-sin(roll);
    rotx(2,0)=0.0;
    rotx(2,1)=sin(roll);
    rotx(2,2)=cos(roll);

    // Pitch

    roty(0,0)=cos(pitch);
    roty(0,1)=0.0;
    roty(0,2)=sin(pitch);
    roty(1,0)=0.0;
    roty(1,1)=1.0;
    roty(1,2)=0.0;
    roty(2,0)=-sin(pitch);
    roty(2,1)=0.0;
    roty(2,2)=cos(pitch);

    // Yaw

    rotz(0,0)=cos(yaw);
    rotz(0,1)=-sin(yaw);
    rotz(0,2)=0.0;
    rotz(1,0)=sin(yaw);
    rotz(1,1)=cos(yaw);
    rotz(1,2)=0.0;
    rotz(2,0)=0.0;
    rotz(2,1)=0.0;
    rotz(2,2)=1.0;


//************************************************** -- Complete Rotation Matrix From BFF to IF -- *****************************************************************************************

    rot_bi=rotz*roty*rotx;


// Intertial Matrix

InertialMatrix(0,0) = 1.0;
InertialMatrix(0,1) = 0.0;
InertialMatrix(0,2) = 0.0;
InertialMatrix(1,0) = 0.0;
InertialMatrix(1,1) = 1.0;
InertialMatrix(1,2) = 0.0;
InertialMatrix(2,0) = 0.0;
InertialMatrix(2,1) = 0.0;
InertialMatrix(2,2) = 1.0;


// Jacobian Matrix 

jacobian(0,0) = InertialMatrix(0,0);
jacobian(0,1) = 0;
jacobian(0,2) = -InertialMatrix(0,0)*sin(-pitch);
jacobian(1,0) = 0;
jacobian(1,1) = (InertialMatrix(1,1)*cos(-roll)*cos(-roll))+(InertialMatrix(2,2)*sin(-roll)*sin(-roll));
jacobian(1,2) = (InertialMatrix(1,1)-InertialMatrix(2,2))*cos(-roll)*sin(-roll)*cos(-pitch);
jacobian(2,0) = -InertialMatrix(0,0)*sin(-pitch);
jacobian(2,1) = (InertialMatrix(1,1)-InertialMatrix(2,2))*cos(-roll)*sin(-roll)*cos(-pitch);
jacobian(2,2) = (InertialMatrix(0,0)*sin(-pitch)*sin(-pitch))+(InertialMatrix(1,1)*(sin(-roll)*sin(-roll))*(cos(-pitch)*cos(-pitch)))+(InertialMatrix(2,2)*(cos(-roll)*cos(-roll))*(cos(-pitch)*cos(-pitch)));

//************************************************** -- Intertial Frame Parameters -- *******************************************************************************************************


    acceleration_if = rot_bi*acceleration_bff;
    acceleration_if(2) = acceleration_if(2) - 1.0;
    acceleration_if *= G_SI;
    /*
    if(l_resetValue<1450){
    velocity_k1if << 0,0,0;
    position_k1if << 0,0,0;
    printf("values reset: position(k1if):%f\t%f\t%f \t velocity(k1if):%f\t%f\t%f \n",position_k1if(0),position_k1if(1),position_k1if(2),velocity_k1if(0),velocity_k1if(1),velocity_k1if(2) );
    }*/ 
   
    velocity_k1if = velocity_kif+acceleration_if*dt;
    position_k1if = position_kif+velocity_k1if*dt;
     printf("roll:%f\tposition(k1if):%f\t%f\t%f \t velocity(k1if):%f\t%f\t%f \n",roll,position_k1if(0),position_k1if(1),position_k1if(2),velocity_k1if(0),velocity_k1if(1),velocity_k1if(2));

 //****************************************************************************************************************************************************************************************  
//********************* -- Reseting the velocity position and yaw values on moving the throttle stick to left -- *************************************************************************** //*****************************************************************************************************************************************************************************************  

    if(l_resetValue<1450){
    velocity_k1if << 0,0,0;
    position_k1if << 0,0,0;
    roll = 0;
    printf("values reset: roll:%f\tposition(k1if):%f\t%f\t%f \t velocity(k1if):%f\t%f\t%f \n",roll,position_k1if(0),position_k1if(1),position_k1if(2),velocity_k1if(0),velocity_k1if(1),velocity_k1if(2) );
    }

    velocity_kif = velocity_k1if;
    position_kif = position_k1if;

    angle_kif(0) = roll;
    angle_kif(1) = pitch;
    angle_kif(2) = yaw; 

   omega_kif = jacobian*omega_bff;
   omega_kif *= M_PI/180.0;


/*
    omega_kif(0) = l_gx*M_PI/180.0;
    omega_kif(1) = l_gy*M_PI/180.0;
    omega_kif(2) = l_gz*M_PI/180.0;
*/


//  printf("gx_if:%f\t gy_if:%f\t gz_if:%f \n",omega_kif(0),omega_kif(1),omega_kif(2));  

    //printf("%f, %f, %f\n", roll, pitch, yaw);

//**************************************************************************************** -- Auto Controller -- ****************************************************************************
    // WARNING: l_gz is in degrees and l_ax is normalized to 1.0

    float k1 = -1, k2 = -0.1; // Thrust gains
    float k3 = -1, k4 = -1, k7 =-1, k8=-8; // alpha gains
    float k5 = 1, k6 = 1,k9 = -1, k10 = -4; // beta gains

    // Setting linear position and velocity gains to zero temporarily because estimates are bad.
    k7 = 0; k8 = 0; k9 = 0; k10 = 0;

    position_des_kif  << 0,0,0;
    velocity_des_kif  << 0,0,0;
    angle_des_kif     << 0,0,0;
    omega_des_kif     << 0,0,0;
/*
   if(l_resetValue<960){
    omega_kif     << 0,0,0;
    position_kif  << 0,0,0;
    velocity_kif  << 0,0,0;
    printf("reset on:\t omega:%f\t%f\t%f \tposition:%f\t%f\t%f \t vel:%f\t%f\t%f  \n",omega_kif(0),omega_kif(1),omega_kif(2),position_kif(0),position_kif(1),position_kif(2),velocity_kif(0),velocity_kif(1),velocity_kif(2) ); 
   }
  else{
    omega_kif     = omega_kif;
    position_kif  = position_kif;
    velocity_kif  = velocity_kif;
     printf("reset off:\t omega:%f\t%f\t%f \tposition:%f\t%f\t%f \t vel:%f\t%f\t%f \n",omega_kif(0),omega_kif(1),omega_kif(2),position_kif(0),position_kif(1),position_kif(2),velocity_kif(0),velocity_kif(1),velocity_kif(2) );

  } 
*/ 

   // float cmd_thrust = mass * G_SI + k1 * (position_kif(2) - position_des_kif(2)) + k2 * (velocity_kif(2) - velocity_des_kif(2));
    float cmd_alpha  = k3 * (angle_kif(1) - angle_des_kif(1)) + k4 * (omega_kif(1)-omega_des_kif(1)) + k7 * (position_kif(0) - position_des_kif(0)) + k8 * (velocity_kif(0) - velocity_des_kif(0));
    float cmd_beta   = k5 * (angle_kif(0) - angle_des_kif(0)) + k6 * (omega_kif(0)-omega_des_kif(0))+ k9 * (position_kif(1) - position_des_kif(1)) + k10* (velocity_kif(1) - velocity_des_kif(1));

        
  float ms_2 = AutoBetaMs(cmd_beta);
  float ms_1 = AutoAlphaMs(cmd_alpha);
  //printf("ms1:%f\t,ms2:%f\n",ms_1,ms_2);
//   printf("cmd_alpha:%f\t,cmd_beta:%f\n",cmd_alpha,cmd_beta);
//      printf("roll:%f\t pitch:%f\n",roll,pitch);
//**************************************************************************************** -- Manual Controller -- *************************************************************************

/*
   float ms_2 = ManualBetaMs(l_periodBetaAngle);
   float ms_1 = ManualAlphaMs(l_periodAlphaAngle);
//printf("ms1:%f\t,ms2:%f\n",ms_1,ms_2);
*/
//***************************************************************************************** -- Thrust Computation -- **********************************************************************

    float   m = 0.001;//0.001202;// 0.001;
    float   c = -0.15;//-0.3126;//-0.15;
    //cout<<l_periodRotorSpeed<<endl;
    float   thrust = m * l_periodRotorSpeed + c;
 //   cout<<thrust<<endl;
//******************************************************************************************* -- Output To Pins -- ************************************************************************
//2op
    Output(ms_1,0,thrust);



    pthread_mutex_lock(&controllerStateMutex);
//*************************************************** -- Storing States of Baton -- *********************************************************************************************************
    g_state[0]=position_k1if(0);
    g_state[1]=position_k1if(1);
    g_state[2]=position_k1if(2);
    g_state[3]=roll;
    g_state[4]=pitch;
    g_state[5]= yaw;
    g_state[6]=velocity_k1if(0);
    g_state[7]=velocity_k1if(1);
    g_state[8]=velocity_k1if(2);
    g_state[9]  = l_gx;
    g_state[10] = l_gy;
    g_state[11] = l_gz;
//*************************************************** -- Storing the inputs from RC Trasmitter -- ******************************************************************************************
    g_inputs[0] = l_periodAlphaAngle;
    g_inputs[1]= l_periodBetaAngle;
    g_inputs[2]= l_periodRotorSpeed;
    pthread_mutex_unlock(&controllerStateMutex);

    // Get the time after the execution of the loop and sleep the appropriate number of microseconds
    gettimeofday(&tv, NULL);
    endTime = 1000000 * tv.tv_sec + tv.tv_usec;

    elapsed = endTime-startTime;


    if ( elapsed < PERIOD_CONTROL_LOOP )
    {
      elapsedOutput_t1=PERIOD_CONTROL_LOOP-elapsed;
      usleep(elapsedOutput_t1);
    }


  }
}



//*********************************************************************************************** -- Inputs Thread -- **********************************************************************

void* inputThread(void *)
{
  struct timeval  tv;
  unsigned long   startTime, endTime, elapsed;
  while (1)
  {
//**************************************************************************** Get the time at the start of the loop
    gettimeofday(&tv, NULL);
 startTime = 1000000 * tv.tv_sec + tv.tv_usec;


    int l_periodAlphaAngle = rcin.read(0);
    int l_periodBetaAngle  = rcin.read(1);
    int l_periodRotorSpeed = rcin.read(2);
    int l_resetValue       = rcin.read(4);
    pthread_mutex_lock(&rcInputMutex);
    g_AlphaControlRad        = l_periodAlphaAngle;
    g_BetaControlRad         = l_periodBetaAngle;
    g_ThrustControlRadPerSec = l_periodRotorSpeed;
    g_ResetValuePeriod       = l_resetValue;
    pthread_mutex_unlock(&rcInputMutex);

//************************************************************************* Get the time after the execution of the loop and sleep the appropriate number of microseconds
    gettimeofday(&tv, NULL);
    endTime = 1000000 * tv.tv_sec + tv.tv_usec;

    elapsed = endTime-startTime;

    if ( elapsed < PERIOD_INPUT_LOOP )
    {
      //        elapsedOutput_t2=elapsed;

     elapsedOutput_t2=PERIOD_INPUT_LOOP-elapsed;
      usleep(elapsedOutput_t2);
    }

  }

}


//****************************************************************************************** -- OUTPUT THREAD -- ***************************************************************************

void* outputThread(void *)
{
  struct timeval  tv;
  unsigned long   startTime, endTime, elapsed;
 // gettimeofday(&tv,NULL);
 // currentTime=1000000*tc.tv_sec+tv.tv_usec;
  while (1)
  {
   // Get the time at the start of the loop
    gettimeofday(&tv, NULL);
    startTime = 1000000 * tv.tv_sec + tv.tv_usec;
    int l_periodAlphaAngle;
    int l_periodBetaAngle;
    int l_periodRotorSpeed;
    float l_state[12];
    float l_inputs[3];
    int elapsedt1;
    int elapsedt2;
    int elapsedt3;

    // Copy the RC controller inputs into a local variable (mutex protected)
    pthread_mutex_lock(&rcInputMutex);
    l_periodAlphaAngle = g_AlphaControlRad;
    l_periodBetaAngle = g_BetaControlRad;
    l_periodRotorSpeed = g_ThrustControlRadPerSec;
    pthread_mutex_unlock(&rcInputMutex);


    // Copy the controller states into a local variable (mutex protected)
    pthread_mutex_lock(&controllerStateMutex);
    for (int i = 0 ; i < 12 ; i++)
      l_state[i] = g_state[i];

    for (int i = 0 ; i < 3 ; i++)
      l_inputs[i] = g_inputs[i];
    pthread_mutex_unlock(&controllerStateMutex);

    //************************************************************************* -- Output Inputs from RC - Controller --*****************************************************************

    // printf("RC thrust: %d\tRC alpha: %d\tRC beta: %d\n", l_period0, l_period1, l_period2);

    //*********************************************************************** -- Outputting the Baton State Variables -- ***********************************************************************
    // printf("System States\n\t");
    //printf("%0.6d\n",startTime);
    //printf("%lu,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f\n",startTime,l_state[0],l_state[1],l_state[2],l_state[3],l_state[4],l_state[5],l_state[6],l_state[7],l_state[8],l_state[9],l_state[10],l_state[11]);

    //printf("ax:%f\t,ay:%f\t,az:%f\n",ax,ay,az);

/*
  for (int i = 0 ; i < 12 ; i++)
    {
      printf("%0.6f: ", l_state[i]);
    }
    printf("\n\n");
*/

    // Get the time after the execution of the loop and sleep the appropriate number of microseconds
    gettimeofday(&tv, NULL);
    endTime = 1000000 * tv.tv_sec + tv.tv_usec;

    elapsed = endTime-startTime;
 ;
     elapsedOutput_t3=PERIOD_OUTPUT_LOOP-elapsed;

      pthread_mutex_lock(&outputMutex);
      elapsedt1=elapsedOutput_t1;
      elapsedt2=elapsedOutput_t2;
      elapsedt3=elapsedOutput_t3;
      pthread_mutex_unlock(&outputMutex);

//********************************************************************* -- Output Sleep Times -- ******************************************************************************************

//       printf("Sleep Time:\t Control Thread:%d\t, Input Thread:%d\t, Output Thread:%d\t\n",elapsedt1,elapsedt2,elapsedt3 );


    if ( elapsed < PERIOD_OUTPUT_LOOP )
    {
       usleep(elapsedOutput_t3);
    }

    }
}

//******************************************************************************************************************************************************************************************
//*************************************************************************************************************** -- FUNCTIONS -- **********************************************************
//******************************************************************************************************************************************************************************************
//2functions
float ManualBetaMs(int period_beta){
float mb = -0.0398;
float cb = 59.647;

float mb_ms = -0.0163;
float cb_ms = 1.378;

float beta = mb*period_beta+cb;
float beta_ms= mb_ms*beta+cb_ms;

return(beta_ms);

}

float ManualAlphaMs(int period_alpha){
  //float alpha_d=alpha*180/M_PI;
  float ma = -0.03125;
  float ca = 47.25;

  float ma_ms = -0.0254;
  float ca_ms = 1.467;

  float alpha = ma*7+ca;
  float alpha_ms= ma_ms*alpha+ca_ms;

  return(alpha_ms);

}

float AutoBetaMs(float beta){
  float beta_d = beta*180/M_PI;
  float mb_ms =0;// 0.0214;//-0.02;//-0.0195;//-0.0173;//-0.0195;
  float cb_ms = 1.5;//1.429;//1.425;//1.429;

  float b_ms = mb_ms*beta_d+cb_ms;
  return(b_ms);
}

float AutoAlphaMs(float alpha){
  float alpha_d = alpha*180/M_PI;
  //cout<<alpha_d<<endl;
  float ma_ms =0;//0.0214;//0;//0.0187;//0.0115;//0.0153;//0;//0.017;//0.0175;//0.0115;//-0.0214;// -0.0254;//-0.0208;//-0.0416;
  float ca_ms = 1.43;//1.53086;//1.52;//1.4963;

  float a_ms = ma_ms*alpha_d+ca_ms;
  return(a_ms);

}

void Output(float ms1,float ms2,float ms3){
   // ms1 = 1.45;
  //  ms2 = 1.45;
  // printf("ms1:%f\t ms2:%f\t , ms3:%f\n",ms1,ms2,ms3);
    pwm1.set_duty_cycle(PWM_OUTPUT_2, ms2);
    pwm2.set_duty_cycle(PWM_OUTPUT_1, ms1);
    pwm3.set_duty_cycle(PWM_OUTPUT_3, ms3);
}

//*******************************************************************************************************************************************************************************************
//************************************************************************************************* -- MAIN -- ******************************************************************************
//*******************************************************************************************************************************************************************************************
int                 main   (int argc, char *argv[])
{

  //1 main
  pthread_t    thread1;
  pthread_t    thread2;
  pthread_t    thread3;

//***************************************************************************** -- Initializing the nutex
  pthread_mutex_init(&rcInputMutex, NULL);
  pthread_mutex_init(&controllerStateMutex, NULL);
  pthread_mutex_init(&outputMutex, NULL);



  //pthread_t thread1;
  int         parameter;
  char        *sensor_name;

  if (check_apm())
  {
    return 1;
  }
  if (argc < 2)
  {
    printf("Enter parameter\n");
    print_help();

    return EXIT_FAILURE;
  }
  //prevent the error message
  opterr = 0;

  while ((parameter = getopt(argc, argv, "i:h")) != -1)
  {
    switch (parameter)
    {
    case 'i':

      sensor_name = optarg;
      break;
  case 'h':
      print_help();
      return EXIT_FAILURE;
    case '?':
      printf("Wrong parameter.\n");
      print_help();
      return EXIT_FAILURE;
    }
  }

  imuMpu = create_inertial_sensor("mpu");
  imuLsm = create_inertial_sensor("lsm");

  if (!imuMpu)
  {
    printf("Wrong sensor name. Select: mpu or lsm\n");
    return EXIT_FAILURE;
  }
  if (!imuLsm)
  {
    printf("Wrong sensor name.\n");
    return EXIT_FAILURE;
  }

  if (!imuMpu->probe())
  {
    printf("Sensor not enable\n");
    return EXIT_FAILURE;
  }
  if (!imuLsm->probe())
  {
    printf("Sensor not enable\n");
    return EXIT_FAILURE;
  }


  //---------------------------Network setup-- -- ---------------------------

  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  servaddr.sin_family = AF_INET;

  if (argc == 5)
  {
    servaddr.sin_addr.s_addr = inet_addr(argv[3]);
    servaddr.sin_port = htons(atoi(argv[4]));
  } else
  {
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    servaddr.sin_port = htons(7000);
  }

  //--------------------IMU setup and main loop-- -- ------------------------
  imuSetup();
  servoSetup();

 //********************************************************************************************* -- Calling the threads -- *****************************************************************
  pthread_create(&thread1, NULL, controlThread, NULL);
  pthread_detach(thread1);
  pthread_create(&thread2, NULL, inputThread, NULL);
  pthread_detach(thread2);
  pthread_create(&thread3, NULL, outputThread, NULL);
  pthread_exit(&thread1);
  pthread_exit(&thread2);
  pthread_exit(&thread3);

}





 	
 
  
   

