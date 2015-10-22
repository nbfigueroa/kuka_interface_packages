/*
 * LWRCore.h
 *
 *  Created on: Feb 1, 2012
 *      Author: klas
 */

#ifndef LWRCORE_H_
#define LWRCORE_H_


#include "StdTools/NCConsole.h"
#include "StdTools/Various.h"
#include "RobotLib/RobotInterface.h"
#include "MathLib/MathLib.h"
#include "RobotLib/Sensor.h"
#include "RobotLib/Robot.h"
#include "KUKARobotModel/LWRRobot.h"
#include "FastResearchInterface.h"
#include "LinuxAbstraction.h"

//#inlcude "DisplayThread"
#include "boost/thread.hpp"
#include "time.h"
#include <deque>
#include <ros/package.h>

#define MEASUREMENT_HISTORY_LENGTH 20

#define MODE_NONE 0
#define MODE_REC  1
#define MODE_PREP 2
#define MODE_GO   3

/* #define CTRL_JPOS 0 */
/* #define CTRL_GCMP 1 */
/* #define CTRL_JIMP 2 */
/* #define CTRL_CART 3 */

    /* enum  { */
    /*     CTRLMODE_NONE = 0, */
    /*     CTRLMODE_POSITION , */
    /*     CTRLMODE_VELOCITY, */
    /*     CTRLMODE_ACCELERATION, */
    /*     CTRLMODE_TORQUE, */
    /*     CTRLMODE_DEFAULT, */
    /* 	//added for LWR compatibility */
    /* 	CTRLMODE_CARTIMPEDANCE, */
    /* 	CTRLMODE_JOINTIMPEDANCE, */
    /* 	CTRLMODE_GRAVITYCOMPENSATION, */
    /* }; */


#ifndef RAD
#define RAD(A)	((A) * M_PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)	((A) * 180.0 / M_PI )
#endif


#define FRI_CONN_TIMEOUT_SEC	30

#define JOINT_MAX_VEL_DEG_SEC  60.0
#define JOINT_MAX_ACC_DEG_SEC  60.0
#define CART_MAX_VEL_M_SEC  0.2
#define CART_MAX_ACC_M_SEC  0.05

#define FRI_JOINT_STIFFNESS 200
#define FRI_JOINT_DAMPING   0.7

#define FRI_CART_STIFFNESS_POS 		300
#define FRI_CART_STIFFNESS_ORIENT 	30
#define FRI_CART_DAMPING_POS 		0.7
#define FRI_CART_DAMPING_ORIENT 	0.7




/*this parameter determines how often the impedance should be updated relative to the position.
 * example: if REL_FREQ_IMPEDANCE_UPDATE = 10, then the desired impedance will be sent to the robot
 * every 10 iterations of desrired pose command update.*/
#define REL_FREQ_IMPEDANCE_UPDATE 10


// a little structure used for storing measurements from the robot
struct LWRMeasurement{

  MathLib::Vector JointPositions;
  MathLib::Vector JointVelocities;
  MathLib::Vector JointAccelerations;
  MathLib::Vector JointTorques;
  double t;

};


class LWRCore : public RobotInterface
{


  FastResearchInterface * mFRI;
  LWRRobot* mLWRRobot;

  bool bRunConsoleLoop;
  boost::thread mDisplayThread;
  //  boost::thread mCoreThread;
  NCConsole mNCConsole;
  streambuf *mStdout;
  stringstream mOutputStream;
  char static_txt[1025];

  float cc[7];

  int nMode;
  int curr_traj_index;
  int nCurrQuality;
  bool bPrep;
  bool bGo;
  bool bFirst;
  int nControl;
  double mCurrJDamp, mCurrJStiff;
  double mCurrCDamp_pos, mCurrCStiff_pos, mCurrCDamp_or, mCurrCStiff_or;

  Vector tempJ_v; //NJOINTS-sized vector for intermediate data storage.
  Vector tempC_v; // vector for intermediate storage of cartesian wrench (6d)

  Vector LWR_state;


  float jnt2[LBR_MNJ];
  float cart2[FRI_CART_FRM_DIM];
  float cart_imp_params[FRI_CART_VEC];
  //yarp::sig::Vector currJoint;
  //yarp::sig::Vector cartVec;
  MathLib::Vector currJoint;
  MathLib::Vector currCart;

  SensorsList mJointSensors;
  ActuatorsList  mJointActuators;


  std::deque<LWRMeasurement> MeasurementHistory;


  /* std::vector<MathLib::Vector> recordedTrajectory; */
  /* std::vector<MathLib::Vector>  recordedTorques; */
  /* std::vector<MathLib::Vector>  recordedCartPos; */
  
 public:

  LWRCore();
     virtual ~LWRCore();

     virtual Status RobotInit();
     virtual Status RobotFree();
     virtual Status RobotStart();
     virtual Status RobotStop();
     virtual Status RobotUpdate();
     virtual Status RobotUpdateCore();

     virtual int RespondToConsoleCommand(const string cmd, const vector<string> &args);

     bool SetControlMode(const int desiredMode);
     void SensorsUpdate();
     void ControlUpdate();


     //these are the methods for user interaction through console. they are run in a separate thread.
     void ConsoleLoop();
     void ConsoleUpdate();
     void SetCommandedJPos(float*);

};






#endif /* LWRCORE_H_ */
