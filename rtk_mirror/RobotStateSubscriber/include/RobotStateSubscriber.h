/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef RobotStateSubscriber_H_
#define RobotStateSubscriber_H_

#include "RobotLib/RobotInterface.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "rtk_mirror/StringService.h"
#include "KUKARobotModel/LWRRobot.h"
#include "CDDynamics.h"
#include "tf/LinearMath/Quaternion.h"
#include <tf/tf.h>


// Translational Stiffness values
#define TRANS_STIFF_X	1200
#define TRANS_STIFF_Y	1200
#define TRANS_STIFF_Z	1200

// Rotational Stiffness values
#define ROT_STIFF_X	300
#define ROT_STIFF_Y	300
#define ROT_STIFF_Z	300

#define INTERFACE_POSITION 0
#define INTERFACE_VELOCITY 1
#define WN 1.0
class RobotStateSubscriber : public RobotInterface
{

    ros::NodeHandle * nh;
    ros::Subscriber jointStateSubscriber;
    ros::Subscriber cartStateSubscriber;
    ros::Subscriber ftStateSubscriber;
    ros::Subscriber stiffStateSubscriber;
    ros::Subscriber chatterSub;

    RevoluteJointActuatorGroup actuators;
    RevoluteJointSensorGroup sensors;
    Vector jointPositions, jointVelocities, prevJointVelocities, jointMax, jointMin;
    Vector cmd_positions, cmd_velocities;
    Vector  des_ee_ft, des_ee_stiff;
    Vector3 des_ee_pos, prev_ee_pos;
    Matrix3 des_ee_orient, prev_ee_orient;
    Vector filtered_joints;
    unsigned int ndof;
    IndicesVector joint_map;
    ros::ServiceServer mService;
//    bool bControl, bFilter, bGrav, bSync;
    bool bFilter, bGrav, bSync;
    int bControl;
    bool sub_pose, sub_stiff, sub_ft;
    CDDynamics *filter;

    CDDynamics *genCart; 		// moving in cartesian space
    Vector currentTarget;
    Vector vTarget;
    Vector forces_old;

    Vector3 eePos;
    Matrix3 eeOrient;
    Vector eeFT, eeStiff;

    double force;
    double force_old;
    double force_new;
    double total_time;
    double spline_duration;


    Vector default_damp, default_stiff;
    int init_mode, nEndEffectorId;
    double dt, module_dt, fri_dt;
public:
    RobotStateSubscriber();
    virtual ~RobotStateSubscriber();

    virtual Status              RobotInit();
    virtual Status              RobotFree();

    virtual Status              RobotStart();
    virtual Status              RobotStop();

    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr & msg);

    void cartStateCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    void ftStateCallback(const geometry_msgs::WrenchStampedConstPtr& msg);
	
    void chatterCallback(const std_msgs::String::ConstPtr& msg);

    void stiffStateCallback(const geometry_msgs::TwistStampedConstPtr& msg);

    bool serviceCallback(rtk_mirror::StringService::Request& reques,rtk_mirror::StringService::Response& response);

    void slerp_interpolator(Vector3 currPos, Matrix3 currOrient, Vector3& targetPos, Matrix3& targetOrient);

    double getIntermediatePoint(double curX, double curXd, double targetX, double targetXd, double duration, double dt);

};



#endif 
