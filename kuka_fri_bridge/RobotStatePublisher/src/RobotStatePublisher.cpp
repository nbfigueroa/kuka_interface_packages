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

#include "RobotStatePublisher.h"
#include <boost/lexical_cast.hpp>

RobotStatePublisher::RobotStatePublisher()
    :RobotInterface() , mLWRRobot(0), ndof(0), nh(0){
}
RobotStatePublisher::~RobotStatePublisher(){
}

RobotInterface::Status RobotStatePublisher::RobotInit(){

    mLWRRobot = (LWRRobot*)mRobot;

    ///--- Set Node Name --- ///
    string nodename = mRobot->GetName();
    nodename += "_BRIDGE";
    nh = mRobot->InitializeROS(nodename);
	GetConsole()->Print("Robot Init Publisher");

    ///--- Setting Sampling Time --- ///
    float new_sampling_time = 0.002;
	mLWRRobot->SetSamplingTime(new_sampling_time);
	dt = mLWRRobot->GetSamplingTime();
	std::ostringstream ss;
	ss << "DT: " << dt;
	std::string msg(ss.str());
	GetConsole()->Print(msg);

    string topicName;
    ///- Publish Joint State Message -///
    topicName = mRobot->GetName();
    topicName += "/joint_states";
    jointStatePublisher = nh->advertise<sensor_msgs::JointState>(topicName,3);

    ///- Publish Joint State Impedance Message -///
    topicName = mRobot->GetName();
    topicName += "/joint_imp_states";
    jointStateImpedancePublisher = nh->advertise<kuka_fri_bridge::JointStateImpedance>(topicName,3);

    ///- Publish Cartesian Pose Message -///
    topicName = mRobot->GetName();
    topicName += "/Pose";
    posePublisher = nh->advertise<geometry_msgs::PoseStamped>(topicName,3);

    ///- Publish Cartesian FT Message -///
    topicName = mRobot->GetName();
    topicName += "/FT";
    ftPublisher = nh->advertise<geometry_msgs::WrenchStamped>(topicName,3);

    ///- Publish CartesianStiffness Message -///
    topicName = mRobot->GetName();
    topicName += "/Stiff";
    stiffPublisher = nh->advertise<geometry_msgs::TwistStamped>(topicName,3);



    mSensorsGroup.SetSensorsList(mRobot->GetSensors());
    int linkid = mRobot->GetLinkIndex("TOOL");


    mKinematicChain.SetRobot(mRobot);
    mKinematicChain.Create(0,0,linkid);
    mKinematicChain.BuildJacobian();

    mJacobian.Resize(6,mRobot->GetDOFCount());
    currVel.Resize(6);
    currVel_filtered.Resize(6);
    jointVel.Resize(mRobot->GetDOFCount());

    velocityFilter.Init(6,4);
    velocityFilter.SetSamplingPeriod(0.002);
    velocityFilter.SetButterworth(20.0);

    eeFT.Resize(6);
    eeStiff.Resize(6);

    ndof = 0;
    while(true)
    {
    	if(mRobot->GetDOFIndex("DOF_" + boost::lexical_cast<string>(ndof++)) < 0)
    		break;
    }

    if(ndof == 1)
    {
    	cout<<"No DOF found!"<<endl;
    	exit(1);
    }
    ndof--;
    joint_map.resize(ndof);

    for(int i=0;i<ndof;i++)
    	joint_map[i] = mRobot->GetDOFIndex("DOF_" + boost::lexical_cast<string>(i));

    ///--- Initializing Joint State Message ---///
    jointStateMsg.position.resize(ndof);
    jointStateMsg.velocity.resize(ndof);
    jointStateMsg.effort.resize(ndof);
    jointStateMsg.name.resize(ndof);

    ///--- Initializing Joint State Impedance Message ---///
    jointStateImpedanceMsg.position.resize(ndof);
    jointStateImpedanceMsg.velocity.resize(ndof);
    jointStateImpedanceMsg.effort.resize(ndof);
    jointStateImpedanceMsg.stiffness.resize(ndof);
    jointStateImpedanceMsg.name.resize(ndof);

    ///--- Get Robot Type (left/right) ---//
    char buf[255];
    pXmlTree options = GetOptionTree();
    string which_arm;
    if(options) {
        which_arm = options->FindData("Options.Arm");
    } else {
        which_arm = "right";
    }

    for(int i=0; i<ndof; ++i) {
    	sprintf(buf, "%s_arm_%d_joint",which_arm.c_str(), i);
    	jointStateMsg.name[i] = buf;
        jointStateImpedanceMsg.name[i] = buf;
    }



    topicName = mRobot->GetName();
    topicName += "/CoreRate";

    return STATUS_OK;
}
RobotInterface::Status RobotStatePublisher::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status RobotStatePublisher::RobotStart(){
    return STATUS_OK;
}    
RobotInterface::Status RobotStatePublisher::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status RobotStatePublisher::RobotUpdate(){

    return STATUS_OK;
}
RobotInterface::Status RobotStatePublisher::RobotUpdateCore(){
    //ratePublisher->publish(emptyMsg);
    // not safe for realtime
    mSensorsGroup.ReadSensors();
    mKinematicChain.Update();
    mJacobian = mKinematicChain.GetJacobian();
    jointVel = mSensorsGroup.GetJointVelocities();
    mJacobian.Mult(jointVel,currVel);
    velocityFilter.SetInput(currVel);
    velocityFilter.Update();
    velocityFilter.GetOutput(currVel_filtered);


    currEEPos  = mRobot->GetReferenceFrame(mRobot->GetLinksCount()-1,0).GetOrigin();
    rot_MathLib = mRobot->GetReferenceFrame(mRobot->GetLinksCount()-1,0).GetOrient();

    eeFT = ((LWRRobot*)mRobot)->GetEstimatedExternalCartForces();
    eeStiff = ((LWRRobot*)mRobot)->GetCartStiffness();


    for(int ii =0;ii<3;ii++){
         for( int jj = 0;jj<3;jj++){
             rot_Eigen(ii,jj) = rot_MathLib(ii,jj);
         }
     }

     Eigen::Quaternion<double> rot_quat(rot_Eigen);

     Vector curr_stiff; curr_stiff.Resize(ndof);
     curr_stiff = ((LWRRobot*)mRobot)->GetJointStiffness();

     for(int i=0;i<ndof;i++){
         ///--- Setting Positions ---//
         jointStateMsg.position[i] = mSensorsGroup.GetJointAngles()(joint_map[i]);
         jointStateImpedanceMsg.position[i] = jointStateMsg.position[i];

         ///--- Setting Velocities ---//
         jointStateMsg.velocity[i] = mSensorsGroup.GetJointVelocities()(joint_map[i]);
         jointStateImpedanceMsg.velocity[i] = jointStateMsg.velocity[i];

         ///--- Setting Efforts ---//
         jointStateMsg.effort[i] = mSensorsGroup.GetJointTorques()(joint_map[i]);
         jointStateImpedanceMsg.effort[i] = jointStateMsg.effort[i];

         ///--- Setting Stiffness ---//
         jointStateImpedanceMsg.stiffness[i] = curr_stiff[i];
     }

     jointStateMsg.header.stamp = ros::Time::now();

     poseStampedMsg.pose.position.x = currEEPos(0);
     poseStampedMsg.pose.position.y = currEEPos(1);
     poseStampedMsg.pose.position.z = currEEPos(2);
     poseStampedMsg.pose.orientation.w = rot_quat.w();
     poseStampedMsg.pose.orientation.x = rot_quat.x();
     poseStampedMsg.pose.orientation.y = rot_quat.y();
     poseStampedMsg.pose.orientation.z = rot_quat.z();
     poseStampedMsg.header.stamp = ros::Time::now();


     ///-- Publish Joint State Msg --///
     jointStatePublisher.publish(jointStateMsg);

     ///-- Publish Joint State Impedance Msg --///
     jointStateImpedancePublisher.publish(jointStateImpedanceMsg);

     ///-- Publish CartPose Msg --///
     posePublisher.publish(poseStampedMsg);

     ///-- Publish Cart FT Msg --///
     ftMsg.header.stamp = ros::Time::now();
     ftMsg.wrench.force.x = eeFT[0];
     ftMsg.wrench.force.y = eeFT[1];
     ftMsg.wrench.force.z = eeFT[2];
     ftMsg.wrench.torque.x = eeFT[3];
     ftMsg.wrench.torque.y = eeFT[4];
     ftMsg.wrench.torque.z = eeFT[5];
     ftPublisher.publish(ftMsg);

     ///-- Publish Cart Stiff Msg --///
     stiffMsg.header.stamp = ros::Time::now();
     stiffMsg.twist.linear.x = eeStiff[0];
     stiffMsg.twist.linear.y = eeStiff[1];
     stiffMsg.twist.linear.z = eeStiff[2];
     stiffMsg.twist.angular.x = eeStiff[3];
     stiffMsg.twist.angular.y = eeStiff[4];
     stiffMsg.twist.angular.z = eeStiff[5];
     stiffPublisher.publish(stiffMsg);


    return STATUS_OK;
}
int RobotStatePublisher::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    return 0;
}



extern "C"{
// These two "C" functions manage the creation and destruction of the class
RobotStatePublisher* create(){return new RobotStatePublisher();}
void destroy(RobotStatePublisher* module){delete module;}
}

