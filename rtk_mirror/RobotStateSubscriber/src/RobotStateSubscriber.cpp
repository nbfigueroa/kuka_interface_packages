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

#include "RobotStateSubscriber.h"


RobotStateSubscriber::RobotStateSubscriber()
:RobotInterface(){
}
RobotStateSubscriber::~RobotStateSubscriber(){
}

RobotInterface::Status RobotStateSubscriber::RobotInit(){

	AddConsoleCommand("control");
	AddConsoleCommand("filter");
	AddConsoleCommand("gain");

	GetConsole()->Print("Robot Init Subscriber");
	float new_sampling_time = 0.002;
	((LWRRobot*)(mRobot))->SetSamplingTime(new_sampling_time);
	dt = ((LWRRobot*)(mRobot))->GetSamplingTime();
	std::ostringstream ss;
	ss << "DT: " << dt;
	std::string msg(ss.str());
	GetConsole()->Print(msg);

	//--- Initialize variables and actuators/sensors ---//
	jointPositions.Resize(mRobot->GetDOFCount());
	jointMin.Resize(mRobot->GetDOFCount());
	jointMax.Resize(mRobot->GetDOFCount());
	actuators.SetActuatorsList(mRobot->GetActuators());
	sensors.SetSensorsList(mRobot->GetSensors());

	jointMax[0] = 160;jointMax[1] = 110;jointMax[2] = 160;jointMax[3] = 110;jointMax[4] = 160;jointMax[5] = 110;jointMax[6] = 160;
	jointMin[0] = -160;jointMin[1] = -110;jointMin[2] = -160;jointMin[3] = -110;jointMin[4] = -160;jointMin[5] = -110;jointMin[6] = -160;

	//--- Initializing type of interface ---//
	pXmlTree options = GetOptionTree();
	string interf;
	if(options) {
		interf = options->CGet("Options.CtrlInterface", string("position"));
	} else {
		interf = "position";
	}

	if(interf == "velocity") {
		init_mode = INTERFACE_VELOCITY;
		cout<<"Using velocity control";
	} else {
		init_mode = INTERFACE_POSITION;
		cout<<"Using position/impedance control";
	}

	//--- Initializing ROS Node and Subscriber Topics ---//
	string nodename = mRobot->GetName();
	nodename += "_MIRROR";

	nh = mRobot->InitializeROS(nodename);

	string topicName = mRobot->GetName();
	topicName += "/joint_cmd";
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

	jointPositions.Resize(ndof);
	jointVelocities.Resize(ndof);
	prevJointVelocities.Resize(ndof);


	jointStateSubscriber = nh->subscribe(topicName,1,&RobotStateSubscriber::jointStateCallback,this);



	nEndEffectorId = mRobot->GetLinkIndex("TOOL");
		cout << "The end effector id is " << nEndEffectorId << endl;
		if (nEndEffectorId == -1)
			GetConsole()->Print("ERROR: End effector not found");

	//--- New Subscribers to have Cartesian Impedance control ---//f
	topicName = mRobot->GetName();
	topicName += "/des_ee_pose";
	cartStateSubscriber = nh->subscribe(topicName,1,&RobotStateSubscriber::cartStateCallback,this);

	topicName = mRobot->GetName();
	topicName += "/des_ee_ft";
	des_ee_ft(6);
	ftStateSubscriber = nh->subscribe(topicName,1,&RobotStateSubscriber::ftStateCallback,this);

	topicName = mRobot->GetName();
	topicName += "/des_ee_stiff";
	des_ee_stiff(6);
	stiffStateSubscriber = nh->subscribe(topicName,1,&RobotStateSubscriber::stiffStateCallback,this);


	mService = nh->advertiseService("StringService",&RobotStateSubscriber::serviceCallback,this);


	bFilter = false;
	bControl=0;
	bGrav = false;

	sub_pose = false;
	sub_stiff = false;
	sub_ft = false;

	Vector default_stiff(7), default_damp(7);
	Vector default_cart_stiff(6);

	// Getting current Cartesian states
	des_ee_pos.Zero(); des_ee_orient.Identity();
	des_ee_stiff.Resize(6); des_ee_stiff.Zero();
	des_ee_ft.Resize(6); des_ee_ft.Zero();
	((LWRRobot*)(mRobot))->GetMeasuredCartPose(des_ee_pos, des_ee_orient);
	des_ee_stiff = ((LWRRobot*)(mRobot))->GetCartStiffness();
	des_ee_ft = ((LWRRobot*)(mRobot))->GetEstimatedExternalCartForces();


	// ===================================================
	// ========  Initialize CDDynamics         ===========
	// ===================================================
    module_dt = dt;
    fri_dt = dt;
	genCart = new CDDynamics(6, dt, 1); // In cartesian space x, y, z, wx, wy, wz
	Vector vel_lim_cart(6);
	vel_lim_cart = DEG2RAD(80);
	genCart->SetVelocityLimits(vel_lim_cart);
	genCart->SetWn(3);

	vTarget.Resize(6);
	vTarget.Zero();
	currentTarget.Resize(6);
	currentTarget.Zero();

	eeFT.Resize(6);
	eeFT.Zero();
	eeStiff.Resize(6);
	eeStiff.Zero();

	forces_old.Resize(6);
	forces_old.Zero();


	force = 0;
	force_old = 0;
	force_new = 0;
	total_time = 0;
	spline_duration = 5.0;

	GetConsole()->Print("RobotInit()");
	return STATUS_OK;
}
RobotInterface::Status RobotStateSubscriber::RobotFree(){
	return STATUS_OK;
}
RobotInterface::Status RobotStateSubscriber::RobotStart(){

	if(mRobot->IsSimulationMode())
		((LWRRobot*)mRobot)->SetControlMode(Robot::CTRLMODE_POSITION);
	else
	{
		((LWRRobot*)mRobot)->SetControlMode(Robot::CTRLMODE_JOINTIMPEDANCE);
		default_damp = ((LWRRobot*)mRobot)->GetJointDamping();
		default_stiff = ((LWRRobot*)mRobot)->GetJointStiffness();
		GetConsole()->Print("Starting Robot in JointImpedance ()");
	}

	// Get initial joint positions
	sensors.ReadSensors();
	cmd_positions = sensors.GetJointPositions();
	cmd_velocities.Resize(cmd_positions.Size());
	cmd_velocities.Zero();
	for(int i=0;i<ndof;i++) {
		jointPositions[i] = cmd_positions[joint_map[i]];
	}
	jointVelocities.Zero();
	prevJointVelocities.Zero();

	// Get/Set Sampling Time
	dt = ((LWRRobot*)(mRobot))->GetSamplingTime();
	cout<<"DT: "<<dt<<endl;


	// Creating filter for joint positions
	if(mRobot->IsSimulationMode())
		filter = new CDDynamics(ndof, dt, WN);
	else
		filter = new CDDynamics(ndof, dt, WN);

	// Applying filter
	filter->SetState(jointPositions);
	filter->SetTarget(jointPositions);
	filtered_joints.Resize(ndof);

	// Sending new positions to robot
	actuators.SetJointPositions(cmd_positions);
	actuators.WriteActuators();

	// Getting current robot states
	des_ee_pos.Zero(); des_ee_orient.Identity();
	des_ee_stiff.Resize(6); des_ee_stiff.Zero();
	des_ee_ft.Resize(6); des_ee_ft.Zero();
	((LWRRobot*)(mRobot))->GetMeasuredCartPose(des_ee_pos, des_ee_orient);
	des_ee_stiff = ((LWRRobot*)(mRobot))->GetCartStiffness();
	des_ee_ft = ((LWRRobot*)(mRobot))->GetEstimatedExternalCartForces();

	bSync = true;
	return STATUS_OK;
}    
RobotInterface::Status RobotStateSubscriber::RobotStop(){
	return STATUS_OK;
}
RobotInterface::Status RobotStateSubscriber::RobotUpdate(){

	return STATUS_OK;
}
RobotInterface::Status RobotStateSubscriber::RobotUpdateCore(){

  // Removing spin from here since the robot object is spinning ros now.
  //ros::spinOnce();

		if(bSync)
		{
				sensors.ReadSensors();
				Vector v = sensors.GetJointPositions();
				for(int i=0;i<ndof;i++)
					jointPositions[i] = v[joint_map[i]];
				jointVelocities.Zero();
				prevJointVelocities.Zero();
				actuators.SetJointPositions(v);
				actuators.WriteActuators();
				if(bFilter)
					filter->SetState(jointPositions);

				GetConsole()->Print("Joints Synced");

			if (bControl == 2){
				((LWRRobot*)(mRobot))->SetControlMode(Robot::CTRLMODE_CARTIMPEDANCE);

				// Synchronize Cartesian Position
				Vector    x_state;
				x_state.Resize(6, false);
				Vector3 tmppos;Matrix3 tmporient;
				((LWRRobot*)mRobot)->GetMeasuredCartPose(tmppos, tmporient);
				x_state(0) = tmppos(0);
				x_state(1) = tmppos(1);
				x_state(2) = tmppos(2);
				x_state.SetSubVector(3, tmporient.GetExactRotationAxis());

				genCart->SetState(x_state);
				vTarget.Set(currentTarget);
				genCart->SetTarget(vTarget);
				genCart->Update(((LWRRobot*)mRobot)->GetSamplingTime());
				genCart->GetState(vTarget);
				des_ee_pos.Set(Vector3(vTarget[0], vTarget[1], vTarget[2]));
				des_ee_orient = Matrix3::SRotationV(Vector3(vTarget[3], vTarget[4], vTarget[5]));
				((LWRRobot*)(mRobot))->SetCartCommand(des_ee_pos, des_ee_orient);

				eePos = des_ee_pos;
				eeOrient = des_ee_orient;

				// Synchronize Cartesian Stiffness
				des_ee_stiff.Resize(6); des_ee_stiff.Zero();
				des_ee_stiff = ((LWRRobot*)mRobot)->GetCartStiffness();
				Vector cartStiffness;
				cartStiffness.Resize(6);
				cartStiffness.SetSubVector(0, Vector3(300, 300, 300));
				cartStiffness.SetSubVector(3, Vector3(200, 200, 200));
				((LWRRobot*)(mRobot))->SetCartStiffness(cartStiffness);

				total_time = 0.0;
				eeStiff = cartStiffness;

				// Synchronize Cartesian Forces/Torques
				des_ee_ft.Resize(6); des_ee_ft.Zero();
				des_ee_ft = ((LWRRobot*)(mRobot))->GetEstimatedExternalCartForces();
				((LWRRobot*)(mRobot))->SetEstimatedExternalCartForces(des_ee_ft);

				eeFT = des_ee_ft;

				GetConsole()->Print("Sent Sync cart Command");
				std::ostringstream ss;
				ss << des_ee_pos[0] << " " << des_ee_pos[1] << " " <<  des_ee_pos[2];
				std::string msg(ss.str());
				GetConsole()->Print(msg);
				GetConsole()->Print("RobotUpdateCore Cart () w/ bSync");
			}
			bSync = false;
			return STATUS_OK;
		}
		if(init_mode == INTERFACE_VELOCITY && bControl==1) {
			for(int i=0; i<jointPositions.Size(); ++i) {
				if((jointPositions[i] >= jointMax[i] && jointVelocities[i] >0) ||
										(jointPositions[i] <= jointMin[i] && jointVelocities[i] < 0))  {

				} else {
					jointPositions[i] += jointVelocities[i]*((LWRRobot*)mRobot)->GetSamplingTime();
				}
			}
			GetConsole()->Print("RobotUpdateCore () inside Interface Velocity");
		}

		///----Filter----///
		if(bFilter && bControl==1)
		{
			//			jointPositions.Print("JP");
			filter->SetTarget(jointPositions);
			filter->Update(((LWRRobot*)mRobot)->GetSamplingTime());
			filter->GetState(filtered_joints);
			//			filtered_joints.Print("F");
			GetConsole()->Print("RobotUpdateCore () inside bFilter");
		}
		else
		{
			if (bControl==1){
				for(int i=0;i<ndof;i++)
								filtered_joints[i] = jointPositions[i];
							GetConsole()->Print("RobotUpdateCore () not bFilter");
			}
		}

		//----Depending on the type of control (bControl=1 is Joint Impedance Control / bControl=2 is Cartesian Impedance Control) do the following:
		if(bControl==1) {
			for(int i=0;i<ndof;i++)
				cmd_positions[joint_map[i]] = filtered_joints[i];
			actuators.SetJointPositions(cmd_positions);
			actuators.WriteActuators();

			GetConsole()->Print("RobotUpdateCore () sending joint commands");
		}
		if(bControl==2){
			((LWRRobot*)(mRobot))->SetControlMode(Robot::CTRLMODE_CARTIMPEDANCE);
			// IF 0 Stiffness on all axis it means we want GRAVCOMP so only set stiffness and update current Pose
			// Otherwise send the desired commands accordingly
			double trans_stiff = eeStiff(0) + eeStiff(1) + eeStiff(2);
			double rot_stiff = eeStiff(3) + eeStiff(4) + eeStiff(5);

			if (trans_stiff + rot_stiff == 0){
				//--- Synchronize Cartesian Stiffness ---//
				des_ee_stiff = eeStiff;
				((LWRRobot*)(mRobot))->SetCartStiffness(des_ee_stiff);

				//--- Synchronize Cartesian Position ---//
				((LWRRobot*)mRobot)->GetMeasuredCartPose(des_ee_pos, des_ee_orient);
				((LWRRobot*)(mRobot))->SetCartCommand(des_ee_pos, des_ee_orient);
			}
			else
			{
				//--- Synchronize Cartesian Forces/Torques ---//
				des_ee_ft = eeFT;
				((LWRRobot*)(mRobot))->SetCartForce(des_ee_ft);

				//--- Synchronize Cartesian Position ---//
				Vector3 currpos;Matrix3 currorient;
				((LWRRobot*)mRobot)->GetMeasuredCartPose(currpos, currorient);
				des_ee_pos = eePos;
				des_ee_orient = eeOrient;
				std::ostringstream sss;
				sss <<"Desired: " <<eePos[0] << " " << eePos[1] << " " <<  eePos[2];
				std::string msg(sss.str());
				GetConsole()->Print(msg);

				// My own interpolator
				slerp_interpolator(currpos,currorient,des_ee_pos,des_ee_orient);
				((LWRRobot*)(mRobot))->SetCartCommand(des_ee_pos, des_ee_orient);

				//--- Synchronize Cartesian Stiffness ---//
				des_ee_stiff = eeStiff;
				((LWRRobot*)(mRobot))->SetCartStiffness(des_ee_stiff);
			}


			std::ostringstream ss;
			ss <<"Position: " <<des_ee_pos[0] << " " << des_ee_pos[1] << " " <<  des_ee_pos[2] << " Stiffness: "<< des_ee_stiff[0] << " " << des_ee_stiff[1] << " " <<  des_ee_stiff[2] << " Force: "<< des_ee_ft[0] << " " << des_ee_ft[1] << " " <<  des_ee_ft[2];
			std::string msg2(ss.str());
			GetConsole()->Print(msg2);
		}


	return STATUS_OK;
}

double RobotStateSubscriber::getIntermediatePoint(double curX, double curXd, double targetX, double targetXd, double duration, double dt){
    // powers of duration
    double duration2 =  duration * duration;
    double duration3 = duration2 * duration;
    // powers of dt
    double dt2 =  dt * dt;
    double dt3 = dt2 * dt;
    // spline coefficients
    double c0 = curX;
    double c1 = curXd;
    double c2 = -(3*curX-3*targetX+2*duration*curXd+duration*targetXd)/duration2;
    double c3 = (2*curX-2*targetX+duration*(curXd+targetXd))/duration3;
    double intermediateX = c0 + c1*dt + c2*dt2 + c3*dt3;
    return intermediateX;
}

void RobotStateSubscriber::slerp_interpolator(Vector3 currPos, Matrix3 currOrient, Vector3& targetPos, Matrix3& targetOrient){

	Vector3 tmpPos; Matrix3 tmpOrient;

	// SLERP INTERPOLATOR for Position
	// General formula: slerp(p0,p1,t) = (1-t)*p0 + p1

    // Adapt interpolation parameter
	double inter_param_p = 0.5;
	double reach_thres (0.05), pos_param (inter_param_p);
	double pos_err = sqrt(pow((targetPos[0]-currPos[0]),2) + pow((targetPos[1]-currPos[1]),2) + pow((targetPos[2]-currPos[2]),2));
	if (pos_err < reach_thres)
		pos_param = inter_param_p + (reach_thres - pos_err)*(1-inter_param_p)/reach_thres;

	for (int i=0;i<3;i++)
		tmpPos[i] = (1-pos_param)*currPos[i] + pos_param*targetPos[i];

	// SLERP INTERPOLATOR for Quaternions
	// General formula: slerp(q0,q1,t) = (q1q0^-1)^t + q0
	Vector3 v1 = currOrient.GetRotationAxis();
	tf::Vector3 ax1(v1(0), v1(1), v1(2));
	tf::Quaternion q_curr(tf::Quaternion(ax1, currOrient.GetRotationAngle()));

    tmpOrient = targetOrient;
    Vector3 v2 = tmpOrient.GetRotationAxis();
    tf::Vector3 ax2(v2(0), v2(1), v2(2));
    tf::Quaternion q_target(tf::Quaternion(ax2, tmpOrient.GetRotationAngle()));

    // Adapt interpolation parameter
    double inter_param_q = 0.15;
	double ori_thres  (0.25), ori_param (inter_param_q);
	double ori_err = acos(abs(q_target.dot(q_curr)));
	if (ori_err < ori_thres)
		ori_param = inter_param_q + (ori_thres - ori_err)*(1-inter_param_q)/ori_thres;

    tf::Quaternion q_tmp = q_curr.slerp(q_target,ori_param);
    tf::Matrix3x3 mat33(q_tmp);

    tmpOrient = MathLib::Matrix3(mat33[0][0], mat33[0][1], mat33[0][2],
								 mat33[1][0], mat33[1][1], mat33[1][2],
								 mat33[2][0], mat33[2][1], mat33[2][2]);

	std::ostringstream ss;
	ss << "Position Error: " << pos_err << " Inter. Param: " << pos_param << " Orientation Error: " <<  ori_err << " Inter. Param: " << ori_param;
	std::string msg(ss.str());
	GetConsole()->Print(msg);

    // Set new target
    targetPos = tmpPos;
    targetOrient = tmpOrient;
}

int RobotStateSubscriber::RespondToConsoleCommand(const string cmd, const vector<string> &args){
	if(cmd == "control")
	{
		bControl  = atoi(args[0].c_str());
		if (bControl==0)
		{
			bControl = 0;
			bSync = true;
			mRobot->SetControlMode(Robot::CTRLMODE_GRAVITYCOMPENSATION);
			GetConsole()->Print("Control off");
		}
		else if (bControl==1)
		{
			bControl = 1;
			bSync = true;
			mRobot->SetControlMode(Robot::CTRLMODE_JOINTIMPEDANCE);
			GetConsole()->Print("Joint Impedance Control on");
		}
		else if (bControl==2)
		{
			bControl = 2;
			bSync = true;
			mRobot->SetControlMode(Robot::CTRLMODE_CARTIMPEDANCE);
			GetConsole()->Print("Cartesian Impedance Control on");
		}

	}
	if (cmd == "filter")
	{
		if(bFilter)
		{
			bFilter = false;
			bSync = true;
			GetConsole()->Print("Filter off");
		}
		else
		{
			bFilter = true;
			bSync = true;
			GetConsole()->Print("Filter on");
		}
	}
	if (cmd == "gain")
	{
		if(!mRobot->IsSimulationMode())
		{
			if(args.size()!=1)
			{
				GetConsole()->Print("Missing gain multiplier argument");
				return 1;
			}
			float tmp = atof(args[0].c_str());
			if(tmp <0 || tmp >1.0)
			{
				GetConsole()->Print("Gain must be between 0 and 1");
				return 1;
			}
			Vector v = default_damp*tmp;
			((LWRRobot*)mRobot)->SetJointDamping(v);
			v = default_stiff*tmp;
			((LWRRobot*)mRobot)->SetJointStiffness(v);
		}
		else
			return 1;
	}

	return 0;
}

void RobotStateSubscriber::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
	if(init_mode == INTERFACE_VELOCITY) {
		if(msg->velocity.size() != ndof) {
			cout<<"Size mismatch in velocity!"<<endl;
			return;
		}

		double d = ((LWRRobot*)mRobot)->GetSamplingTime();
		for(int i=0;i<ndof;i++){
			jointVelocities(i) = msg->velocity[i];
		}

	} else {
		if (bControl==1){
			if(msg->position.size() != ndof) {
				cout<<"Size mismatch in position!"<<endl;
				return;
			}
			for(int i=0;i<ndof;i++){
				jointPositions(i) = msg->position[i];
			}

		}
	}

}


// Callback for the desired cartesian pose
void RobotStateSubscriber::cartStateCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
	const geometry_msgs::PoseStamped* data = msg.get();
	//Setting desired position
	eePos.Set(Vector3(data->pose.position.x, data->pose.position.y, data->pose.position.z));

	//Setting desired orientation
	tf::Pose p;
	p.setOrigin(tf::Vector3(data->pose.position.x, data->pose.position.y, data->pose.position.z));
	p.setRotation(tf::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w));

	tf::Matrix3x3 mat33 = p.getBasis();

	MathLib::Matrix3 mat(mat33[0][0], mat33[0][1], mat33[0][2],
						 mat33[1][0], mat33[1][1], mat33[1][2],
						 mat33[2][0], mat33[2][1], mat33[2][2]);
	eeOrient = mat;
}


// Callback for the desired end effector force/torque
void RobotStateSubscriber::ftStateCallback(const geometry_msgs::WrenchStampedConstPtr& msg) {
	const geometry_msgs::WrenchStamped* data = msg.get();
	eeFT[0] = data->wrench.force.x;
	eeFT[1] = data->wrench.force.y;
	eeFT[2] = data->wrench.force.z;

	eeFT[3] = data->wrench.torque.x;
	eeFT[4] = data->wrench.torque.y;
	eeFT[5] = data->wrench.torque.z;

	//--- Interpolate Forces ---//
//	double curr_force = vCrtEEForce(2);
//	Vector vCrtEEForce; vCrtEEForce.Resize(6); vCrtEEForce.Zero();
//	vCrtEEForce = ((LWRRobot*)(mRobot))->GetEstimatedExternalCartForces();
//	vCrtEEForce.Zero();

	total_time += module_dt;

	for (int i=0; i<6;i++){
		force_new = eeFT(i);
		if(total_time <= spline_duration) {
		    force = getIntermediatePoint(forces_old(i), 0.0, force_new, 0.0, spline_duration, total_time);
		} else {
		    force = force_new;
		    forces_old[i] = force;
		}
		eeFT[i] = force;
	}

}


// Callback for the desired cartesian stiffness.
void RobotStateSubscriber::stiffStateCallback(const geometry_msgs::TwistStampedConstPtr& msg) {
	const geometry_msgs::TwistStamped* data = msg.get();
	eeStiff[0] = data->twist.linear.x;
	eeStiff[1] = data->twist.linear.y;
	eeStiff[2] = data->twist.linear.z;

	eeStiff[3] = data->twist.angular.x;
	eeStiff[4] = data->twist.angular.y;
	eeStiff[5] = data->twist.angular.z;
}


bool RobotStateSubscriber::serviceCallback(rtk_mirror::StringService::Request& request,rtk_mirror::StringService::Response& response){
	std::string command = request.str;
	if(command == "grav"){
		if(bGrav)
		{
			bGrav = false;
			sensors.ReadSensors();
			cmd_positions = sensors.GetJointPositions();
			for(int i=0;i<ndof;i++)
				jointPositions[i] = cmd_positions[joint_map[i]];

			filter->SetState(jointPositions);
			filter->SetTarget(jointPositions);
			filtered_joints.Resize(ndof);
			actuators.SetJointPositions(cmd_positions);
			actuators.WriteActuators();
			if(mRobot->IsSimulationMode())
				((LWRRobot*)mRobot)->SetControlMode(Robot::CTRLMODE_POSITION);
			else
				((LWRRobot*)mRobot)->SetControlMode(Robot::CTRLMODE_JOINTIMPEDANCE);

			response.str =  "Grav. comp. off";
			bSync = true;

		}
		else
		{
			bGrav = true;
			((LWRRobot*)mRobot)->SetControlMode(Robot::CTRLMODE_GRAVITYCOMPENSATION);
			response.str =  "Grav. comp. on";
		}

	}
	else if(command == "control")
	{
		std::vector<std::string> tmp;
		RespondToConsoleCommand("control", tmp);

		if(bControl==0)
			response.str = "Control off";
		else if(bControl==1)
			response.str = " Joint Control off";
		else if(bControl==2)
			response.str = " Cart Control off";

	}
	else if(command == "filter")
	{
		std::vector<std::string> tmp;
		RespondToConsoleCommand("filter", tmp);
		if(bFilter)
		{
			response.str = "Filter on";
			filter->SetState(jointPositions);
			filter->SetTarget(jointPositions);
		}
		else
			response.str = "Filter off";
	}
	else if (command == "gain")
	{
		std::vector<std::string> tmp;
		if(RespondToConsoleCommand("gain", tmp) != 0)
			response.str = "Gain command failed! 0.0<gain<1.0";
		else
			response.str = "Gain set successfully";
	}
	else{
		response.str =  command + " not recoginsed. Available --> filter, control, gravcomp";
	}
	return true;
}

extern "C"{
// These two "C" functions manage the creation and destruction of the class
RobotStateSubscriber* create(){return new RobotStateSubscriber();}
void destroy(RobotStateSubscriber* module){delete module;}
}

