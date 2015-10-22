#include "LWRCore.h"


using namespace std;


LWRCore::LWRCore(): RobotInterface(true)
{
	//get the path to the LWRInterface package
	//  std:string driverPath = ros::package::getPath("LWRInterface");
	//append the known relative path to the driver file
	//  driverPath+="/data/Misc/980039-FRI-Driver.init";

	/*Note, the above is the more general way to find the path, but ros/package library does not work when the program is run as sudo, therefor we use this ugly hard coded path..this will only work if the interface is started from the RobotToolKit base folder*/

	std::string driverPath = "./data/packages/lwr_interface/Misc/980039-FRI-Driver.init";


	mFRI = NULL;
	mFRI = new FastResearchInterface(driverPath.c_str());
	if(!mFRI) {
	  cout<<"Error: Cannot create FRI object"<<endl;
	  throw;
	}
	mFRI->GetCommunicationTimingQuality();

}

LWRCore::~LWRCore()
{}

RobotInterface::Status LWRCore::RobotInit()
{


	AddConsoleCommand("quit");


	currJoint.Resize(LBR_MNJ);
	currCart.Resize(FRI_CART_FRM_DIM);
	tempJ_v.Resize(LBR_MNJ);
	tempC_v.Resize(FRI_CART_VEC);

	LWR_state.Resize(LWR_state_size);

	mCurrJDamp = FRI_JOINT_DAMPING;
	mCurrJStiff = FRI_JOINT_STIFFNESS;

	mCurrCDamp_pos = FRI_CART_DAMPING_POS;
	mCurrCDamp_or = FRI_CART_DAMPING_ORIENT;
	mCurrCStiff_pos = FRI_CART_STIFFNESS_POS;
	mCurrCStiff_or = FRI_CART_STIFFNESS_ORIENT;

	nMode  = MODE_NONE;


	nCurrQuality = mFRI->GetCommunicationTimingQuality();


	// if(!dThread->threadInit())
	// {
	// 	return STATUS_ERROR;
	// }
	//



//	mStdout = cout.rdbuf();
//	cout.rdbuf(mOutputStream.rdbuf());
//	mNCConsole.SetConsole(GetConsole());
//	mNCConsole.InitNCurses();
//	mNCConsole.SetTopStaticLinesCount(7);


	//setting up the sensorslist to point to the sensors of the robot object
	if(mRobot ==NULL){
		cout<<"Error, no robot specified. Use .SetRobot() method before .Init"<<endl;
	}
	else{

		mLWRRobot = (LWRRobot*)mRobot;

		int Nsens = mRobot->GetDOFCount();

		mJointSensors.resize(Nsens);
		mJointActuators.resize(Nsens);
		char ActName[256];
		for(int i=0; i < LBR_MNJ ; i++){
			//find the sensor and actuators. They must be named DOF_0 etc in the config file
			sprintf(ActName,"DOF_%d",i);
			mJointSensors[i] = mRobot->FindSensor(ActName);
			mJointActuators[i] = mRobot->FindActuator(ActName);
		}

//		char txt[256];
//		sprintf(txt,"the number of sensors is %d",Nsens);
//		mNCConsole.Print(txt);

	}

	SensorsUpdate();

	return STATUS_OK;
}

RobotInterface::Status LWRCore::RobotFree()
{
	// delete dThread;
	// delete consoleThread;
	return STATUS_OK;
}

RobotInterface::Status LWRCore::RobotStart()
{
	// if(!dThread->start())
	// {
	// 	return STATUS_ERROR;
	// }
	// if(!consoleThread->start())
	// {
	// 	return STATUS_ERROR;
	// }
	bRunConsoleLoop = true;

	mStdout = cout.rdbuf();
	cout.rdbuf(mOutputStream.rdbuf());
	mNCConsole.SetConsole(GetConsole());
	mNCConsole.InitNCurses();
	mNCConsole.SetTopStaticLinesCount(7);

	mDisplayThread = boost::thread(&LWRCore::ConsoleLoop,  this);

	float aa[7], bb[7];
	mFRI->GetCommandedJointPositions((float*)aa);
	mFRI->GetCommandedJointPositionOffsets((float*)bb);
	for(int i=0;i<7;i++)
	{
		cc[i] = aa[i]-bb[i];
		cout<<cc[i]<<endl;
	}
	// to prevet SetControlMode() from running when we only want to read from FRI
	bFirst = true;
	return STATUS_OK;
}
RobotInterface::Status LWRCore::RobotStop()
{
	bRunConsoleLoop = false;
	usleep(100000);
	mNCConsole.FreeNCurses();
	mDisplayThread.join();

	return STATUS_OK;
}
RobotInterface::Status LWRCore::RobotUpdate(){ 


	return STATUS_OK; }



RobotInterface::Status LWRCore::RobotUpdateCore(){
	//  clock_t startM,stopM;
	//update robot sensors
	//  startM = clock();

	if(bFirst) {
		nControl = Robot::CTRLMODE_NONE;
		mRobot->SetControlMode(Robot::CTRLMODE_NONE);
		bFirst = false;
	}
	mFRI->WaitForKRCTick();
	//change control mode if necessary
	if(mRobot->GetControlMode() != nControl){
		SetControlMode(mRobot->GetControlMode());
	}

	int currMode = 	mFRI->GetFRIMode();
	if(currMode != FRI_STATE_CMD)
		mLWRRobot->SetAlive(false);
	else
		mLWRRobot->SetAlive(true);

//	if(mFRI->DoesAnyDriveSignalAnError())
//	{
//		mFRI->StopRobot();
//		for(int i=0;i<LBR_MNJ;i++)
//			((RevoluteJointActuator*)mJointActuators[i])->mPosition = cc[i];
//	}


	SensorsUpdate();


	//////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////
	//  nControl = Robot::CTRLMODE_POSITION;
	/*NOTE: this line is only for debugging!!!! remember to remove!*/
	//////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////

	//
	if(mFRI->GetFRIMode()==FRI_STATE_CMD){
		//control the robot
		ControlUpdate();
	}
	// run Core at 500 hz is 2ms

	// int clockcs  = 0.002*CLOCKS_PER_SEC;
	//  usleep(clockcs); //sleep 2 ms
	//  stopM = clock();
	//  cout<<"the core update takes:  "<<stopM - startM<<endl;
	// char txt[1024];
	// sprintf(txt, "%d cycles used by the processor.", stopM-startM);
	// mNCConsole.Print(txt);



	return STATUS_OK;}

int cnt=0;
void LWRCore::ControlUpdate(){

	switch (nControl){
	{
	case Robot::CTRLMODE_POSITION:

		float jpc2[LBR_MNJ];
		for(int i=0;i<LBR_MNJ;i++){
			jpc2[i] =(float)((RevoluteJointActuator*)mJointActuators[i])->mPosition;
		}

		SetCommandedJPos(jpc2);
		break;
	}
	{
	case Robot::CTRLMODE_JOINTIMPEDANCE:
		float jpc2[LBR_MNJ];
		float jstiff[LBR_MNJ];
		float jdamp[LBR_MNJ];
		for(int i=0;i<LBR_MNJ;i++){
			jpc2[i] =(float)((RevoluteJointActuator*)mJointActuators[i])->mPosition;
			jstiff[i] = mLWRRobot->GetJointStiffness()(i);
			jdamp[i] = mLWRRobot->GetJointDamping()(i);
		}

		SetCommandedJPos(jpc2);

		cnt++;

		if(cnt==REL_FREQ_IMPEDANCE_UPDATE){
			mFRI->SetCommandedJointStiffness(jstiff);
			mFRI->SetCommandedJointDamping(jdamp);
//                        cout<<"I set the stiffness and damping "<<endl;
			cnt=0;
		}

		break;
	}
	{
	case Robot::CTRLMODE_GRAVITYCOMPENSATION:

		float jpc2[LBR_MNJ];
		mFRI->GetMeasuredJointPositions(jpc2);


				float jstiff[LBR_MNJ];
				float jdamp[LBR_MNJ];
				for(int i=0;i<LBR_MNJ;i++){
					if(!mLWRRobot->GetGravComp(i))
					{
					jstiff[i] = mLWRRobot->GetJointStiffness()(i);
					jdamp[i] = mLWRRobot->GetJointDamping()(i);
					jpc2[i] = (float)((RevoluteJointSensor*)mJointSensors[i])->mPosition;
					}
					else
					{
						jstiff[i] =0;
						jdamp[i] =0;
					}
				}

		cnt++;

				if(cnt==REL_FREQ_IMPEDANCE_UPDATE){
					mFRI->SetCommandedJointStiffness(jstiff);
					mFRI->SetCommandedJointDamping(jdamp);
		//                        cout<<"I set the stiffness and damping "<<endl;
					cnt=0;
				}


		SetCommandedJPos(jpc2);
		break;
	}
        {
        case Robot::CTRLMODE_CARTIMPEDANCE:
    // 1) get the cartesian command from mLWRRobot.
                float *  cart_pose_command;
                cart_pose_command = mLWRRobot->GetCartCommandAsFloat();



    // 2) send the cartesian command to the real robot.

                mFRI->SetCommandedCartPose(cart_pose_command);


                // 3) get the desired stiffness and damping
                float * cart_stiffness;
                float * cart_damping;

                cart_stiffness = mLWRRobot->GetCartStiffnessAsFloat();


                cart_damping = mLWRRobot->GetCartDampingAsFloat();
                    // 4)send the desired stiffness and damping
                cnt++;
                if(cnt==REL_FREQ_IMPEDANCE_UPDATE){
                        mFRI->SetCommandedCartStiffness(cart_stiffness);
                        mFRI->SetCommandedCartDamping(cart_damping);
                        cnt=0;
                }

                //5) set desired force

                float * desiredForce;
                desiredForce = mLWRRobot->GetDesiredForceAsFloat();
                /*cout<<"simdi borda desired force"<<endl;
                for(int i=0;i<6;i++){
                                   cout<<desiredForce[i]<<" ";
                                 }*/
                mFRI->SetCommandedCartForcesAndTorques(desiredForce);

                break;
        }
	}



}

/* this is a lame attempt at trying to avoid some 'regulator limit exceeded',
 * which for some enormously stupid reason seems to occur when the commanded joint positions
 * change to LITTLE between two iterations. Yes, you read right, when two consecutive commanded positions
 * are too similar, these errors occur...go figure..
 */
void LWRCore::SetCommandedJPos(float* JP_commanded){
	float oldJP_commanded[LBR_MNJ];
	mFRI->GetCommandedJointPositions(oldJP_commanded);
	//float diff;
//	for(int i=0;i<LBR_MNJ;i++){
//		diff = fabs(JP_commanded[i] - oldJP_commanded[i]);
//		if(diff < 0.0001){
//			JP_commanded[i] = oldJP_commanded[i];
//		}
//	}
	mFRI->SetCommandedJointPositions(JP_commanded);

}

void LWRCore::ConsoleLoop(){
	while(bRunConsoleLoop == true){
		usleep(10000);			// 10 ms rate of refresh for commandline interface
		ConsoleUpdate();
	}

}

// void LWRCore::ControlUpdate(){





// }



void LWRCore::SensorsUpdate(){
	/*NOTE: CURRENTLY EXTREMELY POOR ESTIMATION OF VELOCITY AND ACCELRATION!
    WE SHOULD PROBABLY USE A FILTER HERE.*/


	//store the time of measurement and the measurement
	LWRMeasurement thisMeasurement;
	thisMeasurement.t = mFRI->ReadData.intf.timestamp;

	float tempJP[LBR_MNJ];
	float tempJP_commanded[LBR_MNJ];
	float tempJT[LBR_MNJ];
	float tempEJT[LBR_MNJ];
	float tempCFT[FRI_CART_VEC];
	float tempCartPose[12];
	float tempCartPose_commanded[12];

	//char txt[5000];
	// get the actual values from FRI
	mFRI->GetMeasuredJointPositions(tempJP);
	mFRI->GetMeasuredCartPose(tempCartPose);
	mFRI->GetMeasuredJointTorques(tempJT);
	mFRI->GetEstimatedExternalJointTorques(tempEJT);
	mFRI->GetEstimatedExternalCartForcesAndTorques(tempCFT);

	mFRI->GetCommandedJointPositions(tempJP_commanded);
	mFRI->GetCommandedCartPose(tempCartPose_commanded);

	//CompleteLWRState vector structure:
	// 7xJoint Positions + 12xCart Pose + 7xJoint Torques +
	//7x Estimated External Joint Torques + 6x Estimated Cart Force +
	// 7x commanded joint positions + 12 x commanded cart pose
	int cn = 0;
	for(int i=0;i<LBR_MNJ;i++){
		LWR_state(cn) = tempJP[i];
		cn++;
	}
	for(int i=0;i<12;i++){
		LWR_state(cn) = tempCartPose[i];
		cn++;
	}
	for(int i=0;i<LBR_MNJ;i++){
		LWR_state(cn) = tempJT[i];
		cn++;
	}
	for(int i=0;i<LBR_MNJ;i++){
		LWR_state(cn) = tempEJT[i];
		cn++;
	}
	for(int i=0;i<FRI_CART_VEC;i++){
		LWR_state(cn) = tempCFT[i];
		cn++;
	}
	for(int i=0;i<LBR_MNJ;i++){
		LWR_state(cn) = tempJP_commanded[i];
		cn++;
	}
	for(int i=0;i<12;i++){
		LWR_state(cn) = tempCartPose_commanded[i];
		cn++;

	}



	mLWRRobot->SetCompleteLWRState(LWR_state);




	for(int i=0;i<LBR_MNJ;i++){
		tempJ_v(i) = tempEJT[i];
	}
	mLWRRobot->SetEstimatedExternalJointTorques(tempJ_v);


	for(int i=0;i<LBR_MNJ;i++){
			tempJ_v(i) = tempJT[i];
		}
	mLWRRobot->SetMeasuredJointTorques(tempJ_v);

	for(int i=0;i<FRI_CART_VEC;i++){
		tempC_v(i) = tempCFT[i];
	}
	//tempC_v.Print();
	mLWRRobot->SetEstimatedExternalCartForces(tempC_v);

        mLWRRobot->SetMeasuredCartPose(tempCartPose);


        float measured_time; //s
        measured_time = mFRI->GetFRICycleTime(); //s
        mLWRRobot->SetSamplingTime(measured_time); //s


	thisMeasurement.JointPositions.Resize(LBR_MNJ);
	thisMeasurement.JointTorques.Resize(LBR_MNJ);


	for(int i=0;i<LBR_MNJ;i++){
		thisMeasurement.JointPositions(i) = (REALTYPE)tempJP[i];
		thisMeasurement.JointTorques(i) = (REALTYPE)tempEJT[i];
	}

	if(MeasurementHistory.size()>0){
		//set velocities, finite diffrence with prioir measurement of position
		thisMeasurement.JointVelocities.Set(MeasurementHistory.front().JointPositions);
		thisMeasurement.JointVelocities -= thisMeasurement.JointPositions;
		double dt = thisMeasurement.t - MeasurementHistory.front().t;
		//  cout<<"this is my time"<<dt<<endl;
		thisMeasurement.JointVelocities /= dt;


		thisMeasurement.JointAccelerations.Set(MeasurementHistory.front().JointVelocities);
		thisMeasurement.JointAccelerations -= thisMeasurement.JointVelocities;

		thisMeasurement.JointAccelerations /= dt;
	}

	else{
		// this happens the first time, so we set the joint velocity to zero here..
		thisMeasurement.JointVelocities.Resize(LBR_MNJ);
		thisMeasurement.JointVelocities.Zero();
		thisMeasurement.JointAccelerations.Resize(LBR_MNJ);
		thisMeasurement.JointVelocities.Zero();
	}


	// put the new measurement in the history deque
	MeasurementHistory.push_front(thisMeasurement);
	//if histry is too long, pop it
	if(MeasurementHistory.size() > MEASUREMENT_HISTORY_LENGTH)
		MeasurementHistory.pop_back();

	// thisMeasurement.JointPositions.Print();
	// thisMeasurement.JointVelocities.Print();
	// thisMeasurement.JointAccelerations.Print();

	//write to sensors of robot object
	for(int i =0;i<LBR_MNJ;i++){
		((RevoluteJointSensor*)mJointSensors[i])->mPosition = thisMeasurement.JointPositions(i);
		((RevoluteJointSensor*)mJointSensors[i])->mVelocity = thisMeasurement.JointVelocities(i);
		((RevoluteJointSensor*)mJointSensors[i])->mAcceleration =thisMeasurement.JointAccelerations(i);
		((RevoluteJointSensor*)mJointSensors[i])->mTorque =thisMeasurement.JointTorques(i);
	}








}

void LWRCore::ConsoleUpdate(){

	string s = mOutputStream.str();

	size_t cpos = 0;
	size_t opos = 0;

	if(s.size()>0){
		for(unsigned int i=0;i<s.size();i++){
			opos = cpos;
			cpos = s.find("\n",opos);
			string ss = s.substr(opos,cpos-opos);
			if(ss.size()>0){
				mNCConsole.Print(ss);
			}
			if(cpos==string::npos)
				break;
			cpos++;
		}
		mOutputStream.str("");
	}

	mFRI->GetMeasuredJointPositions(jnt2);
	sprintf(static_txt, "Joint Position :  %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf", DEG(jnt2[0]),DEG(jnt2[1]),DEG(jnt2[2]),DEG(jnt2[3]),DEG(jnt2[4]),DEG(jnt2[5]),DEG(jnt2[6]) );
	mNCConsole.SetTopStaticLine(0,  static_txt);

	mFRI->GetEstimatedExternalJointTorques(jnt2);
	sprintf(static_txt, "Joint Torques  :  %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf", jnt2[0],jnt2[1],jnt2[2],jnt2[3],jnt2[4],jnt2[5],jnt2[6] );
	mNCConsole.SetTopStaticLine(1,  static_txt);

	int tmp = mFRI->GetFRIMode();
	switch (tmp) {
	case FRI_STATE_CMD:
		sprintf(static_txt, "FRI State      :  COMMAND");
		break;
	case FRI_STATE_MON:
		sprintf(static_txt, "FRI State      :  MONITOR");
		break;
	case FRI_STATE_INVALID:
		sprintf(static_txt, "FRI State      :  INVALID");
		break;
	case FRI_STATE_OFF:
		sprintf(static_txt, "FRI State      :  OFF");
		break;
	default:
		sprintf(static_txt, "FRI State      :  UNKNOWN");
		break;
	}

	mNCConsole.SetTopStaticLine(2,  static_txt);

	tmp = mFRI->GetCommunicationTimingQuality();
	switch (tmp) {
	case FRI_QUALITY_BAD:
		sprintf(static_txt, "Comm. Quality  :  BAD");
		break;
	case FRI_QUALITY_INVALID:
		sprintf(static_txt, "Comm. Quality  :  INVALID");
		break;
	case FRI_QUALITY_OK:
		sprintf(static_txt, "Comm. Quality  :  OK");
		break;
	case FRI_QUALITY_PERFECT:
		sprintf(static_txt, "Comm. Quality  :  PERFECT");
		break;
	case FRI_QUALITY_UNACCEPTABLE:
		sprintf(static_txt, "Comm. Quality  :  UNACCEPTABLE");
		break;
	default:
		sprintf(static_txt, "Comm. Quality  :  UNKNOWN");
		break;
	}

	mNCConsole.SetTopStaticLine(3, static_txt);


	tmp = mFRI->GetCurrentControlScheme();
	switch (tmp) {
	case FastResearchInterface::JOINT_POSITION_CONTROL:
		sprintf(static_txt, "Control Mode   :  JOINT POSITION");
		break;
	case FastResearchInterface::JOINT_IMPEDANCE_CONTROL:
		sprintf(static_txt, "Control Mode   :  JOINT IMPEDANCE");
		break;
	case FastResearchInterface::CART_IMPEDANCE_CONTROL:
		sprintf(static_txt, "Control Mode   :  CARTESIAN IMPDEANCE");
		break;
	default:
		sprintf(static_txt, "Control Mode   :  UNKNOWN");
		break;
	}

	mNCConsole.SetTopStaticLine(4, static_txt);

	float extft[6];
	mFRI->GetEstimatedExternalCartForcesAndTorques(extft);
	    sprintf(static_txt, "External C-FT  :  %-2.6lf  %-2.6lf  %-2.6lf  %-2.6lf  %-2.6lf  %-2.6lf", extft[0],extft[1],extft[2],extft[3],extft[4],extft[5]);
//	if(mFRI->IsRobotArmPowerOn() && !mFRI->DoesAnyDriveSignalAnError())
//		sprintf(static_txt, "Drives         :  GO");
//	else
//		sprintf(static_txt, "Drives         :  NO-GO");

	mNCConsole.SetTopStaticLine(5, static_txt);

	float cartp[12];
		mFRI->GetMeasuredCartPose(cartp);
		sprintf(static_txt, "Msr Cart       :  %-2.6lf  %-2.6lf  %-2.6lf", cartp[3],cartp[7],cartp[11]);
		    mNCConsole.SetTopStaticLine(6, static_txt);

	mNCConsole.Process();
	mNCConsole.Render();

}


bool LWRCore::SetControlMode(int desiredMode){

	if(desiredMode == Robot::CTRLMODE_POSITION){

		mNCConsole.Print("Waiting for script...");
		mNCConsole.Render();
		mFRI->GetMeasuredJointPositions(jnt2);
		SetCommandedJPos(jnt2);
		mFRI-> WaitForKRCTick();
		int result =  mFRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL, FRI_CONN_TIMEOUT_SEC);
		if(result != EOK && result != EALREADY)
		{
			cout<<"Error: "<<result<<endl;
			return false;
		}
		mNCConsole.Print("Robot set in position control mode");


		nControl = Robot::CTRLMODE_POSITION;
	}
	else if(desiredMode == Robot::CTRLMODE_CARTIMPEDANCE ){
		mNCConsole.Print("Waiting for script...");
		mNCConsole.Render();
		mFRI->StopRobot();
                mFRI->GetMeasuredCartPose(cart2);
                mFRI->SetCommandedCartPose(cart2);
		mFRI-> WaitForKRCTick();
		int result =  mFRI->StartRobot(FastResearchInterface::CART_IMPEDANCE_CONTROL, FRI_CONN_TIMEOUT_SEC);
		if(result != EOK && result != EALREADY)
		{
			cout<<"Error: "<<result<<endl;
			return 1;
		}
		for(int i=0;i<3; i++)
			cart_imp_params[i] =  mCurrCStiff_pos;
		for(int i=3;i<6; i++)
			cart_imp_params[i] =  mCurrCStiff_or;

		mFRI->SetCommandedCartStiffness(cart_imp_params);

		for(int i=0;i<3; i++)
			cart_imp_params[i] =  mCurrCDamp_pos;
		for(int i=3;i<6; i++)
			cart_imp_params[i] =  mCurrCDamp_or;

		mFRI->SetCommandedCartDamping(cart_imp_params);


		mNCConsole.Print("Robot set in cartesian impedance control mode");


		nControl = Robot::CTRLMODE_CARTIMPEDANCE;

	}
	else if(desiredMode == Robot::CTRLMODE_JOINTIMPEDANCE){
		mFRI->SetKRLBoolValue(0,true);
		mNCConsole.Print("Waiting for script...");
		mNCConsole.Render();
		mFRI-> WaitForKRCTick();
		int result =  mFRI->StartRobot(FastResearchInterface::JOINT_IMPEDANCE_CONTROL, FRI_CONN_TIMEOUT_SEC);
		if(result != EOK && result != EALREADY)
		{
			cout<<"Error: "<<result<<endl;
			return 1;
		}

		for(int i=0;i<LBR_MNJ; i++)
			jnt2[i] =  mCurrJDamp;
		mFRI->SetCommandedJointDamping(jnt2);

		for(int i=0;i<LBR_MNJ; i++)
			jnt2[i] =  mCurrJStiff;
		mFRI->SetCommandedJointStiffness(jnt2);


		nControl = Robot::CTRLMODE_JOINTIMPEDANCE;
		mNCConsole.Print("Robot set in joint impedance mode.");



	}
	else if(desiredMode == Robot::CTRLMODE_GRAVITYCOMPENSATION){

		mFRI->SetKRLBoolValue(0,true);
		mNCConsole.Print("Waiting for script...");
		mNCConsole.Render();
		mFRI-> WaitForKRCTick();
		int result =  mFRI->StartRobot(FastResearchInterface::JOINT_IMPEDANCE_CONTROL, FRI_CONN_TIMEOUT_SEC);
		if(result != EOK && result != EALREADY)
		{
			cout<<"Error: "<<result<<endl;
			return 1;
		}

		Vector v = mLWRRobot-> GetJointStiffness();
		for(int i=0;i<LBR_MNJ; i++)
			if(mLWRRobot->GetGravComp(i))
			jnt2[i] = 0;
			else
				jnt2[i] = v(i);

		v = mLWRRobot-> GetJointDamping();
		float jdamp[LBR_MNJ];
		for(int i=0;i<LBR_MNJ; i++)
			if(mLWRRobot->GetGravComp(i))
			jdamp[i] = 0;
			else
				jdamp[i] = v(i);


		mFRI->SetCommandedJointStiffness(jnt2);
		mFRI->SetCommandedJointDamping(jdamp);

		nControl = Robot::CTRLMODE_GRAVITYCOMPENSATION;
		mNCConsole.Print("Robot set in grav. comp. mode");


	}

	nMode = MODE_NONE;
	return true;
}

int LWRCore::RespondToConsoleCommand(const string command, const vector<string> & args)
{

	GetConsole()->ClearLine();

	if(command == "quit")
	{
		Stop();
		Free();
		exit(1);
	}



	return 0;
}
