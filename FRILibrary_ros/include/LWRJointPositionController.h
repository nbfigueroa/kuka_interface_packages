//  ---------------------- Doxygen info ----------------------
//! \file LWRJointPositionController.h
//!
//! \brief
//! Header file for the class LWRJointPositionController
//!
//! \details
//! The class LWRJointPositionController constitutes an access
//! easy-to-use joint position control interface through the KUKA
//! Fast Research Interface of the Light-Weight Robot IV.
//! \n
//! \n
//! <b>GNU Lesser Public License</b>
//! \n
//! This file is part of the Fast Research Interface Library.
//! \n\n
//! The Fast Research Interface Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Fast Research Interface Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied 
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU General Public License
//! along with the Fast Research Interface Library. If not, see 
//! http://www.gnu.org/licenses.
//! \n
//! \n
//! Stanford University\n
//! Department of Computer Science\n
//! Artificial Intelligence Laboratory\n
//! Gates Computer Science Building 1A\n
//! 353 Serra Mall\n
//! Stanford, CA 94305-9010\n
//! USA\n
//! \n
//! http://cs.stanford.edu/groups/manips\n
//!
//! \date November 2011
//!
//! \version 1.0
//!
//!	\author Torsten Kroeger, tkr@stanford.edu
//!
//!
//!
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#ifndef __LWRJointPositionController__
#define __LWRJointPositionController__


#include <FastResearchInterface.h>
#include <LWRBaseControllerInterface.h>


//  ---------------------- Doxygen info ----------------------
//! \class LWRJointPositionController
//!
//! \brief
//! Joint position controller interface for the KUKA Light-Weight
//! Robot IV
//  ----------------------------------------------------------

class LWRJointPositionController : public LWRBaseControllerInterface
{


public:


//  ---------------------- Doxygen info ----------------------
//! \fn LWRJointPositionController(const char *InitFileName):LWRBaseControllerInterface(InitFileName)
//!
//! \brief
//! Constructor
//!
//! \copydetails LWRBaseControllerInterface::LWRBaseControllerInterface()
//  ----------------------------------------------------------
	LWRJointPositionController(const char *InitFileName):LWRBaseControllerInterface(InitFileName)
	{
	}


//  ---------------------- Doxygen info ----------------------
//! \fn ~LWRJointPositionController(void)
//!
//! \brief
//! Destructor
//!
//! \copydetails LWRBaseControllerInterface::~LWRBaseControllerInterface()
//  ----------------------------------------------------------
	~LWRJointPositionController(void)
	{
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int StartRobot(const float &TimeOutValueInSeconds	=	120.0)
//!
//! \brief
//! \copybrief FastResearchInterface::StartRobot()
//!
//! \details
//! \copydetails FastResearchInterface::StartRobot()
//  ----------------------------------------------------------
	inline int StartRobot(const float &TimeOutValueInSeconds	=	120.0)
	{
		this->printf("Please start up the robot now by using KUKA Control Panel.\n");

		// start the controller and switch to command mode
		return(this->FRI->StartRobot(		FastResearchInterface::JOINT_POSITION_CONTROL
		                             	,	TimeOutValueInSeconds));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedJointPositions(const float *CommandedJointPositions)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedJointPositions()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedJointPositions()
//  ----------------------------------------------------------
	inline void SetCommandedJointPositions(const float *CommandedJointPositions)
	{
		return(this->FRI->SetCommandedJointPositions(CommandedJointPositions));
	}

};	// class LWRJointPositionController

#endif
