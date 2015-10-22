//  ---------------------- Doxygen info ----------------------
//! \file LWRBaseControllerInterface.h
//!
//! \brief
//! Header file for the class LWRBaseControllerInterface
//!
//! \details
//! The class LWRBaseControllerInterface constitutes the base class
//! for the actual control interface classes:
//!
//!  - class LWRJointPositionController
//!  - class CartImpedanceController
//!  - class LWRJointImpedanceController
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


#ifndef __LWRBaseControllerInterface__
#define __LWRBaseControllerInterface__

#include <FastResearchInterface.h>
#include <friComm.h>
#include <OSAbstraction.h>
#include <errno.h>
#include <stdarg.h>



//  ---------------------- Doxygen info ----------------------
//! \class LWRBaseControllerInterface
//!
//! \brief
//! Base class for other controller interface classes of the
//! KUKA Fast Research Interface for the Light-Weight Robot IV
//!
//! \details
//! This class constitutes the base class for the three different controller interface classes, which represent the
//! actual user API for the Fast Research Library:
//! <ul>
//! <li>LWRJointPositionController for the joint position controller,
//! <li>LWRCartImpedanceController for the Cartesian impedance controller, and
//! <li>LWRJointImpedanceController for the joint impedance controller,\n\n
//! </ul>
//! This base class only features one attribute, which is a pointer to an object of the class FastResearchInterface.
//! Most of the functionality of the class FastResearchInterface remains hidden, and only a small subset that is
//! actually necessary for convenient user API is used in this base class and it derivatives.
//!
//! \sa FastResearchInterface
//! \sa LWRJointPositionController
//! \sa LWRCartImpedanceController
//! \sa LWRJointImpedanceController
//  ----------------------------------------------------------

class LWRBaseControllerInterface
{


public:


//  ---------------------- Doxygen info ----------------------
//! \fn LWRBaseControllerInterface(const char *InitFileName)
//!
//! \brief
//! Constructor
//!
//! \details
//! The constructor creates the actual object of the class FastResearchInterface, which is used by this class.
//!
//! \copydetails FastResearchInterface::FastResearchInterface()
//  ----------------------------------------------------------
	LWRBaseControllerInterface(const char *InitFileName)
	{
		this->FRI			=	new FastResearchInterface(InitFileName);
	}


//  ---------------------- Doxygen info ----------------------
//! \fn ~LWRBaseControllerInterface(void)
//!
//! \brief
//! Destructor
//!
//! \details
//! The destructor deletes the actual object of the class FastResearchInterface, which is used by this class.
//!
//! \copydetails FastResearchInterface::~FastResearchInterface()
//  ----------------------------------------------------------
	~LWRBaseControllerInterface(void)
	{
		delete this->FRI;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn virtual inline int StartRobot(const float &TimeOutValueInSeconds) = 0
//!
//! \brief
//! \copybrief FastResearchInterface::StartRobot()
//!
//! \details
//! \copydetails FastResearchInterface::StartRobot()
//  ----------------------------------------------------------
	virtual inline int StartRobot(const float &TimeOutValueInSeconds) = 0;

//  ---------------------- Doxygen info ----------------------
//! \fn inline int StopRobot(void)
//!
//! \brief
//! \copybrief FastResearchInterface::StopRobot()
//!
//! \details
//! \copydetails FastResearchInterface::StopRobot()
//  ----------------------------------------------------------
	inline int StopRobot(void)
	{
		return(this->FRI->StopRobot());
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetMeasuredJointPositions(float *MeasuredJointPositions)
//!
//! \brief
//! \copybrief FastResearchInterface::GetMeasuredJointPositions()
//!
//! \details
//! \copydetails FastResearchInterface::GetMeasuredJointPositions()
//!
//! \return
//! <ul>
//! <li> \c ENOTCONN if no connection between the remote host and the KRC unit exists.
//! <li> \c EOK if no error occurred.
//! </ul>
//  ----------------------------------------------------------
	inline int GetMeasuredJointPositions(float *MeasuredJointPositions)
	{
		this->FRI->GetMeasuredJointPositions(MeasuredJointPositions);

		if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
		{
			return(ENOTCONN);
		}
		else
		{
			return(EOK);
		}
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetMeasuredJointTorques(float *MeasuredJointTorques)
//!
//! \brief
//! \copybrief FastResearchInterface::GetMeasuredJointTorques()
//!
//! \details
//! \copydetails FastResearchInterface::GetMeasuredJointTorques()
//!
//! \return
//! <ul>
//! <li> \c ENOTCONN if no connection between the remote host and the KRC unit exists.
//! <li> \c EOK if no error occurred.
//! </ul>
//  ----------------------------------------------------------
	inline int GetMeasuredJointTorques(float *MeasuredJointTorques)
	{
		this->FRI->GetMeasuredJointTorques(MeasuredJointTorques);

		if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
		{
			return(ENOTCONN);
		}
		else
		{
			return(EOK);
		}
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int WaitForKRCTick(const unsigned int &TimeoutValueInMicroSeconds = 0)
//!
//! \brief
//! \copybrief FastResearchInterface::WaitForKRCTick()
//!
//! \details
//! \copydetails FastResearchInterface::WaitForKRCTick()
//  ----------------------------------------------------------
	inline int WaitForKRCTick(const unsigned int &TimeoutValueInMicroSeconds = 0)
	{
		return(this->FRI->WaitForKRCTick(TimeoutValueInMicroSeconds));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int WaitForTimerTick(const unsigned int &TimeoutValueInMicroSeconds = 0)
//!
//! \brief
//! \copybrief FastResearchInterface::WaitForTimerTick()
//!
//! \details
//! \copydetails FastResearchInterface::WaitForTimerTick()
//  ----------------------------------------------------------
	inline int WaitForTimerTick(const unsigned int &TimeoutValueInMicroSeconds = 0)
	{
		return(this->FRI->WaitForTimerTick(TimeoutValueInMicroSeconds));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool IsMachineOK(void)
//!
//! \brief
//! \copybrief FastResearchInterface::IsMachineOK()
//!
//! \details
//! \copydetails FastResearchInterface::IsMachineOK()
//  ----------------------------------------------------------
	inline bool IsMachineOK(void)
	{
		return(this->FRI->IsMachineOK());
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline float GetCycleTime(void)
//!
//! \brief
//! \copybrief FastResearchInterface::GetFRICycleTime()
//!
//! \details
//! \copydetails FastResearchInterface::GetFRICycleTime()
//  ----------------------------------------------------------
	inline float GetCycleTime(void)
	{
		return(this->FRI->GetFRICycleTime());
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline const char* GetCompleteRobotStateAndInformation(void)
//!
//! \brief
//! \copybrief FastResearchInterface::GetCompleteRobotStateAndInformation()
//!
//! \details
//! \copydetails FastResearchInterface::GetCompleteRobotStateAndInformation()
//  ----------------------------------------------------------
	inline const char* GetCompleteRobotStateAndInformation(void)
	{
		return(FRI->GetCompleteRobotStateAndInformation());
	}

//  ---------------------- Doxygen info ----------------------
//! \fn inline int printf(const char* Format, ...)
//!
//! \brief
//! \copybrief FastResearchInterface::printf()
//!
//! \details
//! \copydetails FastResearchInterface::printf()
//  ----------------------------------------------------------
	inline int printf(const char* Format, ...)
	{
		int			Result		=	0;
	    va_list		ListOfArguments;

		va_start(ListOfArguments, Format);
		Result = FRI->printf(Format, ListOfArguments);
		va_end(ListOfArguments);

		return(Result);
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int PrepareLogging(const char *FileIdentifier = NULL)
//!
//! \brief
//! \copybrief FastResearchInterface::PrepareLogging()
//!
//! \details
//! \copydetails FastResearchInterface::PrepareLogging()
//  ----------------------------------------------------------
	inline int PrepareLogging(const char *FileIdentifier = NULL)
	{
		return(FRI->PrepareLogging(FileIdentifier));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int StartLogging(void)
//!
//! \brief
//! \copybrief FastResearchInterface::StartLogging()
//!
//! \details
//! \copydetails FastResearchInterface::StartLogging()
//  ----------------------------------------------------------
	inline int StartLogging(void)
	{
		return(FRI->StartLogging());
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int StopLogging(void)
//!
//! \brief
//! \copybrief FastResearchInterface::StopLogging()
//!
//! \details
//! \copydetails FastResearchInterface::StopLogging()
//  ----------------------------------------------------------
	inline int StopLogging(void)
	{
		return(FRI->StopLogging());
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int WriteLoggingDataFile(void)
//!
//! \brief
//! \copybrief FastResearchInterface::WriteLoggingDataFile()
//!
//! \details
//! \copydetails FastResearchInterface::WriteLoggingDataFile()
//  ----------------------------------------------------------
	inline int WriteLoggingDataFile(void)
	{
		return(FRI->WriteLoggingDataFile());
	}


protected:


//  ---------------------- Doxygen info ----------------------
//! \var FastResearchInterface *FRI
//!
//! \brief
//! A pointer to the actual object of the class FastResearchInterface
//  ----------------------------------------------------------
	FastResearchInterface		*FRI;

};	// class LWRBaseControllerInterface

#endif
