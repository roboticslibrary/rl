//
// Copyright (c) 2013, Andre Gaschler, Markus Rickert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef _RL_HAL_WEISSWSG50_H_
#define _RL_HAL_WEISSWSG50_H_

#include <string>

#include "Device.h"
#include "DeviceException.h"
#include "Gripper.h"
#include "types.h"
#include "TcpSocket.h"

namespace rl
{
	namespace hal
	{
		/**
		 * Weiss Robotics WSG 50 / Schunk WSG 50
		 * 
		 * See "WSG Series of Intelligent Servo-Electric Grippers: Command Set Reference Manual":
		 * <http://www.schunk.com/schunk_files/attachments/WSG__Command_Set_Reference_Manual__2013-01_EN.pdf>
		 * 
		 * Important: Before use, you usually need to set "Part Width Tolerance"
		 * and "Default Clamping Travel" to maximum in the web interface.
		 * Otherwise, the gripper will not grasp if object width is different
		 * from the given value.
		 */
		class WeissWsg50 : public Gripper
		{
		public:
			class Exception : public DeviceException
			{
			public:
				enum Error
				{
					ERROR_SUCCESS = 0,
					ERROR_NOT_AVAILABLE,
					ERROR_NO_SENSOR,
					ERROR_NOT_INITIALIZED,
					ERROR_ALREADY_RUNNING,
					ERROR_FEATURE_NOT_SUPPORTED,
					ERROR_INCONSISTENT_DATA,
					ERROR_TIMEOUT,
					ERROR_READ_ERROR,
					ERROR_WRITE_ERROR,
					ERROR_INSUFFICIENT_RESOURCES,
					ERROR_CHECKSUM_ERROR,
					ERROR_NO_PARAM_EXPECTED,
					ERROR_NOT_ENOUGH_PARAMS,
					ERROR_CMD_UNKNOWN,
					ERROR_CMD_FORMAT_ERROR,
					ERROR_ACCESS_DENIED,
					ERROR_ALREADY_OPEN,
					ERROR_CMD_FAILED,
					ERROR_CMD_ABORTED,
					ERROR_INVALID_HANDLE,
					ERROR_NOT_FOUND,
					ERROR_NOT_OPEN,
					ERROR_IO_ERROR,
					ERROR_INVALID_PARAMETER,
					ERROR_INDEX_OUT_OF_BOUNDS,
					ERROR_CMD_PENDING,
					ERROR_OVERRUN,
					ERROR_RANGE_ERROR,
					ERROR_AXIS_BLOCKED,
					ERROR_FILE_EXISTS
				};
				
				Exception(const Error& error);
				
				virtual ~Exception() throw();
				
				Error getError() const;
				
				virtual const char* what() const throw();
				
			protected:
				
			private:
				Error error;
			};
			
			enum GraspingState
			{
				GRASPING_STATE_IDLE = 0,
				GRASPING_STATE_GRASPING,
				GRASPING_STATE_NO_PART_FOUND,
				GRASPING_STATE_PART_LOST,
				GRASPING_STATE_HOLDING,
				GRASPING_STATE_RELEASING
			};
			
			enum SystemState
			{
				SYSTEM_STATE_SCRIPT_FAILURE,
				SYSTEM_STATE_SCRIPT_RUNNING,
				SYSTEM_STATE_CMD_FAILURE,
				SYSTEM_STATE_FINGER_FAULT,
				SYSTEM_STATE_CURR_FAULT,
				SYSTEM_STATE_POWER_FAULT,
				SYSTEM_STATE_TEMP_FAULT,
				SYSTEM_STATE_TEMP_WARNING,
				SYSTEM_STATE_FAST_STOP,
				SYSTEM_STATE_OVERDRIVE_MODE,
				SYSTEM_STATE_TARGET_POS_REACHED,
				SYSTEM_STATE_AXIS_STOPPED,
				SYSTEM_STATE_SOFT_LIMIT_PLUS,
				SYSTEM_STATE_SOFT_LIMIT_MINUS,
				SYSTEM_STATE_BLOCKED_PLUS,
				SYSTEM_STATE_BLOCKED_MINUS,
				SYSTEM_STATE_MOVING,
				SYSTEM_STATE_REFERENCED
			};
			
			/**
			 * @param hostname tcp hostname
			 * @param port tcp port
			 * @param acceleration [0.1,..,5] [m/s^2]
			 * @param forceLimit [5,..,80] [N]
			 * @param period default automatic update period [ms]
			 */
			WeissWsg50(
				const ::std::string& hostname = "192.168.1.20",
				const unsigned short int& port = 1000,
				const float& acceleration = 3.0f,
				const float& forceLimit = 40.0f,
				const unsigned int& period = 10
			);
			
			virtual ~WeissWsg50();
			
			void acknowledgeFaults();
			
			void close();
			
			void doAcknowledgeFaults();
			
			void doFastStop();
			
			void doGetAcceleration();
			
			void doClearSoftLimits();
			
			void doGetForce(const bool& doAutomaticUpdate, const bool& doUpdateOnChange, const unsigned int& period = 0);
			
			void doGetForceLimit();
			
			void doGetGraspingState(const bool& doAutomaticUpdate, const bool& doUpdateOnChange, const unsigned int& period = 0);
			
			void doGetGraspingStatistics(const bool& doReset, int& numberGraspsTotal, int& numberGraspsNoPart, int& numberGraspsLostPart);
			
			void doGetOpeningWidth(const bool& doAutomaticUpdate, const bool& doUpdateOnChange, const unsigned int& period = 0);
			
			void doGetSoftLimits();
			
			void doGetSpeed(const bool& doAutomaticUpdate, const bool& doUpdateOnChange, const unsigned int& period = 0);
			
			void doGetSystemInformation(bool& isWsg50, int& hardwareRevision, int& firmwareRevision, int& serialNumber);
			
			void doGetSystemLimits();
			
			void doGetSystemState(const bool& doAutomaticUpdate, const bool& doUpdateOnChange, const unsigned int& period = 0);
			
			/**
			 * @return [@htmlonly &#176C @endhtmlonly]
			 */
			float doGetTemperature();
			
			/**
			 * @param width [0,..,0.11] [m]
			 * @param speed [0,..,0.4] [m/s]
			 */
			void doGraspPart(const float& width, const float& speed);
			
			/**
			 * Perform necessary homing motion for calibration.
			 * This function call is blocking until the calibration is complete.
			 * 
			 * @param direction 0 = default from system configuration, 1 = positive movement, 2 = negative movement
			 */
			void doHomingMotion(const unsigned int& direction = 0);
			
			void doOverdriveMode(const bool& doOverdriveMode);
			
			/**
			 * @param width [0,..,0.11] [m]
			 * @param speed [0,..,0.4] [m/s]
			 */
			void doPrePositionFingers(const float& width, const float& speed = 0.2f, const bool& doRelativeMovement = false, const bool& doStopOnBlock = false);
			
			/**
			 * @param width [0,..,0.11] [m]
			 * @param speed [0.005,..,0.4] [m/s]
			 */
			void doReleasePart(const float& width = 0.11f, const float& speed = 0.4f);
			
			/**
			 * @param acceleration [0.1,..,5] [m/s^2]
			 */
			void doSetAcceleration(const float& acceleration = 2.0f);
			
			/**
			 * @param force [5,..,80] [N]
			 */
			void doSetForceLimit(const float& force = 40.0f);
			
			/**
			 * @param limitMinus [m]
			 * @param limitPlus [m]
			 */
			void doSetSoftLimits(const float& limitMinus, const float& limitPlus);
			
			void doStop();
			
			void doTareForceSensor();
			
			/**
			 * @return [m/s^2]
			 */
			float getAcceleration() const;
			
			/**
			 * @return [N]
			 */
			float getForce() const;
			
			/**
			 * @return [N]
			 */
			float getForceLimit() const;
			
			GraspingState getGraspingState() const;
			
			/**
			 * @return [m]
			 */
			float getOpeningWidth() const;
			
			/**
			 * @return [m/s]
			 */
			float getSpeed() const;
			
			SystemState getSystemState() const;
			
			void halt();
			
			void open();
			
			void release();
			
			void shut();
			
			void start();
			
			void step();
			
			void stop();
			
		protected:
			
		private:
			uint16_t crc(const uint8_t* buf, const ::std::size_t& len) const;
			
			void doDisconnectAnnouncement();
			
			::std::size_t recv(uint8_t* buf);
			
			::std::size_t recv(uint8_t* buf, const ::std::size_t& len, const uint8_t& command);
			
			void send(uint8_t* buf, const ::std::size_t& len);
			
			static const ::std::size_t HEADER_SIZE = 6;
			
			float acceleration;
			
			float accelerationMaximum;
			
			float accelerationMinimum;
			
			float force;
			
			float forceLimit;
			
			float forceMinimum;
			
			float forceNominal;
			
			float forceOverdrive;
			
			GraspingState graspingState;
			
			float limitMinus;
			
			float limitPlus;
			
			float openingWidth;
			
			unsigned int period;
			
			float speed;
			
			float speedMaximum;
			
			float speedMinimum;
			
			float stroke;
			
			SystemState systemState;
			
			TcpSocket tcp;
		};
	}
}

#endif // _RL_HAL_WEISSWSG50_H_
