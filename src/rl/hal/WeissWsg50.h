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

#ifndef RL_HAL_WEISSWSG50_H
#define RL_HAL_WEISSWSG50_H

#include <cstdint>
#include <string>

#include "CyclicDevice.h"
#include "Device.h"
#include "Gripper.h"
#include "Socket.h"

namespace rl
{
	namespace hal
	{
		/**
		 * Weiss Robotics Universal Gripper WSG 50.
		 *
		 * See "WSG Series of Intelligent Servo-Electric Grippers: Command Set Reference Manual":
		 * <http://www.weiss-robotics.de/en/download.html?cid=1&fid=37&id=314>
		 *
		 * Important: Before use, you usually need to set "Part Width Tolerance"
		 * and "Default Clamping Travel" to maximum in the web interface.
		 * Otherwise, the gripper will not grasp if object width is different
		 * from the given value.
		 */
		class RL_HAL_EXPORT WeissWsg50 : public CyclicDevice, public Gripper
		{
		public:
			enum class GraspingState
			{
				idle = 0,
				gripping = 1,
				noPartFound = 2,
				partLost = 3,
				holding = 4,
				releasing = 5,
				positioning = 6,
				error = 7
			};
			
			enum class SystemState
			{
				referenced = 1,
				moving = 2,
				axisBlockedMinus = 4,
				axisBlockedPlus = 8,
				softLimitMinus = 16,
				softLimitPlus = 32,
				axisStopped = 64,
				targetPositionReached = 128,
				overdriveMode = 256,
				forceControlMode = 512,
				fastStop = 4096,
				temperatureWarning = 8192,
				temperatureFault = 16384,
				powerFault = 32768,
				currentFault = 65536,
				fingerFault = 131072,
				commandFailure = 262144,
				scriptRunning = 524288,
				scriptFailure = 1048576
			};
			
			/**
			 * @param[in] address TCP hostname
			 * @param[in] port TCP port
			 * @param[in] acceleration [0.1,..,5] [m/s^2]
			 * @param[in] forceLimit [5,..,80] [N]
			 * @param[in] period Default automatic update period [ms]
			 */
			WeissWsg50(
				const ::std::string& address = "192.168.1.20",
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
			 * @param[in] width[in] [0,..,0.11] [m]
			 * @param[in] speed[in] [0,..,0.4] [m/s]
			 */
			void doGraspPart(const float& width, const float& speed);
			
			/**
			 * Perform necessary homing motion for calibration.
			 *
			 * This function call is blocking until the calibration is complete.
			 *
			 * @param[in] direction 0 = default from system configuration, 1 = positive movement, 2 = negative movement
			 */
			void doHomingMotion(const unsigned int& direction = 0);
			
			void doOverdriveMode(const bool& doOverdriveMode);
			
			/**
			 * @param[in] width [0,..,0.11] [m]
			 * @param[in] speed [0,..,0.4] [m/s]
			 */
			void doPrePositionFingers(const float& width, const float& speed, const bool& doRelativeMovement = false, const bool& doStopOnBlock = false);
			
			/**
			 * @param[in] width [0,..,0.11] [m]
			 * @param[in] speed [0.005,..,0.4] [m/s]
			 */
			void doReleasePart(const float& width, const float& speed);
			
			/**
			 * @param[in] acceleration [0.1,..,5] [m/s^2]
			 */
			void doSetAcceleration(const float& acceleration);
			
			/**
			 * @param[in] force [5,..,80] [N]
			 */
			void doSetForceLimit(const float& force);
			
			/**
			 * @param[in] limitMinus [m]
			 * @param[in] limitPlus [m]
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
			
			::std::uint8_t getGraspingStateBits() const;
			
			/**
			 * @return [m]
			 */
			float getOpeningWidth() const;
			
			/**
			 * @return [m/s]
			 */
			float getSpeed() const;
			
			::std::uint32_t getSystemStateBits() const;
			
			void halt();
			
			void open();
			
			void release();
			
			void shut();
			
			void start();
			
			void step();
			
			void stop();
			
		protected:
			
		private:
			::std::uint16_t crc(const ::std::uint8_t* buf, const ::std::size_t& len) const;
			
			void doDisconnectAnnouncement();
			
			::std::size_t recv(::std::uint8_t* buf);
			
			::std::size_t recv(::std::uint8_t* buf, const ::std::size_t& len, const ::std::uint8_t& command);
			
			void send(::std::uint8_t* buf, const ::std::size_t& len);
			
			static constexpr ::std::size_t headerSize = 6;
			
			float acceleration;
			
			float accelerationMaximum;
			
			float accelerationMinimum;
			
			float force;
			
			float forceLimit;
			
			float forceMinimum;
			
			float forceNominal;
			
			float forceOverdrive;
			
			::std::uint8_t graspingState;
			
			float limitMinus;
			
			float limitPlus;
			
			float openingWidth;
			
			unsigned int period;
			
			Socket socket;
			
			float speed;
			
			float speedMaximum;
			
			float speedMinimum;
			
			float stroke;
			
			::std::uint32_t systemState;
		};
	}
}

#endif // RL_HAL_WEISSWSG50_H
