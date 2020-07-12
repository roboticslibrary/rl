//
// Copyright (c) 2009, Markus Rickert
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

#ifndef RL_HAL_MITSUBISHIH7_H
#define RL_HAL_MITSUBISHIH7_H

#include <cstdint>
#include <string>

#include "CartesianPositionActuator.h"
#include "CartesianPositionSensor.h"
#include "CyclicDevice.h"
#include "Gripper.h"
#include "JointPositionActuator.h"
#include "JointPositionSensor.h"
#include "MitsubishiR3.h"
#include "Socket.h"

namespace rl
{
	namespace hal
	{
		/**
		 * Mitsubishi Electric MELFA Ethernet interface (software version H7 or later).
		 */
		class RL_HAL_EXPORT MitsubishiH7 :
			public CartesianPositionActuator,
			public CartesianPositionSensor,
			public CyclicDevice,
			public Gripper,
			public JointPositionActuator,
			public JointPositionSensor
		{
		public:
			enum class Mode
			{
				joint = 1,
				pose = 0,
				pulse = 2
			};
			
			RL_HAL_DEPRECATED static constexpr Mode MODE_JOINT = Mode::joint;
			RL_HAL_DEPRECATED static constexpr Mode MODE_POSE = Mode::pose;
			RL_HAL_DEPRECATED static constexpr Mode MODE_PULSE = Mode::pulse;
			
			MitsubishiH7(
				const ::std::size_t& dof,
				const ::std::string& addressServer,
				const ::std::string& addressClient,
				const unsigned short int& portTcp = 10001,
				const unsigned short int& portUdp = 10000,
				const Mode& mode = Mode::joint,
				const ::std::uint16_t& haltIoData = 0x00AA,
				const ::std::uint16_t& releaseIoData = 0x00A6,
				const ::std::uint16_t& shutIoData = 0x000A9
			);
			
			virtual ~MitsubishiH7();
			
			void close();
			
			::rl::math::Transform getCartesianPosition() const;
			
			::Eigen::Matrix<::std::int32_t, ::Eigen::Dynamic, 1> getCurrentFeedback() const;
			
			::std::size_t getFilter() const;
			
			::std::uint16_t getIoData() const;
			
			::rl::math::Vector getJointPosition() const;
			
			Mode getMode() const;
			
			::Eigen::Matrix<::std::int32_t, ::Eigen::Dynamic, 1> getMotorPulse() const;
			
			void halt();
			
			void loadProgram(const ::std::string& name, const ::std::string& program);
			
			void open();
			
			void release();
			
			void setCartesianPosition(const ::rl::math::Transform& x);
			
			void setFilter(const ::std::size_t& filter);
			
			void setInput(const ::std::uint16_t& bitTop);
			
			void setJointPosition(const ::rl::math::Vector& q);
			
			void setMode(const Mode& mode);
			
			void setMotorPulse(const ::Eigen::Matrix<::std::int32_t, ::Eigen::Dynamic, 1>& p);
			
			void setOutput(const ::std::uint16_t& bitTop, const ::std::uint16_t& bitMask, const ::std::uint16_t& ioData);
			
			void shut();
			
			void start();
			
			void startProgram(const ::std::string& name);
			
			void step();
			
			void stop();
			
			void stopProgram();
			
		protected:
			enum class MxtCommand
			{
				/** Real-time external command invalid. */
				null = 0,
				/** Real-time external command valid. */
				move = 1,
				/** Real-time external command end. */
				end = 255
			};
			
			enum class MxtIo
			{
				
				/** No data. */
				null = 0,
				/** Output signal. */
				out = 1,
				/** Input signal. */
				in = 2
			};
			
			enum class MxtType
			{
				/** No data. */
				null = 0,
				/** XYZ data. */
				pose = 1,
				/** Joint data. */
				joint = 2,
				/** Motor pulse data. */
				pulse = 3,
				/** XYZ data (position after filter process). */
				poseFilter = 4,
				/** Joint data (position after filter process). */
				jointFilter = 5,
				/** Motor pulse data (position after filter process). */
				pulseFilter = 6,
				/** XYZ data (encoder feedback value). */
				poseFeedback = 7,
				/** Joint data (encoder feedback value). */
				jointFeedback = 8,
				/** Motor pulse data (encoder feedback value). */
				pulseFeedback = 9,
				/** Current command [\%]. */
				currentCommand = 10,
				/** Current feedback [\%]. */
				currentFeedback = 11
			};
			
			struct World
			{
				/** X axis coordinate value [mm]. */
				float x;
				/** Y axis coordinate value [mm]. */
				float y;
				/** Z axis coordinate value [mm]. */
				float z;
				/** A axis coordinate value [rad]. */
				float a;
				/** B axis coordinate value [rad]. */
				float b;
				/** C axis coordinate value [rad]. */
				float c;
				/** Additional axis 1 [mm or rad]. */
				float l1;
				/** Additional axis 2 [mm or rad]. */
				float l2;
			};
			
			struct Joint
			{
				/** J1 axis angle [rad]. */
				float j1;
				/** J2 axis angle [rad]. */
				float j2;
				/** J3 axis angle [rad]. */
				float j3;
				/** J4 axis angle [rad]. */
				float j4;
				/** J5 axis angle [rad]. */
				float j5;
				/** J6 axis angle [rad]. */
				float j6;
				/** Additional axis 1 (J7 axis angle) [mm or rad]. */
				float j7;
				/** Additional axis 2 (J8 axis angle) [mm or rad]. */
				float j8;
			};
			
			struct Pose
			{
				World w;
				/** Structural flag 1. */
				::std::uint32_t sflg1;
				/** Structural flag 2. */
				::std::uint32_t sflg2;
			};
			
			struct Pulse
			{
				/** Motor 1 axis. */
				::std::int32_t p1;
				/** Motor 2 axis. */
				::std::int32_t p2;
				/** Motor 3 axis. */
				::std::int32_t p3;
				/** Motor 4 axis. */
				::std::int32_t p4;
				/** Motor 5 axis. */
				::std::int32_t p5;
				/** Motor 6 axis. */
				::std::int32_t p6;
				/** Additional axis 1 (Motor 7 axis). */
				::std::int32_t p7;
				/** Additional axis 2 (Motor 8 axis). */
				::std::int32_t p8;
			};
			
			struct Command
			{
				union Data
				{
					/** XYZ type [mm or rad]. */
					Pose pos;
					/** Joint type [rad]. */
					Joint jnt;
					/** Motor pulse type [the pulse] or current type [\%]. */
					Pulse pls;
					/** Integer type [%% / non-unit]. */
					::std::int32_t lng[8];
				};
				/** Command. */
				::std::uint16_t command;
				/** Transmission data type designation. */
				::std::uint16_t sendType;
				/** Reply data type designation. */
				::std::uint16_t recvType;
				/** Reservation. */
				::std::uint16_t reserve;
				/** Position data. */
				Data dat;
				/** Transmission input/output signal data designation. */
				::std::uint16_t sendIoType;
				/** Reply input/output signal data designation. */
				::std::uint16_t recvIoType;
				/** Head bit no. of input or output signal. */
				::std::uint16_t bitTop;
				/** Bit mask pattern designation (valid only for commanding) [0x0000-0xFFFF]. */
				::std::uint16_t bitMask;
				/** Input or output signal data value (for monitoring) [0x0000-0xFFFF]. Output signal data value (for commanding) [0x0000-0xFFFF]. */
				::std::uint16_t ioData;
				/** Timeout time counter. */
				::std::uint16_t tCount;
				/** Counter for communication data. */
				::std::uint32_t cCount;
				/** Reply data type designation. */
				::std::uint16_t recvType1;
				/** Reservation. */
				::std::uint16_t reserve1;
				/** Position data. */
				Data dat1;
				/** Reply data type designation. */
				::std::uint16_t recvType2;
				/** Reservation. */
				::std::uint16_t reserve2;
				/** Position data. */
				Data dat2;
				/** Reply data type designation. */
				::std::uint16_t recvType3;
				/** Reservation. */
				::std::uint16_t reserve3;
				/** Position data. */
				Data dat3;
			};
			
		private:
			/** Client ip address or hostname. */
			::std::string addressClient;
			
			/** Server ip address or hostname. */
			::std::string addressServer;
			
			/** Internal filter value. */
			::std::size_t filter;
			
			::std::uint16_t haltIoData;
			
			Command in;
			
			Mode mode;
			
			Command out;
			
			MitsubishiR3 r3;
			
			::std::uint16_t releaseIoData;
			
			::std::uint16_t shutIoData;
			
			Socket socket;
		};
	}
}

#endif // RL_HAL_MITSUBISHIH7_H
