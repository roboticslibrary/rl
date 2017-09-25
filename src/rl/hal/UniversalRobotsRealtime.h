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

#ifndef RL_HAL_UNIVERSALROBOTSREALTIME_H
#define RL_HAL_UNIVERSALROBOTSREALTIME_H

#include <cstdint>

#include "CartesianForceSensor.h"
#include "CartesianPositionSensor.h"
#include "CartesianVelocitySensor.h"
#include "CyclicDevice.h"
#include "DigitalInputReader.h"
#include "DigitalOutputReader.h"
#include "Endian.h"
#include "JointCurrentSensor.h"
#include "JointPositionSensor.h"
#include "JointVelocitySensor.h"
#include "Socket.h"

namespace rl
{
	namespace hal
	{
		/**
		 * Universal Robots realtime interface (pre-3.0, 3.0, 3.1, 3.2).
		 */
		class UniversalRobotsRealtime :
			public CartesianForceSensor,
			public CartesianPositionSensor,
			public CartesianVelocitySensor,
			public CyclicDevice,
			public DigitalInputReader,
			public DigitalOutputReader,
			public JointCurrentSensor,
			public JointPositionSensor,
			public JointVelocitySensor
		{
		public:
			UniversalRobotsRealtime(const ::std::string& address);
			
			virtual ~UniversalRobotsRealtime();
			
			void close();
			
			void doScript(const ::std::string& script);
			
			::rl::math::ForceVector getCartesianForce() const;
			
			::rl::math::Transform getCartesianPosition() const;
			
			::rl::math::MotionVector getCartesianVelocity() const;
			
			::boost::dynamic_bitset<> getDigitalInput() const;
			
			bool getDigitalInput(const ::std::size_t& i) const;
			
			::std::size_t getDigitalInputCount() const;
			
			::boost::dynamic_bitset<> getDigitalOutput() const;
			
			bool getDigitalOutput(const ::std::size_t& i) const;
			
			::std::size_t getDigitalOutputCount() const;
			
			::rl::math::Vector getJointCurrent() const;
			
			::rl::math::Vector getJointPosition() const;
			
			::rl::math::Vector getJointVelocity() const;
			
			void open();
			
			void start();
			
			void step();
			
			void stop();
			
		protected:
			
		private:
			struct Message
			{
				void unserialize(::std::uint8_t* ptr);
				
				template<typename T>
				void unserialize(::std::uint8_t*& ptr, T& t)
				{
					::std::memcpy(&t, ptr, sizeof(t));
					Endian::bigToHost(t);
					ptr += sizeof(t);
				}
				
				template<typename T, ::std::size_t N>
				void unserialize(::std::uint8_t*& ptr, T (&t)[N])
				{
					for (::std::size_t i = 0; i < N; ++i)
					{
						this->unserialize(ptr, t[i]);
					}
				}
				
				::std::uint32_t messageSize;
				
				double time;
				
				double qTarget[6];
				
				double qdTarget[6];
				
				double qddTarget[6];
				
				double iTarget[6];
				
				double mTarget[6];
				
				double qActual[6];
				
				double qdActual[6];
				
				double iActual[6];
				
				double iControl[6];
				
				double toolVectorActual[6];
				
				double tcpSpeedActual[6];
				
				double tcpForce[6];
				
				double toolVectorTarget[6];
				
				double tcpSpeedTarget[6];
				
				::std::int64_t digitalInputBits;
				
				double motorTemperatures[6];
				
				double controllerTimer;
				
				double testValue;
				
				double robotMode;
				
				double jointModes[6];
				
				double safetyMode;
				
				double toolAccelerometerValues[3];
				
				double speedScaling;
				
				double linearMomentumNorm;
				
				double vMain;
				
				double vRobot;
				
				double iRobot;
				
				double vActual[6];
				
				::std::int64_t digitalOutputs;
				
				double programState;
			};
			
			Message in;
			
			Socket socket;
		};
		
		template<>
		void UniversalRobotsRealtime::Message::unserialize(::std::uint8_t*& ptr, ::std::int64_t& t);
	}
}

#endif // RL_HAL_UNIVERSALROBOTSREALTIME_H
