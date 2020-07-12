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

#ifndef RL_HAL_UNIVERSALROBOTSDASHBOARD_H
#define RL_HAL_UNIVERSALROBOTSDASHBOARD_H

#include <string>
#include <utility>

#include "Com.h"
#include "Socket.h"

namespace rl
{
	namespace hal
	{
		/**
		 * Universal Robots dashboard server.
		 */
		class RL_HAL_EXPORT UniversalRobotsDashboard : public Com
		{
		public:
			enum class ProgramState
			{
				stopped,
				playing,
				paused
			};
			
			RL_HAL_DEPRECATED static constexpr ProgramState PROGRAM_STATE_STOPPED = ProgramState::stopped;
			RL_HAL_DEPRECATED static constexpr ProgramState PROGRAM_STATE_PLAYING = ProgramState::playing;
			RL_HAL_DEPRECATED static constexpr ProgramState PROGRAM_STATE_PAUSED = ProgramState::paused;
			
			enum class RobotMode
			{
				noController,
				disconnected,
				confirmSafety,
				booting,
				powerOff,
				powerOn,
				idle,
				backdrive,
				running
			};
			
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_NO_CONTROLLER = RobotMode::noController;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_DISCONNECTED = RobotMode::disconnected;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_CONFIRM_SAFETY = RobotMode::confirmSafety;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_BOOTING = RobotMode::booting;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_POWER_OFF = RobotMode::powerOff;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_POWER_ON = RobotMode::powerOn;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_IDLE = RobotMode::idle;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_BACKDRIVE = RobotMode::backdrive;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_RUNNING = RobotMode::running;
			
			enum class SafetyMode
			{
				normal,
				reduced,
				protectiveStop,
				recovery,
				safeguardStop,
				systemEmergencyStop,
				robotEmergencyStop,
				violation,
				fault
			};
			
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_NORMAL = SafetyMode::normal;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_REDUCED = SafetyMode::reduced;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_PROTECTIVE_STOP = SafetyMode::protectiveStop;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_RECOVERY = SafetyMode::recovery;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_SAFEGUARD_STOP = SafetyMode::safeguardStop;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_SYSTEM_EMERGENCY_STOP = SafetyMode::systemEmergencyStop;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_ROBOT_EMERGENCY_STOP = SafetyMode::robotEmergencyStop;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_VIOLATION = SafetyMode::violation;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_FAULT = SafetyMode::fault;
			
			UniversalRobotsDashboard(const ::std::string& address);
			
			virtual ~UniversalRobotsDashboard();
			
			void close();
			
			void doAddToLog(const ::std::string& message);
			
			void doBrakeRelease();
			
			void doClosePopup();
			
			void doCloseSafetyPopup();
			
			::std::string doGetLoadedProgram();
			
			::std::pair<bool, ::std::string> doIsProgramSaved();
			
			void doLoad(const ::std::string& program);
			
			void doLoadInstallation(const ::std::string& installation);
			
			void doPause();
			
			void doPlay();
			
			::std::string doPolyscopeVersion();
			
			void doPopup(const ::std::string& text);
			
			void doPowerOff();
			
			void doPowerOn();
			
			::std::pair<ProgramState, ::std::string> doProgramState();
			
			RobotMode doRobotmode();
			
			bool doRunning();
			
			SafetyMode doSafetymode();
			
			void doShutdown();
			
			void doStop();
			
			void doQuit();
			
			void doUnlockProtectiveStop();
			
			void open();
			
		protected:
			
		private:
			void send(const ::std::string& command);
			
			Socket socket;
		};
	}
}

#endif // RL_HAL_UNIVERSALROBOTSDASHBOARD_H
