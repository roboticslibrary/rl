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
		class UniversalRobotsDashboard : public Com
		{
		public:
			enum ProgramState
			{
				PROGRAM_STATE_STOPPED,
				PROGRAM_STATE_PLAYING,
				PROGRAM_STATE_PAUSED
			};
			
			enum RobotMode
			{
				ROBOT_MODE_NO_CONTROLLER,
				ROBOT_MODE_DISCONNECTED,
				ROBOT_MODE_CONFIRM_SAFETY,
				ROBOT_MODE_BOOTING,
				ROBOT_MODE_POWER_OFF,
				ROBOT_MODE_POWER_ON,
				ROBOT_MODE_IDLE,
				ROBOT_MODE_BACKDRIVE,
				ROBOT_MODE_RUNNING
			};
			
			enum SafetyMode
			{
				SAFETY_MODE_NORMAL,
				SAFETY_MODE_REDUCED,
				SAFETY_MODE_PROTECTIVE_STOP,
				SAFETY_MODE_RECOVERY,
				SAFETY_MODE_SAFEGUARD_STOP,
				SAFETY_MODE_SYSTEM_EMERGENCY_STOP,
				SAFETY_MODE_ROBOT_EMERGENCY_STOP,
				SAFETY_MODE_VIOLATION,
				SAFETY_MODE_FAULT
			};
			
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
