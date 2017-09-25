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

#ifndef RL_HAL_MITSUBISHIR3_H
#define RL_HAL_MITSUBISHIR3_H

#include <array>
#include <cstdint>
#include <string>

#include "Com.h"
#include "ComException.h"
#include "Socket.h"

namespace rl
{
	namespace hal
	{
		/**
		 * Mitsubishi Electric R3 protocol.
		 */
		class MitsubishiR3 : public Com
		{
		public:
			enum HandSts
			{
				HANDSTS_NOTUSED = -1,
				/** Hand open. */
				HANDSTS_OPEN = 1,
				/** Hand closed. */
				HANDSTS_CLOSED = 2
			};
			
			enum HandType
			{
				HANDTYPE_NOTUSED = -1,
				/** Single-solenoid. */
				HANDTYPE_SINGLE = 0,
				/** Double-solenoid. */
				HANDTYPE_DOUBLE = 1
			};
			
			enum TaskCond
			{
				/** START. */
				TASKCOND_START,
				/** ALWAYS. */
				TASKCOND_ALWAYS,
				/** ERROR. */
				TASKCOND_ERROR
			};
			
			struct EditSts
			{
				/** Editing. */
				bool isEditing;
				/** Running. */
				bool isRunning;
				/** Changed. */
				bool isChanged;
			};
			
			class Exception : public ComException
			{
			public:
				Exception(const int& errorNo, const ::std::string& what_arg);
				
				virtual ~Exception() throw();
				
				int getErrorNo() const;
				
			protected:
				
			private:
				int errorNo;
			};
			
			struct Hand
			{
				/** Signal number allocated in hand (-1: not used). */
				int outputNo;
				/** Hand output status. */
				HandSts handSts;
				/** Hand type. */
				HandType handType;
			};
			
			/** Mech info (Mech 1, Mech 2, Mech 3). */
			typedef bool MechInfo[3];
			
			struct RunSts
			{
				/** Cycle / Repeat. */
				bool isRepeat;
				/** Cycle stop ON / Cycle stop OFF. */
				bool isCycleStopOff;
				/** MLOCK OFF / MLOCK ON. */
				bool isMlockOn;
				/** Auto / Teach. */
				bool isTeach;
				/** Running of Teach mode. */
				bool isTeachRunning;
				/** Servo OFF / Servo ON. */
				bool isServoOn;
				/** STOP / RUN. */
				bool isRun;
				/** Operation disable / Operation enable. */
				bool isOperationEnable;
			};
			
			struct StopSts
			{
				/** EMG STOP. */
				bool isEmgStop;
				/** STOP. */
				bool isStop;
				/** WAIT. */
				bool isWait;
				/** STOP signal ON /OFF. */
				bool isStopSignalOff;
				/** Program select enable. */
				bool isProgramSelectEnable;
				/** (reserve). */
				bool isReserve;
				/** Pseudo input. */
				bool isPseudoInput;
			};
			
			struct CalibState
			{
				/** Install status. */
				bool isDefined;
				/** Completion axis pattern (J1, J2, J3, J4, J5, J6, J7, J8). */
				bool isAxis[8];
			};
			
			struct RunState
			{
				/** Program name loaded into task slot. */
				char programName[256];
				/** Execution line number. */
				int lineNo;
				/** A present override value is read. */
				int override;
				/** Edit status. */
				EditSts editSts;
				/** Run status. */
				RunSts runSts;
				/** Stop status. */
				StopSts stopSts;
				/** Error number (0: no error). */
				int errorNo;
				/** Execution step number. */
				int stepNo;
				/** Mech info. */
				MechInfo mechInfo;
				/** Program name of slot table. */
				char taskPrgName[256];
				/** Operation mode of slot table (REP / CYC). */
				bool isTaskModeCycle;
				/** Stating conditions of slot table. */
				TaskCond taskCond;
				/** Priority of slot table (1 - 31). */
				int taskPri;
				/** Mech number under use. */
				int mechNo;
			};
			
			struct StopSignalState
			{
				/** T/B (RS-422). */
				bool isTb;
				/** P/C (RS-232C). */
				bool isPc;
				/** I/O. */
				bool isIo;
				/** O/P. */
				bool isOp;
				/** I/O EMG. */
				bool isIoEmg;
				/** O/P EMG. */
				bool isOpEmg;
				/** T/B EMG. */
				bool isTbEmg;
			};
			
			struct StopState
			{
				/** Run status. */
				RunSts runSts;
				/** Stop status. */
				StopSts stopSts;
				/** Error number (0: no error). */
				int errorNo;
				/** Execution step number. */
				int stepNo;
				/** Mech number under use. */
				int mechNo;
			};
			
			MitsubishiR3(
				const ::std::string& address,
				const unsigned short int& port = 10001
			);
			
			virtual ~MitsubishiR3();
			
			void close();
			
			/**
			 * The install status is read.
			 * 
			 * @param[out] state Install state
			 */
			void doCalib(CalibState& state);
			
			/**
			 * Operation enable or disable.
			 * 
			 * When the command which needs the operation right such as program start,
			 * servo ON and more is used, the operation right should be made effective.
			 * 
			 * @param[in] doOn OFF / ON
			 */
			void doCntl(const bool& doOn);
			
			/**
			 * The origin is set by the data input.
			 * 
			 * @param[in] j1 Joint 1 data
			 * @param[in] j2 Joint 2 data
			 * @param[in] j3 Joint 3 data
			 * @param[in] j4 Joint 4 data
			 * @param[in] j5 Joint 5 data
			 * @param[in] j6 Joint 6 data
			 * @param[in] checksum Checksum
			 */
			void doDatinst(const ::std::string& j1, const ::std::string& j2, const ::std::string& j3, const ::std::string& j4, const ::std::string& j5, const ::std::string& j6, const ::std::string& checksum);
			
			/**
			 * The origin is set by the data input.
			 * 
			 * @param[in] j1 Joint 1 data
			 * @param[in] j2 Joint 2 data
			 * @param[in] j3 Joint 3 data
			 * @param[in] j4 Joint 4 data
			 * @param[in] j5 Joint 5 data
			 * @param[in] j6 Joint 6 data
			 * @param[in] j7 Joint 7 data
			 * @param[in] j8 Joint 8 data
			 * @param[in] j8 Joint 8 data
			 * @param[in] checksum Checksum
			 */
			void doDatinst(const ::std::string& j1, const ::std::string& j2, const ::std::string& j3, const ::std::string& j4, const ::std::string& j5, const ::std::string& j6, const ::std::string& j7, const ::std::string& j8, const ::std::string& checksum);
			
			/**
			 * The stop state is read.
			 * 
			 * @param[out] state Stop state
			 */
			StopState doDstate();
			
			/**
			 * Clear program contents.
			 * 
			 * It is effective in the edit slot.
			 */
			void doEclr();
			
			/**
			 * More line and position are registered in the program.
			 * 
			 * It is effective in the edit slot.
			 * 
			 * @param[in] program Line data and positional data (<line or position>[0b<line or position>...])
			 */
			void doEmdat(const ::std::string& program);
			
			/**
			 * The content of the error is read.
			 * 
			 * @param[in] errorNo Error number
			 * @return Content of the error
			 */
			::std::string doErrormes(const int& errorNo);
			
			/**
			 * The instruction is executed directly.
			 * 
			 * @param[in] instruction Instruction of MELFA-BASIC IV or MOVEMASTER commands
			 */
			void doExec(const ::std::string& instruction);
			
			/**
			 * The hand is openend and closed.
			 * 
			 * @param[in] doOpen CLOSE / OPEN
			 * @param[in] handNo Hand number (1 - 8) is specified
			 */
			void doHnd(const bool& doOpen, const int& handNo);
			
			/**
			 * The setting and the output of the hand are read.
			 * 
			 * @param[out] state Hand state
			 */
			::std::array<Hand, 8> doHndsts();
			
			/**
			 * The input signal is pseudo-input.
			 * 
			 * @param[in] inNo Input signal number
			 * @param[in] inVal Pseudo-input signal value by 4 hex number fixation
			 */
			void doInEquals(const ::std::size_t& inNo, const ::std::string& inVal);
			
			/**
			 * Open the program for edit.
			 * 
			 * @param[in] programName Edit program name
			 */
			void doLoad(const ::std::string& programName);
			
			/**
			 * The program is closed annulling the content of the edit.
			 */
			void doNew();
			
			/**
			 * The output signal is compelling output.
			 * 
			 * @param[in] outNo Output signal number
			 * @param[in] outVal Output signal value by 4 hex number fixation
			 */
			void doOutEquals(const ::std::size_t& outNo, const ::std::string& outVal);
			
			/**
			 * The error is reset.
			 */
			void doRstalrm();
			
			/**
			 * Power supply reset (reboot) of the controller is executed.
			 */
			void doRstpwr();
			
			/**
			 * The program is started.
			 * 
			 * @param[in] programName Program name
			 * @param[in] doModeCycle Repeat start / cycle start
			 */
			void doRun(const ::std::string& programName, const bool& doModeCycle = false);
			
			/**
			 * The content of the edit is preserved and the program is closed.
			 */
			void doSave();
			
			/**
			 * The program resets all slots.
			 */
			void doSlotinit();
			
			/**
			 * The servo power supply is turned on and off.
			 * 
			 * @param[in] doOn OFF / ON
			 */
			void doSrv(const bool& doOn);
			
			/**
			 * The run state is read.
			 * 
			 * @param[out] state Run state
			 */
			RunState doState();
			
			/**
			 * The start is stopped.
			 */
			void doStop();
			
			/**
			 * The state of the stop signal is read.
			 * 
			 * @param[out] state Stop signal state
			 */
			StopSignalState doStpsig();
			
			void loadProgram(const ::std::string& name, const ::std::string& program);
			
			void open();
			
			void startProgram(const ::std::string& name);
			
			void stopProgram();
			
		protected:
			
		private:
			Socket socket;
		};
	}
}

#endif // RL_HAL_MITSUBISHIR3_H
