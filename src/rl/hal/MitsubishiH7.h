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

#ifndef _RL_HAL_MITSUBISHIH7_H_
#define _RL_HAL_MITSUBISHIH7_H_

#include <string>
#include <rl/math/Transform.h>

#include "CartesianPositionActuator.h"
#include "CartesianPositionSensor.h"
#include "Gripper.h"
#include "JointPositionActuator.h"
#include "JointPositionSensor.h"
#include "types.h"

namespace rl
{
	namespace hal
	{
		class TcpSocket;
		class UdpSocket;
		
		class MitsubishiH7 : public CartesianPositionActuator, public CartesianPositionSensor, public Gripper, public JointPositionActuator, public JointPositionSensor
		{
		public:
			enum Mode
			{
				MODE_JOINT = 1,
				MODE_POSE = 0,
				MODE_PULSE = 2
			};
			
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
			
			typedef Hand HandState[8];
			
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
			
			MitsubishiH7(
				const ::std::size_t& dof,
				const ::std::string& server,
				const ::std::string& client,
				const unsigned short int& tcp = 10001,
				const unsigned short int& udp = 10000,
				const Mode& mode = MODE_JOINT,
				const uint16_t& haltIoData = 0x00AA,
				const uint16_t& releaseIoData = 0x00A6,
				const uint16_t& shutIoData = 0x000A9
			);
			
			virtual ~MitsubishiH7();
			
			/**
			 * The install status is read.
			 * @param state install state
			 */
			void calibCmd(CalibState& state) const;
			
			void close();
			
			/**
			 * Operation enable or disable.
			 * When the command which needs the operation right such as program start,
			 * servo ON and more is used, the operation right should be made effective.
			 * @param doOn OFF / ON
			 */
			void cntlCmd(const bool& doOn) const;
			
			/**
			 * The origin is set by the data input.
			 * @param j1 j1 data
			 * @param j2 j2 data
			 * @param j3 j3 data
			 * @param j4 j4 data
			 * @param j5 j5 data
			 * @param j6 j6 data
			 * @param checksum checksum
			 */
			void datinstCmd(const ::std::string& j1, const ::std::string& j2, const ::std::string& j3, const ::std::string& j4, const ::std::string& j5, const ::std::string& j6, const ::std::string& checksum) const;
			
			/**
			 * The origin is set by the data input.
			 * @param j1 j1 data
			 * @param j2 j2 data
			 * @param j3 j3 data
			 * @param j4 j4 data
			 * @param j5 j5 data
			 * @param j6 j6 data
			 * @param j7 j7 data
			 * @param j8 j8 data
			 * @param checksum checksum
			 */
			void datinstCmd(const ::std::string& j1, const ::std::string& j2, const ::std::string& j3, const ::std::string& j4, const ::std::string& j5, const ::std::string& j6, const ::std::string& j7, const ::std::string& j8, const ::std::string& checksum) const;
			
			/**
			 * The stop state is read.
			 * @param state stop state
			 */
			void dstateCmd(StopState& state) const;
			
			/**
			 * Clear program contents.
			 * It is effective in the edit slot.
			 */
			void eclrCmd() const;
			
			/**
			 * More line and position are registered in the program.
			 * It is effective in the edit slot.
			 * @param program line data and positional data (<line or position>[0b<line or position>...])
			 */
			void emdatCmd(const ::std::string& program) const;
			
			/**
			 * The content of the error is read.
			 * @param errorNo error number
			 * @return content of the error
			 */
			::std::string errormesCmd(const int& errorNo) const;
			
			/**
			 * The instruction is executed directly.
			 * @param instruction instruction of MELFA-BASIC IV or MOVEMASTER commands
			 */
			void execCmd(const ::std::string& instruction) const;
			
			void getCartesianPosition(::rl::math::Transform& x) const;
			
			void getCurrentFeedback(::Eigen::Matrix< int32_t, ::Eigen::Dynamic, 1 >& c) const;
			
			::std::size_t getFilter() const;
			
			uint16_t getIoData() const;
			
			void getJointPosition(::rl::math::Vector& q) const;
			
			Mode getMode() const;
			
			void getMotorPulse(::Eigen::Matrix< int32_t, ::Eigen::Dynamic, 1 >& p) const;
			
			void halt();
			
			/**
			 * The hand is openend and closed.
			 * @param doOpen CLOSE / OPEN
			 * @param handNo hand number (1 - 8) is specified
			 */
			void hndCmd(const bool& doOpen, const int& handNo) const;
			
			/**
			 * The setting and the output of the hand are read.
			 * @param state hand state
			 */
			void hndstsCmd(HandState& state) const;
			
			/**
			 * The input signal is pseudo-input.
			 * @param inNo input signal number
			 * @param inVal pseudo-input signal value by 4 hex number fixation
			 */
			void inEqualsCmd(const ::std::size_t& inNo, const ::std::string& inVal) const;
			
			/**
			 * Open the program for edit.
			 * @param programName edit program name
			 */
			void loadCmd(const ::std::string& programName) const;
			
			void loadProgram(const ::std::string& name, const ::std::string& program) const;
			
			/**
			 * The program is closed annulling the content of the edit.
			 */
			void newCmd() const;
			
			void open();
			
			/**
			 * The output signal is compelling output.
			 * @param outNo output signal number
			 * @param outVal output signal value by 4 hex number fixation
			 */
			void outEqualsCmd(const ::std::size_t& outNo, const ::std::string& outVal) const;
			
			void release();
			
			/**
			 * The error is reset.
			 */
			void rstalrmCmd() const;
			
			/**
			 * Power supply reset (reboot) of the controller is executed.
			 */
			void rstpwrCmd() const;
			
			/**
			 * The program is started.
			 * @param programName program name
			 * @param doModeCycle repeat start / cycle start
			 */
			void runCmd(const ::std::string& programName, const bool& doModeCycle = false) const;
			
			/**
			 * The content of the edit is preserved and the program is closed.
			 */
			void saveCmd() const;
			
			void setCartesianPosition(const ::rl::math::Transform& x);
			
			void setFilter(const ::std::size_t& filter);
			
			void setInput(const uint16_t& bitTop);
			
			void setJointPosition(const ::rl::math::Vector& q);
			
			void setMode(const Mode& mode);
			
			void setMotorPulse(const ::Eigen::Matrix< int32_t, ::Eigen::Dynamic, 1 >& p);
			
			void setOutput(const uint16_t& bitTop, const uint16_t& bitMask, const uint16_t& ioData);
			
			void shut();
			
			/**
			 * The program resets all slots.
			 */
			void slotinitCmd() const;
			
			/**
			 * The servo power supply is turned on and off.
			 * @param doOn OFF / ON
			 */
			void srvCmd(const bool& doOn) const;
			
			void start();
			
			void startProgram(const ::std::string& name) const;
			
			/**
			 * The run state is read.
			 * @param state run state
			 */
			void stateCmd(RunState& state) const;
			
			void step();
			
			/**
			 * The start is stopped.
			 */
			void stopCmd() const;
			
			void stop();
			
			void stopProgram() const;
			
			/**
			 * The state of the stop signal is read.
			 * @param state stop signal state
			 */
			void stpsigCmd(StopSignalState& state) const;
			
		protected:
			struct MxtWorld
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
			
			struct MxtJoint
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
			
			struct MxtPose
			{
				MxtWorld w;
				/** Structural flag 1. */
				uint32_t sflg1;
				/** Structural flag 2. */
				uint32_t sflg2;
			};
			
			struct MxtPulse
			{
				/** Motor 1 axis. */
				int32_t p1;
				/** Motor 2 axis. */
				int32_t p2;
				/** Motor 3 axis. */
				int32_t p3;
				/** Motor 4 axis. */
				int32_t p4;
				/** Motor 5 axis. */
				int32_t p5;
				/** Motor 6 axis. */
				int32_t p6;
				/** Additional axis 1 (Motor 7 axis). */
				int32_t p7;
				/** Additional axis 2 (Motor 8 axis). */
				int32_t p8;
			};
			
			struct MxtCommand
			{
				union MxtData
				{
					/** XYZ type [mm or rad]. */
					MxtPose pos;
					/** Joint type [rad]. */
					MxtJoint jnt;
					/** Motor pulse type [the pulse] or current type [\%]. */
					MxtPulse pls;
					/** Integer type [%% / non-unit]. */
					int32_t lng[8];
				};
				/** Command. */
				uint16_t command;
				/** Transmission data type designation. */
				uint16_t sendType;
				/** Reply data type designation. */
				uint16_t recvType;
				/** Reservation. */
				uint16_t reserve;
				/** Position data. */
				MxtData dat;
				/** Transmission input/output signal data designation. */
				uint16_t sendIoType;
				/** Reply input/output signal data designation. */
				uint16_t recvIoType;
				/** Head bit no. of input or output signal. */
				uint16_t bitTop;
				/** Bit mask pattern designation (valid only for commanding) [0x0000-0xFFFF]. */
				uint16_t bitMask;
				/** Input or output signal data value (for monitoring) [0x0000-0xFFFF]. Output signal data value (for commanding) [0x0000-0xFFFF]. */
				uint16_t ioData;
				/** Timeout time counter. */
				uint16_t tCount;
				/** Counter for communication data. */
				uint32_t cCount;
				/** Reply data type designation. */
				uint16_t recvType1;
				/** Reservation. */
				uint16_t reserve1;
				/** Position data. */
				MxtData dat1;
				/** Reply data type designation. */
				uint16_t recvType2;
				/** Reservation. */
				uint16_t reserve2;
				/** Position data. */
				MxtData dat2;
				/** Reply data type designation. */
				uint16_t recvType3;
				/** Reservation. */
				uint16_t reserve3;
				/** Position data. */
				MxtData dat3;
			};
			
		private:
			/** Client ip address or hostname. */
			::std::string client;
			
			/** Internal filter value. */
			::std::size_t filter;
			
			uint16_t haltIoData;
			
			MxtCommand in;
			
			Mode mode;
			
			MxtCommand out;
			
			uint16_t releaseIoData;
			
			/** Server ip address or hostname. */
			::std::string server;
			
			uint16_t shutIoData;
			
			TcpSocket* tcp;
			
			UdpSocket* udp;
		};
	}
}

#endif // _RL_HAL_MITSUBISHIH7_H_
