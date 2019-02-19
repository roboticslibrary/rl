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

#include <array>
#include <bitset>
#include <cassert>
#include <boost/algorithm/string.hpp>

#include "MitsubishiR3.h"

namespace rl
{
	namespace hal
	{
		MitsubishiR3::MitsubishiR3(
			const ::std::string& address,
			const unsigned short int& port
		) :
			Com(),
			socket(Socket::Tcp(Socket::Address::Ipv4(address, port)))
		{
		}
		
		MitsubishiR3::~MitsubishiR3()
		{
		}
		
		void
		MitsubishiR3::close()
		{
			this->socket.close();
			this->setConnected(false);
		}
		
		MitsubishiR3::CalibState
		MitsubishiR3::doCalib()
		{
			this->send("1;1;CALIB");
			::std::string reply = this->recv();
			
			CalibState state;
			
			if ("QoK" == reply.substr(0, 3))
			{
				if ("N" == reply.substr(3, 1))
				{
					state.isDefined = true;
				}
				else
				{
					state.isDefined = false;
				}
				
				::std::bitset<8> isAxis(::std::stoi(reply.substr(4, 2), nullptr, 16));
				
				for (::std::size_t i = 0; i < 8; ++i)
				{
					state.isAxis[i] = isAxis[i];
				}
			}
			
			return state;
		}
		
		void
		MitsubishiR3::doCntl(const bool& doOn)
		{
			this->send("1;1;CNTL" + ::std::string(doOn ? "ON" : "OFF"));
			this->recv();
		}
		
		void
		MitsubishiR3::doDatinst(const ::std::string& j1, const ::std::string& j2, const ::std::string& j3, const ::std::string& j4, const ::std::string& j5, const ::std::string& j6, const ::std::string& checksum)
		{
			this->send("1;1;DATINST" + j1 + ";" + j2 + ";" + j3 + ";" + j4 + ";" + j5 + ";" + j6 + ";" + checksum);
			this->recv();
		}
		
		void
		MitsubishiR3::doDatinst(const ::std::string& j1, const ::std::string& j2, const ::std::string& j3, const ::std::string& j4, const ::std::string& j5, const ::std::string& j6, const ::std::string& j7, const ::std::string& j8, const ::std::string& checksum)
		{
			this->send("1;1;DATINST" + j1 + ";" + j2 + ";" + j3 + ";" + j4 + ";" + j5 + ";" + j6 + ";" + j7 + ";" + j8 + ";" + checksum);
			this->recv();
		}
		
		MitsubishiR3::StopState
		MitsubishiR3::doDstate()
		{
			this->send("1;1;DSTATE");
			::std::string reply = this->recv();
			
			StopState state;
			
			if ("QoK" == reply.substr(0, 3))
			{
				::std::bitset<8> runSts(::std::stoi(reply.substr(3, 2), nullptr, 16));
				
				state.runSts.isRepeat = runSts[0];
				state.runSts.isCycleStopOff = runSts[1];
				state.runSts.isMlockOn = runSts[2];
				state.runSts.isTeach = runSts[3];
				state.runSts.isTeachRunning = runSts[4];
				state.runSts.isServoOn = runSts[5];
				state.runSts.isRun = runSts[6];
				state.runSts.isOperationEnable = runSts[7];
				
				::std::bitset<8> stopSts(::std::stoi(reply.substr(5, 2), nullptr, 16));
				
				state.stopSts.isEmgStop = stopSts[0];
				state.stopSts.isStop = stopSts[1];
				state.stopSts.isWait = stopSts[2];
				state.stopSts.isStopSignalOff = stopSts[3];
				state.stopSts.isProgramSelectEnable = stopSts[4];
				state.stopSts.isPseudoInput = stopSts[6];
				
				::std::vector< ::std::string> tokens;
				::std::string tmp = reply.substr(7);
				::boost::split(tokens, tmp, ::boost::is_any_of(";"));
				
				state.errorNo = ::std::stoi(tokens[0]);
				state.stepNo = ::std::stoi(tokens[1]);
				state.mechNo = ::std::stoi(tokens[2]);
			}
			
			return state;
		}
		
		void
		MitsubishiR3::doEclr()
		{
			this->send("1;9;ECLR");
			this->recv();
		}
		
		void
		MitsubishiR3::doEmdat(const ::std::string& program)
		{
			// max. 256 - (9 = strlen("1;9;EMDAT")) - 1 = 246 chars
			
			for (::std::size_t begin = 0, end = 0; begin < program.length(); begin += end + 1)
			{
				::std::string lines = program.substr(begin, 246);
				
				if (lines.length() < 246)
				{
					end = program.length();
					
					if ("\v" == lines.substr(lines.length() - 1))
					{
						lines = lines.substr(0, lines.length() - 1);
					}
				}
				else
				{
					end = lines.find_last_of('\v');
					lines = lines.substr(0, end);
				}
				
				this->send("1;9;EMDAT" + lines);
				this->recv();
			}
		}
		
		::std::string
		MitsubishiR3::doErrormes(const int& errorNo)
		{
			this->send("1;1;ERRORMES" + ::std::to_string(errorNo));
			::std::string reply = this->recv();
			return reply.substr(3);
		}
		
		void
		MitsubishiR3::doExec(const ::std::string& instruction)
		{
			this->send("1;9;EXEC" + instruction);
			this->recv();
		}
		
		void
		MitsubishiR3::doHnd(const bool& doOpen, const int& handNo)
		{
			assert(0 < handNo && handNo < 9);
			this->send("1;1;HND" + ::std::string(doOpen ? "ON" : "OFF") + ::std::to_string(handNo));
			this->recv();
		}
		
		::std::array<MitsubishiR3::Hand, 8>
		MitsubishiR3::doHndsts()
		{
			this->send("1;1;HNDSTS");
			::std::string reply = this->recv();
			
			::std::array<Hand, 8> state;
			
			if ("QoK" == reply.substr(0, 3))
			{
				::std::array<Hand, 8> state;
				
				::std::vector< ::std::string> tokens;
				::std::string tmp = reply.substr(3);
				::boost::split(tokens, tmp, ::boost::is_any_of(";"));
				
				for (::std::size_t i = 0; i < 8; ++i)
				{
					state[i].outputNo = ::std::stoi(tokens[i * 3 + 0]);
					state[i].handSts = static_cast<HandSts>(::std::stoi(tokens[i * 3 + 1]));
					state[i].handType = static_cast<HandType>(::std::stoi(tokens[i * 3 + 2]));
				}
			}
			
			return state;
		}
		
		void
		MitsubishiR3::doInEquals(const ::std::size_t& inNo, const ::std::string& inVal)
		{
			this->send("1;1;IN=" + ::std::to_string(inNo) + ";" + inVal);
			this->recv();
		}
		
		void
		MitsubishiR3::doLoad(const ::std::string& programName)
		{
			this->send("1;1;LOAD=" + programName);
			this->recv();
		}
		
		void
		MitsubishiR3::doNew()
		{
			this->send("1;1;NEW");
			this->recv();
		}
		
		void
		MitsubishiR3::doOutEquals(const ::std::size_t& outNo, const ::std::string& outVal)
		{
			this->send("1;1;OUT=" + ::std::to_string(outNo) + ";" + outVal);
			this->recv();
		}
		
		void
		MitsubishiR3::doRstalrm()
		{
			this->send("1;1;RSTALRM");
			this->recv();
		}
		
		void
		MitsubishiR3::doRstpwr()
		{
			this->send("1;1;RSTPWR");
			this->recv();
		}
		
		void
		MitsubishiR3::doRun(const ::std::string& programName, const bool& doModeCycle)
		{
			this->send("1;1;RUN" + programName + ";" + ::std::string(doModeCycle ? "1" : "0"));
			this->recv();
		}
		
		void
		MitsubishiR3::doSave()
		{
			this->send("1;1;SAVE");
			this->recv();
		}
		
		void
		MitsubishiR3::doSlotinit()
		{
			this->send("1;1;SLOTINIT");
			this->recv();
		}
		
		void
		MitsubishiR3::doSrv(const bool& doOn)
		{
			this->send("1;1;SRV" + ::std::string(doOn ? "ON" : "OFF"));
			this->recv();
		}
		
		MitsubishiR3::RunState
		MitsubishiR3::doState()
		{
			this->send("1;1;STATE");
			::std::string reply = this->recv();
			
			RunState state;
			
			if ("QoK" == reply.substr(0, 3))
			{
				::std::vector< ::std::string> tokens;
				::std::string tmp = reply.substr(3);
				::boost::split(tokens, tmp, ::boost::is_any_of(";"));
				
				state.programName = tokens[0];
				
				state.lineNo = ::std::stoi(tokens[1]);
				state.override = ::std::stoi(tokens[2]);
				
				::std::bitset<8> editSts(::std::stoi(tokens[3], nullptr, 16));
				
				state.editSts.isEditing = editSts[0];
				state.editSts.isRunning = editSts[1];
				state.editSts.isChanged = editSts[2];
				
				::std::bitset<8> runSts(::std::stoi(tokens[4].substr(0, 2), nullptr, 16));
				
				state.runSts.isRepeat = runSts[0];
				state.runSts.isCycleStopOff = runSts[1];
				state.runSts.isMlockOn = runSts[2];
				state.runSts.isTeach = runSts[3];
				state.runSts.isTeachRunning = runSts[4];
				state.runSts.isServoOn = runSts[5];
				state.runSts.isRun = runSts[6];
				state.runSts.isOperationEnable = runSts[7];
				
				::std::bitset<8> stopSts(::std::stoi(tokens[4].substr(2, 2), nullptr, 16));
				
				state.stopSts.isEmgStop = stopSts[0];
				state.stopSts.isStop = stopSts[1];
				state.stopSts.isWait = stopSts[2];
				state.stopSts.isStopSignalOff = stopSts[3];
				state.stopSts.isProgramSelectEnable = stopSts[4];
				state.stopSts.isPseudoInput = stopSts[6];
				
				state.errorNo = ::std::stoi(tokens[4].substr(4));
				state.stepNo = ::std::stoi(tokens[5]);
				
				::std::bitset<8> mechInfo(::std::stoi(tokens[6], nullptr, 16));
				
				for (::std::size_t i = 0; i < 3; ++i)
				{
					state.mechInfo[i] = mechInfo[i];
				}
				
				state.taskPrgName = tokens[14];
				
				if ("CYC" == tokens[15])
				{
					state.isTaskModeCycle = true;
				}
				else
				{
					state.isTaskModeCycle = false;
				}
				
				if ("START" == tokens[16])
				{
					state.taskCond = TASKCOND_START;
				}
				else if ("ALWAYS" == tokens[16])
				{
					state.taskCond = TASKCOND_ALWAYS;
				}
				else if ("ERROR" == tokens[16])
				{
					state.taskCond = TASKCOND_ERROR;
				}
				
				state.taskPri = ::std::stoi(tokens[17]);
				state.mechNo = ::std::stoi(tokens[18]);
			}
			
			return state;
		}
		
		void
		MitsubishiR3::doStop()
		{
			this->send("1;1;STOP");
			this->recv();
		}
		
		MitsubishiR3::StopSignalState
		MitsubishiR3::doStpsig()
		{
			this->send("1;1;STPSIG");
			::std::string reply = this->recv();
			
			StopSignalState state;
			
			if ("QoK" == reply.substr(0, 3))
			{
				::std::bitset<4> stopSts(::std::stoi(reply.substr(3, 1), nullptr, 16));
				
				state.isTb = stopSts[0];
				state.isPc = stopSts[1];
				state.isIo = stopSts[2];
				state.isOp = stopSts[3];
				
				::std::bitset<4> emgSts(::std::stoi(reply.substr(5, 1), nullptr, 16));
				
				state.isIoEmg = emgSts[0];
				state.isOpEmg = emgSts[1];
				state.isTbEmg = emgSts[2];
			}
			
			return state;
		}
		
		void
		MitsubishiR3::loadProgram(const ::std::string& name, const ::std::string& program)
		{
			this->doNew();
			this->doLoad(name);
			this->doEclr();
			this->doEmdat(program);
			this->doSave();
		}
		
		void
		MitsubishiR3::open()
		{
			this->socket.open();
			this->socket.connect();
			this->setConnected(true);
		}
		
		::std::string
		MitsubishiR3::recv()
		{
			::std::array<char, 256> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(buffer.data(), size);
			
			if ("QeR" == reply.substr(0, 3))
			{
				int errorNo = ::std::stoi(reply.substr(3));
				throw Exception(errorNo, ::std::string(this->doErrormes(errorNo) + " (" + ::std::to_string(errorNo) + ")"));
			}
			
			return reply;
		}
		
		void
		MitsubishiR3::send(const ::std::string& command)
		{
			this->socket.send(command.c_str(), command.size());
		}
		
		void
		MitsubishiR3::startProgram(const ::std::string& name)
		{
			this->doCntl(true);
			this->doSlotinit();
			this->doRun(name, true);
			this->doCntl(false);
		}
		
		void
		MitsubishiR3::stopProgram()
		{
			this->doStop();
		}
		
		MitsubishiR3::Exception::Exception(const int& errorNo, const ::std::string& what_arg) :
			ComException(what_arg),
			errorNo(errorNo)
		{
		}
		
		MitsubishiR3::Exception::~Exception() throw()
		{
		}
		
		int
		MitsubishiR3::Exception::getErrorNo() const
		{
			return this->errorNo;
		}
	}
}
