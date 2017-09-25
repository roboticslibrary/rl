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

#include <cassert>
#include <cstdio>
#include <cstring>
#include <sstream>

#include "MitsubishiR3.h"

#ifdef WIN32
#ifndef snprintf
#define snprintf _snprintf_s
#endif // snprintf
#ifndef strncpy_s
#define strncpy strncpy_s
#endif // strncpy_s
#endif // WIN32

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
		
		void
		MitsubishiR3::doCalib(CalibState& state)
		{
			char buf[256] = "1;1;CALIB";
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'o' == buf[1] && 'K' == buf[2])
			{
				if ('N' == buf[3])
				{
					state.isDefined = true;
				}
				else
				{
					state.isDefined = false;
				}
				
				char hex[2] = {buf[4], '\0'};
				int dec = ::strtol(hex, nullptr, 16);
				// 00000001
				state.isAxis[0] = (dec & 1) ? true : false;
				// 00000010
				state.isAxis[1] = (dec & 2) ? true : false;
				// 00000100
				state.isAxis[2] = (dec & 4) ? true : false;
				// 00001000
				state.isAxis[3] = (dec & 8) ? true : false;
				hex[0] = buf[5];
				dec = ::strtol(hex, nullptr, 16);
				// 00010000
				state.isAxis[4] = (dec & 1) ? true : false;
				// 00100000
				state.isAxis[5] = (dec & 2) ? true : false;
				// 01000000
				state.isAxis[6] = (dec & 4) ? true : false;
				// 10000000
				state.isAxis[7] = (dec & 8) ? true : false;
			}
			else if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiR3::doCntl(const bool& doOn)
		{
			char buf[256];
			
			::snprintf(buf, 255, "1;1;CNTL%s", (doOn ? "ON" : "OFF"));
			buf[255] = '\0';
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiR3::doDatinst(const ::std::string& j1, const ::std::string& j2, const ::std::string& j3, const ::std::string& j4, const ::std::string& j5, const ::std::string& j6, const ::std::string& checksum)
		{
			char buf[256];
			
			::snprintf(buf, 255, "1;1;DATINST%s;%s;%s;%s;%s;%s;%s", j1.c_str(), j2.c_str(), j3.c_str(), j4.c_str(), j5.c_str(), j6.c_str(), checksum.c_str());
			buf[255] = '\0';
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiR3::doDatinst(const ::std::string& j1, const ::std::string& j2, const ::std::string& j3, const ::std::string& j4, const ::std::string& j5, const ::std::string& j6, const ::std::string& j7, const ::std::string& j8, const ::std::string& checksum)
		{
			char buf[256];
			
			::snprintf(buf, 255, "1;1;DATINST%s;%s;%s;%s;%s;%s;%s;%s;%s", j1.c_str(), j2.c_str(), j3.c_str(), j4.c_str(), j5.c_str(), j6.c_str(), j7.c_str(), j8.c_str(), checksum.c_str());
			buf[255] = '\0';
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		MitsubishiR3::StopState
		MitsubishiR3::doDstate()
		{
			StopState state;
			
			char buf[256] = "1;1;DSTATE";
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'o' == buf[1] && 'K' == buf[2])
			{
				char* elem = buf + 2;
				char* semicolon = nullptr;
				int i = 0;
				
				while (nullptr != elem)
				{
					++elem;
					semicolon = ::strchr(elem, ';');
					
					if (nullptr != semicolon)
					{
						buf[semicolon - buf] = '\0';
					}
					
					switch (i)
					{
					case 0:
						// run status
						{
							char hex[2] = {elem[1], '\0'};
							int dec = ::strtol(hex, nullptr, 16);
							// 00000001
							state.runSts.isRepeat = (dec & 1) ? true : false;
							// 00000010
							state.runSts.isCycleStopOff = (dec & 2) ? true : false;
							// 00000100
							state.runSts.isMlockOn = (dec & 4) ? true : false;
							// 00001000
							state.runSts.isTeach = (dec & 8) ? true : false;
							hex[0] = elem[0];
							dec = ::strtol(hex, nullptr, 16);
							// 00010000
							state.runSts.isTeachRunning = (dec & 1) ? true : false;
							// 00100000
							state.runSts.isServoOn = (dec & 2) ? true : false;
							// 01000000
							state.runSts.isRun = (dec & 4) ? true : false;
							// 10000000
							state.runSts.isOperationEnable = (dec & 8) ? true : false;
						}
						// stop status
						{
							char hex[2] = {elem[1], '\0'};
							int dec = ::strtol(hex, nullptr, 16);
							// 00000001
							state.stopSts.isEmgStop = (dec & 1) ? true : false;
							// 00000010
							state.stopSts.isStop = (dec & 2) ? true : false;
							// 00000100
							state.stopSts.isWait = (dec & 4) ? true : false;
							// 00001000
							state.stopSts.isStopSignalOff = (dec & 8) ? true : false;
							hex[0] = elem[0];
							dec = ::strtol(hex, nullptr, 16);
							// 00010000
							state.stopSts.isProgramSelectEnable = (dec & 1) ? true : false;
							// 00100000
							state.stopSts.isReserve = (dec & 2) ? true : false;
							// 01000000
							state.stopSts.isPseudoInput = (dec & 4) ? true : false;
						}
						// error number
						{
							elem += 4;
							state.errorNo = ::atoi(elem);
						}
						break;
					case 1:
						// execution step number
						state.stepNo = ::atoi(elem);
						break;
					case 2:
						// mech number under use
						state.mechNo = ::atoi(elem);
						break;
					default:
						break;
					}
					
					elem = semicolon;
					++i;
				}
			}
			else if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
			
			return state;
		}
		
		void
		MitsubishiR3::doEclr()
		{
			char buf[256] = "1;9;ECLR";
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiR3::doEmdat(const ::std::string& program)
		{
			// max. 256 - 9 (::std::strlen("1;9;EMDAT")) - 1 = 246 chars
			
			::std::size_t begin = 0;
			char buf[256];
			::std::size_t end = 0;
			::std::string lines;
			
			while (begin < program.length())
			{
				lines = program.substr(begin, 246);
				
				if (lines.length() < 246)
				{
					if ("\v" == lines.substr(lines.length() - 1))
					{
						lines = lines.substr(0, lines.length() - 1);
					}
					
					end = program.length();
				}
				else
				{
					end = lines.find_last_of('\v');
					lines = lines.substr(0, end);
				}
				
				lines = "1;9;EMDAT" + lines;
				
				this->socket.send(lines.c_str(), lines.length());
				this->socket.recv(buf, 256);
				
				if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
				{
					char* errorNo = nullptr;
					char* errorLine = nullptr;
					char* charPos = nullptr;
					
					char* elem = buf + 2;
					char* semicolon = nullptr;
					int i = 0;
					
					while (nullptr != elem)
					{
						++elem;
						semicolon = ::strchr(elem, ';');
						
						if (nullptr != semicolon)
						{
							buf[semicolon - buf] = '\0';
						}
						
						switch (i)
						{
						case 0:
							// error number
							errorNo = elem;
							break;
						case 1:
							// error line
							errorLine = elem;
							break;
						case 2:
							// character position
							charPos = elem;
							break;
						default:
							break;
						}
						
						elem = semicolon;
						++i;
					}
					
					throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
				}
				
				begin += end + 1;
			}
		}
		
		::std::string
		MitsubishiR3::doErrormes(const int& errorNo)
		{
			char buf[256];
			
			::snprintf(buf, 255, "1;1;ERRORMES%i", errorNo);
			buf[255] = '\0';
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
			
			return buf + 3;
		}
		
		void
		MitsubishiR3::doExec(const ::std::string& instruction)
		{
			char buf[256];
			
			::snprintf(buf, 255, "1;9;EXEC%s", instruction.c_str());
			buf[255] = '\0';
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiR3::doHnd(const bool& doOpen, const int& handNo)
		{
			assert(0 < handNo && handNo < 9);
			
			char buf[256];
			
			::snprintf(buf, 255, "1;1;HND%s%i", (doOpen ? "ON" : "OFF"), handNo);
			buf[255] = '\0';
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		::std::array<MitsubishiR3::Hand, 8>
		MitsubishiR3::doHndsts()
		{
			::std::array<Hand, 8> state;
			
			char buf[256] = "1;1;HNDSTS";
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'o' == buf[1] && 'K' == buf[2])
			{
				char* elem = buf + 2;
				char* semicolon = nullptr;
				int i = 0;
				int j = 0;
				
				while (nullptr != elem)
				{
					++elem;
					semicolon = ::strchr(elem, ';');
					
					if (nullptr != semicolon)
					{
						buf[semicolon - buf] = '\0';
					}
					
					switch (j)
					{
					case 0:
						// signal number allocated in hand
						state[i].outputNo = ::atoi(elem);
						break;
					case 1:
						// hand output status
						switch (::atoi(elem))
						{
						case -1:
							state[i].handSts = HANDSTS_NOTUSED;
							break;
						case 1:
							state[i].handSts = HANDSTS_OPEN;
							break;
						case 2:
							state[i].handSts = HANDSTS_CLOSED;
							break;
						default:
							break;
						}
						break;
					case 2:
						// hand type
						switch (::atoi(elem))
						{
						case -1:
							state[i].handType = HANDTYPE_NOTUSED;
							break;
						case 0:
							state[i].handType = HANDTYPE_SINGLE;
							break;
						case 1:
							state[i].handType = HANDTYPE_DOUBLE;
							break;
						default:
							break;
						}
						
						++i;
						
						break;
					default:
						break;
					}
					
					elem = semicolon;
					++j;
					j %= 3;
				}
			}
			else if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
			
			return state;
		}
		
		void
		MitsubishiR3::doInEquals(const ::std::size_t& inNo, const ::std::string& inVal)
		{
			char buf[256];
			
			::snprintf(buf, 255, "1;1;IN=%zi;%s", inNo, inVal.c_str());
			buf[255] = '\0';
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiR3::doLoad(const ::std::string& programName)
		{
			char buf[256];
			
			::snprintf(buf, 255, "1;1;LOAD=%s", programName.c_str());
			buf[255] = '\0';
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiR3::doNew()
		{
			char buf[256] = "1;1;NEW";
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiR3::doOutEquals(const ::std::size_t& outNo, const ::std::string& outVal)
		{
			char buf[256];
			
			::snprintf(buf, 255, "1;1;OUT=%zi;%s", outNo, outVal.c_str());
			buf[255] = '\0';
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiR3::doRstalrm()
		{
			char buf[256] = "1;1;RSTALRM";
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiR3::doRstpwr()
		{
			char buf[256] = "1;1;RSTPWR";
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiR3::doRun(const ::std::string& programName, const bool& doModeCycle)
		{
			char buf[256];
			
			::snprintf(buf, 255, "1;1;RUN%s;%i", programName.c_str(), (doModeCycle ? 1 : 0));
			buf[255] = '\0';
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiR3::doSave()
		{
			char buf[256] = "1;1;SAVE";
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiR3::doSlotinit()
		{
			char buf[256] = "1;1;SLOTINIT";
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiR3::doSrv(const bool& doOn)
		{
			char buf[256];
			
			::snprintf(buf, 255, "1;1;SRV%s", (doOn ? "ON" : "OFF"));
			buf[255] = '\0';
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		MitsubishiR3::RunState
		MitsubishiR3::doState()
		{
			RunState state;
			
			char buf[256] = "1;1;STATE";
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'o' == buf[1] && 'K' == buf[2])
			{
				char* elem = buf + 2;
				char* semicolon = nullptr;
				int i = 0;
				
				while (nullptr != elem)
				{
					++elem;
					semicolon = ::strchr(elem, ';');
					
					if (nullptr != semicolon)
					{
						buf[semicolon - buf] = '\0';
					}
					
					switch (i)
					{
					case 0:
						// program name loaded into task slot
						::strncpy(state.programName, elem, 255);
						state.programName[255] = '\0';
						break;
					case 1:
						// execution line number
						state.lineNo = ::atoi(elem);
						break;
					case 2:
						// a present override value is read
						state.override = ::atoi(elem);
						break;
					case 3:
						// edit status
						{
							int dec = ::strtol(elem, nullptr, 16);
							// 00000001
							state.editSts.isEditing = (dec & 1) ? true : false;
							// 00000010
							state.editSts.isRunning = (dec & 2) ? true : false;
							// 00000100
							state.editSts.isChanged = (dec & 4) ? true : false;
						}
						break;
					case 4:
						// run status
						{
							char hex[2] = {elem[1], '\0'};
							int dec = ::strtol(hex, nullptr, 16);
							// 00000001
							state.runSts.isRepeat = (dec & 1) ? true : false;
							// 00000010
							state.runSts.isCycleStopOff = (dec & 2) ? true : false;
							// 00000100
							state.runSts.isMlockOn = (dec & 4) ? true : false;
							// 00001000
							state.runSts.isTeach = (dec & 8) ? true : false;
							hex[0] = elem[0];
							dec = ::strtol(hex, nullptr, 16);
							// 00010000
							state.runSts.isTeachRunning = (dec & 1) ? true : false;
							// 00100000
							state.runSts.isServoOn = (dec & 2) ? true : false;
							// 01000000
							state.runSts.isRun = (dec & 4) ? true : false;
							// 10000000
							state.runSts.isOperationEnable = (dec & 8) ? true : false;
						}
						// stop status
						{
							char hex[2] = {elem[3], '\0'};
							int dec = ::strtol(hex, nullptr, 16);
							// 00000001
							state.stopSts.isEmgStop = (dec & 1) ? true : false;
							// 00000010
							state.stopSts.isStop = (dec & 2) ? true : false;
							// 00000100
							state.stopSts.isWait = (dec & 4) ? true : false;
							// 00001000
							state.stopSts.isStopSignalOff = (dec & 8) ? true : false;
							hex[0] = elem[2];
							dec = ::strtol(hex, nullptr, 16);
							// 00010000
							state.stopSts.isProgramSelectEnable = (dec & 1) ? true : false;
							// 00100000
							state.stopSts.isReserve = (dec & 2) ? true : false;
							// 01000000
							state.stopSts.isPseudoInput = (dec & 4) ? true : false;
						}
						// error number
						{
							state.errorNo = ::atoi(elem + 4);
						}
						break;
					case 5:
						// execution step number
						state.stepNo = ::atoi(elem);
						break;
					case 6:
						// mech info
						{
							int dec = ::strtol(elem, nullptr, 16);
							// 00000001
							state.mechInfo[0] = (dec & 1) ? true : false;
							// 00000010
							state.mechInfo[1] = (dec & 2) ? true : false;
							// 00000100
							state.mechInfo[2] = (dec & 4) ? true : false;
						}
						break;
					case 14:
						// program name of slot table
						::strncpy(state.taskPrgName, elem, 255);
						state.taskPrgName[255] = '\0';
						break;
					case 15:
						// operation mode of slot table
						if (0 == ::std::strcmp(elem, "CYC"))
						{
							state.isTaskModeCycle = true;
						}
						else
						{
							state.isTaskModeCycle = false;
						}
						break;
					case 16:
						// stating conditions of slot table
						if (0 == ::std::strcmp(elem, "START"))
						{
							state.taskCond = TASKCOND_START;
						}
						else if (0 == ::std::strcmp(elem, "ALWAYS"))
						{
							state.taskCond = TASKCOND_ALWAYS;
						}
						else if (0 == ::std::strcmp(elem, "ERROR"))
						{
							state.taskCond = TASKCOND_ERROR;
						}
						break;
					case 17:
						// priority of slot table
						state.taskPri = ::atoi(elem);
						break;
					case 18:
						// mech number under use
						state.mechNo = ::atoi(elem);
						break;
					default:
						break;
					}
					
					elem = semicolon;
					++i;
				}
			}
			else if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
			
			return state;
		}
		
		void
		MitsubishiR3::doStop()
		{
			char buf[256] = "1;1;STOP";
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		MitsubishiR3::StopSignalState
		MitsubishiR3::doStpsig()
		{
			StopSignalState state;
			
			char buf[256] = "1;1;STPSIG";
			
			this->socket.send(buf, ::std::strlen(buf));
			this->socket.recv(buf, 256);
			
			if ('Q' == buf[0] && 'o' == buf[1] && 'K' == buf[2])
			{
				char* elem = buf + 2;
				char* semicolon = nullptr;
				int i = 0;
				
				while (nullptr != elem)
				{
					++elem;
					semicolon = ::strchr(elem, ';');
					
					if (nullptr != semicolon)
					{
						buf[semicolon - buf] = '\0';
					}
					
					switch (i)
					{
					case 0:
						// the state of the stop signal
						{
							int dec = ::strtol(elem, nullptr, 16);
							// 00000001
							state.isTb = (dec & 1) ? true : false;
							// 00000010
							state.isPc = (dec & 2) ? true : false;
							// 00000100
							state.isIo = (dec & 4) ? true : false;
							// 00001000
							state.isOp = (dec & 8) ? true : false;
						}
						break;
					case 1:
						// the state of the emg stop signal
						{
							int dec = ::strtol(elem, nullptr, 16);
							// 00000001
							state.isIoEmg = (dec & 1) ? true : false;
							// 00000010
							state.isOpEmg = (dec & 2) ? true : false;
							// 00000100
							state.isTbEmg = (dec & 4) ? true : false;
						}
						break;
					default:
						break;
					}
					
					elem = semicolon;
					++i;
				}
			}
			else if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw Exception(::atoi(errorNo), ::std::string(this->doErrormes(::atoi(errorNo)) + " (" + errorNo + ")"));
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
