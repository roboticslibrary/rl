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
#include <rl/math/Rotation.h>

#ifdef WIN32
#include <winsock.h>
#else // WIN32
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#endif // WIN32

#include "ComException.h"
#include "DeviceException.h"
#include "Exception.h"
#include "MitsubishiH7.h"
#include "MitsubishiH7Exception.h"
#include "TcpSocket.h"
#include "TimeoutException.h"
#include "UdpSocket.h"

#ifdef WIN32
#ifndef snprintf
#define snprintf _snprintf_s
#endif // snprintf
#ifndef strncpy_s
#define strncpy strncpy_s
#endif // strncpy_s
#endif // WIN32

/** Real-time external command invalid. */
#define MXT_CMD_NULL 0
/** Real-time external command valid. */
#define MXT_CMD_MOVE 1
/** Real-time external command end. */
#define MXT_CMD_END 255

/** No data. */
#define MXT_IO_NULL 0
/** Output signal. */
#define MXT_IO_OUT 1
/** Input signal. */
#define MXT_IO_IN 2

/** No data. */
#define MXT_TYP_NULL 0
/** XYZ data. */
#define MXT_TYP_POSE 1
/** Joint data. */
#define MXT_TYP_JOINT 2
/** Motor pulse data. */
#define MXT_TYP_PULSE 3
/** XYZ data (position after filter process). */
#define MXT_TYP_FPOSE 4
/** Joint data (position after filter process). */
#define MXT_TYP_FJOINT 5
/** Motor pulse data (position after filter process). */
#define MXT_TYP_FPULSE 6
/** XYZ data (encoder feedback value). */
#define MXT_TYP_FBPOSE 7
/** Joint data (encoder feedback value). */
#define MXT_TYP_FBJOINT 8
/** Motor pulse data (encoder feedback value). */
#define MXT_TYP_FBPULSE 9
/** Current command [\%]. */
#define MXT_TYP_CMDCUR 10
/** Current feedback [\%]. */
#define MXT_TYP_FBKCUR 11

namespace rl
{
	namespace hal
	{
		MitsubishiH7::MitsubishiH7(
			const ::std::size_t& dof,
			const ::std::string& server,
			const ::std::string& client,
			const unsigned short int& tcp,
			const unsigned short int& udp,
			const Mode& mode,
			const uint16_t& haltIoData,
			const uint16_t& releaseIoData,
			const uint16_t& shutIoData
		) :
			AxisController(dof, 0.00711f),
			CartesianPositionActuator(dof, 0.00711f),
			CartesianPositionSensor(dof, 0.00711f),
			Gripper(),
			JointPositionActuator(dof, 0.00711f),
			JointPositionSensor(dof, 0.00711f),
			client(client),
			filter(0),
			haltIoData(haltIoData),
			in(),
			mode(mode),
			out(),
			releaseIoData(releaseIoData),
			server(server),
			shutIoData(shutIoData),
			tcp(new TcpSocket(server, tcp)),
			udp(new UdpSocket(server, udp))
		{
			assert(dof < 9);
			
			this->out.cCount = 0;
		}
		
		MitsubishiH7::~MitsubishiH7()
		{
			delete this->tcp;
			delete this->udp;
		}
		
		void
		MitsubishiH7::calibCmd(CalibState& state) const
		{
			char buf[256] = "1;1;CALIB";
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
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
				int dec = strtol(hex, NULL, 16);
				// 00000001
				state.isAxis[0] = (dec & 1) ? true : false;
				// 00000010
				state.isAxis[1] = (dec & 2) ? true : false;
				// 00000100
				state.isAxis[2] = (dec & 4) ? true : false;
				// 00001000
				state.isAxis[3] = (dec & 8) ? true : false;
				hex[0] = buf[5];
				dec = strtol(hex, NULL, 16);
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
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::close()
		{
			this->tcp->close();
			this->udp->close();
			this->setConnected(false);
		}
		
		void
		MitsubishiH7::cntlCmd(const bool& doOn) const
		{
			char buf[256];
			
			snprintf(buf, 255, "1;1;CNTL%s", (doOn ? "ON" : "OFF"));
			buf[255] = '\0';
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::datinstCmd(const ::std::string& j1, const ::std::string& j2, const ::std::string& j3, const ::std::string& j4, const ::std::string& j5, const ::std::string& j6, const ::std::string& checksum) const
		{
			char buf[256];
			
			snprintf(buf, 255, "1;1;DATINST%s;%s;%s;%s;%s;%s;%s", j1.c_str(), j2.c_str(), j3.c_str(), j4.c_str(), j5.c_str(), j6.c_str(), checksum.c_str());
			buf[255] = '\0';
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::datinstCmd(const ::std::string& j1, const ::std::string& j2, const ::std::string& j3, const ::std::string& j4, const ::std::string& j5, const ::std::string& j6, const ::std::string& j7, const ::std::string& j8, const ::std::string& checksum) const
		{
			char buf[256];
			
			snprintf(buf, 255, "1;1;DATINST%s;%s;%s;%s;%s;%s;%s;%s;%s", j1.c_str(), j2.c_str(), j3.c_str(), j4.c_str(), j5.c_str(), j6.c_str(), j7.c_str(), j8.c_str(), checksum.c_str());
			buf[255] = '\0';
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::dstateCmd(StopState& state) const
		{
			char buf[256] = "1;1;DSTATE";
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'o' == buf[1] && 'K' == buf[2])
			{
				char* elem = buf + 2;
				char* semicolon = NULL;
				int i = 0;
				
				while (NULL != elem)
				{
					++elem;
					semicolon = strchr(elem, ';');
					
					if (NULL != semicolon)
					{
						buf[semicolon - buf] = '\0';
					}
					
					switch (i)
					{
					case 0:
						// run status
						{
							char hex[2] = {elem[1], '\0'};
							int dec = strtol(hex, NULL, 16);
							// 00000001
							state.runSts.isRepeat = (dec & 1) ? true : false;
							// 00000010
							state.runSts.isCycleStopOff = (dec & 2) ? true : false;
							// 00000100
							state.runSts.isMlockOn = (dec & 4) ? true : false;
							// 00001000
							state.runSts.isTeach = (dec & 8) ? true : false;
							hex[0] = elem[0];
							dec = strtol(hex, NULL, 16);
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
							int dec = strtol(hex, NULL, 16);
							// 00000001
							state.stopSts.isEmgStop = (dec & 1) ? true : false;
							// 00000010
							state.stopSts.isStop = (dec & 2) ? true : false;
							// 00000100
							state.stopSts.isWait = (dec & 4) ? true : false;
							// 00001000
							state.stopSts.isStopSignalOff = (dec & 8) ? true : false;
							hex[0] = elem[0];
							dec = strtol(hex, NULL, 16);
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
							state.errorNo = atoi(elem);
						}
						break;
					case 1:
						// execution step number
						state.stepNo = atoi(elem);
						break;
					case 2:
						// mech number under use
						state.mechNo = atoi(elem);
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
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::eclrCmd() const
		{
			char buf[256] = "1;9;ECLR";
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::emdatCmd(const ::std::string& program) const
		{
			// max. 256 - 9 (strlen("1;9;EMDAT")) - 1 = 246 chars
			
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
				
				this->tcp->write(lines.c_str(), lines.length());
				this->tcp->read(buf, 256);
				
				if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
				{
					char* errorNo = NULL;
					char* errorLine = NULL;
					char* charPos = NULL;
					
					char* elem = buf + 2;
					char* semicolon = NULL;
					int i = 0;
					
					while (NULL != elem)
					{
						++elem;
						semicolon = strchr(elem, ';');
						
						if (NULL != semicolon)
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
					
					throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
				}
				
				begin += end + 1;
			}
		}
		
		::std::string
		MitsubishiH7::errormesCmd(const int& errorNo) const
		{
			char buf[256];
			
			snprintf(buf, 255, "1;1;ERRORMES%i", errorNo);
			buf[255] = '\0';
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
			
			return buf + 3;
		}
		
		void
		MitsubishiH7::execCmd(const ::std::string& instruction) const
		{
			char buf[256];
			
			snprintf(buf, 255, "1;9;EXEC%s", instruction.c_str());
			buf[255] = '\0';
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::getCartesianPosition(::rl::math::Transform& x) const
		{
			x.setIdentity();
			
			x = ::rl::math::AngleAxis(this->in.dat1.pos.w.c, ::rl::math::Vector3::UnitZ()) *
				::rl::math::AngleAxis(this->in.dat1.pos.w.b, ::rl::math::Vector3::UnitY()) *
				::rl::math::AngleAxis(this->in.dat1.pos.w.a, ::rl::math::Vector3::UnitX());
			
			x.translation().x() = this->in.dat1.pos.w.x / 1000.0f;
			x.translation().y() = this->in.dat1.pos.w.y / 1000.0f;
			x.translation().z() = this->in.dat1.pos.w.z / 1000.0f;
		}
		
		void
		MitsubishiH7::getCurrentFeedback(::Eigen::Matrix< int32_t, ::Eigen::Dynamic, 1 >& c) const
		{
			assert(c.size() >= this->getDof());
			
			switch (this->getDof())
			{
			case 8:
				c(7) = this->in.dat3.pls.p8;
			case 7:
				c(6) = this->in.dat3.pls.p7;
			case 6:
				c(5) = this->in.dat3.pls.p6;
			case 5:
				c(4) = this->in.dat3.pls.p5;
			case 4:
				c(3) = this->in.dat3.pls.p4;
			case 3:
				c(2) = this->in.dat3.pls.p3;
			case 2:
				c(1) = this->in.dat3.pls.p2;
			case 1:
				c(0) = this->in.dat3.pls.p1;
			default:
				break;
			}
		}
		
		::std::size_t
		MitsubishiH7::getFilter() const
		{
			return this->filter;
		}
		
		uint16_t
		MitsubishiH7::getIoData() const
		{
			return this->in.ioData;
		}
		
		void
		MitsubishiH7::getJointPosition(::rl::math::Vector& q) const
		{
			assert(q.size() >= this->getDof());
			
			switch (this->getDof())
			{
			case 8:
				q(7) = this->in.dat.jnt.j8;
			case 7:
				q(6) = this->in.dat.jnt.j7;
			case 6:
				q(5) = this->in.dat.jnt.j6;
			case 5:
				q(4) = this->in.dat.jnt.j5;
			case 4:
				q(3) = this->in.dat.jnt.j4;
			case 3:
				q(2) = this->in.dat.jnt.j3;
			case 2:
				q(1) = this->in.dat.jnt.j2;
			case 1:
				q(0) = this->in.dat.jnt.j1;
			default:
				break;
			}
		}
		
		MitsubishiH7::Mode
		MitsubishiH7::getMode() const
		{
			return this->mode;
		}
		
		void
		MitsubishiH7::getMotorPulse(::Eigen::Matrix< int32_t, ::Eigen::Dynamic, 1 >& p) const
		{
			assert(p.size() >= this->getDof());
			
			switch (this->getDof())
			{
			case 8:
				p(7) = this->in.dat2.pls.p8;
			case 7:
				p(6) = this->in.dat2.pls.p7;
			case 6:
				p(5) = this->in.dat2.pls.p6;
			case 5:
				p(4) = this->in.dat2.pls.p5;
			case 4:
				p(3) = this->in.dat2.pls.p4;
			case 3:
				p(2) = this->in.dat2.pls.p3;
			case 2:
				p(1) = this->in.dat2.pls.p2;
			case 1:
				p(0) = this->in.dat2.pls.p1;
			default:
				break;
			}
		}
		
		void
		MitsubishiH7::halt()
		{
			this->setOutput(900, 0x00FF, this->haltIoData);
		}
		
		void
		MitsubishiH7::hndCmd(const bool& doOpen, const int& handNo) const
		{
			assert(0 < handNo && handNo < 9);
			
			char buf[256];
			
			snprintf(buf, 255, "1;1;HND%s%i", (doOpen ? "ON" : "OFF"), handNo);
			buf[255] = '\0';
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::hndstsCmd(HandState& state) const
		{
			char buf[256] = "1;1;HNDSTS";
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'o' == buf[1] && 'K' == buf[2])
			{
				char* elem = buf + 2;
				char* semicolon = NULL;
				int i = 0;
				int j = 0;
				
				while (NULL != elem)
				{
					++elem;
					semicolon = strchr(elem, ';');
					
					if (NULL != semicolon)
					{
						buf[semicolon - buf] = '\0';
					}
					
					switch (j)
					{
					case 0:
						// signal number allocated in hand
						state[i].outputNo = atoi(elem);
						break;
					case 1:
						// hand output status
						switch (atoi(elem))
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
						switch (atoi(elem))
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
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::inEqualsCmd(const ::std::size_t& inNo, const ::std::string& inVal) const
		{
			char buf[256];
			
			snprintf(buf, 255, "1;1;IN=%zi;%s", inNo, inVal.c_str());
			buf[255] = '\0';
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::loadCmd(const ::std::string& programName) const
		{
			char buf[256];
			
			snprintf(buf, 255, "1;1;LOAD=%s", programName.c_str());
			buf[255] = '\0';
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::loadProgram(const ::std::string& name, const ::std::string& program) const
		{
			this->newCmd();
			this->loadCmd(name);
			this->eclrCmd();
			this->emdatCmd(program);
			this->saveCmd();
		}
		
		void
		MitsubishiH7::newCmd() const
		{
			char buf[256] = "1;1;NEW";
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::open()
		{
			this->tcp->open();
			this->udp->open();
			this->setConnected(true);
		}
		
		void
		MitsubishiH7::outEqualsCmd(const ::std::size_t& outNo, const ::std::string& outVal) const
		{
			char buf[256];
			
			snprintf(buf, 255, "1;1;OUT=%zi;%s", outNo, outVal.c_str());
			buf[255] = '\0';
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::release()
		{
			this->setOutput(900, 0x00FF, this->releaseIoData);
		}
		
		void
		MitsubishiH7::rstalrmCmd() const
		{
			char buf[256] = "1;1;RSTALRM";
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::rstpwrCmd() const
		{
			char buf[256] = "1;1;RSTPWR";
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::runCmd(const ::std::string& programName, const bool& doModeCycle) const
		{
			char buf[256];
			
			snprintf(buf, 255, "1;1;RUN%s;%i", programName.c_str(), (doModeCycle ? 1 : 0));
			buf[255] = '\0';
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::saveCmd() const
		{
			char buf[256] = "1;1;SAVE";
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::setCartesianPosition(const ::rl::math::Transform& x)
		{
			::rl::math::Vector3 abc = x.rotation().eulerAngles(2, 1, 0).reverse();
			
			this->out.dat.pos.w.a = static_cast< float >(abc(0));
			this->out.dat.pos.w.b = static_cast< float >(abc(1));
			this->out.dat.pos.w.c = static_cast< float >(abc(2));
			
			this->out.dat.pos.w.x = static_cast< float >(x.translation().x() * 1000.0f);
			this->out.dat.pos.w.y = static_cast< float >(x.translation().y() * 1000.0f);
			this->out.dat.pos.w.z = static_cast< float >(x.translation().z() * 1000.0f);
			
			this->out.command = MXT_CMD_MOVE;
			this->out.sendType = MXT_TYP_POSE;
		}
		
		void
		MitsubishiH7::setFilter(const ::std::size_t& filter)
		{
			this->filter = filter;
		}
		
		void
		MitsubishiH7::setInput(const uint16_t& bitTop)
		{
			this->out.bitTop = bitTop;
			this->out.recvIoType = MXT_IO_IN;
			this->out.sendIoType = MXT_IO_NULL;
		}
		
		void
		MitsubishiH7::setJointPosition(const ::rl::math::Vector& q)
		{
			assert(q.size() >= this->getDof());
			
			this->out.dat.jnt.j1 = 0.0f;
			this->out.dat.jnt.j2 = 0.0f;
			this->out.dat.jnt.j3 = 0.0f;
			this->out.dat.jnt.j4 = 0.0f;
			this->out.dat.jnt.j5 = 0.0f;
			this->out.dat.jnt.j6 = 0.0f;
			this->out.dat.jnt.j7 = 0.0f;
			this->out.dat.jnt.j8 = 0.0f;
			
			switch (this->getDof())
			{
			case 8:
				this->out.dat.jnt.j8 = static_cast< float >(q(7));
			case 7:
				this->out.dat.jnt.j7 = static_cast< float >(q(6));
			case 6:
				this->out.dat.jnt.j6 = static_cast< float >(q(5));
			case 5:
				this->out.dat.jnt.j5 = static_cast< float >(q(4));
			case 4:
				this->out.dat.jnt.j4 = static_cast< float >(q(3));
			case 3:
				this->out.dat.jnt.j3 = static_cast< float >(q(2));
			case 2:
				this->out.dat.jnt.j2 = static_cast< float >(q(1));
			case 1:
				this->out.dat.jnt.j1 = static_cast< float >(q(0));
			default:
				break;
			}
			
			this->out.command = MXT_CMD_MOVE;
			this->out.sendType = MXT_TYP_JOINT;
		}
		
		void
		MitsubishiH7::setMode(const Mode& mode)
		{
			this->mode = mode;
		}
		
		void
		MitsubishiH7::setMotorPulse(const ::Eigen::Matrix< int32_t, ::Eigen::Dynamic, 1 >& p)
		{
			assert(p.size() >= this->getDof());
			
			this->out.dat2.pls.p1 = 0;
			this->out.dat2.pls.p2 = 0;
			this->out.dat2.pls.p3 = 0;
			this->out.dat2.pls.p4 = 0;
			this->out.dat2.pls.p5 = 0;
			this->out.dat2.pls.p6 = 0;
			this->out.dat2.pls.p7 = 0;
			this->out.dat2.pls.p8 = 0;
			
			switch (this->getDof())
			{
			case 8:
				this->out.dat2.pls.p8 = p(7);
			case 7:
				this->out.dat2.pls.p7 = p(6);
			case 6:
				this->out.dat2.pls.p6 = p(5);
			case 5:
				this->out.dat2.pls.p5 = p(4);
			case 4:
				this->out.dat2.pls.p4 = p(3);
			case 3:
				this->out.dat2.pls.p3 = p(2);
			case 2:
				this->out.dat2.pls.p2 = p(1);
			case 1:
				this->out.dat2.pls.p1 = p(0);
			default:
				break;
			}
			
			this->out.command = MXT_CMD_MOVE;
			this->out.sendType = MXT_TYP_PULSE;
		}
		
		void
		MitsubishiH7::setOutput(const uint16_t& bitTop, const uint16_t& bitMask, const uint16_t& ioData)
		{
			this->out.bitMask = bitMask;
			this->out.bitTop = bitTop;
			this->out.ioData = ioData;
			this->out.recvIoType = MXT_IO_NULL;
			this->out.sendIoType = MXT_IO_OUT;
		}
		
		void
		MitsubishiH7::shut()
		{
			this->setOutput(900, 0x00FF, this->shutIoData);
		}
		
		void
		MitsubishiH7::slotinitCmd() const
		{
			char buf[256] = "1;1;SLOTINIT";
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::srvCmd(const bool& doOn) const
		{
			char buf[256];
			
			snprintf(buf, 255, "1;1;SRV%s", (doOn ? "ON" : "OFF"));
			buf[255] = '\0';
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::start()
		{
			this->stopProgram();
			
			struct hostent* client = gethostbyname(this->client.c_str());
			struct sockaddr_in clientAddr;
			memset(&clientAddr, 0, sizeof(clientAddr));
			memcpy(&clientAddr.sin_addr, client->h_addr, sizeof(client->h_addr));
			
			::std::stringstream program;
			
			program << "10 OPEN \"ENET:" << inet_ntoa(clientAddr.sin_addr) << "\" AS #1" << "\v";
			program << "20 MXT 1," << this->mode << "," << this->filter << "\v";
			program << "30 HLT" << "\v";
			program << "40 END";
			
			this->loadProgram("1432", program.str());
			
			this->cntlCmd(true);
			this->srvCmd(true);
			this->cntlCmd(false);
			
			StopState state;
			
			state.runSts.isServoOn = false;
			
			while (!state.runSts.isServoOn)
			{
				this->dstateCmd(state);
			}
			
			this->startProgram("1432");
			
			this->setRunning(true);
		}
		
		void
		MitsubishiH7::startProgram(const ::std::string& name) const
		{
			this->cntlCmd(true);
			this->slotinitCmd();
			this->runCmd(name, true);
			this->cntlCmd(false);
		}
		
		void
		MitsubishiH7::stateCmd(RunState& state) const
		{
			char buf[256] = "1;1;STATE";
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'o' == buf[1] && 'K' == buf[2])
			{
				char* elem = buf + 2;
				char* semicolon = NULL;
				int i = 0;
				
				while (NULL != elem)
				{
					++elem;
					semicolon = strchr(elem, ';');
					
					if (NULL != semicolon)
					{
						buf[semicolon - buf] = '\0';
					}
					
					switch (i)
					{
					case 0:
						// program name loaded into task slot
						strncpy(state.programName, elem, 255);
						state.programName[255] = '\0';
						break;
					case 1:
						// execution line number
						state.lineNo = atoi(elem);
						break;
					case 2:
						// a present override value is read
						state.override = atoi(elem);
						break;
					case 3:
						// edit status
						{
							int dec = strtol(elem, NULL, 16);
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
							int dec = strtol(hex, NULL, 16);
							// 00000001
							state.runSts.isRepeat = (dec & 1) ? true : false;
							// 00000010
							state.runSts.isCycleStopOff = (dec & 2) ? true : false;
							// 00000100
							state.runSts.isMlockOn = (dec & 4) ? true : false;
							// 00001000
							state.runSts.isTeach = (dec & 8) ? true : false;
							hex[0] = elem[0];
							dec = strtol(hex, NULL, 16);
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
							int dec = strtol(hex, NULL, 16);
							// 00000001
							state.stopSts.isEmgStop = (dec & 1) ? true : false;
							// 00000010
							state.stopSts.isStop = (dec & 2) ? true : false;
							// 00000100
							state.stopSts.isWait = (dec & 4) ? true : false;
							// 00001000
							state.stopSts.isStopSignalOff = (dec & 8) ? true : false;
							hex[0] = elem[0];
							dec = strtol(hex, NULL, 16);
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
							state.errorNo = atoi(elem);
						}
						break;
					case 5:
						// execution step number
						state.stepNo = atoi(elem);
						break;
					case 6:
						// mech info
						{
							int dec = strtol(elem, NULL, 16);
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
						strncpy(state.taskPrgName, elem, 255);
						state.taskPrgName[255] = '\0';
						break;
					case 15:
						// operation mode of slot table
						if (0 == strcmp(elem, "CYC"))
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
						if (0 == strcmp(elem, "START"))
						{
							state.taskCond = TASKCOND_START;
						}
						else if (0 == strcmp(elem, "ALWAYS"))
						{
							state.taskCond = TASKCOND_ALWAYS;
						}
						else if (0 == strcmp(elem, "ERROR"))
						{
							state.taskCond = TASKCOND_ERROR;
						}
						break;
					case 17:
						// priority of slot table
						state.taskPri = atoi(elem);
						break;
					case 18:
						// mech number under use
						state.mechNo = atoi(elem);
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
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::step()
		{
			this->out.recvType = MXT_TYP_JOINT;
			this->out.recvType1 = MXT_TYP_POSE;
			this->out.recvType2 = MXT_TYP_PULSE;
			this->out.recvType3 = MXT_TYP_FBKCUR;
			
			this->udp->write(&out, sizeof(MxtCommand));
			
			++this->out.cCount;
			
			try
			{
				this->udp->select(true, false, this->filter * this->getUpdateRate() + 6 * this->getUpdateRate());
			}
			catch (const TimeoutException& e)
			{
				throw e;
			}
			
			this->udp->read(&this->in, sizeof(MxtCommand));
			
			this->out.bitMask = 0;
			this->out.bitTop = 0;
			this->out.command = MXT_CMD_NULL;
			this->out.ioData = 0;
			this->out.recvType = MXT_TYP_NULL;
			this->out.recvType1 = MXT_TYP_NULL;
			this->out.recvType2 = MXT_TYP_NULL;
			this->out.recvType3 = MXT_TYP_NULL;
			this->out.recvIoType = MXT_IO_NULL;
			this->out.sendType = MXT_TYP_NULL;
			this->out.sendIoType = MXT_IO_NULL;
		}
		
		void
		MitsubishiH7::stop()
		{
			this->out.bitMask = 0;
			this->out.bitTop = 0;
			this->out.command = MXT_CMD_END;
			this->out.ioData = 0;
			this->out.recvType = MXT_TYP_NULL;
			this->out.recvType1 = MXT_TYP_NULL;
			this->out.recvType2 = MXT_TYP_NULL;
			this->out.recvType3 = MXT_TYP_NULL;
			this->out.recvIoType = MXT_IO_NULL;
			this->out.sendType = MXT_TYP_NULL;
			this->out.sendIoType = MXT_IO_NULL;
			
			this->udp->write(&out, sizeof(MxtCommand));
			
			this->stopProgram();
			
			StopState state;
			
			this->dstateCmd(state);
			
			if (state.runSts.isServoOn)
			{
				this->srvCmd(false);
				
				while (state.runSts.isServoOn)
				{
					this->dstateCmd(state);
				}
			}
			
			this->setRunning(false);
		}
		
		void
		MitsubishiH7::stopCmd() const
		{
			char buf[256] = "1;1;STOP";
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'e' == buf[1] && 'R' == buf[2])
			{
				char* errorNo = buf + 3;
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
		
		void
		MitsubishiH7::stopProgram() const
		{
			this->stopCmd();
		}
		
		void
		MitsubishiH7::stpsigCmd(StopSignalState& state) const
		{
			char buf[256] = "1;1;STPSIG";
			
			this->tcp->write(buf, strlen(buf));
			this->tcp->read(buf, 256);
			
			if ('Q' == buf[0] && 'o' == buf[1] && 'K' == buf[2])
			{
				char* elem = buf + 2;
				char* semicolon = NULL;
				int i = 0;
				
				while (NULL != elem)
				{
					++elem;
					semicolon = strchr(elem, ';');
					
					if (NULL != semicolon)
					{
						buf[semicolon - buf] = '\0';
					}
					
					switch (i)
					{
					case 0:
						// the state of the stop signal
						{
							int dec = strtol(elem, NULL, 16);
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
							int dec = strtol(elem, NULL, 16);
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
				throw MitsubishiH7Exception(atoi(errorNo), ::std::string(this->errormesCmd(atoi(errorNo)) + " (" + errorNo + ")"));
			}
		}
	}
}
