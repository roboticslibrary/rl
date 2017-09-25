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
#include <cstdint>

#include "ComException.h"
#include "UniversalRobotsDashboard.h"

namespace rl
{
	namespace hal
	{
		UniversalRobotsDashboard::UniversalRobotsDashboard(const ::std::string& address) :
			Com(),
			socket(Socket::Tcp(Socket::Address::Ipv4(address, 29999)))
		{
		}
		
		UniversalRobotsDashboard::~UniversalRobotsDashboard()
		{
			if (this->isConnected())
			{
				this->close();
			}
		}
		
		void
		UniversalRobotsDashboard::close()
		{
			this->socket.close();
			this->setConnected(false);
		}
		
		void
		UniversalRobotsDashboard::doAddToLog(const ::std::string& message)
		{
			this->send("addToLog " + message + "\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Added log message\n" == reply)
			{
			}
			else if ("No log message to add\n" == reply)
			{
				throw ComException("No log message to add");
			}
			else
			{
				throw ComException("Bad reply: addToLog " + message);
			}
		}
		
		void
		UniversalRobotsDashboard::doBrakeRelease()
		{
			this->send("brake release\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Brake releasing\n" == reply)
			{
			}
			else
			{
				throw ComException("Bad reply: brake release");
			}
		}
		
		void
		UniversalRobotsDashboard::doClosePopup()
		{
			this->send("close popup\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("closing popup\n" == reply)
			{
			}
			else
			{
				throw ComException("Bad reply: close popup");
			}
		}
		
		void
		UniversalRobotsDashboard::doCloseSafetyPopup()
		{
			this->send("close safety popup\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("closing safety popup\n" == reply)
			{
			}
			else
			{
				throw ComException("Bad reply: close safety popup");
			}
		}
		
		::std::string
		UniversalRobotsDashboard::doGetLoadedProgram()
		{
			this->send("get loaded program\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Loaded program" == reply.substr(0, 14))
			{
				return reply.substr(16, reply.size() - 17);
			}
			else if ("No program loaded\n" == reply)
			{
				return ::std::string();
			}
			else
			{
				throw ComException("Bad reply: get loaded program");
			}
		}
		
		::std::pair<bool, ::std::string>
		UniversalRobotsDashboard::doIsProgramSaved()
		{
			this->send("isProgramSaved\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("true" == reply.substr(0, 4))
			{
				return ::std::make_pair(true, reply.substr(5, reply.size() - 6));
			}
			else if ("false" == reply.substr(0, 5))
			{
				return ::std::make_pair(false, reply.substr(6, reply.size() - 7));
			}
			else
			{
				throw ComException("Bad reply: isProgramSaved");
			}
		}
		
		void
		UniversalRobotsDashboard::doLoad(const ::std::string& program)
		{
			this->send("load " + program + "\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Loading program" == reply.substr(0, 15))
			{
			}
			else if ("File not found" == reply.substr(0, 14))
			{
				throw ComException("File " + program + " not found");
			}
			else if ("Error while loading program" == reply.substr(0, 27))
			{
				throw ComException("Error while loading program " + program);
			}
			else
			{
				throw ComException("Bad reply: load " + program);
			}
		}
		
		void
		UniversalRobotsDashboard::doLoadInstallation(const ::std::string& installation)
		{
			this->send("load installation " + installation + "\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Loading installation" == reply.substr(0, 20))
			{
			}
			else if ("File not found" == reply.substr(0, 14))
			{
				throw ComException("File " + installation + " not found");
			}
			else if ("Failed to load installation" == reply.substr(0, 27))
			{
				throw ComException("Failed to load installation " + installation);
			}
			else
			{
				throw ComException("Bad reply: load installation " + installation);
			}
		}
		
		void
		UniversalRobotsDashboard::doPause()
		{
			this->send("pause\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Pausing program\n" == reply)
			{
			}
			else if ("Failed to execute: pause\n" == reply)
			{
				throw ComException("Failed to execute pause");
			}
			else
			{
				throw ComException("Bad reply: pause");
			}
		}
		
		void
		UniversalRobotsDashboard::doPlay()
		{
			this->send("play\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Starting program\n" == reply)
			{
			}
			else if ("Failed to execute: play\n" == reply)
			{
				throw ComException("Failed to execute play");
			}
			else
			{
				throw ComException("Bad reply: play");
			}
		}
		
		::std::string
		UniversalRobotsDashboard::doPolyscopeVersion()
		{
			this->send("PolyscopeVersion\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			return reply.substr(0, reply.size() - 1);
		}
		
		void
		UniversalRobotsDashboard::doPopup(const ::std::string& text)
		{
			this->send("popup " + text + "\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("showing popup\n" == reply)
			{
			}
			else
			{
				throw ComException("Bad reply: popup " + text);
			}
		}
		
		void
		UniversalRobotsDashboard::doPowerOff()
		{
			this->send("power off\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Powering off\n" == reply)
			{
			}
			else
			{
				throw ComException("Bad reply: power off");
			}
		}
		
		void
		UniversalRobotsDashboard::doPowerOn()
		{
			this->send("power on\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Powering on\n" == reply)
			{
			}
			else
			{
				throw ComException("Bad reply: power on");
			}
		}
		
		::std::pair<UniversalRobotsDashboard::ProgramState, ::std::string>
		UniversalRobotsDashboard::doProgramState()
		{
			this->send("programState\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("STOPPED" == reply.substr(0, 7))
			{
				return ::std::make_pair(PROGRAM_STATE_STOPPED, reply.substr(8, reply.size() - 9));
			}
			else if ("PLAYING" == reply.substr(0, 7))
			{
				return ::std::make_pair(PROGRAM_STATE_PLAYING, reply.substr(8, reply.size() - 9));
			}
			else if ("PAUSED" == reply.substr(0, 6))
			{
				return ::std::make_pair(PROGRAM_STATE_PAUSED, reply.substr(7, reply.size() - 8));
			}
			else
			{
				throw ComException("Bad reply: programState");
			}
		}
		
		UniversalRobotsDashboard::RobotMode
		UniversalRobotsDashboard::doRobotmode()
		{
			this->send("robotmode\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Robotmode: NO_CONTROLLER\n" == reply)
			{
				return ROBOT_MODE_NO_CONTROLLER;
			}
			else if ("Robotmode: DISCONNECTED\n" == reply)
			{
				return ROBOT_MODE_DISCONNECTED;
			}
			else if ("Robotmode: CONFIRM_SAFETY\n" == reply)
			{
				return ROBOT_MODE_CONFIRM_SAFETY;
			}
			else if ("Robotmode: BOOTING\n" == reply)
			{
				return ROBOT_MODE_BOOTING;
			}
			else if ("Robotmode: POWER_OFF\n" == reply)
			{
				return ROBOT_MODE_POWER_OFF;
			}
			else if ("Robotmode: POWER_ON\n" == reply)
			{
				return ROBOT_MODE_POWER_ON;
			}
			else if ("Robotmode: IDLE\n" == reply)
			{
				return ROBOT_MODE_IDLE;
			}
			else if ("Robotmode: BACKDRIVE\n" == reply)
			{
				return ROBOT_MODE_BACKDRIVE;
			}
			else if ("Robotmode: RUNNING\n" == reply)
			{
				return ROBOT_MODE_RUNNING;
			}
			else
			{
				throw ComException("Bad reply: robotmode");
			}
		}
		
		bool
		UniversalRobotsDashboard::doRunning()
		{
			this->send("running\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Robot running: True\n" == reply)
			{
				return true;
			}
			else if ("Robot running: False\n" == reply)
			{
				return false;
			}
			else
			{
				throw ComException("Bad reply: running");
			}
		}
		
		UniversalRobotsDashboard::SafetyMode
		UniversalRobotsDashboard::doSafetymode()
		{
			this->send("safetymode\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Safetymode: NORMAL\n" == reply)
			{
				return SAFETY_MODE_NORMAL;
			}
			else if ("Safetymode: REDUCED\n" == reply)
			{
				return SAFETY_MODE_REDUCED;
			}
			else if ("Safetymode: PROTECTIVE_STOP\n" == reply)
			{
				return SAFETY_MODE_PROTECTIVE_STOP;
			}
			else if ("Safetymode: RECOVERY\n" == reply)
			{
				return SAFETY_MODE_RECOVERY;
			}
			else if ("Safetymode: SAFEGUARD_STOP\n" == reply)
			{
				return SAFETY_MODE_SAFEGUARD_STOP;
			}
			else if ("Safetymode: SYSTEM_EMERGENCY_STOP\n" == reply)
			{
				return SAFETY_MODE_SYSTEM_EMERGENCY_STOP;
			}
			else if ("Safetymode: ROBOT_EMERGENCY_STOP\n" == reply)
			{
				return SAFETY_MODE_ROBOT_EMERGENCY_STOP;
			}
			else if ("Safetymode: VIOLATION\n" == reply)
			{
				return SAFETY_MODE_VIOLATION;
			}
			else if ("Safetymode: FAULT\n" == reply)
			{
				return SAFETY_MODE_FAULT;
			}
			else
			{
				throw ComException("Bad reply: safetymode");
			}
		}
		
		void
		UniversalRobotsDashboard::doShutdown()
		{
			this->send("shutdown\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Shutting down\n" == reply)
			{
			}
			else
			{
				throw ComException("Bad reply: shutdown");
			}
		}
		
		void
		UniversalRobotsDashboard::doStop()
		{
			this->send("stop\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Stopped\n" == reply)
			{
			}
			else if ("Failed to execute: stop\n" == reply)
			{
				throw ComException("Failed to execute stop");
			}
			else
			{
				throw ComException("Bad reply: stop");
			}
		}
		
		void
		UniversalRobotsDashboard::doQuit()
		{
			this->send("quit\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Disconnected\n" == reply)
			{
			}
			else
			{
				throw ComException("Bad reply: quit");
			}
		}
		
		void
		UniversalRobotsDashboard::doUnlockProtectiveStop()
		{
			this->send("unlock protective stop\n");
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
			
			if ("Protective stop releasing\n" == reply)
			{
			}
			else
			{
				throw ComException("Bad reply: unlock protective stop");
			}
		}
		
		void
		UniversalRobotsDashboard::open()
		{
			this->socket.open();
			this->socket.connect();
			this->socket.setOption(::rl::hal::Socket::OPTION_NODELAY, 1);
			this->setConnected(true);
			
			::std::array< ::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket.recv(buffer.data(), buffer.size());
			::std::string reply(reinterpret_cast<char*>(buffer.data()), size);
		}
		
		void
		UniversalRobotsDashboard::send(const ::std::string& command)
		{
			this->socket.send(command.c_str(), command.size());
		}
	}
}
