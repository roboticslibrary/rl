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

#include <QTextStream>
#include <rl/math/Rotation.h>
#include <rl/sg/Body.h>
#include <rl/sg/Shape.h>

#include "ConfigurationModel.h"
#include "MainWindow.h"
#include "Socket.h"

Socket::Socket(QObject* parent) :
	QTcpSocket(parent)
{
	QObject::connect(this, SIGNAL(disconnected()), this, SLOT(deleteLater()));
	QObject::connect(this, SIGNAL(readyRead()), this, SLOT(readClient()));
	MainWindow::instance()->setServerConnectionStatus("Connected");
}

Socket::~Socket()
{
	MainWindow::instance()->setServerConnectionStatus("Disconnected");
}

void
Socket::readClient()
{
	QTextStream textStream(this);
	
	while (this->canReadLine())
	{
		MainWindow::instance()->setServerConnectionStatus("Received command");
		std::size_t cmd = 0;
		textStream >> cmd;
		
		switch (cmd)
		{
		case 0:
			{
				std::size_t i = 0;
				textStream >> i;
				
				std::size_t j = 0;
				textStream >> j;
				
				rl::math::Real x = 0;
				textStream >> x;
				
				rl::math::Real y = 0;
				textStream >> y;
				
				rl::math::Real z = 0;
				textStream >> z;
				
				rl::math::Real a = 0;
				textStream >> a;
				
				rl::math::Real b = 0;
				textStream >> b;
				
				rl::math::Real c = 0;
				textStream >> c;
				
				rl::math::Transform t;
				t = rl::math::AngleAxis(a, rl::math::Vector3::UnitZ()) *
					rl::math::AngleAxis(b, rl::math::Vector3::UnitY()) *
					rl::math::AngleAxis(c, rl::math::Vector3::UnitX());
				t.translation().x() = x;
				t.translation().y() = y;
				t.translation().z() = z;
				
				if (MainWindow::instance()->scene->getNumModels() > i)
				{
					if (MainWindow::instance()->scene->getModel(i)->getNumBodies() > j)
					{
						MainWindow::instance()->scene->getModel(i)->getBody(j)->setFrame(t);
					}
				}
			}
			break;
		case 1:
			{
				std::size_t i = 0;
				textStream >> i;
				
				std::size_t j = 0;
				textStream >> j;
				
				std::size_t k = 0;
				textStream >> k;
				
				rl::math::Real x = 0;
				textStream >> x;
				
				rl::math::Real y = 0;
				textStream >> y;
				
				rl::math::Real z = 0;
				textStream >> z;
				
				rl::math::Real a = 0;
				textStream >> a;
				
				rl::math::Real b = 0;
				textStream >> b;
				
				rl::math::Real c = 0;
				textStream >> c;
				
				rl::math::Transform t;
				t = rl::math::AngleAxis(a, rl::math::Vector3::UnitZ()) *
					rl::math::AngleAxis(b, rl::math::Vector3::UnitY()) *
					rl::math::AngleAxis(c, rl::math::Vector3::UnitX());
				t.translation().x() = x;
				t.translation().y() = y;
				t.translation().z() = z;
				
				if (MainWindow::instance()->scene->getNumModels() > i)
				{
					if (MainWindow::instance()->scene->getModel(i)->getNumBodies() > j)
					{
						if (MainWindow::instance()->scene->getModel(i)->getBody(j)->getNumShapes() > k)
						{
							MainWindow::instance()->scene->getModel(i)->getBody(j)->getShape(k)->setTransform(t);
						}
					}
				}
			}
			break;
		case 2:
			{
				std::size_t i = 0;
				textStream >> i;
				
				if (i < 1)
				{
					rl::math::Vector q(MainWindow::instance()->kinematicModels->getDof());
					q.setZero();
					
					for (std::ptrdiff_t j = 0; j < q.size(); ++j)
					{
						textStream >> q(j);
					}
					
					MainWindow::instance()->positionModel->setData(q);
				}
			}
			break;
		case 5:
			{
				std::size_t i = 0;
				textStream >> i;
				
				if (i < 1)
				{
					rl::math::Vector torque(MainWindow::instance()->kinematicModels->getDof());
					torque.setZero();
					
					for (std::ptrdiff_t j = 0; j < torque.size(); ++j)
					{
						textStream >> torque(j);
					}
					
					MainWindow::instance()->torqueModel->setData(torque);
				}
			}
			break;
		default:
			break;
		}
		
		textStream << endl;
	}
}
