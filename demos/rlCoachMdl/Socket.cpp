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

#include <QHostAddress>
#include <QStatusBar>
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
}

Socket::~Socket()
{
}

void
Socket::readClient()
{
	MainWindow::instance()->statusBar()->showMessage("Received data from " + this->peerAddress().toString() + ":" + QString::number(this->peerPort()), 1000);
	
	QTextStream textStream(this);
	
	for (QString line = textStream.readLine(); QString() != line; line = textStream.readLine())
	{
		QStringList list = line.split(" ");
		
		if (list.size() < 1)
		{
			continue;
		}
		
		std::size_t cmd = list[0].toUInt();
		
		switch (cmd)
		{
		case 0:
			{
				if (9 != list.size())
				{
					continue;
				}
				
				std::size_t i = list[1].toUInt();
				std::size_t j = list[2].toUInt();
				rl::math::Real x = list[3].toDouble();
				rl::math::Real y = list[4].toDouble();
				rl::math::Real z = list[5].toDouble();
				rl::math::Real a = list[6].toDouble();
				rl::math::Real b = list[7].toDouble();
				rl::math::Real c = list[8].toDouble();
				
				rl::math::Transform t;
				t = rl::math::AngleAxis(c, rl::math::Vector3::UnitZ()) *
					rl::math::AngleAxis(b, rl::math::Vector3::UnitY()) *
					rl::math::AngleAxis(a, rl::math::Vector3::UnitX());
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
				if (10 != list.size())
				{
					continue;
				}
				
				std::size_t i = list[1].toUInt();
				std::size_t j = list[2].toUInt();
				std::size_t k = list[3].toUInt();
				rl::math::Real x = list[4].toDouble();
				rl::math::Real y = list[5].toDouble();
				rl::math::Real z = list[6].toDouble();
				rl::math::Real a = list[7].toDouble();
				rl::math::Real b = list[8].toDouble();
				rl::math::Real c = list[9].toDouble();
				
				rl::math::Transform t;
				t = rl::math::AngleAxis(c, rl::math::Vector3::UnitZ()) *
					rl::math::AngleAxis(b, rl::math::Vector3::UnitY()) *
					rl::math::AngleAxis(a, rl::math::Vector3::UnitX());
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
				if (list.size() < 2)
				{
					continue;
				}
				
				std::size_t i = list[1].toUInt();
				
				if (2 + MainWindow::instance()->kinematicModels[i]->getDofPosition() != list.size())
				{
					continue;
				}
				
				if (i < MainWindow::instance()->kinematicModels.size())
				{
					rl::math::Vector q(MainWindow::instance()->kinematicModels[i]->getDofPosition());
					q.setZero();
					
					for (std::ptrdiff_t j = 0; j < q.size(); ++j)
					{
						q(j) = list[2 + j].toDouble();
					}
					
					MainWindow::instance()->configurationModels[i]->setData(q);
				}
			}
			break;
		case 6:
			{
				textStream << cmd;
				
				if (list.size() < 2)
				{
					textStream << endl;
					continue;
				}
				
				std::size_t i = list[1].toUInt();
				textStream << " " << i;
				
				if (i < MainWindow::instance()->kinematicModels.size())
				{
					rl::math::Vector q = MainWindow::instance()->kinematicModels[i]->getPosition();
					
					for (std::size_t i = 0; i < q.size(); ++i)
					{
						textStream << " " << q(i);
					}
				}
				
				textStream << endl;
			}
			break;
		default:
			break;
		}
	}
}
