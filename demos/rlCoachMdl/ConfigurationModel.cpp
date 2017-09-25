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

#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Revolute.h>
#include <rl/sg/Body.h>

#include "ConfigurationModel.h"
#include "MainWindow.h"

ConfigurationModel::ConfigurationModel(QObject* parent) :
	QAbstractTableModel(parent),
	id(0)
{
}

ConfigurationModel::~ConfigurationModel()
{
}

int
ConfigurationModel::columnCount(const QModelIndex& parent) const
{
	return 1;
}

QVariant
ConfigurationModel::data(const QModelIndex& index, int role) const
{
	if (nullptr == MainWindow::instance()->kinematicModels[this->id])
	{
		return QVariant();
	}
	
	if (!index.isValid())
	{
		return QVariant();
	}
	
	rl::math::Vector q = MainWindow::instance()->kinematicModels[this->id]->getPosition();
	rl::math::Vector maximum = MainWindow::instance()->kinematicModels[this->id]->getMaximum();
	rl::math::Vector minimum = MainWindow::instance()->kinematicModels[this->id]->getMinimum();
	Eigen::Matrix<rl::math::Unit, Eigen::Dynamic, 1> qUnits = MainWindow::instance()->kinematicModels[this->id]->getPositionUnits();
	
	switch (role)
	{
	case Qt::DisplayRole:
		switch (qUnits(index.row()))
		{
		case rl::math::UNIT_METER:
			return QString::number(q(index.row()), 'f', 4) + QString(" m");
			break;
		case rl::math::UNIT_RADIAN:
			return QString::number(q(index.row()) * rl::math::RAD2DEG, 'f', 2) + QChar(176);
			break;
		default:
			return q(index.row());
			break;
		}
		break;
	case Qt::EditRole:
		if (rl::math::UNIT_RADIAN == qUnits(index.row()))
		{
			return q(index.row()) * rl::math::RAD2DEG;
		}
		else
		{
			return q(index.row());
		}
		break;
	case Qt::ForegroundRole:
		if (q(index.row()) < minimum(index.row()) || q(index.row()) > maximum(index.row()))
		{
			return QBrush(Qt::red);
		}
		break;
	case Qt::TextAlignmentRole:
		return QVariant(Qt::AlignRight | Qt::AlignVCenter);
		break;
	default:
		break;
	}
	
	return QVariant();
}

Qt::ItemFlags
ConfigurationModel::flags(const QModelIndex &index) const
{
	if (!index.isValid())
	{
		return Qt::NoItemFlags;
	}
	
	return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
}

QVariant
ConfigurationModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (nullptr == MainWindow::instance()->kinematicModels[this->id])
	{
		return QVariant();
	}
	
	if (Qt::DisplayRole == role && Qt::Vertical == orientation)
	{
		if (rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(MainWindow::instance()->kinematicModels[this->id].get()))
		{
			for (std::size_t i = 0, j = 0; i < kinematic->getJoints(); ++i)
			{
				rl::mdl::Joint* joint = kinematic->getJoint(i);
				
				for (std::size_t k = 0; k < joint->getDofPosition(); ++j, ++k)
				{
					if (section == j)
					{
						return QString::fromStdString(joint->getName());
					}
				}
			}
		}
	}
	
	return QVariant();
}

void
ConfigurationModel::operationalChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight)
{
	this->beginResetModel();
	this->endResetModel();
}

int
ConfigurationModel::rowCount(const QModelIndex& parent) const
{
	if (nullptr == MainWindow::instance()->kinematicModels[this->id])
	{
		return 0;
	}
	
	return MainWindow::instance()->kinematicModels[this->id]->getDofPosition();
}

bool
ConfigurationModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if (nullptr == MainWindow::instance()->kinematicModels[this->id])
	{
		return false;
	}
	
	if (index.isValid() && Qt::EditRole == role)
	{
		if (rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(MainWindow::instance()->kinematicModels[this->id].get()))
		{
			rl::math::Vector q = kinematic->getPosition();
			Eigen::Matrix<rl::math::Unit, Eigen::Dynamic, 1> qUnits = kinematic->getPositionUnits();
			
			if (rl::math::UNIT_RADIAN == qUnits(index.row()))
			{
				q(index.row()) = value.value<rl::math::Real>() * rl::math::DEG2RAD;
			}
			else
			{
				q(index.row()) = value.value<rl::math::Real>();
			}
			
			kinematic->setPosition(q);
			kinematic->forwardPosition();
			
			for (std::size_t i = 0; i < MainWindow::instance()->geometryModels[this->id]->getNumBodies(); ++i)
			{
				MainWindow::instance()->geometryModels[this->id]->getBody(i)->setFrame(kinematic->getFrame(i));
			}
			
			emit dataChanged(index, index);
			
			return true;
		}
	}
	
	return false;
}

bool
ConfigurationModel::setData(const rl::math::Vector& q)
{
	if (nullptr == MainWindow::instance()->kinematicModels[this->id])
	{
		return false;
	}
	
	if (rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(MainWindow::instance()->kinematicModels[this->id].get()))
	{
		kinematic->setPosition(q);
		kinematic->forwardPosition();
		
		for (std::size_t i = 0; i < MainWindow::instance()->geometryModels[this->id]->getNumBodies(); ++i)
		{
			MainWindow::instance()->geometryModels[this->id]->getBody(i)->setFrame(kinematic->getFrame(i));
		}
		
		emit dataChanged(this->createIndex(0, 0), this->createIndex(this->rowCount(), this->columnCount()));
		
		return true;
	}
	
	return false;
}
