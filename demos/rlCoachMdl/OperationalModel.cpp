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

#include <QStatusBar>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Exception.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/sg/Body.h>

#ifdef RL_MDL_NLOPT
#include <rl/mdl/NloptInverseKinematics.h>
#endif

#include "ConfigurationModel.h"
#include "OperationalModel.h"
#include "MainWindow.h"

OperationalModel::OperationalModel(QObject* parent) :
	QAbstractTableModel(parent),
	id(0)
{
}

OperationalModel::~OperationalModel()
{
}

int
OperationalModel::columnCount(const QModelIndex& parent) const
{
	return 6;
}

void
OperationalModel::configurationChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight)
{
	this->beginResetModel();
	this->endResetModel();
}

QVariant
OperationalModel::data(const QModelIndex& index, int role) const
{
	if (nullptr == MainWindow::instance()->kinematicModels[this->id])
	{
		return QVariant();
	}
	
	if (!index.isValid())
	{
		return QVariant();
	}
	
	const rl::math::Transform& transform = MainWindow::instance()->kinematicModels[this->id]->getOperationalPosition(index.row());
	rl::math::Transform::ConstTranslationPart position = transform.translation();
	rl::math::Vector3 orientation = transform.rotation().eulerAngles(2, 1, 0).reverse();
	
	switch (role)
	{
	case Qt::DisplayRole:
		switch (index.column())
		{
		case 0:
		case 1:
		case 2:
			return QString::number(position(index.column()), 'f', 4) + QString(" m");
			break;
		case 3:
		case 4:
		case 5:
			return QString::number(orientation(index.column() - 3) * rl::math::RAD2DEG, 'f', 2) + QChar(176);
			break;
		default:
			break;
		}
		break;
	case Qt::EditRole:
		switch (index.column())
		{
		case 0:
		case 1:
		case 2:
			return position(index.column());
			break;
		case 3:
		case 4:
		case 5:
			return orientation(index.column() - 3) * rl::math::RAD2DEG;
			break;
		default:
			break;
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
OperationalModel::flags(const QModelIndex &index) const
{
	if (!index.isValid())
	{
		return Qt::NoItemFlags;
	}
	
	return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
}

QVariant
OperationalModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (nullptr == MainWindow::instance()->kinematicModels[this->id])
	{
		return QVariant();
	}
	
	if (Qt::DisplayRole == role && Qt::Horizontal == orientation)
	{
		switch (section)
		{
		case 0:
			return "x";
			break;
		case 1:
			return "y";
			break;
		case 2:
			return "z";
			break;
		case 3:
			return "a";
			break;
		case 4:
			return "b";
			break;
		case 5:
			return "c";
			break;
		default:
			break;
		}
	}
	
	if (Qt::DisplayRole == role && Qt::Vertical == orientation)
	{
		return section;
	}
	
	return QVariant();
}

int
OperationalModel::rowCount(const QModelIndex& parent) const
{
	if (nullptr == MainWindow::instance()->kinematicModels[this->id])
	{
		return 0;
	}
	
	return MainWindow::instance()->kinematicModels[this->id]->getOperationalDof();
}

bool
OperationalModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if (nullptr == MainWindow::instance()->kinematicModels[this->id])
	{
		return false;
	}
	
	if (index.isValid() && Qt::EditRole == role)
	{
		if (rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(MainWindow::instance()->kinematicModels[this->id].get()))
		{
			rl::math::Transform transform = kinematic->getOperationalPosition(index.row());
			rl::math::Vector3 orientation = transform.linear().eulerAngles(2, 1, 0).reverse();
			
			switch (index.column())
			{
			case 0:
			case 1:
			case 2:
				transform.translation()(index.column()) = value.value<rl::math::Real>();
				break;
			case 3:
				transform.linear() = (
					rl::math::AngleAxis(orientation.z(), rl::math::Vector3::UnitZ()) *
					rl::math::AngleAxis(orientation.y(), rl::math::Vector3::UnitY()) *
					rl::math::AngleAxis(value.value<rl::math::Real>() * rl::math::DEG2RAD, rl::math::Vector3::UnitX())
				).toRotationMatrix();
				break;
			case 4:
				transform.linear() = (
					rl::math::AngleAxis(orientation.z(), rl::math::Vector3::UnitZ()) *
					rl::math::AngleAxis(value.value<rl::math::Real>() * rl::math::DEG2RAD, rl::math::Vector3::UnitY()) *
					rl::math::AngleAxis(orientation.x(), rl::math::Vector3::UnitX())
				).toRotationMatrix();
				break;
			case 5:
				transform.linear() = (
					rl::math::AngleAxis(value.value<rl::math::Real>() * rl::math::DEG2RAD, rl::math::Vector3::UnitZ()) *
					rl::math::AngleAxis(orientation.y(), rl::math::Vector3::UnitY()) *
					rl::math::AngleAxis(orientation.x(), rl::math::Vector3::UnitX())
				).toRotationMatrix();
				break;
			default:
				break;
			}
			
			rl::math::Vector q = kinematic->getPosition();
			
			std::shared_ptr<rl::mdl::InverseKinematics> ik;
			
			if ("JacobianInverseKinematics" == MainWindow::instance()->ikAlgorithmComboBox->currentText())
			{
				ik = std::make_shared<rl::mdl::JacobianInverseKinematics>(kinematic);
				rl::mdl::JacobianInverseKinematics* jacobianIk = static_cast<rl::mdl::JacobianInverseKinematics*>(ik.get());
				jacobianIk->duration = std::chrono::milliseconds(MainWindow::instance()->ikDurationSpinBox->cleanText().toUInt());
				jacobianIk->svd = "SVD" == MainWindow::instance()->ikJacobianInverseComboBox->currentText() ? true : false;
				jacobianIk->transpose = "Transpose" == MainWindow::instance()->ikJacobianComboBox->currentText() ? true : false;
			}
#ifdef RL_MDL_NLOPT
			else if ("NloptInverseKinematics" == MainWindow::instance()->ikAlgorithmComboBox->currentText())
			{
				ik = std::make_shared<rl::mdl::NloptInverseKinematics>(kinematic);
				rl::mdl::NloptInverseKinematics* nloptIk = static_cast<rl::mdl::NloptInverseKinematics*>(ik.get());
				nloptIk->duration = std::chrono::milliseconds(MainWindow::instance()->ikDurationSpinBox->cleanText().toUInt());
			}
#endif
			
#if 0
			ik->goals.push_back(::std::make_pair(transform, index.row()));
#else
			for (std::size_t i = 0; i < kinematic->getOperationalDof(); ++i)
			{
				ik->goals.push_back(::std::make_pair(i == index.row() ? transform : kinematic->getOperationalPosition(i), i));
			}
#endif
			
			std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
			bool solved = ik->solve();
			std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now();
			
			if (solved)
			{
				MainWindow::instance()->statusBar()->showMessage("IK solved in " + QString::number(std::chrono::duration<double>(stop - start).count() * rl::math::UNIT2MILLI) + " ms", 2000);
				
				kinematic->forwardPosition();
				
				for (std::size_t i = 0; i < MainWindow::instance()->geometryModels[this->id]->getNumBodies(); ++i)
				{
					MainWindow::instance()->geometryModels[this->id]->getBody(i)->setFrame(kinematic->getFrame(i));
				}
				
				emit dataChanged(this->createIndex(0, 0), this->createIndex(this->rowCount(), this->columnCount()));
				
				return true;
			}
			else
			{
				MainWindow::instance()->statusBar()->showMessage("IK failed", 2000);
				
				kinematic->setPosition(q);
				kinematic->forwardPosition();
			}
		}
	}
	
	return false;
}
