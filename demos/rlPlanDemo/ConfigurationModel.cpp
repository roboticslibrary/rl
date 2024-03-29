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

#include <rl/math/Constants.h>

#include "ConfigurationModel.h"
#include "MainWindow.h"
#include "Thread.h"
#include "Viewer.h"

ConfigurationModel::ConfigurationModel(QObject* parent) :
	QAbstractTableModel(parent)
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
	if (nullptr == MainWindow::instance()->model)
	{
		return QVariant();
	}
	
	if (!index.isValid())
	{
		return QVariant();
	}
	
	switch (role)
	{
	case Qt::DisplayRole:
	case Qt::EditRole:
		{
			Eigen::Matrix<rl::math::Units, Eigen::Dynamic, 1> qUnits = MainWindow::instance()->model->getPositionUnits();
			
			if (rl::math::Units::radian == qUnits(index.row()))
			{
				return (*MainWindow::instance()->q)(index.row()) * rl::math::constants::rad2deg;
			}
			else
			{
				return (*MainWindow::instance()->q)(index.row());
			}
		}
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
	if (Qt::DisplayRole == role && Qt::Vertical == orientation)
	{
		return QString::number(section);
	}
	
	return QVariant();
}

void
ConfigurationModel::invalidate()
{
	this->beginResetModel();
	this->endResetModel();
}

int
ConfigurationModel::rowCount(const QModelIndex& parent) const
{
	if (nullptr == MainWindow::instance()->model)
	{
		return 0;
	}
	
	return MainWindow::instance()->model->getDofPosition();
}

bool
ConfigurationModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if (nullptr == MainWindow::instance()->model)
	{
		return false;
	}
	
	if (MainWindow::instance()->thread->isRunning())
	{
		return false;
	}
	
	if (index.isValid() && Qt::EditRole == role)
	{
		Eigen::Matrix<rl::math::Units, Eigen::Dynamic, 1> qUnits = MainWindow::instance()->model->getPositionUnits();
		
		if (rl::math::Units::radian == qUnits(index.row()))
		{
			(*MainWindow::instance()->q)(index.row()) = value.value<rl::math::Real>() * rl::math::constants::deg2rad;
		}
		else
		{
			(*MainWindow::instance()->q)(index.row()) = value.value<rl::math::Real>();
		}
		
		MainWindow::instance()->thread->drawConfiguration(*MainWindow::instance()->q);
		
		emit dataChanged(index, index);
		
		return true;
	}
	
	return false;
}
