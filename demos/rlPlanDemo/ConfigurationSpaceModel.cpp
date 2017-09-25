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

#include "ConfigurationSpaceModel.h"
#include "ConfigurationSpaceScene.h"
#include "MainWindow.h"
#include "Thread.h"
#include "Viewer.h"

ConfigurationSpaceModel::ConfigurationSpaceModel(QObject* parent) :
	QAbstractTableModel(parent),
	configurationSpaceScene(nullptr)
{
}

ConfigurationSpaceModel::~ConfigurationSpaceModel()
{
}

int
ConfigurationSpaceModel::columnCount(const QModelIndex& parent) const
{
	return 1;
}

QVariant
ConfigurationSpaceModel::data(const QModelIndex& index, int role) const
{
	if (nullptr == this->configurationSpaceScene)
	{
		return QVariant();
	}
	
	if (nullptr == this->configurationSpaceScene->model)
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
		switch (index.row())
		{
		case 0:
			return static_cast<unsigned int>(this->configurationSpaceScene->axis0);
			break;
		case 1:
			return static_cast<unsigned int>(this->configurationSpaceScene->axis1);
			break;
		case 2:
			return this->configurationSpaceScene->delta0;
			break;
		case 3:
			return this->configurationSpaceScene->delta1;
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
ConfigurationSpaceModel::flags(const QModelIndex &index) const
{
	if (!index.isValid())
	{
		return Qt::NoItemFlags;
	}
	
	return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
}

QVariant
ConfigurationSpaceModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (nullptr == this->configurationSpaceScene)
	{
		return QVariant();
	}
	
	if (nullptr == this->configurationSpaceScene->model)
	{
		return QVariant();
	}
	
	if (Qt::DisplayRole == role && Qt::Vertical == orientation)
	{
		switch (section)
		{
		case 0:
			return "axis0";
			break;
		case 1:
			return "axis1";
			break;
		case 2:
			return "delta0";
			break;
		case 3:
			return "delta1";
			break;
		default:
			break;
		}
	}
	
	return QVariant();
}

void
ConfigurationSpaceModel::invalidate()
{
	this->beginResetModel();
	this->endResetModel();
}

int
ConfigurationSpaceModel::rowCount(const QModelIndex& parent) const
{
	if (nullptr == this->configurationSpaceScene)
	{
		return 0;
	}
	
	
	if (nullptr == this->configurationSpaceScene->model)
	{
		return 0;
	}
	return 4;
}

bool
ConfigurationSpaceModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if (nullptr == this->configurationSpaceScene)
	{
		return false;
	}
	
	if (nullptr == this->configurationSpaceScene->model)
	{
		return false;
	}
	
	if (MainWindow::instance()->thread->isRunning())
	{
		return false;
	}
	
	if (index.isValid() && Qt::EditRole == role)
	{
		switch (index.row())
		{
		case 0:
			if (value.value<std::size_t>() < this->configurationSpaceScene->model->getDofPosition())
			{
				this->configurationSpaceScene->axis0 = value.value<std::size_t>();
			}
			else
			{
				return false;
			}
			break;
		case 1:
			if (value.value<std::size_t>() < this->configurationSpaceScene->model->getDofPosition())
			{
				this->configurationSpaceScene->axis1 = value.value<std::size_t>();
			}
			else
			{
				return false;
			}
			break;
		case 2:
			if (value.value<rl::math::Real>() > 0)
			{
				this->configurationSpaceScene->delta0 = value.value<rl::math::Real>();
			}
			else
			{
				return false;
			}
			break;
		case 3:
			if (value.value<rl::math::Real>() > 0)
			{
				this->configurationSpaceScene->delta1 = value.value<rl::math::Real>();
			}
			else
			{
				return false;
			}
			break;
		default:
			break;
		}
		
		this->configurationSpaceScene->init();
		
		emit dataChanged(index, index);
		
		return true;
	}
	
	return false;
}
