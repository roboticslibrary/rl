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

#include <QTableView>

#include "MainWindow.h"
#include "QuaternionModel.h"

QuaternionModel::QuaternionModel(QObject* parent) :
	QAbstractTableModel(parent),
	quaternion(nullptr)
{
}

QuaternionModel::~QuaternionModel()
{
}

int
QuaternionModel::columnCount(const QModelIndex& parent) const
{
	return 4;
}

QVariant
QuaternionModel::data(const QModelIndex& index, int role) const
{
	if (nullptr == this->quaternion)
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
		switch (index.column())
		{
		case 0:
		case 1:
		case 2:
			return this->quaternion->vec()(index.column());
			break;
		case 3:
			return this->quaternion->w();
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
QuaternionModel::flags(const QModelIndex &index) const
{
	if (!index.isValid())
	{
		return Qt::NoItemFlags;
	}
	
	return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
}

QVariant
QuaternionModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (nullptr == this->quaternion)
	{
		return QVariant();
	}
	
	if (Qt::DisplayRole == role && Qt::Horizontal == orientation)
	{
		switch (section)
		{
		case 0:
			return "X";
			break;
		case 1:
			return "Y";
			break;
		case 2:
			return "Z";
			break;
		case 3:
			return "W";
			break;
		default:
			break;
		}
	}
	
	return QVariant();
}

void
QuaternionModel::invalidate()
{
	this->beginResetModel();
	this->endResetModel();
}

int
QuaternionModel::rowCount(const QModelIndex& parent) const
{
	return 1;
}

bool
QuaternionModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if (nullptr == this->quaternion)
	{
		return false;
	}
	
	if (index.isValid() && Qt::EditRole == role)
	{
		switch (index.column())
		{
		case 0:
		case 1:
		case 2:
			this->quaternion->vec()(index.column()) = value.value<rl::math::Real>();
			break;
		case 3:
			this->quaternion->w() = value.value<rl::math::Real>();
			break;
		default:
			break;
		}
		
		emit dataChanged(this->createIndex(index.row(), index.column()), this->createIndex(index.row(), index.column()));
		
		return true;
	}
	
	return false;
}
