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
#include <rl/math/Unit.h>

#include "EulerAnglesModel.h"
#include "MainWindow.h"

EulerAnglesModel::EulerAnglesModel(QObject* parent) :
	QAbstractTableModel(parent),
	eulerAngles(nullptr),
	eulerAnglesRadians(nullptr),
	eulerAxes(nullptr)
{
}

EulerAnglesModel::~EulerAnglesModel()
{
}

int
EulerAnglesModel::columnCount(const QModelIndex& parent) const
{
	return 3;
}

QVariant
EulerAnglesModel::data(const QModelIndex& index, int role) const
{
	if (nullptr == this->eulerAngles || nullptr == this->eulerAnglesRadians || nullptr == this->eulerAxes)
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
		return *this->eulerAnglesRadians ? (*this->eulerAngles)(index.column()) : (*this->eulerAngles)(index.column()) * rl::math::RAD2DEG;
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
EulerAnglesModel::flags(const QModelIndex &index) const
{
	if (!index.isValid())
	{
		return Qt::NoItemFlags;
	}
	
	return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
}

QVariant
EulerAnglesModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (nullptr == this->eulerAngles || nullptr == this->eulerAnglesRadians || nullptr == this->eulerAxes)
	{
		return QVariant();
	}
	
	if (Qt::DisplayRole == role && Qt::Horizontal == orientation)
	{
		switch ((*this->eulerAxes)[section])
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
		default:
			break;
		}
	}
	
	return QVariant();
}

void
EulerAnglesModel::invalidate()
{
	this->beginResetModel();
	this->endResetModel();
}

int
EulerAnglesModel::rowCount(const QModelIndex& parent) const
{
	return 1;
}

bool
EulerAnglesModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if (nullptr == this->eulerAngles || nullptr == this->eulerAnglesRadians || nullptr == this->eulerAxes)
	{
		return false;
	}
	
	if (index.isValid() && Qt::EditRole == role)
	{
		(*this->eulerAngles)(index.column()) = *this->eulerAnglesRadians ? value.value<rl::math::Real>() : value.value<rl::math::Real>() * rl::math::DEG2RAD;
		emit dataChanged(this->createIndex(index.row(), index.column()), this->createIndex(index.row(), index.column()));
		return true;
	}
	
	return false;
}
