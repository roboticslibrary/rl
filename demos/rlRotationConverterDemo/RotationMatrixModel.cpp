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
#include "RotationMatrixModel.h"

RotationMatrixModel::RotationMatrixModel(QObject* parent) :
	QAbstractTableModel(parent),
	rotation(nullptr)
{
}

RotationMatrixModel::~RotationMatrixModel()
{
}

int
RotationMatrixModel::columnCount(const QModelIndex& parent) const
{
	return 3;
}

QVariant
RotationMatrixModel::data(const QModelIndex& index, int role) const
{
	if (nullptr == this->rotation)
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
		return (*this->rotation)(index.row(), index.column());
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
RotationMatrixModel::flags(const QModelIndex &index) const
{
	if (!index.isValid())
	{
		return Qt::NoItemFlags;
	}
	
	return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
}

QVariant
RotationMatrixModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (nullptr == this->rotation)
	{
		return QVariant();
	}
	
	if (Qt::DisplayRole == role)
	{
		return section;
	}
	
	return QVariant();
}

void
RotationMatrixModel::invalidate()
{
	this->beginResetModel();
	this->endResetModel();
}

int
RotationMatrixModel::rowCount(const QModelIndex& parent) const
{
	return 3;
}

bool
RotationMatrixModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if (nullptr == this->rotation)
	{
		return false;
	}
	
	if (index.isValid() && Qt::EditRole == role)
	{
		(*this->rotation)(index.row(), index.column()) = value.value<rl::math::Real>();
		emit dataChanged(this->createIndex(index.row(), index.column()), this->createIndex(index.row(), index.column()));
		return true;
	}
	
	return false;
}
