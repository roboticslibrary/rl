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

#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>

#include "BodyModel.h"
#include "MainWindow.h"

BodyModel::BodyModel(QObject* parent) :
	QAbstractTableModel(parent),
	collision(nullptr),
	view(nullptr)
{
}

BodyModel::~BodyModel()
{
}

int
BodyModel::columnCount(const QModelIndex& parent) const
{
	return 1;
}

QVariant
BodyModel::data(const QModelIndex& index, int role) const
{
	if (nullptr == this->collision || nullptr == this->view)
	{
		return QVariant();
	}
	
	if (!index.isValid())
	{
		return QVariant();
	}
	
	rl::math::Transform transform;
	this->collision->getFrame(transform);
	rl::math::Transform::TranslationPart position = transform.translation();
	rl::math::Vector3 orientation = transform.rotation().eulerAngles(2, 1, 0).reverse();
	
	switch (role)
	{
	case Qt::DisplayRole:
		switch (index.row())
		{
		case 0:
		case 1:
		case 2:
			return QString::number(position(index.row()), 'f', 4) + QString(" m");
			break;
		case 3:
		case 4:
		case 5:
			return QString::number(orientation(index.row() - 3) * rl::math::RAD2DEG, 'f', 2) + QChar(176);
			break;
		default:
			break;
		}
		break;
	case Qt::EditRole:
		switch (index.row())
		{
		case 0:
		case 1:
		case 2:
			return position(index.row());
			break;
		case 3:
		case 4:
		case 5:
			return orientation(index.row() - 3) * rl::math::RAD2DEG;
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
BodyModel::flags(const QModelIndex &index) const
{
	if (!index.isValid())
	{
		return Qt::NoItemFlags;
	}
	
	return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
}

QVariant
BodyModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (nullptr == this->collision || nullptr == this->view)
	{
		return QVariant();
	}
	
	if (Qt::DisplayRole == role && Qt::Vertical == orientation)
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
	
	return QVariant();
}

int
BodyModel::rowCount(const QModelIndex& parent) const
{
	if (nullptr == this->collision || nullptr == this->view)
	{
		return 0;
	}
	
	return 6;
}

void
BodyModel::setBody(rl::sg::Body* collision, rl::sg::Body* view)
{
	this->beginResetModel();
	this->collision = collision;
	this->view = view;
	this->endResetModel();
}

bool
BodyModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if (nullptr == this->collision || nullptr == this->view)
	{
		return false;
	}
	
	if (index.isValid() && Qt::EditRole == role)
	{
		rl::math::Transform transform;
		this->collision->getFrame(transform);
		rl::math::Transform::TranslationPart position = transform.translation();
		rl::math::Vector3 orientation = transform.linear().eulerAngles(2, 1, 0).reverse();
		
		switch (index.row())
		{
		case 0:
		case 1:
		case 2:
			transform.translation()(index.row()) = value.value<rl::math::Real>();
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
		
		this->collision->setFrame(transform);
		this->view->setFrame(transform);
		
		MainWindow::instance()->test();
		emit dataChanged(this->createIndex(0, 0), this->createIndex(this->rowCount(), this->columnCount()));
		
		return true;
	}
	
	return false;
}
