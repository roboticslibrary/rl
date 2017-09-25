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

#include <rl/plan/Eet.h>
#include <rl/plan/Prm.h>
#include <rl/plan/Rrt.h>
#include <rl/plan/RrtGoalBias.h>
#include <rl/plan/Verifier.h>

#include "MainWindow.h"
#include "PlannerModel.h"
#include "Thread.h"

PlannerModel::PlannerModel(QObject* parent) :
	QAbstractTableModel(parent)
{

}

PlannerModel::~PlannerModel()
{
}

int
PlannerModel::columnCount(const QModelIndex& parent) const
{
	return 1;
}

QVariant
PlannerModel::data(const QModelIndex& index, int role) const
{
	if (nullptr == MainWindow::instance()->planner)
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
			return std::chrono::duration_cast<std::chrono::duration<double>>(
				MainWindow::instance()->planner->duration
			).count();
			break;
		default:
			break;
		}
		
		if (rl::plan::Eet* eet = dynamic_cast<rl::plan::Eet*>(MainWindow::instance()->planner.get()))
		{
			switch (index.row())
			{
			case 4:
				return eet->alpha;
				break;
			case 5:
				return eet->beta;
				break;
			case 6:
				return eet->distanceWeight;
				break;
			case 7:
				return eet->gamma;
				break;
			case 8:
				return eet->max.x();
				break;
			case 9:
				return eet->max.y();
				break;
			case 10:
				return eet->max.z();
				break;
			case 11:
				return eet->min.x();
				break;
			case 12:
				return eet->min.y();
				break;
			case 13:
				return eet->min.z();
				break;
			default:
				break;
			}
		}
		
		if (rl::plan::Prm* prm = dynamic_cast<rl::plan::Prm*>(MainWindow::instance()->planner.get()))
		{
			switch (index.row())
			{
			case 1:
				return static_cast<unsigned int>(prm->degree);
				break;
			case 2:
				return prm->verifier->delta;
				break;
			case 3:
				return static_cast<unsigned int>(prm->k);
				break;
			case 4:
				return prm->radius;
				break;
			default:
				break;
			}
		}
		
		if (rl::plan::Rrt* rrt = dynamic_cast<rl::plan::Rrt*>(MainWindow::instance()->planner.get()))
		{
			switch (index.row())
			{
			case 1:
				return rrt->delta;
				break;
			case 2:
				return rrt->epsilon;
				break;
			default:
				break;
			}
		}
		
		if (rl::plan::RrtGoalBias* rrtGoalBias = dynamic_cast<rl::plan::RrtGoalBias*>(MainWindow::instance()->planner.get()))
		{
			switch (index.row())
			{
			case 3:
				return rrtGoalBias->probability;
				break;
			default:
				break;
			}
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
PlannerModel::flags(const QModelIndex &index) const
{
	if (!index.isValid())
	{
		return Qt::NoItemFlags;
	}
	
	return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
}

QVariant
PlannerModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (nullptr == MainWindow::instance()->planner)
	{
		return QVariant();
	}
	
	if (Qt::DisplayRole == role && Qt::Vertical == orientation)
	{
		switch (section)
		{
		case 0:
			return "duration";
			break;
		default:
			break;
		}
		
		if (dynamic_cast<rl::plan::Eet*>(MainWindow::instance()->planner.get()))
		{
			switch (section)
			{
			case 4:
				return "alpha";
				break;
			case 5:
				return "beta";
				break;
			case 6:
				return "distanceWeight";
				break;
			case 7:
				return "gamma";
				break;
			case 8:
				return "max.x";
				break;
			case 9:
				return "max.y";
				break;
			case 10:
				return "max.z";
				break;
			case 11:
				return "min.x";
				break;
			case 12:
				return "min.y";
				break;
			case 13:
				return "min.z";
				break;
			default:
				break;
			}
		}
		
		if (dynamic_cast<rl::plan::Prm*>(MainWindow::instance()->planner.get()))
		{
			switch (section)
			{
			case 1:
				return "degree";
				break;
			case 2:
				return "delta";
				break;
			case 3:
				return "k";
				break;
			case 4:
				return "radius";
				break;
			default:
				break;
			}
		}
		
		if (dynamic_cast<rl::plan::Rrt*>(MainWindow::instance()->planner.get()))
		{
			switch (section)
			{
			case 1:
				return "delta";
				break;
			case 2:
				return "epsilon";
				break;
			default:
				break;
			}
		}
		
		if (dynamic_cast<rl::plan::RrtGoalBias*>(MainWindow::instance()->planner.get()))
		{
			switch (section)
			{
			case 3:
				return "probability";
				break;
			default:
				break;
			}
		}
	}
	
	return QVariant();
}

void
PlannerModel::invalidate()
{
	this->beginResetModel();
	this->endResetModel();
}

int
PlannerModel::rowCount(const QModelIndex& parent) const
{
	if (nullptr == MainWindow::instance()->planner)
	{
		return 0;
	}
	
	if (dynamic_cast<rl::plan::Prm*>(MainWindow::instance()->planner.get()))
	{
		return 5;
	}
	else if (dynamic_cast<rl::plan::Eet*>(MainWindow::instance()->planner.get()))
	{
		return 14;
	}
	else if (dynamic_cast<rl::plan::RrtGoalBias*>(MainWindow::instance()->planner.get()))
	{
		return 4;
	}
	else if (dynamic_cast<rl::plan::Rrt*>(MainWindow::instance()->planner.get()))
	{
		return 3;
	}
	
	return 0;
}

bool
PlannerModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if (nullptr == MainWindow::instance()->planner)
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
			MainWindow::instance()->planner->duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
				std::chrono::duration<double>(value.value<double>())
			);
			break;
		default:
			break;
		}
		
		if (rl::plan::Eet* eet = dynamic_cast<rl::plan::Eet*>(MainWindow::instance()->planner.get()))
		{
			switch (index.row())
			{
			case 4:
				eet->alpha = value.value<rl::math::Real>();
				break;
			case 5:
				eet->beta = value.value<rl::math::Real>();
				break;
			case 6:
				eet->distanceWeight = value.value<rl::math::Real>();
				break;
			case 7:
				eet->gamma = value.value<rl::math::Real>();
				break;
			case 8:
				eet->max.x() = value.value<rl::math::Real>();
				break;
			case 9:
				eet->max.y() = value.value<rl::math::Real>();
				break;
			case 10:
				eet->max.z() = value.value<rl::math::Real>();
				break;
			case 11:
				eet->min.x() = value.value<rl::math::Real>();
				break;
			case 12:
				eet->min.y() = value.value<rl::math::Real>();
				break;
			case 13:
				eet->min.z() = value.value<rl::math::Real>();
				break;
			default:
				break;
			}
		}
		
		if (rl::plan::Prm* prm = dynamic_cast<rl::plan::Prm*>(MainWindow::instance()->planner.get()))
		{
			switch (index.row())
			{
			case 1:
				prm->degree = value.value<std::size_t>();
				break;
			case 2:
				prm->verifier->delta = value.value<rl::math::Real>();
				break;
			case 3:
				prm->k = value.value<std::size_t>();
				break;
			case 4:
				prm->radius = value.value<rl::math::Real>();
				break;
			default:
				break;
			}
		}
		
		if (rl::plan::Rrt* rrt = dynamic_cast<rl::plan::Rrt*>(MainWindow::instance()->planner.get()))
		{
			switch (index.row())
			{
			case 1:
				rrt->delta = value.value<rl::math::Real>();
				break;
			case 2:
				rrt->epsilon = value.value<rl::math::Real>();
				break;
			default:
				break;
			}
		}
		
		if (rl::plan::RrtGoalBias* rrtGoalBias = dynamic_cast<rl::plan::RrtGoalBias*>(MainWindow::instance()->planner.get()))
		{
			switch (index.row())
			{
			case 3:
				rrtGoalBias->probability = value.value<rl::math::Real>();
				break;
			default:
				break;
			}
		}
		
		emit dataChanged(index, index);
		
		return true;
	}
	
	return false;
}
