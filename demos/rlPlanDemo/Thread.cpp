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

#include <fstream>
#include <QApplication>
#include <QDateTime>
#include <QMutexLocker>
#include <rl/math/Quaternion.h>
#include <rl/math/Unit.h>
#include <rl/plan/Eet.h>
#include <rl/plan/GnatNearestNeighbors.h>
#include <rl/plan/KdtreeBoundingBoxNearestNeighbors.h>
#include <rl/plan/KdtreeNearestNeighbors.h>
#include <rl/plan/LinearNearestNeighbors.h>
#include <rl/plan/Prm.h>
#include <rl/plan/Rrt.h>

#include "MainWindow.h"
#include "Thread.h"
#include "Viewer.h"

Thread::Thread(QObject* parent) :
	QThread(parent),
	animate(true),
	quit(false),
	swept(false),
	running(false)
{
}

Thread::~Thread()
{
}

void
Thread::drawConfiguration(const rl::math::Vector& q)
{
	emit configurationRequested(q);
}

void
Thread::drawConfigurationEdge(const rl::math::Vector& q0, const rl::math::Vector& q1, const bool& free)
{
	emit configurationEdgeRequested(q0, q1, free);
}

void
Thread::drawConfigurationPath(const rl::plan::VectorList& path)
{
	emit configurationPathRequested(path);
}

void
Thread::drawConfigurationVertex(const rl::math::Vector& q, const bool& free)
{
	emit configurationVertexRequested(q, free);
}

void
Thread::drawLine(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1)
{
	emit lineRequested(xyz0, xyz1);
}

void
Thread::drawPoint(const rl::math::Vector& xyz)
{
	emit pointRequested(xyz);
}

void
Thread::drawSphere(const rl::math::Vector& center, const rl::math::Real& radius)
{
	emit sphereRequested(center, radius);
}

void
Thread::drawSweptVolume(const rl::plan::VectorList& path)
{
	emit sweptVolumeRequested(path);
}

void
Thread::drawWork(const rl::math::Transform& t)
{
	emit workRequested(t);
}

void
Thread::drawWorkEdge(const rl::math::Vector& q0, const rl::math::Vector& q1)
{
//	emit workEdgeRequested(q0, q1);
}

void
Thread::drawWorkPath(const rl::plan::VectorList& path)
{
	emit workPathRequested(path);
}

void
Thread::drawWorkVertex(const rl::math::Vector& q)
{
//	emit workVertexRequested(q);
}

void
Thread::reset()
{
	emit resetRequested();
}

void
Thread::resetEdges()
{
	emit edgeResetRequested();
}

void
Thread::resetLines()
{
	emit lineResetRequested();
}

void
Thread::resetPoints()
{
	emit pointResetRequested();
}

void
Thread::resetSpheres()
{
	emit sphereResetRequested();
}

void
Thread::resetVertices()
{
	emit vertexResetRequested();
}

void
Thread::run()
{
	QMutexLocker lock(&MainWindow::instance()->mutex);
	
	this->running = true;
	
	if (nullptr != MainWindow::instance()->planner->viewer)
	{
		emit statusChanged("Showing start configuration.");
		this->drawConfiguration(*MainWindow::instance()->planner->start);
		QThread::usleep(static_cast<std::size_t>(2.0 * 1000.0 * 1000.0));
	}
	
	if (!this->running) return;
	
	if (nullptr != MainWindow::instance()->planner->viewer)
	{
		emit statusChanged("Showing goal configuration.");
		this->drawConfiguration(*MainWindow::instance()->planner->goal);
		QThread::usleep(static_cast<std::size_t>(2.0 * 1000.0 * 1000.0));
	}
	
	if (!this->running) return;
	
	if (!MainWindow::instance()->planner->verify())
	{
		return;
	}
	
	emit statusChanged("Solving...");
	
	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	bool solved = MainWindow::instance()->planner->solve();
	std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now();
	
	double plannerDuration = std::chrono::duration_cast<std::chrono::duration<double>>(stop - start).count();
	
	emit statusChanged("Planner " + QString(solved ? "succeeded" : "failed") + " in " + QString::number(plannerDuration * 1000) + " ms.");
	
	std::fstream benchmark;
	benchmark.open("benchmark.csv", std::ios::app | std::ios::in | std::ios::out);
	int peek = benchmark.peek();
	
	if (std::ifstream::traits_type::eof() == peek)
	{
		benchmark.clear();
		benchmark << "Date,Time,Solved,Engine,Planner,Robot,Nearest Neighbors,Vertices,Edges,Total CD,Free CD,Exploration Duration (s),Duration (s), Path Length" << std::endl;
	}
	else
	{
		benchmark.seekp(peek);
	}
	
	benchmark << QDateTime::currentDateTime().toString("yyyy-MM-dd,HH:mm:ss.zzz").toStdString();
	benchmark << ",";
	benchmark << (solved ? "true" : "false");
	benchmark << ",";
	benchmark << MainWindow::instance()->engine.toUpper().toStdString();
	benchmark << ",";
	benchmark << MainWindow::instance()->planner->getName();
	benchmark << ",";
	benchmark << MainWindow::instance()->model->getManufacturer();
	benchmark << (!MainWindow::instance()->model->getManufacturer().empty() && !MainWindow::instance()->model->getName().empty() ? " " : "");
	benchmark << MainWindow::instance()->model->getName();
	benchmark << ",";
	
	if (!MainWindow::instance()->nearestNeighbors.empty())
	{
		if (rl::plan::GnatNearestNeighbors* gnatNearestNeighbors = dynamic_cast<rl::plan::GnatNearestNeighbors*>(MainWindow::instance()->nearestNeighbors.front().get()))
		{
			benchmark << "GNAT";
			
			boost::optional<std::size_t> checks = gnatNearestNeighbors->getChecks();
			
			if (checks)
			{
				benchmark << " (" << checks.get() << " Checks)";
			}
		}
		else if (rl::plan::KdtreeBoundingBoxNearestNeighbors* kdtreeBoundingBoxNearestNeighbors = dynamic_cast<rl::plan::KdtreeBoundingBoxNearestNeighbors*>(MainWindow::instance()->nearestNeighbors.front().get()))
		{
			benchmark << "k-d Tree Bounding Box";
			
			boost::optional<std::size_t> checks = kdtreeBoundingBoxNearestNeighbors->getChecks();
			
			if (checks)
			{
				benchmark << " (" << checks.get() << " Checks)";
			}
		}
		else if (rl::plan::KdtreeNearestNeighbors* kdtreeNearestNeighbors = dynamic_cast<rl::plan::KdtreeNearestNeighbors*>(MainWindow::instance()->nearestNeighbors.front().get()))
		{
			benchmark << "k-d Tree";
			
			boost::optional<std::size_t> checks = kdtreeNearestNeighbors->getChecks();
			
			if (checks)
			{
				benchmark << " (" << checks.get() << " Checks)";
			}
		}
		else if (rl::plan::LinearNearestNeighbors* linearNearestNeighbors = dynamic_cast<rl::plan::LinearNearestNeighbors*>(MainWindow::instance()->nearestNeighbors.front().get()))
		{
			benchmark << "Linear";
		}
	}
	
	benchmark << ",";
	
	if (rl::plan::Prm* prm = dynamic_cast<rl::plan::Prm*>(MainWindow::instance()->planner.get()))
	{
		benchmark << prm->getNumVertices();
	}
	else if (rl::plan::Rrt* rrt = dynamic_cast<rl::plan::Rrt*>(MainWindow::instance()->planner.get()))
	{
		benchmark << rrt->getNumVertices();
	}
	
	benchmark << ",";
	
	if (rl::plan::Prm* prm = dynamic_cast<rl::plan::Prm*>(MainWindow::instance()->planner.get()))
	{
		benchmark << prm->getNumEdges();
	}
	else if (rl::plan::Rrt* rrt = dynamic_cast<rl::plan::Rrt*>(MainWindow::instance()->planner.get()))
	{
		benchmark << rrt->getNumEdges();
	}
	
	benchmark << ",";
	benchmark << MainWindow::instance()->model->getTotalQueries();
	benchmark << ",";
	benchmark << MainWindow::instance()->model->getFreeQueries();
	benchmark << ",";
	
	if (rl::plan::Eet* eet = dynamic_cast<rl::plan::Eet*>(MainWindow::instance()->planner.get()))
	{
		benchmark << std::chrono::duration_cast<std::chrono::duration<double>>(eet->getExplorationDuration()).count();
	}
	else
	{
		benchmark << 0;
	}
	
	benchmark << ",";
	benchmark << plannerDuration;
	
	rl::plan::VectorList path;
	
	if (solved)
	{
		path = MainWindow::instance()->planner->getPath();
		
		rl::plan::VectorList::iterator i = path.begin();
		rl::plan::VectorList::iterator j = ++path.begin();
		
		rl::math::Real length = 0;
		
		for (; i != path.end() && j != path.end(); ++i, ++j)
		{
			length += MainWindow::instance()->model->distance(*i, *j);
		}
		
		benchmark << ",";
		benchmark << length;
	}
	
	benchmark << std::endl;
	benchmark.close();
	
	if (!this->running) return;
	
	if (this->quit)
	{
		QApplication::quit();
		return;
	}
	
	if (solved)
	{
		if (nullptr != MainWindow::instance()->planner->viewer)
		{
			this->drawConfigurationPath(path);
		}
		
		if (!this->running) return;
		
		if (nullptr != MainWindow::instance()->optimizer)
		{
			if (nullptr != MainWindow::instance()->planner->viewer)
			{
				QThread::usleep(static_cast<std::size_t>(2.0 * 1000.0 * 1000.0));
			}
			
			emit statusChanged("Planner " + QString(solved ? "succeeded" : "failed") + " in " + QString::number(plannerDuration) + " ms. Optimizing...");
			
			start = std::chrono::steady_clock::now();
			MainWindow::instance()->optimizer->process(path);
			stop = std::chrono::steady_clock::now();
			
			double optimizerDuration = std::chrono::duration_cast<std::chrono::duration<double>>(stop - start).count() * 1000;
			
			emit statusChanged("Planner " + QString(solved ? "succeeded" : "failed") + " in " + QString::number(plannerDuration) + " ms. Optimizer finished in " + QString::number(optimizerDuration) + " ms.");
			
			if (nullptr != MainWindow::instance()->planner->viewer)
			{
				this->drawConfigurationPath(path);
			}
		}
		
		if (nullptr == MainWindow::instance()->planner->viewer) return;
		
		if (this->swept)
		{
			this->drawSweptVolume(path);
		}
		
		rl::math::Vector diff(MainWindow::instance()->model->getDofPosition());
		rl::math::Vector inter(MainWindow::instance()->model->getDofPosition());
		
		while (this->animate)
		{
			if (!this->running) break;
			
			rl::plan::VectorList::iterator i = path.begin();
			rl::plan::VectorList::iterator j = ++path.begin();
			
			if (i != path.end() && j != path.end())
			{
				this->drawConfiguration(*i);
				QThread::usleep(static_cast<std::size_t>(0.01 * 1000.0 * 1000.0));
			}
			
			rl::math::Real delta = MainWindow::instance()->viewer->delta;
			
			for (; i != path.end() && j != path.end(); ++i, ++j)
			{
				diff = *j - *i;
				
				rl::math::Real steps = std::ceil(MainWindow::instance()->model->distance(*i, *j) / delta);
				
				for (std::size_t k = 1; k < steps + 1; ++k)
				{
					if (!this->running) break;
					
					MainWindow::instance()->model->interpolate(*i, *j, k / steps, inter);
					this->drawConfiguration(inter);
					QThread::usleep(static_cast<std::size_t>(0.01 * 1000.0 * 1000.0));
				}
			}
			
			if (!this->running) break;
			
			rl::plan::VectorList::reverse_iterator ri = path.rbegin();
			rl::plan::VectorList::reverse_iterator rj = ++path.rbegin();
			
			if (ri != path.rend() && rj != path.rend())
			{
				this->drawConfiguration(*ri);
				QThread::usleep(static_cast<std::size_t>(0.01 * 1000.0 * 1000.0));
			}
			
			for (; ri != path.rend() && rj != path.rend(); ++ri, ++rj)
			{
				diff = *rj - *ri;
				
				rl::math::Real steps = std::ceil(MainWindow::instance()->model->distance(*ri, *rj) / delta);
				
				for (std::size_t k = 1; k < steps + 1; ++k)
				{
					if (!this->running) break;
					
					MainWindow::instance()->model->interpolate(*ri, *rj, k / steps, inter);
					this->drawConfiguration(inter);
					QThread::usleep(static_cast<std::size_t>(0.01 * 1000.0 * 1000.0));
				}
			}
		}
	}
}

void
Thread::showMessage(const std::string& message)
{
	emit messageRequested(message);
}

void
Thread::stop()
{
	if (this->running)
	{
		this->running = false;
		
		while (!this->isFinished())
		{
			QThread::usleep(0);
		}
	}
}
