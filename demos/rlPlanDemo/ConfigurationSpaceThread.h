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

#ifndef CONFIGURATIONSPACETHREAD_H
#define CONFIGURATIONSPACETHREAD_H

#include <QThread>
#include <rl/plan/Model.h>

class ConfigurationSpaceThread : public QThread
{
	Q_OBJECT
	
public:
	ConfigurationSpaceThread(QObject* parent = nullptr);
	
	virtual ~ConfigurationSpaceThread();
	
	void run();
	
	void stop();
	
	std::size_t axis0;
	
	std::size_t axis1;
	
	rl::math::Real delta0;
	
	rl::math::Real delta1;
	
	rl::plan::Model* model;
	
protected:
	
private:
	bool running;
	
signals:
	void addCollision(const qreal& x, const qreal& y, const qreal& w, const qreal& h, const int& rgb);
};

#endif // CONFIGURATIONSPACETHREAD_H
