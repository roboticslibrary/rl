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

#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <rl/math/Transform.h>

#include "MainWindow.h"
#include "TestWidget.h"

TestWidget::TestWidget(rl::sg::Body* body0, rl::sg::Body* body1, QWidget* parent, Qt::WFlags f) :
	QWidget(parent, f),
	body0(body0),
	body1(body1)
{
	QVBoxLayout* mainLayout = new QVBoxLayout(this);
	mainLayout->setMargin(4);
	
	rl::math::Transform frame;
	this->body0->getFrame(frame);
	
	QDoubleSpinBox* xDoubleSpinBox = new QDoubleSpinBox(this);
	mainLayout->addWidget(xDoubleSpinBox);
	xDoubleSpinBox->setRange(-std::numeric_limits< rl::math::Real >::max(), std::numeric_limits< rl::math::Real >::max());
	xDoubleSpinBox->setSingleStep(0.1f);
	xDoubleSpinBox->setValue(frame(0, 3));
	QObject::connect(xDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setX(double)));
	
	QDoubleSpinBox* yDoubleSpinBox = new QDoubleSpinBox(this);
	mainLayout->addWidget(yDoubleSpinBox);
	yDoubleSpinBox->setRange(-std::numeric_limits< rl::math::Real >::max(), std::numeric_limits< rl::math::Real >::max());
	yDoubleSpinBox->setSingleStep(0.1f);
	yDoubleSpinBox->setValue(frame(1, 3));
	QObject::connect(yDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setY(double)));
	
	QDoubleSpinBox* zDoubleSpinBox = new QDoubleSpinBox(this);
	mainLayout->addWidget(zDoubleSpinBox);
	zDoubleSpinBox->setRange(-std::numeric_limits< rl::math::Real >::max(), std::numeric_limits< rl::math::Real >::max());
	zDoubleSpinBox->setSingleStep(0.1f);
	zDoubleSpinBox->setValue(frame(2, 3));
	QObject::connect(zDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setZ(double)));
}

TestWidget::~TestWidget()
{
}

void
TestWidget::setX(double x)
{
	rl::math::Transform frame;
	this->body0->getFrame(frame);
	frame(0, 3) = x;
	this->body0->setFrame(frame);
	this->body1->setFrame(frame);
	MainWindow::instance()->test();
}

void
TestWidget::setY(double y)
{
	rl::math::Transform frame;
	this->body0->getFrame(frame);
	frame(1, 3) = y;
	this->body0->setFrame(frame);
	this->body1->setFrame(frame);
	MainWindow::instance()->test();
}

void
TestWidget::setZ(double z)
{
	rl::math::Transform frame;
	this->body0->getFrame(frame);
	frame(2, 3) = z;
	this->body0->setFrame(frame);
	this->body1->setFrame(frame);
	MainWindow::instance()->test();
}
