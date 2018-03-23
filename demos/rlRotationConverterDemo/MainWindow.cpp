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

#include <QGridLayout>
#include <QComboBox>
#include <QGroupBox>
#include <QHeaderView>
#include <QScrollBar>
#include <QSpinBox>
#include <rl/math/Vector.h>

#include "AngleAxisModel.h"
#include "Delegate.h"
#include "EulerAnglesModel.h"
#include "GroupBox.h"
#include "MainWindow.h"
#include "QuaternionModel.h"
#include "RotationMatrixModel.h"
#include "TableView.h"

MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags f) :
	QMainWindow(parent, f),
	inputAngleAxis(0, rl::math::Vector3::UnitZ()),
	inputAngleAxisGroupBox(new GroupBox(this)),
	inputAngleAxisModel(new AngleAxisModel(this)),
	inputAngleAxisTableView(new TableView(this)),
	inputEulerAngles(rl::math::Vector3::Zero()),
	inputEulerAnglesGroupBox(new GroupBox(this)),
	inputEulerAnglesModel(new EulerAnglesModel(this)),
	inputEulerAnglesTableView(new TableView(this)),
	inputEulerAxes(),
	inputPrecision(8),
	inputQuaternion(rl::math::Quaternion::Identity()),
	inputQuaternionGroupBox(new GroupBox(this)),
	inputQuaternionModel(new QuaternionModel(this)),
	inputQuaternionTableView(new TableView(this)),
	inputRotationMatrix(rl::math::Rotation::Identity()),
	inputRotationMatrixGroupBox(new GroupBox(this)),
	inputRotationMatrixModel(new RotationMatrixModel(this)),
	inputRotationMatrixTableView(new TableView(this)),
	inputUnitRadians(false),
	outputAngleAxis(0, rl::math::Vector3::UnitZ()),
	outputAngleAxisModel(new AngleAxisModel(this)),
	outputEulerAngles(rl::math::Vector3::Zero()),
	outputEulerAnglesModel(new EulerAnglesModel(this)),
	outputEulerAxes(),
	outputPrecision(8),
	outputQuaternion(rl::math::Quaternion::Identity()),
	outputQuaternionModel(new QuaternionModel(this)),
	outputRotationMatrix(rl::math::Rotation::Identity()),
	outputRotationMatrixModel(new RotationMatrixModel(this)),
	outputUnitRadians(false)
{
	MainWindow::singleton = this;
	
	this->inputEulerAxes[0] = 0;
	this->inputEulerAxes[1] = 1;
	this->inputEulerAxes[2] = 2;
	
	this->outputEulerAxes[0] = 0;
	this->outputEulerAxes[1] = 1;
	this->outputEulerAxes[2] = 2;
	
	QWidget* centralWidget = new QWidget(this);
	this->setCentralWidget(centralWidget);
	
	QGridLayout* gridLayout = new QGridLayout(centralWidget);
	
	Delegate* inputDelegate = new Delegate(this);
	inputDelegate->precision = &this->inputPrecision;
	
	Delegate* outputDelegate = new Delegate(this);
	outputDelegate->precision = &this->outputPrecision;
	
	// settings
	
	QGroupBox* inputSettingsGroupBox = new QGroupBox(this);
	inputSettingsGroupBox->setFlat(true);
	inputSettingsGroupBox->setTitle("Settings (Input)");
	gridLayout->addWidget(inputSettingsGroupBox, gridLayout->rowCount(), 0);
	
	QComboBox* inputUnitComboBox = new QComboBox(this);
	inputUnitComboBox->addItem("Degrees");
	inputUnitComboBox->addItem("Radians");
	
	QComboBox* inputEulerAxesComboBox = new QComboBox(this);
	
	QSpinBox* inputPrecisionSpinBox = new QSpinBox(this);
	inputPrecisionSpinBox->setMaximum(std::numeric_limits<double>::digits10);
	inputPrecisionSpinBox->setSuffix(" Digits");
	inputPrecisionSpinBox->setValue(this->inputPrecision);
	
	QHBoxLayout* inputSettingsLayout = new QHBoxLayout(inputSettingsGroupBox);
	inputSettingsLayout->addWidget(inputUnitComboBox);
	inputSettingsLayout->addWidget(inputEulerAxesComboBox);
	inputSettingsLayout->addWidget(inputPrecisionSpinBox);
	
	QObject::connect(
		inputUnitComboBox,
		SIGNAL(currentIndexChanged(int)),
		this,
		SLOT(inputUnitChanged(int))
	);
	
	QObject::connect(
		inputEulerAxesComboBox,
		SIGNAL(currentIndexChanged(int)),
		this,
		SLOT(inputEulerAxesChanged(int))
	);
	
	QObject::connect(
		inputPrecisionSpinBox,
		SIGNAL(valueChanged(int)),
		this,
		SLOT(inputPrecisionChanged(int))
	);
	
	QGroupBox* outputSettingsGroupBox = new QGroupBox(this);
	outputSettingsGroupBox->setFlat(true);
	outputSettingsGroupBox->setTitle("Settings (Output)");
	gridLayout->addWidget(outputSettingsGroupBox, gridLayout->rowCount() - 1, 1);
	
	QComboBox* outputUnitComboBox = new QComboBox(this);
	outputUnitComboBox->addItem("Degrees");
	outputUnitComboBox->addItem("Radians");
	
	QComboBox* outputEulerAxesComboBox = new QComboBox(this);
	
	QSpinBox* outputPrecisionSpinBox = new QSpinBox(this);
	outputPrecisionSpinBox->setMaximum(std::numeric_limits<double>::digits10);
	outputPrecisionSpinBox->setSuffix(" Digits");
	outputPrecisionSpinBox->setValue(this->outputPrecision);
	
	QHBoxLayout* outputSettingsLayout = new QHBoxLayout(outputSettingsGroupBox);
	outputSettingsLayout->addWidget(outputUnitComboBox);
	outputSettingsLayout->addWidget(outputEulerAxesComboBox);
	outputSettingsLayout->addWidget(outputPrecisionSpinBox);
	
	QObject::connect(
		outputUnitComboBox,
		SIGNAL(currentIndexChanged(int)),
		this,
		SLOT(outputUnitChanged(int))
	);
	
	QObject::connect(
		outputEulerAxesComboBox,
		SIGNAL(currentIndexChanged(int)),
		this,
		SLOT(outputEulerAxesChanged(int))
	);
	
	QObject::connect(
		outputPrecisionSpinBox,
		SIGNAL(valueChanged(int)),
		this,
		SLOT(outputPrecisionChanged(int))
	);
	
	for (std::size_t i = 0; i < 12; ++i)
	{
		QString text = "Euler ";
		
		for (std::size_t j = 0; j < 3; ++j)
		{
			if (j > 0)
			{
				text += "-";
			}
			
			switch (MainWindow::EULER_ANGLES[i][j])
			{
			case 0:
				text += "X";
				break;
			case 1:
				text += "Y";
				break;
			case 2:
				text += "Z";
				break;
			default:
				break;
			}
		}
		
		inputEulerAxesComboBox->addItem(text);
		outputEulerAxesComboBox->addItem(text);
	}
	
	// rotation matrix
	
	inputRotationMatrixGroupBox->setCheckable(true);
	inputRotationMatrixGroupBox->setFlat(true);
	inputRotationMatrixGroupBox->setTitle("Rotation Matrix (Input)");
	gridLayout->addWidget(inputRotationMatrixGroupBox, gridLayout->rowCount(), 0);
	
	inputRotationMatrixModel->rotation = &this->inputRotationMatrix;
	
	inputRotationMatrixTableView->precision = &this->inputPrecision;
	inputRotationMatrixTableView->setItemDelegate(inputDelegate);
	inputRotationMatrixTableView->setModel(inputRotationMatrixModel);
	inputRotationMatrixTableView->verticalHeader()->setMinimumWidth(20);
	
	QVBoxLayout* inputRotationMatrixLayout = new QVBoxLayout(inputRotationMatrixGroupBox);
	inputRotationMatrixLayout->addWidget(inputRotationMatrixTableView);
	
	QObject::connect(
		inputRotationMatrixGroupBox,
		SIGNAL(toggled(bool)),
		this,
		SLOT(rotationMatrixToggled(bool))
	);
	
	QObject::connect(
		inputRotationMatrixModel,
		SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
		this,
		SLOT(rotationMatrixChanged(const QModelIndex&, const QModelIndex&))
	);
	
	QGroupBox* outputRotationMatrixGroupBox = new QGroupBox(this);
	outputRotationMatrixGroupBox->setFlat(true);
	outputRotationMatrixGroupBox->setTitle("Rotation Matrix (Output)");
	gridLayout->addWidget(outputRotationMatrixGroupBox, gridLayout->rowCount() - 1, 1);
	
	outputRotationMatrixModel->rotation = &this->outputRotationMatrix;
	
	TableView* outputRotationMatrixTableView = new TableView(this);
	outputRotationMatrixTableView->precision = &this->outputPrecision;
	outputRotationMatrixTableView->setEditTriggers(QAbstractItemView::NoEditTriggers);
	outputRotationMatrixTableView->setItemDelegate(outputDelegate);
	outputRotationMatrixTableView->setModel(outputRotationMatrixModel);
	outputRotationMatrixTableView->verticalHeader()->setMinimumWidth(20);
	
	QVBoxLayout* outputRotationMatrixLayout = new QVBoxLayout(outputRotationMatrixGroupBox);
	outputRotationMatrixLayout->addWidget(outputRotationMatrixTableView);
	
	// angle axis
	
	inputAngleAxisGroupBox->setCheckable(true);
	inputAngleAxisGroupBox->setFlat(true);
	inputAngleAxisGroupBox->setTitle("Angle Axis (Input)");
	gridLayout->addWidget(inputAngleAxisGroupBox, gridLayout->rowCount(), 0);
	
	inputAngleAxisModel->angleAxis = &this->inputAngleAxis;
	inputAngleAxisModel->angleRadians = &this->inputUnitRadians;
	
	inputAngleAxisTableView->precision = &this->inputPrecision;
	inputAngleAxisTableView->setItemDelegate(inputDelegate);
	inputAngleAxisTableView->setModel(inputAngleAxisModel);
	inputAngleAxisTableView->verticalHeader()->setMinimumWidth(20);
	
	QVBoxLayout* inputAngleAxisLayout = new QVBoxLayout(inputAngleAxisGroupBox);
	inputAngleAxisLayout->addWidget(inputAngleAxisTableView);
	
	QObject::connect(
		inputAngleAxisGroupBox,
		SIGNAL(toggled(bool)),
		this,
		SLOT(angleAxisToggled(bool))
	);
	
	QObject::connect(
		inputAngleAxisModel,
		SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
		this,
		SLOT(angleAxisChanged(const QModelIndex&, const QModelIndex&))
	);
	
	QGroupBox* outputAngleAxisGroupBox = new QGroupBox(this);
	outputAngleAxisGroupBox->setFlat(true);
	outputAngleAxisGroupBox->setTitle("Angle Axis (Output)");
	gridLayout->addWidget(outputAngleAxisGroupBox, gridLayout->rowCount() - 1, 1);
	
	outputAngleAxisModel->angleAxis = &this->outputAngleAxis;
	outputAngleAxisModel->angleRadians = &this->outputUnitRadians;
	
	TableView* outputAngleAxisTableView = new TableView(this);
	outputAngleAxisTableView->precision = &this->outputPrecision;
	outputAngleAxisTableView->setEditTriggers(QAbstractItemView::NoEditTriggers);
	outputAngleAxisTableView->setItemDelegate(outputDelegate);
	outputAngleAxisTableView->setModel(outputAngleAxisModel);
	outputAngleAxisTableView->verticalHeader()->setMinimumWidth(20);
	
	QVBoxLayout* outputAngleAxisLayout = new QVBoxLayout(outputAngleAxisGroupBox);
	outputAngleAxisLayout->addWidget(outputAngleAxisTableView);
	
	// quaternion
	
	inputQuaternionGroupBox->setCheckable(true);
	inputQuaternionGroupBox->setFlat(true);
	inputQuaternionGroupBox->setTitle("Quaternion (Input)");
	gridLayout->addWidget(inputQuaternionGroupBox, gridLayout->rowCount(), 0);
	
	inputQuaternionModel->quaternion = &this->inputQuaternion;
	
	inputQuaternionTableView->precision = &this->inputPrecision;
	inputQuaternionTableView->setItemDelegate(inputDelegate);
	inputQuaternionTableView->setModel(inputQuaternionModel);
	inputQuaternionTableView->verticalHeader()->setMinimumWidth(20);
	
	QVBoxLayout* inputQuaternionLayout = new QVBoxLayout(inputQuaternionGroupBox);
	inputQuaternionLayout->addWidget(inputQuaternionTableView);
	
	QObject::connect(
		inputQuaternionGroupBox,
		SIGNAL(toggled(bool)),
		this,
		SLOT(quaternionToggled(bool))
	);
	
	QObject::connect(
		inputQuaternionModel,
		SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
		this,
		SLOT(quaternionChanged(const QModelIndex&, const QModelIndex&))
	);
	
	QGroupBox* outputQuaternionGroupBox = new QGroupBox(this);
	outputQuaternionGroupBox->setFlat(true);
	outputQuaternionGroupBox->setTitle("Quaternion (Output)");
	gridLayout->addWidget(outputQuaternionGroupBox, gridLayout->rowCount() - 1, 1);
	
	outputQuaternionModel->quaternion = &this->outputQuaternion;
	
	TableView* outputQuaternionTableView = new TableView(this);
	outputQuaternionTableView->precision = &this->outputPrecision;
	outputQuaternionTableView->setEditTriggers(QAbstractItemView::NoEditTriggers);
	outputQuaternionTableView->setItemDelegate(outputDelegate);
	outputQuaternionTableView->setModel(outputQuaternionModel);
	outputQuaternionTableView->verticalHeader()->setMinimumWidth(20);
	
	QVBoxLayout* outputQuaternionLayout = new QVBoxLayout(outputQuaternionGroupBox);
	outputQuaternionLayout->addWidget(outputQuaternionTableView);
	
	// eulerAngles
	
	inputEulerAnglesGroupBox->setCheckable(true);
	inputEulerAnglesGroupBox->setFlat(true);
	inputEulerAnglesGroupBox->setTitle("Euler Angles (Input)");
	gridLayout->addWidget(inputEulerAnglesGroupBox, gridLayout->rowCount(), 0);
	
	inputEulerAnglesModel->eulerAngles = &this->inputEulerAngles;
	inputEulerAnglesModel->eulerAnglesRadians = &this->inputUnitRadians;
	inputEulerAnglesModel->eulerAxes = &this->inputEulerAxes;
	
	inputEulerAnglesTableView->precision = &this->inputPrecision;
	inputEulerAnglesTableView->setItemDelegate(inputDelegate);
	inputEulerAnglesTableView->setModel(inputEulerAnglesModel);
	inputEulerAnglesTableView->verticalHeader()->setMinimumWidth(20);
	
	QVBoxLayout* inputEulerAnglesLayout = new QVBoxLayout(inputEulerAnglesGroupBox);
	inputEulerAnglesLayout->addWidget(inputEulerAnglesTableView);
	
	QObject::connect(
		inputEulerAnglesGroupBox,
		SIGNAL(toggled(bool)),
		this,
		SLOT(eulerAnglesToggled(bool))
	);
	
	QObject::connect(
		inputEulerAnglesModel,
		SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
		this,
		SLOT(eulerAnglesChanged(const QModelIndex&, const QModelIndex&))
	);
	
	QGroupBox* outputEulerAnglesGroupBox = new QGroupBox(this);
	outputEulerAnglesGroupBox->setFlat(true);
	outputEulerAnglesGroupBox->setTitle("Euler Angles (Output)");
	gridLayout->addWidget(outputEulerAnglesGroupBox, gridLayout->rowCount() - 1, 1);
	
	outputEulerAnglesModel->eulerAngles = &this->outputEulerAngles;
	outputEulerAnglesModel->eulerAnglesRadians = &this->outputUnitRadians;
	outputEulerAnglesModel->eulerAxes = &this->outputEulerAxes;
	
	TableView* outputEulerAnglesTableView = new TableView(this);
	outputEulerAnglesTableView->precision = &this->outputPrecision;
	outputEulerAnglesTableView->setEditTriggers(QAbstractItemView::NoEditTriggers);
	outputEulerAnglesTableView->setItemDelegate(outputDelegate);
	outputEulerAnglesTableView->setModel(outputEulerAnglesModel);
	outputEulerAnglesTableView->verticalHeader()->setMinimumWidth(20);
	
	QVBoxLayout* outputEulerAnglesLayout = new QVBoxLayout(outputEulerAnglesGroupBox);
	outputEulerAnglesLayout->addWidget(outputEulerAnglesTableView);
	
	inputAngleAxisGroupBox->setChecked(false);
	inputQuaternionGroupBox->setChecked(false);
	inputEulerAnglesGroupBox->setChecked(false);
}

MainWindow::~MainWindow()
{
	MainWindow::singleton = nullptr;
}

void
MainWindow::angleAxisChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight)
{
	this->fromAngleAxis();
}

void
MainWindow::angleAxisToggled(bool on)
{
	if (on)
	{
		this->inputAngleAxisTableView->setFocus();
		this->inputEulerAnglesGroupBox->setChecked(false);
		this->inputEulerAnglesTableView->clearSelection();
		this->inputQuaternionGroupBox->setChecked(false);
		this->inputQuaternionTableView->clearSelection();
		this->inputRotationMatrixGroupBox->setChecked(false);
		this->inputRotationMatrixTableView->clearSelection();
		
		this->fromAngleAxis();
	}
	else
	{
		this->inputAngleAxisTableView->clearSelection();
	}
}

void
MainWindow::eulerAnglesChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight)
{
	this->fromEulerAngles();
}

void
MainWindow::eulerAnglesToggled(bool on)
{
	if (on)
	{
		this->inputAngleAxisGroupBox->setChecked(false);
		this->inputAngleAxisTableView->clearSelection();
		this->inputEulerAnglesTableView->setFocus();
		this->inputQuaternionGroupBox->setChecked(false);
		this->inputQuaternionTableView->clearSelection();
		this->inputRotationMatrixGroupBox->setChecked(false);
		this->inputRotationMatrixTableView->clearSelection();
		
		this->fromEulerAngles();
	}
	else
	{
		this->inputEulerAnglesTableView->clearSelection();
	}
}

void
MainWindow::fromAngleAxis()
{
	this->outputAngleAxis = this->inputAngleAxis;
	this->outputQuaternion = this->inputAngleAxis;
	this->outputRotationMatrix = this->inputAngleAxis;
	
	this->outputEulerAngles = this->outputRotationMatrix.eulerAngles(this->outputEulerAxes[0], this->outputEulerAxes[1], this->outputEulerAxes[2]);
	
	this->outputAngleAxisModel->invalidate();
	this->outputEulerAnglesModel->invalidate();
	this->outputQuaternionModel->invalidate();
	this->outputRotationMatrixModel->invalidate();
}

void
MainWindow::fromEulerAngles()
{
	this->outputQuaternion = rl::math::AngleAxis(
		this->inputEulerAngles[0],
		rl::math::Vector3::Unit(this->inputEulerAxes[0])
	) * rl::math::AngleAxis(
		this->inputEulerAngles[1],
		rl::math::Vector3::Unit(this->inputEulerAxes[1])
	) * rl::math::AngleAxis(
		this->inputEulerAngles[2],
		rl::math::Vector3::Unit(this->inputEulerAxes[2])
	);
	
	this->outputAngleAxis = this->outputQuaternion;
	this->outputRotationMatrix = this->outputQuaternion;
	
	this->outputEulerAngles = this->outputRotationMatrix.eulerAngles(this->outputEulerAxes[0], this->outputEulerAxes[1], this->outputEulerAxes[2]);
	
	this->outputAngleAxisModel->invalidate();
	this->outputEulerAnglesModel->invalidate();
	this->outputQuaternionModel->invalidate();
	this->outputRotationMatrixModel->invalidate();
}

void
MainWindow::fromQuaternion()
{
	this->outputAngleAxis = this->inputQuaternion;
	this->outputQuaternion = this->inputQuaternion;
	this->outputRotationMatrix = this->inputQuaternion;
	
	this->outputEulerAngles = this->outputRotationMatrix.eulerAngles(this->outputEulerAxes[0], this->outputEulerAxes[1], this->outputEulerAxes[2]);
	
	this->outputAngleAxisModel->invalidate();
	this->outputEulerAnglesModel->invalidate();
	this->outputQuaternionModel->invalidate();
	this->outputRotationMatrixModel->invalidate();
}

void
MainWindow::fromRotationMatrix()
{
	this->outputAngleAxis = this->inputRotationMatrix;
	this->outputQuaternion = this->inputRotationMatrix;
	this->outputRotationMatrix = this->inputRotationMatrix;
	
	this->outputEulerAngles = this->outputRotationMatrix.eulerAngles(this->outputEulerAxes[0], this->outputEulerAxes[1], this->outputEulerAxes[2]);
	
	this->outputAngleAxisModel->invalidate();
	this->outputEulerAnglesModel->invalidate();
	this->outputQuaternionModel->invalidate();
	this->outputRotationMatrixModel->invalidate();
}

void
MainWindow::inputEulerAxesChanged(int index)
{
	if (index < 0 || index > 11)
	{
		return;
	}
	
	for (std::size_t i = 0; i < 3; ++i)
	{
		this->inputEulerAxes[i] = MainWindow::EULER_ANGLES[index][i];
	}
	
	this->inputEulerAnglesModel->invalidate();
	
	if (this->inputEulerAnglesGroupBox->isChecked())
	{
		this->fromEulerAngles();
	}
}

void
MainWindow::inputPrecisionChanged(int precision)
{
	this->inputPrecision = precision;
	
	this->inputAngleAxisModel->invalidate();
	this->inputEulerAnglesModel->invalidate();
	this->inputQuaternionModel->invalidate();
	this->inputRotationMatrixModel->invalidate();
}

void
MainWindow::inputUnitChanged(int index)
{
	if (index < 0 || index > 1)
	{
		return;
	}
	
	switch (index)
	{
	case 0:
		this->inputUnitRadians = false;
		break;
	case 1:
		this->inputUnitRadians = true;
		break;
	default:
		break;
	}
	
	this->inputAngleAxisModel->invalidate();
	this->inputEulerAnglesModel->invalidate();
}

MainWindow*
MainWindow::instance()
{
	if (nullptr == MainWindow::singleton)
	{
		new MainWindow();
	}
	
	return MainWindow::singleton;
}

void
MainWindow::outputEulerAxesChanged(int index)
{
	if (index < 0 || index > 11)
	{
		return;
	}
	
	for (std::size_t i = 0; i < 3; ++i)
	{
		this->outputEulerAxes[i] = MainWindow::EULER_ANGLES[index][i];
	}
	
	this->outputEulerAngles = this->outputRotationMatrix.eulerAngles(this->outputEulerAxes[0], this->outputEulerAxes[1], this->outputEulerAxes[2]);
	
	this->outputEulerAnglesModel->invalidate();
}

void
MainWindow::outputPrecisionChanged(int precision)
{
	this->outputPrecision = precision;
	
	this->outputAngleAxisModel->invalidate();
	this->outputEulerAnglesModel->invalidate();
	this->outputQuaternionModel->invalidate();
	this->outputRotationMatrixModel->invalidate();
}

void
MainWindow::outputUnitChanged(int index)
{
	if (index < 0 || index > 1)
	{
		return;
	}
	
	switch (index)
	{
	case 0:
		this->outputUnitRadians = false;
		break;
	case 1:
		this->outputUnitRadians = true;
		break;
	default:
		break;
	}
	
	this->outputAngleAxisModel->invalidate();
	this->outputEulerAnglesModel->invalidate();
}

void
MainWindow::quaternionChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight)
{
	this->fromQuaternion();
}

void
MainWindow::quaternionToggled(bool on)
{
	if (on)
	{
		this->inputAngleAxisGroupBox->setChecked(false);
		this->inputAngleAxisTableView->clearSelection();
		this->inputEulerAnglesGroupBox->setChecked(false);
		this->inputEulerAnglesTableView->clearSelection();
		this->inputQuaternionTableView->setFocus();
		this->inputRotationMatrixGroupBox->setChecked(false);
		this->inputRotationMatrixTableView->clearSelection();
		
		this->fromQuaternion();
	}
	else
	{
		this->inputQuaternionTableView->clearSelection();
	}
}

void
MainWindow::rotationMatrixChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight)
{
	this->fromRotationMatrix();
}

void
MainWindow::rotationMatrixToggled(bool on)
{
	if (on)
	{
		this->inputAngleAxisGroupBox->setChecked(false);
		this->inputAngleAxisTableView->clearSelection();
		this->inputEulerAnglesGroupBox->setChecked(false);
		this->inputEulerAnglesTableView->clearSelection();
		this->inputQuaternionGroupBox->setChecked(false);
		this->inputQuaternionTableView->clearSelection();
		this->inputRotationMatrixTableView->setFocus();
		
		this->fromRotationMatrix();
	}
	else
	{
		this->inputRotationMatrixTableView->clearSelection();
	}
}

std::size_t MainWindow::EULER_ANGLES[12][3] = {
	{0, 1, 0},
	{0, 2, 0},
	{1, 0, 1},
	{1, 2, 1},
	{2, 0, 2},
	{2, 1, 2},
	{0, 1, 2},
	{0, 2, 1},
	{1, 0, 2},
	{1, 2, 0},
	{2, 0, 1},
	{2, 1, 0}
};
