//
// Copyright (c) 2009, Andre Gaschler
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

#ifndef CONFIGURATIONMODEL_H
#define CONFIGURATIONMODEL_H

#include <QAbstractTableModel>
#include <rl/math/Vector.h>

class ConfigurationModel : public QAbstractTableModel
{
	Q_OBJECT
	
public:
	ConfigurationModel(QObject* parent = nullptr);
	
	virtual ~ConfigurationModel();
	
	int columnCount(const QModelIndex& parent = QModelIndex()) const;
	
	virtual QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const = 0;
	
	Qt::ItemFlags flags(const QModelIndex &index) const;
	
	QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;
	
	virtual int rowCount(const QModelIndex& parent = QModelIndex()) const = 0;
	
	bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole); 
	
	bool setData(const rl::math::Vector& q); 
	
public slots:
	void operationalChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight);	
	
protected:
	
private:
	
};

class PositionModel : public ConfigurationModel
{
public:
	PositionModel(QObject* parent = nullptr)
	: ConfigurationModel(parent)
	{
	}
	
	virtual ~PositionModel()
	{
	}
	
	int rowCount(const QModelIndex& parent = QModelIndex()) const;
	
	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;
};

class VelocityModel : public ConfigurationModel
{
public:
	VelocityModel(QObject* parent = nullptr)
	: ConfigurationModel(parent)
	{
	}
	
	virtual ~VelocityModel()
	{
	}
	
	int rowCount(const QModelIndex& parent = QModelIndex()) const;
	
	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;
};

class AccelerationModel : public ConfigurationModel
{
public:
	AccelerationModel(QObject* parent = nullptr)
	: ConfigurationModel(parent)
	{
	}
	
	virtual ~AccelerationModel()
	{
	}
	
	int rowCount(const QModelIndex& parent = QModelIndex()) const;
	
	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;
};

class TorqueModel : public ConfigurationModel
{
public:
	TorqueModel(QObject* parent = nullptr)
	: ConfigurationModel(parent)
	{
	}
	
	virtual ~TorqueModel()
	{
	}
	
	int rowCount(const QModelIndex& parent = QModelIndex()) const;
	
	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;
};

#endif // CONFIGURATIONMODEL_H
