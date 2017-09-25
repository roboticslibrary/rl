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

#ifndef CONFIGURATIONSPACEMODEL_H
#define CONFIGURATIONSPACEMODEL_H

#include <QAbstractTableModel>

class ConfigurationSpaceScene;

class ConfigurationSpaceModel : public QAbstractTableModel
{
public:
	ConfigurationSpaceModel(QObject* parent = nullptr);
	
	virtual ~ConfigurationSpaceModel();
	
	int columnCount(const QModelIndex& parent = QModelIndex()) const;
	
	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;
	
	Qt::ItemFlags flags(const QModelIndex &index) const;
	
	QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;
	
	void invalidate();
	
	int rowCount(const QModelIndex& parent = QModelIndex()) const;
	
	bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole); 
	
	ConfigurationSpaceScene* configurationSpaceScene;
	
protected:
	
private:
	
};

#endif // CONFIGURATIONSPACEMODEL_H
