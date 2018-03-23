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

#include <QApplication>
#include <QClipboard>
#include <QHeaderView>
#include <QKeyEvent>
#include <QScrollBar>

#include "TableView.h"

TableView::TableView(QWidget* parent) :
	QTableView(parent),
	precision(nullptr)
{
}

TableView::~TableView()
{
}

void
TableView::keyPressEvent(QKeyEvent* event)
{
	if (event->matches(QKeySequence::Copy))
	{
		QString text;
		
		if (!this->selectionModel()->hasSelection())
		{
			this->selectAll();
		}
		
		QItemSelectionRange range = this->selectionModel()->selection().first();
		
		for (int i = range.top(); i <= range.bottom(); ++i)
		{
			QStringList rowContents;
			
			for (int j = range.left(); j <= range.right(); ++j)
			{
				rowContents << QString::number(this->model()->index(i, j).data().toDouble(), 'g', nullptr != this->precision ? *this->precision : 8);
			}
			
			text += rowContents.join("\t");
			text += "\n";
		}
		
		QApplication::clipboard()->setText(text);
	}
	else if (event->matches(QKeySequence::Paste) && QAbstractItemView::NoEditTriggers != this->editTriggers())
	{
		QString text = QApplication::clipboard()->text();
		
		QStringList rowContents = text.split("\n", QString::SkipEmptyParts);
		
		if (!this->selectionModel()->hasSelection())
		{
			this->selectAll();
		}
		
		QModelIndex first = this->selectedIndexes().first();
		
		for (int i = 0; i < rowContents.size(); ++i)
		{
			QStringList columnContents = rowContents.at(i).split("\t");
			
			for (int j = 0; j < columnContents.size(); ++j)
			{
				this->model()->setData(this->model()->index(first.row() + i, first.column() + j), columnContents[j]);
			}
		}
	}
	
	QTableView::keyPressEvent(event);
}

QSize
TableView::minimumSizeHint() const
{
	return this->sizeHint();
}

QSize
TableView::sizeHint() const
{
	int horizontalScrollBarHeight = this->horizontalScrollBar()->isVisible() ? this->horizontalScrollBar()->height() : 2;
	int horizontalHeaderHeight = this->horizontalHeader()->height();
	int rowTotalHeight = 0;
	
	for (int i = 0; i < this->verticalHeader()->count(); ++i)
	{
		rowTotalHeight += this->verticalHeader()->sectionSize(i);
	}
	
	int verticalScrollBarWidth = this->verticalScrollBar()->isVisible() ? this->verticalScrollBar()->width() : 2;
	int verticalHeaderWidth = this->verticalHeader()->width();
	int columnTotalWidth = 0;
	
	for (int i = 0; i < this->horizontalHeader()->count(); ++i)
	{
		columnTotalWidth += this->horizontalHeader()->sectionSize(i);
	}
	
	return QSize(
		verticalHeaderWidth + columnTotalWidth + verticalScrollBarWidth,
		horizontalHeaderHeight + rowTotalHeight + horizontalScrollBarHeight
	);
}
