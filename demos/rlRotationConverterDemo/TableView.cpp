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
	QTableView(parent)
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
		
		QModelIndexList indices = this->selectedIndexes();
		qSort(indices);
		
		QModelIndexList::iterator i = indices.begin();
		QModelIndexList::iterator j = ++indices.begin();
		
		for (; i != indices.end(); ++i, ++j)
		{
			text += i->data().toString();
			
			if (j != indices.end() && j->row() > i->row())
			{
				text += "\n";
			}
			else if (j != indices.end() && j->column() > i->column())
			{
				text += " ";
			}
		}
		
		QApplication::clipboard()->setText(text);
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
