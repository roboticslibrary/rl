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
#include <QFileDialog>
#include <QMessageBox>
#include <QWidget>
#include <stdexcept>
#include <Inventor/SoDB.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <rl/sg/so/Scene.h>

int
main(int argc, char** argv)
{
	try
	{
		SoDB::init();
		
		QWidget* widget = SoQt::init(argc, argv, argv[0]);
		widget->resize(800, 600);
		
		QString filename;
		
		if (argc < 2)
		{
			filename = QFileDialog::getOpenFileName(widget, "", filename, "All Formats (*.xml)");
		}
		else
		{
			filename = argv[1];
		}
		
		rl::sg::so::Scene scene;
		
		if (!filename.isEmpty())
		{
			scene.load(filename.toStdString());
		}
		
		SoQtExaminerViewer viewer(widget, nullptr, true, SoQtFullViewer::BUILD_POPUP);
		viewer.setSceneGraph(scene.root);
		viewer.setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
		viewer.show();
		
		widget->setWindowTitle(filename + (filename.isEmpty() ? "" : " - ") + "rlViewDemo");
		
		SoQt::show(widget);
		SoQt::mainLoop();
		
		return EXIT_SUCCESS;
	}
	catch (const std::exception& e)
	{
		QApplication application(argc, argv);
		QMessageBox::critical(nullptr, "Error", e.what());
		return EXIT_FAILURE;
	}
}
