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

#ifdef WIN32
#include <windows.h>
#endif // WIN32

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else // __APPLE__
#include <GL/gl.h>
#endif // __APPLE__

#include <Inventor/nodes/SoSubNode.h>

#include "SoGradientBackground.h"

SO_NODE_SOURCE(SoGradientBackground);

SoGradientBackground::SoGradientBackground() :
	SoNode(),
	color0(),
	color1()
{
	SO_NODE_CONSTRUCTOR(SoGradientBackground);
	
	SO_NODE_ADD_FIELD(color0, (1.0f, 1.0f, 1.0f));
	SO_NODE_ADD_FIELD(color1, (0.0f, 0.0f, 0.0f));
}

SoGradientBackground::~SoGradientBackground()
{
}

SbBool
SoGradientBackground::affectsState() const
{
	return false;
}

void
SoGradientBackground::exitClass()
{
}

void
SoGradientBackground::GLRender(SoGLRenderAction* action)
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	
	glPushAttrib(GL_ENABLE_BIT);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	
	glBegin(GL_QUADS);
	
	glColor3f(this->color0.getValue()[0], this->color0.getValue()[1], this->color0.getValue()[2]);
	glVertex2f(1.0, 1.0);
	glVertex2f(-1.0, 1.0);
	
	glColor3f(this->color1.getValue()[0], this->color1.getValue()[1], this->color1.getValue()[2]);
	glVertex2f(-1.0, -1.0);
	glVertex2f(1.0, -1.0);
	
	glEnd();
	
	glPopAttrib();
	
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

void
SoGradientBackground::initClass()
{
	SO_NODE_INIT_CLASS(SoGradientBackground, SoNode, "Node");
}
