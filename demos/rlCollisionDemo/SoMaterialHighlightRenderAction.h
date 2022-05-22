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

#ifndef SOMATERIALHIGHLIGHTRENDERACTION_H
#define SOMATERIALHIGHLIGHTRENDERACTION_H

#include <Inventor/SbColor.h>
#include <Inventor/actions/SoGLRenderAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/threads/SbStorage.h>

class SoMaterialHighlightRenderAction : public SoGLRenderAction
{
	SO_ACTION_HEADER(SoMaterialHighlightRenderAction);
	
public:
	SoMaterialHighlightRenderAction();
	
	SoMaterialHighlightRenderAction(const SbViewportRegion& viewportRegion);
	
	virtual ~SoMaterialHighlightRenderAction();
	
	virtual void apply(SoNode* node);
	
	virtual void apply(SoPath* path);
	
	virtual void apply(const SoPathList& pathList, SbBool obeysRules = false);
	
	const SbColor& getDiffuseColor();
	
	const SbColor& getEmissiveColor();
	
	const SbColor& getSpecularColor();
	
	static void initClass(void);
	
	SbBool isVisible() const;
	
	void setVisible(const SbBool visible);
	
	void setDiffuseColor(const SbColor& diffuseColor);
	
	void setEmissiveColor(const SbColor& emissiveColor);
	
	void setSpecularColor(const SbColor& specularColor);
	
protected:
	
private:
	SoMaterialHighlightRenderAction(const SoMaterialHighlightRenderAction& rhs);
	
	static void allocColorPacker(void* data);
	
	static void freeColorPacker(void* data);
	
	void highlight(SoPath* path, const SoPathList* pathList);
	
	SoMaterialHighlightRenderAction& operator=(const SoMaterialHighlightRenderAction& rhs);
	
	SbStorage colorPackerStorage;
	
	SbColor diffuseColor;
	
	SbColor emissiveColor;
	
	SoTempPath* postProcPath;
	
	SoSearchAction searchAction;
	
	SbColor specularColor;
	
	SbBool visible;
};

#endif // SOMATERIALHIGHLIGHTRENDERACTION_H
