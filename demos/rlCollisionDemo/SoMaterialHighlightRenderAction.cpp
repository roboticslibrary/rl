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

#include <cassert>
#include <Inventor/SbName.h>
#include <Inventor/actions/SoSubAction.h>
#include <Inventor/elements/SoDrawStyleElement.h>
#include <Inventor/elements/SoLazyElement.h>
#include <Inventor/elements/SoLinePatternElement.h>
#include <Inventor/elements/SoLineWidthElement.h>
#include <Inventor/elements/SoOverrideElement.h>
#include <Inventor/elements/SoPolygonOffsetElement.h>
#include <Inventor/elements/SoTextureOverrideElement.h>
#include <Inventor/elements/SoTextureQualityElement.h>
#include <Inventor/elements/SoMaterialBindingElement.h>
#include <Inventor/elements/SoNormalElement.h>
#include <Inventor/lists/SoEnabledElementsList.h>
#include <Inventor/lists/SoPathList.h>
#include <Inventor/misc/SoState.h>
#include <Inventor/nodes/SoSelection.h>

#include "SoMaterialHighlightRenderAction.h"

SO_ACTION_SOURCE(SoMaterialHighlightRenderAction)

SoMaterialHighlightRenderAction::SoMaterialHighlightRenderAction() :
	SoGLRenderAction(SbViewportRegion()),
	colorPackerStorage(
		sizeof(void*),
		SoMaterialHighlightRenderAction::allocColorPacker,
		SoMaterialHighlightRenderAction::freeColorPacker
	),
	diffuseColor(SbColor(0.8f, 0.8f, 0.8f)),
	emissiveColor(0.0f, 0.0f, 0.25f),
	postProcPath(new SoTempPath(32)),
	searchAction(),
	specularColor(0.0f, 0.0f, 1.0f),
	visible(true)
{
	SO_ACTION_CONSTRUCTOR(SoMaterialHighlightRenderAction);
	this->postProcPath->ref();
}

SoMaterialHighlightRenderAction::SoMaterialHighlightRenderAction(const SbViewportRegion& viewportRegion) :
	SoGLRenderAction(viewportRegion),
	colorPackerStorage(
		sizeof(void*),
		SoMaterialHighlightRenderAction::allocColorPacker,
		SoMaterialHighlightRenderAction::freeColorPacker
	),
	diffuseColor(SbColor(0.8f, 0.8f, 0.8f)),
	emissiveColor(0.0f, 0.0f, 0.25f),
	postProcPath(new SoTempPath(32)),
	searchAction(),
	specularColor(0.0f, 0.0f, 1.0f),
	visible(true)
{
	SO_ACTION_CONSTRUCTOR(SoMaterialHighlightRenderAction);
	this->postProcPath->ref();
}

SoMaterialHighlightRenderAction::~SoMaterialHighlightRenderAction()
{
	this->postProcPath->unref();
}

void
SoMaterialHighlightRenderAction::allocColorPacker(void* data)
{
	SoColorPacker** colorPacker = static_cast<SoColorPacker**>(data);
	*colorPacker = new SoColorPacker();
}

void
SoMaterialHighlightRenderAction::apply(SoNode* node)
{
	SoGLRenderAction::apply(node);
	
	if (this->visible)
	{
		const SbBool searchall = false;
		this->searchAction.setInterest(searchall ? SoSearchAction::ALL : SoSearchAction::FIRST);
		this->searchAction.setType(SoSelection::getClassTypeId());
		this->searchAction.apply(node);
		
		if (searchall)
		{
			const SoPathList& pathList = this->searchAction.getPaths();
			
			if (pathList.getLength() > 0)
			{
				for (int i = 0; i < pathList.getLength(); ++i)
				{
					SoFullPath* path = static_cast<SoFullPath*>(pathList[i]);
					assert(path);
					SoSelection* selection = static_cast<SoSelection*>(path->getTail());
					
					if (selection->getNumSelected() > 0)
					{
						this->highlight(path, selection->getList());
					}
				}
			}
		}
		else
		{
			SoFullPath* path = static_cast<SoFullPath*>(this->searchAction.getPath());
			
			if (path)
			{
				SoSelection* selection = static_cast<SoSelection*>(path->getTail());
				assert(selection->getTypeId().isDerivedFrom(SoSelection::getClassTypeId()));
				
				if (selection->getNumSelected() > 0)
				{
					this->highlight(path, selection->getList());
				}
			}
		}
		
		this->searchAction.reset();
	}
}

void
SoMaterialHighlightRenderAction::apply(SoPath* path)
{
	SoGLRenderAction::apply(path);
}

void
SoMaterialHighlightRenderAction::apply(const SoPathList& pathList, SbBool obeysRules)
{
	SoGLRenderAction::apply(pathList, obeysRules);
}

void
SoMaterialHighlightRenderAction::freeColorPacker(void* data)
{
	SoColorPacker** colorPacker = static_cast<SoColorPacker**>(data);
	delete *colorPacker;
}

const SbColor&
SoMaterialHighlightRenderAction::getDiffuseColor()
{
	return this->diffuseColor;
}

const SbColor&
SoMaterialHighlightRenderAction::getEmissiveColor()
{
	return this->emissiveColor;
}

const SbColor&
SoMaterialHighlightRenderAction::getSpecularColor()
{
	return this->specularColor;
}

void
SoMaterialHighlightRenderAction::highlight(SoPath* path, const SoPathList* pathList)
{
	int pos = static_cast<SoFullPath*>(path)->getLength() - 1;
	assert(pos >= 0);
	this->postProcPath->setHead(path->getHead());
	
	for (int i = 1; i < pos; ++i)
	{
		this->postProcPath->append(path->getIndex(i));
	}
	
	SoState* state = this->getState();
	state->push();
	
	int oldNumPasses = this->getNumPasses();
	this->setNumPasses(1);
	
	SoColorPacker** colorPacker = static_cast<SoColorPacker**>(this->colorPackerStorage.get());
	
	SoLazyElement::setDiffuse(state, path->getHead(), 1, &this->diffuseColor, *colorPacker);
	SoLazyElement::setEmissive(state, &this->emissiveColor);
	SoLazyElement::setSpecular(state, &this->specularColor);
	SoMaterialBindingElement::set(state, nullptr, SoMaterialBindingElement::OVERALL);
	SoLineWidthElement::set(state, 5.0f);
	SoTextureQualityElement::set(state, 0.0f);
	
	SoOverrideElement::setDiffuseColorOverride(state, nullptr, true);
	SoOverrideElement::setEmissiveColorOverride(state, nullptr, true);
	SoOverrideElement::setSpecularColorOverride(state, nullptr, true);
	SoOverrideElement::setMaterialBindingOverride(state, nullptr, true);
	SoOverrideElement::setLineWidthOverride(state, nullptr, true);
	SoTextureOverrideElement::setQualityOverride(state, true);
	
	for (int i = 0; i < pathList->getLength(); ++i)
	{
		SoFullPath* fullPath = static_cast<SoFullPath*>((*pathList)[i]);
		this->postProcPath->append(fullPath->getHead());
		
		for (int j = 1; j < fullPath->getLength(); ++j)
		{
			this->postProcPath->append(fullPath->getIndex(j));
		}
		
		this->SoGLRenderAction::apply(this->postProcPath);
		this->postProcPath->truncate(pos);
	}
	
	this->setNumPasses(oldNumPasses);
	state->pop();
}

void
SoMaterialHighlightRenderAction::initClass()
{
	SO_ACTION_INIT_CLASS(SoMaterialHighlightRenderAction, SoGLRenderAction);
}

SbBool
SoMaterialHighlightRenderAction::isVisible() const
{
	return this->visible;
}

void
SoMaterialHighlightRenderAction::setDiffuseColor(const SbColor& diffuseColor)
{
	this->diffuseColor = diffuseColor;
}

void
SoMaterialHighlightRenderAction::setEmissiveColor(const SbColor& emissiveColor)
{
	this->emissiveColor = emissiveColor;
}

void
SoMaterialHighlightRenderAction::setSpecularColor(const SbColor& specularColor)
{
	this->specularColor = specularColor;
}

void
SoMaterialHighlightRenderAction::setVisible(const SbBool visible)
{
	this->visible = visible;
}
