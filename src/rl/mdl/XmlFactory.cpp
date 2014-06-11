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

#include <queue>
#include <rl/math/Quaternion.h>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/xml/Attribute.h>
#include <rl/xml/Document.h>
#include <rl/xml/DomParser.h>
#include <rl/xml/Node.h>
#include <rl/xml/Object.h>
#include <rl/xml/Path.h>

#include "Body.h"
#include "Cylindrical.h"
#include "Dynamic.h"
#include "Exception.h"
#include "Fixed.h"
#include "Helical.h"
#include "Joint.h"
#include "Kinematic.h"
#include "Model.h"
#include "Prismatic.h"
#include "Revolute.h"
#include "Spherical.h"
#include "World.h"
#include "XmlFactory.h"

namespace rl
{
	namespace mdl
	{
		XmlFactory::XmlFactory()
		{
		}
		
		XmlFactory::~XmlFactory()
		{
		}
		
		void
		XmlFactory::load(const ::std::string& filename, Model* model)
		{
			::rl::xml::DomParser parser;
			
			::rl::xml::Document doc = parser.readFile(filename, "", XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
			
			doc.substitute(XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
			
			::rl::xml::Path path(doc);
			
			::rl::xml::Object instance = path.eval("//model");
			
			for (int i = 0; i < ::std::min(1, instance.getNodeNr()); ++i)
			{
				// manufacturer
				
				model->setManufacturer(path.eval("string(manufacturer)", instance.getNodeTab(i)).getStringval());
				
				// name
				
				model->setName(path.eval("string(name)", instance.getNodeTab(i)).getStringval());
				
				// frame
				
				::rl::xml::Object frames = path.eval("body|frame|world", instance.getNodeTab(i));
				
				::std::map< ::std::string, Frame* > id2frame;
				
				for (int j = 0; j < frames.getNodeNr(); ++j)
				{
					if ("body" == frames.getNodeTab(j).getName())
					{
						Body* b = new Body();
						
						model->add(b);
						
						b->setCenterOfMass(
							path.eval("number(cm/x)", frames.getNodeTab(j)).getFloatval(0),
							path.eval("number(cm/y)", frames.getNodeTab(j)).getFloatval(0),
							path.eval("number(cm/z)", frames.getNodeTab(j)).getFloatval(0)
						);
						
						b->setInertia(
							path.eval("number(i/xx)", frames.getNodeTab(j)).getFloatval(1),
							path.eval("number(i/yy)", frames.getNodeTab(j)).getFloatval(1),
							path.eval("number(i/zz)", frames.getNodeTab(j)).getFloatval(1),
							path.eval("number(i/yz)", frames.getNodeTab(j)).getFloatval(0),
							path.eval("number(i/xz)", frames.getNodeTab(j)).getFloatval(0),
							path.eval("number(i/xy)", frames.getNodeTab(j)).getFloatval(0)
						);
						
						b->setMass(
							path.eval("number(m)", frames.getNodeTab(j)).getFloatval(1)
						);
						
						b->setName(path.eval("string(@id)", frames.getNodeTab(j)).getStringval());
						
						id2frame[path.eval("string(@id)", frames.getNodeTab(j)).getStringval()] = b;
					}
					else if ("frame" == frames.getNodeTab(j).getName())
					{
						Frame* f = new Frame();
						
						model->add(f);
						
						f->setName(path.eval("string(@id)", frames.getNodeTab(j)).getStringval());
						
						id2frame[path.eval("string(@id)", frames.getNodeTab(j)).getStringval()] = f;
					}
					else if ("world" == frames.getNodeTab(j).getName())
					{
						World* w = new World();
						
						model->add(w);
						
						w->t = ::rl::math::AngleAxis(
							path.eval("number(rotation/z)", frames.getNodeTab(j)).getFloatval(0) * ::rl::math::DEG2RAD,
							::rl::math::Vector3::UnitZ()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/y)", frames.getNodeTab(j)).getFloatval(0) * ::rl::math::DEG2RAD,
							::rl::math::Vector3::UnitY()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/x)", frames.getNodeTab(j)).getFloatval(0) * ::rl::math::DEG2RAD,
							::rl::math::Vector3::UnitX()
						);
						
						w->t.translation().x() = path.eval("number(translation/x)", frames.getNodeTab(j)).getFloatval(0);
						w->t.translation().y() = path.eval("number(translation/y)", frames.getNodeTab(j)).getFloatval(0);
						w->t.translation().z() = path.eval("number(translation/z)", frames.getNodeTab(j)).getFloatval(0);
						
						w->setGravity(
							path.eval("number(g/x)", frames.getNodeTab(j)).getFloatval(0),
							path.eval("number(g/y)", frames.getNodeTab(j)).getFloatval(0),
							path.eval("number(g/z)", frames.getNodeTab(j)).getFloatval(0)
						);
						
						w->setName(path.eval("string(@id)", frames.getNodeTab(j)).getStringval());
						
						id2frame[path.eval("string(@id)", frames.getNodeTab(j)).getStringval()] = w;
					}
				}
				
				// selfcollision
				
				for (int j = 0; j < frames.getNodeNr(); ++j)
				{
					if ("body" == frames.getNodeTab(j).getName())
					{
						Body* b1 = dynamic_cast< Body* >(id2frame[path.eval("string(@id)", frames.getNodeTab(j)).getStringval()]);
						
						::rl::xml::Object ignores = path.eval("ignore", frames.getNodeTab(j));
						
						for (int k = 0; k < ignores.getNodeNr(); ++k)
						{
							if (ignores.getNodeTab(k).hasAttribute("idref"))
							{
								Body* b2 = dynamic_cast< Body* >(id2frame[ignores.getNodeTab(k).getAttribute("idref").getValue()]);
								
								b1->selfcollision.insert(b2);
								b2->selfcollision.insert(b1);
							}
							else
							{
								b1->collision = false;
							}
						}
					}
				}
				
				// transforms
				
				::rl::xml::Object transforms = path.eval("cylindrical|fixed|helical|prismatic|revolute|spherical", instance.getNodeTab(i));
				
				for (int j = 0; j < transforms.getNodeNr(); ++j)
				{
					Frame* a = id2frame[path.eval("string(frame/a/@idref)", transforms.getNodeTab(j)).getStringval()];
					Frame* b = id2frame[path.eval("string(frame/b/@idref)", transforms.getNodeTab(j)).getStringval()];
					
					if ("cylindrical" == transforms.getNodeTab(j).getName())
					{
						Cylindrical* c = new Cylindrical();
						
						model->add(c, a, b);
						
						c->max(0) = path.eval("number(max1)", transforms.getNodeTab(j)).getFloatval(::std::numeric_limits< ::rl::math::Real >::max());
						c->min(0) = path.eval("number(min1)", transforms.getNodeTab(j)).getFloatval(-::std::numeric_limits< ::rl::math::Real >::max());
						c->offset(0) = path.eval("number(offset1)", transforms.getNodeTab(j)).getFloatval(0);
						c->speed(0) = path.eval("number(speed1)", transforms.getNodeTab(j)).getFloatval(0);
						c->wraparound(0) = path.eval("count(wraparound1) > 0", transforms.getNodeTab(j)).getBoolval();
						
						c->max(1) = path.eval("number(max2)", transforms.getNodeTab(j)).getFloatval(::std::numeric_limits< ::rl::math::Real >::max());
						c->min(1) = path.eval("number(min2)", transforms.getNodeTab(j)).getFloatval(-::std::numeric_limits< ::rl::math::Real >::max());
						c->offset(1) = path.eval("number(offset2)", transforms.getNodeTab(j)).getFloatval(0);
						c->speed(1) = path.eval("number(speed2)", transforms.getNodeTab(j)).getFloatval(0);
						c->wraparound(1) = path.eval("count(wraparound2) > 0", transforms.getNodeTab(j)).getBoolval();
						
						c->S(0, 0) = path.eval("number(axis1/x)", transforms.getNodeTab(j)).getFloatval(0);
						c->S(1, 0) = path.eval("number(axis1/y)", transforms.getNodeTab(j)).getFloatval(0);
						c->S(2, 0) = path.eval("number(axis1/z)", transforms.getNodeTab(j)).getFloatval(1);
						
						c->S(3, 1) = path.eval("number(axis2/x)", transforms.getNodeTab(j)).getFloatval(0);
						c->S(4, 1) = path.eval("number(axis2/y)", transforms.getNodeTab(j)).getFloatval(0);
						c->S(5, 1) = path.eval("number(axis2/z)", transforms.getNodeTab(j)).getFloatval(1);
						
						c->setName(path.eval("string(@id)", transforms.getNodeTab(j)).getStringval());
					}
					else if ("fixed" == transforms.getNodeTab(j).getName())
					{
						Fixed* f = new Fixed();
						
						model->add(f, a, b);
						
						f->t = ::rl::math::AngleAxis(
							path.eval("number(rotation/z)", transforms.getNodeTab(j)).getFloatval(0) * ::rl::math::DEG2RAD,
							::rl::math::Vector3::UnitZ()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/y)", transforms.getNodeTab(j)).getFloatval(0) * ::rl::math::DEG2RAD,
							::rl::math::Vector3::UnitY()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/x)", transforms.getNodeTab(j)).getFloatval(0) * ::rl::math::DEG2RAD,
							::rl::math::Vector3::UnitX()
						);
						
						f->t.translation().x() = path.eval("number(translation/x)", transforms.getNodeTab(j)).getFloatval(0);
						f->t.translation().y() = path.eval("number(translation/y)", transforms.getNodeTab(j)).getFloatval(0);
						f->t.translation().z() = path.eval("number(translation/z)", transforms.getNodeTab(j)).getFloatval(0);
						
						f->x.rotation() = f->t.linear().transpose();
						f->x.translation() = f->t.translation();
						
						f->setName(path.eval("string(@id)", transforms.getNodeTab(j)).getStringval());
					}
					else if ("helical" == transforms.getNodeTab(j).getName())
					{
						Helical* h = new Helical();
						
						model->add(h, a, b);
						
						h->max(0) = path.eval("number(max)", transforms.getNodeTab(j)).getFloatval(::std::numeric_limits< ::rl::math::Real >::max());
						h->min(0) = path.eval("number(min)", transforms.getNodeTab(j)).getFloatval(-::std::numeric_limits< ::rl::math::Real >::max());
						h->offset(0) = path.eval("number(offset)", transforms.getNodeTab(j)).getFloatval(0);
						h->speed(0) = path.eval("number(speed)", transforms.getNodeTab(j)).getFloatval(0);
						h->wraparound(0) = path.eval("count(wraparound) > 0", transforms.getNodeTab(j)).getBoolval();
						
						h->S(0, 0) = path.eval("number(axis1/x)", transforms.getNodeTab(j)).getFloatval(0);
						h->S(1, 0) = path.eval("number(axis1/y)", transforms.getNodeTab(j)).getFloatval(0);
						h->S(2, 0) = path.eval("number(axis1/z)", transforms.getNodeTab(j)).getFloatval(1);
						
						h->S(3, 0) = path.eval("number(axis2/x)", transforms.getNodeTab(j)).getFloatval(0);
						h->S(4, 0) = path.eval("number(axis2/y)", transforms.getNodeTab(j)).getFloatval(0);
						h->S(5, 0) = path.eval("number(axis2/z)", transforms.getNodeTab(j)).getFloatval(1);
						
						h->setPitch(path.eval("number(pitch)", transforms.getNodeTab(j)).getFloatval(1));
						
						h->setName(path.eval("string(@id)", transforms.getNodeTab(j)).getStringval());
					}
					else if ("prismatic" == transforms.getNodeTab(j).getName())
					{
						Prismatic* p = new Prismatic();
						
						model->add(p, a, b);
						
						p->max(0) = path.eval("number(max)", transforms.getNodeTab(j)).getFloatval(::std::numeric_limits< ::rl::math::Real >::max());
						p->min(0) = path.eval("number(min)", transforms.getNodeTab(j)).getFloatval(-::std::numeric_limits< ::rl::math::Real >::max());
						p->offset(0) = path.eval("number(offset)", transforms.getNodeTab(j)).getFloatval(0);
						p->speed(0) = path.eval("number(speed)", transforms.getNodeTab(j)).getFloatval(0);
						p->wraparound(0) = path.eval("count(wraparound) > 0", transforms.getNodeTab(j)).getBoolval();
						
						p->S(3, 0) = path.eval("number(axis/x)", transforms.getNodeTab(j)).getFloatval(0);
						p->S(4, 0) = path.eval("number(axis/y)", transforms.getNodeTab(j)).getFloatval(0);
						p->S(5, 0) = path.eval("number(axis/z)", transforms.getNodeTab(j)).getFloatval(1);
						
						p->setName(path.eval("string(@id)", transforms.getNodeTab(j)).getStringval());
					}
					else if ("revolute" == transforms.getNodeTab(j).getName())
					{
						Revolute* r = new Revolute();
						
						model->add(r, a, b);
						
						r->max(0) = path.eval("number(max)", transforms.getNodeTab(j)).getFloatval(::std::numeric_limits< ::rl::math::Real >::max());
						r->min(0) = path.eval("number(min)", transforms.getNodeTab(j)).getFloatval(-::std::numeric_limits< ::rl::math::Real >::max());
						r->offset(0) = path.eval("number(offset)", transforms.getNodeTab(j)).getFloatval(0);
						r->speed(0) = path.eval("number(speed)", transforms.getNodeTab(j)).getFloatval(0);
						r->wraparound(0) = path.eval("count(wraparound) > 0", transforms.getNodeTab(j)).getBoolval();
						
						r->S(0, 0) = path.eval("number(axis/x)", transforms.getNodeTab(j)).getFloatval(0);
						r->S(1, 0) = path.eval("number(axis/y)", transforms.getNodeTab(j)).getFloatval(0);
						r->S(2, 0) = path.eval("number(axis/z)", transforms.getNodeTab(j)).getFloatval(1);
						
						r->max *= ::rl::math::DEG2RAD;
						r->min *= ::rl::math::DEG2RAD;
						r->offset *= ::rl::math::DEG2RAD;
						r->speed *= ::rl::math::DEG2RAD;
						
						r->setName(path.eval("string(@id)", transforms.getNodeTab(j)).getStringval());
					}
					else if ("spherical" == transforms.getNodeTab(j).getName())
					{
						Spherical* s = new Spherical();
						
						model->add(s, a, b);
						
						s->setName(path.eval("string(@id)", transforms.getNodeTab(j)).getStringval());
					}
				}
			}
			
			model->update();
		}
	}
}
