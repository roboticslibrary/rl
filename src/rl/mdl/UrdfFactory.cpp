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

#include <iostream> // TODO
#include <queue>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp> 
#include <rl/math/Quaternion.h>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/xml/Attribute.h>
#include <rl/xml/Document.h>
#include <rl/xml/DomParser.h>
#include <rl/xml/Node.h>
#include <rl/xml/Object.h>
#include <rl/xml/Path.h>
#include <rl/xml/Stylesheet.h>

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
#include "UrdfFactory.h"
#include "World.h"

namespace rl
{
	namespace mdl
	{
		UrdfFactory::UrdfFactory()
		{
		}
		
		UrdfFactory::~UrdfFactory()
		{
		}
		
		void
		UrdfFactory::load(const ::std::string& filename, Model* model)
		{
			::rl::xml::DomParser parser;
			
			::rl::xml::Document document = parser.readFile(filename, "", XML_PARSE_NOENT);
			document.substitute(XML_PARSE_NOENT);
			
			::rl::xml::Path path(document);
			
			::rl::xml::NodeSet robots = path.eval("/robot").getValue< ::rl::xml::NodeSet>();
			
			if (robots.empty())
			{
				throw Exception("URDF is missing robot node");
			}
			
			path.setNode(robots[0]);
			
			// name
			
			model->setName(path.eval("string(@name)").getValue< ::std::string>());
::std::cout << model->getName() << ::std::endl;
			
			// links
			
			::rl::xml::NodeSet links = path.eval("link").getValue< ::rl::xml::NodeSet>();
			
			::std::map< ::std::string, Frame*> name2frame;
			
			for (int j = 0; j < links.size(); ++j)
			{
::std::cout << "link: " << j << ::std::endl;
				::rl::xml::Path path(document, links[j]);
				
				if (path.eval("count(inertial) > 0").getValue<bool>())
				{
					Body* b = new Body();
					
					model->add(b);
					
					if (path.eval("count(inertial/origin/@xyz) > 0").getValue<bool>())
					{
						::std::vector< ::std::string> xyz;
						::std::string tmp = path.eval("string(inertial/origin/@xyz)").getValue< ::std::string>();
						::boost::split(xyz, tmp, ::boost::algorithm::is_space(), ::boost::algorithm::token_compress_on);
						
						b->setCenterOfMass(
							::boost::lexical_cast< ::rl::math::Real>(xyz[0]),
							::boost::lexical_cast< ::rl::math::Real>(xyz[1]),
							::boost::lexical_cast< ::rl::math::Real>(xyz[2])
						);
					}
					else
					{
						b->setCenterOfMass(0, 0, 0);
					}
::std::cout << "\tcog: " << b->cm.transpose() << ::std::endl;
					
					if (path.eval("count(inertial/origin/@abc) > 0").getValue<bool>())
					{
						throw Exception("URDF link/inertial/origin/@abc not supported");
					}
					
					b->setInertia(
						path.eval("number(inertial/inertia/@ixx)").getValue< ::rl::math::Real>(1),
						path.eval("number(inertial/inertia/@iyy)").getValue< ::rl::math::Real>(1),
						path.eval("number(inertial/inertia/@izz)").getValue< ::rl::math::Real>(1),
						path.eval("number(inertial/inertia/@iyz)").getValue< ::rl::math::Real>(0),
						path.eval("number(inertial/inertia/@ixz)").getValue< ::rl::math::Real>(0),
						path.eval("number(inertial/inertia/@ixy)").getValue< ::rl::math::Real>(0)
					);
::std::cout << "\tinertia: " << b->ic.voigt6().transpose() << ::std::endl;
					
					b->setMass(
						path.eval("number(inertial/mass/@value)").getValue< ::rl::math::Real>(1)
					);
::std::cout << "\tmass: " << b->m << ::std::endl;
					
					b->setName(path.eval("string(@name)").getValue< ::std::string>());
::std::cout << "\tname: " << b->getName() << ::std::endl;
					
					name2frame[path.eval("string(@name)").getValue< ::std::string>()] = b;
				}
				else
				{
					Frame* f = new Frame();
					
					model->add(f);
					
					f->setName(path.eval("string(@name)").getValue< ::std::string>());
::std::cout << "\tname: " << f->getName() << ::std::endl;
					
					name2frame[path.eval("string(@name)").getValue< ::std::string>()] = f;
				}
			}
			
			// joints
			
			::std::map< ::std::string, ::std::string> child2parent;
			
			::rl::xml::NodeSet joints = path.eval("joint").getValue< ::rl::xml::NodeSet>();
			
			for (int j = 0; j < joints.size(); ++j)
			{
::std::cout << "joint: " << j << ::std::endl;
				::rl::xml::Path path(document, joints[j]);
				
				child2parent[path.eval("string(child/@link)").getValue< ::std::string>()] = path.eval("string(parent/@link)").getValue< ::std::string>();
				
				Frame* parent = name2frame[path.eval("string(parent/@link)").getValue< ::std::string>()];
				Frame* child = name2frame[path.eval("string(child/@link)").getValue< ::std::string>()];
::std::cout << "\tparent: " << parent->getName() << ::std::endl;
::std::cout << "\tchild: " << child->getName() << ::std::endl;
				
				if (nullptr == parent || nullptr == child)
				{
					throw Exception("URDF joint has missing parent or child link");
				}
				
				// frame
				
				Frame* frame = new Frame();
				model->add(frame);
				
				// fixed
				
				Fixed* fixed = new Fixed();
				
				if (path.eval("count(origin/@rpy) > 0").getValue<bool>())
				{
					::std::vector< ::std::string> rpy;
					::std::string tmp = path.eval("string(origin/@rpy)").getValue< ::std::string>();
					::boost::split(rpy, tmp, ::boost::algorithm::is_space(), ::boost::algorithm::token_compress_on);
					
					fixed->t = ::rl::math::AngleAxis(
						::boost::lexical_cast< ::rl::math::Real>(rpy[2]),
						 ::rl::math::Vector3::UnitZ()
					) * ::rl::math::AngleAxis(
						::boost::lexical_cast< ::rl::math::Real>(rpy[1]),
						::rl::math::Vector3::UnitY()
					) * ::rl::math::AngleAxis(
						::boost::lexical_cast< ::rl::math::Real>(rpy[0]),
						::rl::math::Vector3::UnitX()
					);
				}
				
				if (path.eval("count(origin/@xyz) > 0").getValue<bool>())
				{
					::std::vector< ::std::string> xyz;
					::std::string tmp = path.eval("string(origin/@xyz)").getValue< ::std::string>();
					::boost::split(xyz, tmp, ::boost::algorithm::is_space(), ::boost::algorithm::token_compress_on);
					
					fixed->t.translation().x() = ::boost::lexical_cast< ::rl::math::Real>(xyz[0]);
					fixed->t.translation().y() = ::boost::lexical_cast< ::rl::math::Real>(xyz[1]);
					fixed->t.translation().z() = ::boost::lexical_cast< ::rl::math::Real>(xyz[2]);
				}
::std::cout << "\torigin.translation: " << fixed->t.translation().transpose() << ::std::endl;
::std::cout << "\torigin.rotation: " << fixed->t.rotation().eulerAngles(2, 1, 0).reverse().transpose() * ::rl::math::RAD2DEG << ::std::endl;
				
				fixed->x.rotation() = fixed->t.linear().transpose();
				fixed->x.translation() = fixed->t.translation();
				
				// joint
				
				::std::string type = path.eval("string(@type)").getValue< ::std::string>();
::std::cout << "\ttype: " << type << ::std::endl;
				
				if ("fixed" == type)
				{
					model->add(fixed, parent, child);
					fixed->setName(path.eval("string(@name)").getValue< ::std::string>());
::std::cout << "\tname: " << fixed->getName() << ::std::endl;
				}
				else if ("floating" == type)
				{
					throw Exception("URDF floating joint not supported");
				}
				else if ("planar" == type)
				{
					throw Exception("URDF planar joint not supported");
				}
				else if ("prismatic" == type)
				{
					Prismatic* p = new Prismatic();
					
					model->add(fixed, parent, frame);
					model->add(p, frame, child);
					
					p->max(0) = path.eval("number(limit/@upper)").getValue< ::rl::math::Real>(::std::numeric_limits< ::rl::math::Real>::max());
::std::cout << "\tmax: " << p->max(0) << ::std::endl;
					p->min(0) = path.eval("number(limit/@lower)").getValue< ::rl::math::Real>(-::std::numeric_limits< ::rl::math::Real>::max());
::std::cout << "\tmin: " << p->min(0) << ::std::endl;
					p->offset(0) = 0;
					p->speed(0) = path.eval("number(limit/@velocity)").getValue< ::rl::math::Real>(0);
::std::cout << "\tspeed: " << p->speed(0) << ::std::endl;
					p->wraparound(0) = false;
					
					if (path.eval("count(axis/@xyz) > 0").getValue<bool>())
					{
						::std::vector< ::std::string> xyz;
						::std::string tmp = path.eval("string(axis/@xyz)").getValue< ::std::string>();
						::boost::split(xyz, tmp, ::boost::algorithm::is_space(), ::boost::algorithm::token_compress_on);
						
						p->S(3, 0) = ::boost::lexical_cast< ::rl::math::Real>(xyz[0]);
						p->S(4, 0) = ::boost::lexical_cast< ::rl::math::Real>(xyz[1]);
						p->S(5, 0) = ::boost::lexical_cast< ::rl::math::Real>(xyz[2]);
					}
					else
					{
						p->S(3, 0) = 1;
						p->S(4, 0) = 0;
						p->S(5, 0) = 0;
					}
::std::cout << "\taxis: " << p->S(3, 0) << " " << p->S(4, 0) << " " << p->S(5, 0) << ::std::endl;
					
					p->setName(path.eval("string(@name)").getValue< ::std::string>());
::std::cout << "\tname: " << p->getName() << ::std::endl;
				}
				else if ("revolute" == type)
				{
					Revolute* r = new Revolute();
					
					model->add(fixed, parent, frame);
					model->add(r, frame, child);
					
					r->max(0) = path.eval("number(limit/@upper)").getValue< ::rl::math::Real>(::std::numeric_limits< ::rl::math::Real>::max());
::std::cout << "\tmax: " << r->max(0) * ::rl::math::RAD2DEG << ::std::endl;
					r->min(0) = path.eval("number(limit/@lower)").getValue< ::rl::math::Real>(-::std::numeric_limits< ::rl::math::Real>::max());
::std::cout << "\tmin: " << r->min(0) * ::rl::math::RAD2DEG << ::std::endl;
					r->offset(0) = 0;
					r->speed(0) = path.eval("number(limit/@velocity)").getValue< ::rl::math::Real>(0);
::std::cout << "\tspeed: " << r->speed(0) * ::rl::math::RAD2DEG << ::std::endl;
					r->wraparound(0) = "continuous" == path.eval("string(@type)").getValue< ::std::string>() ? true : false;
					
					if (path.eval("count(axis/@xyz) > 0").getValue<bool>())
					{
						::std::vector< ::std::string> xyz;
						::std::string tmp = path.eval("string(axis/@xyz)").getValue< ::std::string>();
						::boost::split(xyz, tmp, ::boost::algorithm::is_space(), ::boost::algorithm::token_compress_on);
						
						r->S(0, 0) = ::boost::lexical_cast< ::rl::math::Real>(xyz[0]);
						r->S(1, 0) = ::boost::lexical_cast< ::rl::math::Real>(xyz[1]);
						r->S(2, 0) = ::boost::lexical_cast< ::rl::math::Real>(xyz[2]);
					}
					else
					{
						r->S(0, 0) = 1;
						r->S(1, 0) = 0;
						r->S(2, 0) = 0;
					}
::std::cout << "\taxis: " << r->S(0, 0) << " " << r->S(1, 0) << " " << r->S(2, 0) << ::std::endl;
					
					r->setName(path.eval("string(@name)").getValue< ::std::string>());
::std::cout << "\tname: " << r->getName() << ::std::endl;
				}
			}
			
			// root
			
			World* world = new World();
			model->add(world);
			world->setGravity(0, 0, 9.80665);
			
			Frame* root = nullptr;
			
			for (::std::map< ::std::string, Frame*>::const_iterator frame = name2frame.begin(); frame != name2frame.end(); ++frame)
			{
				if (child2parent.end() == child2parent.find(frame->first))
				{
					if (nullptr == root)
					{
						root = name2frame[frame->first];
::std::cout << "root: " << root->getName() << ::std::endl;
						Fixed* fixed = new Fixed();
						model->add(fixed, world, root);
					}
					else
					{
						throw Exception("URDF has more than one root node");
					}
				}
			}
			
			if (nullptr == root)
			{
				throw Exception("URDF has no root node");
			}
			
			model->update();
		}
	}
}
