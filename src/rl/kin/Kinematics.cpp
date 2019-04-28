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

#include <algorithm>
#include <rl/math/algorithm.h>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/xml/Attribute.h>
#include <rl/xml/Document.h>
#include <rl/xml/DomParser.h>
#include <rl/xml/Node.h>
#include <rl/xml/Object.h>
#include <rl/xml/Path.h>
#include <rl/xml/Stylesheet.h>

#include "Element.h"
#include "Exception.h"
#include "Frame.h"
#include "Kinematics.h"
#include "Link.h"
#include "Joint.h"
#include "Prismatic.h"
#include "Puma.h"
#include "Revolute.h"
#include "Rhino.h"
#include "Transform.h"

namespace rl
{
	namespace kin
	{
		Kinematics::Kinematics() :
			elements(),
			frames(),
			leaves(),
			links(),
			jacobian(),
			jacobianInverse(),
			joints(),
			manufacturer(),
			name(),
			root(),
			tools(),
			transforms(),
			tree(),
			randDistribution(0, 1),
			randEngine(::std::random_device()())
		{
		}
		
		Kinematics::~Kinematics()
		{
		}
		
		bool
		Kinematics::areColliding(const ::std::size_t& i, const ::std::size_t& j) const
		{
			assert(i < this->links.size());
			assert(j < this->links.size());
			
			if (this->links[i]->selfcollision.count(this->links[j]) > 0 || this->links[j]->selfcollision.count(this->links[i]) > 0)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		
		void
		Kinematics::clamp(::rl::math::Vector& q) const
		{
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				if (this->joints[i]->wraparound)
				{
					::rl::math::Real range = ::std::abs(this->joints[i]->max - this->joints[i]->min);
					
					while (q(i) > this->joints[i]->max)
					{
						q(i) -= range;
					}
					
					while (q(i) < this->joints[i]->min)
					{
						q(i) += range;
					}
				}
				else
				{
					q(i) = ::rl::math::clamp(q(i), this->joints[i]->min, this->joints[i]->max);
				}
			}
		}
		
		::std::shared_ptr<Kinematics>
		Kinematics::create(const ::std::string& filename)
		{
			::std::shared_ptr<Kinematics> kinematics;
			
			::rl::xml::DomParser parser;
			
			::rl::xml::Document document = parser.readFile(filename, "", XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
			document.substitute(XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
			
			if ("stylesheet" == document.getRootElement().getName() || "transform" == document.getRootElement().getName())
			{
				if ("1.0" == document.getRootElement().getProperty("version"))
				{
					if (document.getRootElement().hasNamespace() && "http://www.w3.org/1999/XSL/Transform" == document.getRootElement().getNamespace().getHref())
					{
						rl::xml::Stylesheet stylesheet(document);
						document = stylesheet.apply();
					}
				}
			}
			
			::rl::xml::Path path(document);
			
			::rl::xml::NodeSet instances = path.eval("(/rl/kin|/rlkin)/kinematics|(/rl/kin|/rlkin)/puma|(/rl/kin|/rlkin)/rhino").getValue< ::rl::xml::NodeSet>();
			
			for (int i = 0; i < ::std::min(1, instances.size()); ++i)
			{
				::rl::xml::Path path(document, instances[i]);
				
				if ("puma" == instances[i].getName())
				{
					kinematics = ::std::make_shared<Puma>();
				}
				else if ("rhino" == instances[i].getName())
				{
					kinematics = ::std::make_shared<Rhino>();
				}
				else if ("kinematics" == instances[i].getName())
				{
					kinematics = ::std::make_shared<Kinematics>();
				}
				else
				{
					continue;
				}
				
				// manufacturer
				
				kinematics->manufacturer = path.eval("string(manufacturer)").getValue< ::std::string>();
				
				// name
				
				kinematics->name = path.eval("string(name)").getValue< ::std::string>();
				
				// frames
				
				::rl::xml::NodeSet frames = path.eval("frame|link|world").getValue< ::rl::xml::NodeSet>();
				
				::std::map< ::std::string, Vertex> id2vertex;
				
				for (int j = 0; j < frames.size(); ++j)
				{
					::rl::xml::Path path(document, frames[j]);
					
					Vertex v = ::boost::add_vertex(kinematics->tree);
					
					if ("frame" == frames[j].getName())
					{
						Frame* f = new Frame();
						
						f->name = path.eval("string(@id)").getValue< ::std::string>();
						
						f->frame.setIdentity();
						
						f->frame = ::rl::math::AngleAxis(
							path.eval("number(rotation/z)").getValue< ::rl::math::Real>(0) * ::rl::math::DEG2RAD,
							::rl::math::Vector3::UnitZ()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/y)").getValue< ::rl::math::Real>(0) * ::rl::math::DEG2RAD,
							::rl::math::Vector3::UnitY()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/x)").getValue< ::rl::math::Real>(0) * ::rl::math::DEG2RAD,
							::rl::math::Vector3::UnitX()
						);
						
						f->frame.translation().x() = path.eval("number(translation/x)").getValue< ::rl::math::Real>(0);
						f->frame.translation().y() = path.eval("number(translation/y)").getValue< ::rl::math::Real>(0);
						f->frame.translation().z() = path.eval("number(translation/z)").getValue< ::rl::math::Real>(0);
						
						kinematics->tree[v].reset(f);
					}
					else if ("link" == frames[j].getName())
					{
						Link* l = new Link();
						
						l->name = path.eval("string(@id)").getValue< ::std::string>();
						
						kinematics->tree[v].reset(l);
					}
					else if ("world" == frames[j].getName())
					{
						Frame* f = new Frame();
						
						f->name = path.eval("string(@id)").getValue< ::std::string>();
						
						f->frame.setIdentity();
						
						f->frame = ::rl::math::AngleAxis(
							path.eval("number(rotation/z)").getValue< ::rl::math::Real>(0) * ::rl::math::DEG2RAD,
							::rl::math::Vector3::UnitZ()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/y)").getValue< ::rl::math::Real>(0) * ::rl::math::DEG2RAD,
							::rl::math::Vector3::UnitY()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/x)").getValue< ::rl::math::Real>(0) * ::rl::math::DEG2RAD,
							::rl::math::Vector3::UnitX()
						);
						
						f->frame.translation().x() = path.eval("number(translation/x)").getValue< ::rl::math::Real>(0);
						f->frame.translation().y() = path.eval("number(translation/y)").getValue< ::rl::math::Real>(0);
						f->frame.translation().z() = path.eval("number(translation/z)").getValue< ::rl::math::Real>(0);
						
						kinematics->root = v;
						
						kinematics->tree[v].reset(f);
					}
					
					::std::string id = path.eval("string(@id)").getValue< ::std::string>();
					
					if (id2vertex.find(id) != id2vertex.end())
					{
						throw Exception("Frame with ID " + id + " not unique in file " + filename);
					}
					
					id2vertex[id] = v;
				}
				
				for (int j = 0; j < frames.size(); ++j)
				{
					::rl::xml::Path path(document, frames[j]);
					
					if ("link" == frames[j].getName())
					{
						::std::string v1Id = path.eval("string(@id)").getValue< ::std::string>();
						
						if (id2vertex.find(v1Id) == id2vertex.end())
						{
							throw Exception("Link with ID " + v1Id + " not found in file " + filename);
						}
						
						Vertex v1 = id2vertex[v1Id];
						Link* l1 = dynamic_cast<Link*>(kinematics->tree[v1].get());
						
						::rl::xml::NodeSet ignores = path.eval("ignore").getValue< ::rl::xml::NodeSet>();
						
						for (int k = 0; k < ignores.size(); ++k)
						{
							if (!ignores[k].getProperty("idref").empty())
							{
								::std::string v2IdRef = ignores[k].getProperty("idref");
								
								if (id2vertex.find(v2IdRef) == id2vertex.end())
								{
									throw Exception("Link with IDREF " + v2IdRef + " in Link with ID " + v1Id + " not found in file " + filename);
								}
								
								Vertex v2 = id2vertex[v2IdRef];
								Link* l2 = dynamic_cast<Link*>(kinematics->tree[v2].get());
								
								l1->selfcollision.insert(l2);
								l2->selfcollision.insert(l1);
							}
							else
							{
								l1->collision = false;
							}
						}
					}
				}
				
				// transforms
				
				::rl::xml::NodeSet transforms = path.eval("prismatic|revolute|transform").getValue< ::rl::xml::NodeSet>();
				
				for (int j = 0; j < transforms.size(); ++j)
				{
					::rl::xml::Path path(document, transforms[j]);
					
					::std::string name = path.eval("string(@id)").getValue< ::std::string>();
					
					::std::string aIdRef = path.eval("string(frame/a/@idref)").getValue< ::std::string>();
					
					if (id2vertex.find(aIdRef) == id2vertex.end())
					{
						throw Exception("Frame A with IDREF " + aIdRef + " in Transform with ID " + name + " not found in file " + filename);
					}
					
					Vertex a = id2vertex[aIdRef];
					
					::std::string bIdRef = path.eval("string(frame/b/@idref)").getValue< ::std::string>();
					
					if (id2vertex.find(bIdRef) == id2vertex.end())
					{
						throw Exception("Frame B with IDREF " + bIdRef + " in Transform with ID " + name + " not found in file " + filename);
					}
					
					Vertex b = id2vertex[bIdRef];
					
					Edge e = ::boost::add_edge(a, b, kinematics->tree).first;
					
					if ("prismatic" == transforms[j].getName())
					{
						Prismatic* p = new Prismatic();
						
						p->name = name;
						
						p->a = path.eval("number(dh/a)").getValue< ::rl::math::Real>(0);
						p->alpha = path.eval("number(dh/alpha)").getValue< ::rl::math::Real>(0);
						p->d = path.eval("number(dh/d)").getValue< ::rl::math::Real>(0);
						p->max = path.eval("number(max)").getValue< ::rl::math::Real>(::std::numeric_limits< ::rl::math::Real>::max());
						p->min = path.eval("number(min)").getValue< ::rl::math::Real>(-::std::numeric_limits< ::rl::math::Real>::max());
						p->offset = path.eval("number(offset)").getValue< ::rl::math::Real>(0);
						p->speed = path.eval("number(speed)").getValue< ::rl::math::Real>(0);
						p->theta = path.eval("number(dh/theta)").getValue< ::rl::math::Real>(0);
						p->wraparound = path.eval("count(wraparound) > 0").getValue<bool>();
						
						p->alpha *= ::rl::math::DEG2RAD;
						p->theta *= ::rl::math::DEG2RAD;
						
						kinematics->tree[e].reset(p);
					}
					else if ("revolute" == transforms[j].getName())
					{
						Revolute* r = new Revolute();
						
						r->name = name;
						
						r->a = path.eval("number(dh/a)").getValue< ::rl::math::Real>(0);
						r->alpha = path.eval("number(dh/alpha)").getValue< ::rl::math::Real>(0);
						r->d = path.eval("number(dh/d)").getValue< ::rl::math::Real>(0);
						r->max = path.eval("number(max)").getValue< ::rl::math::Real>(::std::numeric_limits< ::rl::math::Real>::max());
						r->min = path.eval("number(min)").getValue< ::rl::math::Real>(-::std::numeric_limits< ::rl::math::Real>::max());
						r->offset = path.eval("number(offset)").getValue< ::rl::math::Real>(0);
						r->speed = path.eval("number(speed)").getValue< ::rl::math::Real>(0);
						r->theta = path.eval("number(dh/theta)").getValue< ::rl::math::Real>(0);
						r->wraparound = path.eval("count(wraparound) > 0").getValue<bool>();
						
						r->alpha *= ::rl::math::DEG2RAD;
						r->max *= ::rl::math::DEG2RAD;
						r->min *= ::rl::math::DEG2RAD;
						r->offset *= ::rl::math::DEG2RAD;
						r->speed *= ::rl::math::DEG2RAD;
						r->theta *= ::rl::math::DEG2RAD;
						
						kinematics->tree[e].reset(r);
					}
					else if ("transform" == transforms[j].getName())
					{
						Transform* t = new Transform();
						
						t->name = name;
						
						t->transform.setIdentity();
						
						t->transform = ::rl::math::AngleAxis(
							path.eval("number(rotation/z)").getValue< ::rl::math::Real>(0) * ::rl::math::DEG2RAD,
							::rl::math::Vector3::UnitZ()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/y)").getValue< ::rl::math::Real>(0) * ::rl::math::DEG2RAD,
							::rl::math::Vector3::UnitY()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/x)").getValue< ::rl::math::Real>(0) * ::rl::math::DEG2RAD,
							::rl::math::Vector3::UnitX()
						);
						
						t->transform.translation().x() = path.eval("number(translation/x)").getValue< ::rl::math::Real>(0);
						t->transform.translation().y() = path.eval("number(translation/y)").getValue< ::rl::math::Real>(0);
						t->transform.translation().z() = path.eval("number(translation/z)").getValue< ::rl::math::Real>(0);
						
						kinematics->tree[e].reset(t);
					}
				}
			}
			
			kinematics->update();
			
			if (::std::dynamic_pointer_cast<Puma>(kinematics))
			{
				if (kinematics->joints.size() != 6 || kinematics->links.size() != 7 || kinematics->transforms.size() != 8 || kinematics->frames.size() != 9)
				{
					if (kinematics->joints.size() != 6)
					{
						throw Exception("Puma kinematics with incorrect number of joints");
					}
					
					if (kinematics->links.size() != 7)
					{
						throw Exception("Puma kinematics with incorrect number of links");
					}
					
					if (kinematics->transforms.size() != 8)
					{
						throw Exception("Puma kinematics with incorrect number of transforms");
					}
					
					if (kinematics->frames.size() != 9)
					{
						throw Exception("Puma kinematics with incorrect number of frames");
					}
				}
			}
			else if (::std::dynamic_pointer_cast<Rhino>(kinematics))
			{
				if (kinematics->joints.size() != 5 || kinematics->links.size() != 6 || kinematics->transforms.size() != 7 || kinematics->frames.size() != 8)
				{
					if (kinematics->joints.size() != 5)
					{
						throw Exception("Rhino kinematics with incorrect number of joints");
					}
					
					if (kinematics->links.size() != 6)
					{
						throw Exception("Rhino kinematics with incorrect number of links");
					}
					
					if (kinematics->transforms.size() != 7)
					{
						throw Exception("Rhino kinematics with incorrect number of transforms");
					}
					
					if (kinematics->frames.size() != 8)
					{
						throw Exception("Rhino kinematics with incorrect number of frames");
					}
				}
			}
			
			return kinematics;
		}
		
		::rl::math::Real
		Kinematics::distance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
		{
			return this->inverseOfTransformedDistance(this->transformedDistance(q1, q2));
		}
		
		const ::rl::math::Transform&
		Kinematics::forwardPosition(const ::std::size_t& i) const
		{
			return this->tree[this->leaves[i]]->frame;
		}
		
		void
		Kinematics::forwardForce(const ::rl::math::Vector& tau, ::rl::math::Vector& f) const
		{
			assert(tau.size() <= this->getDof());
			assert(f.size() <= this->getOperationalDof() * 6);
			
			f = this->jacobianInverse.transpose() * tau;
		}
		
		void
		Kinematics::forwardVelocity(const ::rl::math::Vector& qdot, ::rl::math::Vector& xdot) const
		{
			assert(qdot.size() <= this->getDof());
			assert(xdot.size() <= this->getOperationalDof() * 6);
			
			xdot = this->jacobian * qdot;
		}
		
		::rl::math::Vector
		Kinematics::generatePositionGaussian(const ::rl::math::Vector& mean, const ::rl::math::Vector& sigma)
		{
			::rl::math::Vector rand(this->getDof());
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				rand(i) = this->rand();
			}
			
			return this->generatePositionGaussian(rand, mean, sigma);
		}
		
		::rl::math::Vector
		Kinematics::generatePositionGaussian(const ::rl::math::Vector& rand, const ::rl::math::Vector& mean, const ::rl::math::Vector& sigma) const
		{
			::rl::math::Vector q(this->getDof());
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				q(i) = mean(i) + rand(i) * sigma(i);
			}
			
			this->clamp(q);
			
			return q;
		}
		
		::rl::math::Vector
		Kinematics::generatePositionUniform()
		{
			::rl::math::Vector rand(this->getDof());
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				rand(i) = this->rand();
			}
			
			return this->generatePositionUniform(rand);
		}
		
		::rl::math::Vector
		Kinematics::generatePositionUniform(const ::rl::math::Vector& rand) const
		{
			::rl::math::Vector q(this->getDof());
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				q(i) = this->getMinimum(i) + rand(i) * (this->getMaximum(i) - this->getMinimum(i));
			}
			
			return q;
		}
		
		::std::size_t
		Kinematics::getBodies() const
		{
			return this->links.size();
		}
		
		::std::size_t
		Kinematics::getDof() const
		{
			return this->joints.size();
		}
		
		const ::rl::math::Transform&
		Kinematics::getFrame(const ::std::size_t& i) const
		{
			assert(i < this->links.size());
			
			return this->links[i]->frame;
		}
		
		const ::rl::math::Matrix&
		Kinematics::getJacobian() const
		{
			return this->jacobian;
		}
		
		const ::rl::math::Matrix&
		Kinematics::getJacobianInverse() const
		{
			return this->jacobianInverse;
		}
		
		Joint*
		Kinematics::getJoint(const ::std::size_t& i) const
		{
			assert(i < this->joints.size());
			
			return this->joints[i];
		}
		
		::rl::math::Real
		Kinematics::getManipulabilityMeasure() const
		{
			return ::std::sqrt((this->jacobian * this->jacobian.transpose()).determinant());
		}
		
		::std::string
		Kinematics::getManufacturer() const
		{
			return this->manufacturer;
		}
		
		::rl::math::Real
		Kinematics::getMaximum(const ::std::size_t& i) const
		{
			assert(i < this->joints.size());
			
			return this->joints[i]->max;
		}
		
		void
		Kinematics::getMaximum(::rl::math::Vector& max) const
		{
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				max(i) = this->joints[i]->max;
			}
		}
		
		::rl::math::Real
		Kinematics::getMinimum(const ::std::size_t& i) const
		{
			assert(i < this->joints.size());
			
			return this->joints[i]->min;
		}
		
		void
		Kinematics::getMinimum(::rl::math::Vector& min) const
		{
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				min(i) = this->joints[i]->min;
			}
		}
		
		::std::string
		Kinematics::getName() const
		{
			return this->name;
		}
		
		::std::size_t
		Kinematics::getOperationalDof() const
		{
			return this->leaves.size();
		}
		
		void
		Kinematics::getPosition(::rl::math::Vector& q) const
		{
			assert(q.size() == this->getDof());
			
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				q(i) = this->joints[i]->getPosition();
			}
		}
		
		void
		Kinematics::getPositionUnits(::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1>& units) const
		{
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				units(i) = this->joints[i]->getPositionUnit();
			}
		}
		
		void
		Kinematics::getSpeed(::rl::math::Vector& speed) const
		{
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				speed(i) = this->joints[i]->speed;
			}
		}
		
		void
		Kinematics::getSpeedUnits(::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1>& units) const
		{
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				units(i) = this->joints[i]->getSpeedUnit();
			}
		}
		
		void
		Kinematics::getWraparounds(::Eigen::Matrix<bool, ::Eigen::Dynamic, 1>& wraparounds) const
		{
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				wraparounds(i) = this->joints[i]->wraparound;
			}
		}
		
		void
		Kinematics::interpolate(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, const ::rl::math::Real& alpha, ::rl::math::Vector& q) const
		{
			assert(q1.size() == this->getDof());
			assert(q2.size() == this->getDof());
			assert(alpha >= 0);
			assert(alpha <= 1);
			assert(q.size() == this->getDof());
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				if (this->joints[i]->wraparound)
				{
					::rl::math::Real diff = ::std::abs(q2(i) - q1(i));
					::rl::math::Real range = ::std::abs(this->joints[i]->max - this->joints[i]->min);
					
					if (::std::abs(range - diff) < diff)
					{
						if (q1(i) > q2(i))
						{
							q(i) = (1 - alpha) * q1(i) + alpha * (q2(i) + range);
						}
						else
						{
							q(i) = (1 - alpha) * (q1(i) + range) + alpha * q2(i);
						}
						
						while (q(i) > this->joints[i]->max)
						{
							q(i) -= range;
						}
						
						while (q(i) < this->joints[i]->min)
						{
							q(i) += range;
						}
					}
					else
					{
						q(i) = (1 - alpha) * q1(i) + alpha * q2(i);
					}
				}
				else
				{
					q(i) = (1 - alpha) * q1(i) + alpha * q2(i);
				}
			}
		}
		
		void
		Kinematics::inverseForce(const ::rl::math::Vector& f, ::rl::math::Vector& tau) const
		{
			assert(f.size() <= this->getOperationalDof() * 6);
			assert(tau.size() <= this->getDof());
			
			tau = this->jacobian.transpose() * f;
		}
		
		::rl::math::Real
		Kinematics::inverseOfTransformedDistance(const ::rl::math::Real& d) const
		{
			return ::std::sqrt(d);
		}
		
		bool
		Kinematics::inversePosition(const ::rl::math::Transform& x, ::rl::math::Vector& q, const ::std::size_t& leaf, const ::rl::math::Real& delta, const ::rl::math::Real& epsilon, const ::std::size_t& iterations, const ::std::chrono::nanoseconds& duration)
		{
			assert(q.size() == this->getDof());
			
			::std::chrono::steady_clock::time_point start = ::std::chrono::steady_clock::now();
			double remaining = ::std::chrono::duration<double>(duration).count();
			::std::size_t iteration = 0;
			
			this->getPosition(q);
			::rl::math::Vector q2(this->getDof());
			::rl::math::Vector dq(this->getDof());
			::rl::math::Vector dx(6 * this->getOperationalDof());
			
			do
			{
				do
				{
					this->updateFrames();
					dx.setZero();
					
					::rl::math::VectorBlock dxi = dx.segment(6 * leaf, 6);
					dxi = this->forwardPosition(leaf).toDelta(x);
					
					if (dx.squaredNorm() < ::std::pow(epsilon, 2))
					{
						this->normalize(q);
						this->setPosition(q);
						
						if (this->isValid(q))
						{
							return true;
						}
					}
					
					this->updateJacobian();
					this->updateJacobianInverse();
					this->inverseVelocity(dx, dq);
					
					this->step(q, dq, q2);
					
					if (this->transformedDistance(q, q2) > ::std::pow(delta, 2))
					{
						this->interpolate(q, q2, delta, q2);
					}
					
					q = q2;
					this->setPosition(q);
					
					remaining = ::std::chrono::duration<double>(duration - (::std::chrono::steady_clock::now() - start)).count();
					
					if (0 == ++iteration % 100)
					{
						break;
					}
				}
				while (remaining > 0 && iteration < iterations);
				
				q = this->generatePositionUniform();
				this->setPosition(q);
				
				remaining = ::std::chrono::duration<double>(duration - (::std::chrono::steady_clock::now() - start)).count();
			}
			while (remaining > 0 && iteration < iterations);
			
			return false;
		}
		
		void
		Kinematics::inverseVelocity(const ::rl::math::Vector& xdot, ::rl::math::Vector& qdot) const
		{
			assert(xdot.size() <= this->getOperationalDof() * 6);
			assert(qdot.size() <= this->getDof());
			
			qdot = this->jacobianInverse * xdot;
		}
		
		bool
		Kinematics::isColliding(const ::std::size_t& i) const
		{
			assert(i < this->links.size());
			
			return this->links[i]->collision;
		}
		
		bool
		Kinematics::isSingular() const
		{
#if 0
			return !(this->getManipulabilityMeasure() > 0);
#else
			::Eigen::JacobiSVD< ::rl::math::Matrix> svd(this->jacobian);
			return (::std::abs(svd.singularValues()(svd.singularValues().size() - 1)) > ::std::numeric_limits< ::rl::math::Real>::epsilon()) ? false : true;
#endif
		}
		
		bool
		Kinematics::isValid(const ::rl::math::Vector& q) const
		{
			assert(q.size() == this->getDof());
			
			for (::std::ptrdiff_t i = 0; i < q.size(); ++i)
			{
				if (q(i) < this->joints[i]->min || q(i) > this->joints[i]->max)
				{
					return false;
				}
			}
			
			return true;
		}
		
		void
		Kinematics::normalize(::rl::math::Vector& q) const
		{
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				this->joints[i]->normalize(q(i));
			}
		}
		
		::std::uniform_real_distribution< ::rl::math::Real>::result_type
		Kinematics::rand()
		{
			return this->randDistribution(this->randEngine);
		}
		
		void
		Kinematics::seed(const ::std::mt19937::result_type& value)
		{
			this->randEngine.seed(value);
		}
		
		void
		Kinematics::setColliding(const ::std::size_t& i, const bool& doesCollide)
		{
			assert(i < this->getBodies());
			
			this->links[i]->collision = doesCollide;
		}
		
		void
		Kinematics::setColliding(const ::std::size_t& i, const ::std::size_t& j, const bool& doCollide)
		{
			assert(i < this->getBodies());
			assert(j < this->getBodies());
			
			if (doCollide)
			{
				this->links[i]->selfcollision.erase(this->links[j]);
				this->links[j]->selfcollision.erase(this->links[i]);
			}
			else
			{
				this->links[i]->selfcollision.insert(this->links[j]);
				this->links[j]->selfcollision.insert(this->links[i]);
			}
		}
		
		void
		Kinematics::setPosition(const ::rl::math::Vector& q)
		{
			assert(q.size() == this->getDof());
			
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				this->joints[i]->setPosition(q(i));
			}
		}
		
		void
		Kinematics::step(const ::rl::math::Vector& q1, const ::rl::math::Vector& dq, ::rl::math::Vector& q2) const
		{
			assert(q1.size() == this->getDof());
			assert(dq.size() == this->getDof());
			assert(q2.size() == this->getDof());
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				q2(i) = q1(i) + dq(i);
			}
			
			this->clamp(q2);
		}
		
		::rl::math::Transform&
		Kinematics::tool(const ::std::size_t& i)
		{
			assert(i < this->tools.size());
			
			return this->tree[this->tools[i]]->transform;
		}
		
		const ::rl::math::Transform&
		Kinematics::tool(const ::std::size_t& i) const
		{
			assert(i < this->tools.size());
			
			return this->tree[this->tools[i]]->transform;
		}
		
		::rl::math::Real
		Kinematics::transformedDistance(const ::rl::math::Real& d) const
		{
			return ::std::pow(d, 2);
		}
		
		::rl::math::Real
		Kinematics::transformedDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
		{
			assert(q1.size() == this->getDof());
			assert(q2.size() == this->getDof());
			
			::rl::math::Real d = 0;
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				::rl::math::Real delta = ::std::abs(q2(i) - q1(i));
				
				if (this->joints[i]->wraparound)
				{
					::rl::math::Real range = ::std::abs(this->joints[i]->max - this->joints[i]->min);
					d += this->transformedDistance(::std::min(delta, ::std::abs(range - delta)));
				}
				else
				{
					d += this->transformedDistance(delta);
				}
			}
			
			return d;
		}
		
		::rl::math::Real
		Kinematics::transformedDistance(const ::rl::math::Real& q1, const ::rl::math::Real& q2, const ::std::size_t& i) const
		{
			::rl::math::Real delta = ::std::abs(q1 - q2);
			
			if (this->joints[i]->wraparound)
			{
				::rl::math::Real range = ::std::abs(this->joints[i]->max - this->joints[i]->min);
				return this->transformedDistance(::std::max(delta, ::std::abs(range - delta)));
			}
			else
			{
				return this->transformedDistance(delta);
			}
		}
		
		void
		Kinematics::update()
		{
			this->elements.clear();
			this->joints.clear();
			this->leaves.clear();
			this->links.clear();
			this->tools.clear();
			this->transforms.clear();
			
			this->update(this->root);
			
			for (::std::vector<Vertex>::iterator i = this->leaves.begin(); i != this->leaves.end(); ++i)
			{
				Vertex v = *i;
				
				while (v != this->root)
				{
					Edge e = *::boost::in_edges(v, this->tree).first;
					Transform* transform = this->tree[e].get();
					
					if (Joint* joint = dynamic_cast<Joint*>(transform))
					{
						joint->leaves.insert(*i);
					}
					
					v = ::boost::source(e, this->tree);
				}
			}
			
			this->jacobian.resize(this->leaves.size() * 6, this->joints.size());
			this->jacobianInverse.resize(this->joints.size(), this->leaves.size() * 6);
		}
		
		void
		Kinematics::update(Vertex& u)
		{
			Frame* frame = this->tree[u].get();
			this->elements.push_back(frame);
			this->frames.push_back(frame);
			
			if (Link* link = dynamic_cast<Link*>(frame))
			{
				this->links.push_back(link);
			}
			
			if (::boost::out_degree(u, this->tree) > 0)
			{
				for (OutEdgeIteratorPair i = ::boost::out_edges(u, this->tree); i.first != i.second; ++i.first)
				{
					Edge e = *i.first;
					Vertex v = ::boost::target(e, this->tree);
					
					Transform* transform = this->tree[e].get();
					this->elements.push_back(transform);
					this->transforms.push_back(transform);
					transform->in = this->tree[u].get();
					transform->out = this->tree[v].get();
					
					if (Joint* joint = dynamic_cast<Joint*>(transform))
					{
						joint->leaves.clear();
						this->joints.push_back(joint);
					}
					
					this->update(v);
				}
			}
			else
			{
				this->leaves.push_back(u);
				
				for (InEdgeIteratorPair i = ::boost::in_edges(u, this->tree); i.first != i.second; ++i.first)
				{
					this->tools.push_back(*i.first);
				}
			}
		}
		
		void
		Kinematics::updateFrames()
		{
			for (::std::vector<Transform*>::iterator i = this->transforms.begin(); i != this->transforms.end(); ++i)
			{
				(*i)->updateFrames();
			}
		}
		
		void
		Kinematics::updateJacobian()
		{
			this->jacobian.setZero();
			
			for (::std::size_t i = 0; i < this->leaves.size(); ++i)
			{
				for (::std::size_t j = 0; j < this->joints.size(); ++j)
				{
					if (this->joints[j]->leaves.count(this->leaves[i]) > 0)
					{
						::rl::math::MatrixBlock jacobian = this->jacobian.block(6 * i, j, 6, 1); // TODO
						this->joints[j]->jacobian(this->tree[this->leaves[i]]->frame, jacobian);
					}
				}
			}
		}
		
		void
		Kinematics::updateJacobianInverse(const ::rl::math::Real& lambda, const bool& doSvd)
		{
			if (doSvd)
			{
				this->jacobianInverse.setZero();
				
				::Eigen::JacobiSVD< ::rl::math::Matrix> svd(this->jacobian, ::Eigen::ComputeFullU | ::Eigen::ComputeFullV);
				
				::rl::math::Real wMin = svd.singularValues().minCoeff();
				::rl::math::Real lambdaSqr = wMin < static_cast< ::rl::math::Real>(1.0e-9) ? (1 - ::std::pow((wMin / static_cast< ::rl::math::Real>(1.0e-9)), 2)) * ::std::pow(lambda, 2) : 0;
				
				for (::std::ptrdiff_t i = 0; i < svd.nonzeroSingularValues(); ++i)
				{
					this->jacobianInverse.noalias() += (
						svd.singularValues()(i) / (::std::pow(svd.singularValues()(i), 2) + lambdaSqr) *
						svd.matrixV().col(i) * svd.matrixU().col(i).transpose()
					);
				}
			}
			else
			{
				this->jacobianInverse = this->jacobian.transpose() * (
					this->jacobian * this->jacobian.transpose() + ::std::pow(lambda, 2) *
					::rl::math::Matrix::Identity(this->getOperationalDof() * 6, this->getOperationalDof() * 6)
				).inverse();
			}
		}
		
		::rl::math::Transform&
		Kinematics::world()
		{
			return this->tree[this->root]->frame;
		}
		
		const ::rl::math::Transform&
		Kinematics::world() const
		{
			return this->tree[this->root]->frame;
		}
	}
}
