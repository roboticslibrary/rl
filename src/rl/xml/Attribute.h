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

#ifndef RL_XML_ATTRIBUTE_H
#define RL_XML_ATTRIBUTE_H

#include <string>
#include <boost/shared_array.hpp>
#include <libxml/parser.h>

#include "Namespace.h"

namespace rl
{
	namespace xml
	{
		class Attribute
		{
		public:
			explicit Attribute(::xmlAttrPtr attr) :
				attr(attr)
			{
			}
			
			Attribute(::xmlNodePtr node, const ::std::string& name, const ::std::string& value) :
				attr(
					::xmlNewProp(
						node,
						reinterpret_cast<const ::xmlChar*>(name.c_str()),
						reinterpret_cast<const ::xmlChar*>(value.c_str())
					)
				)
			{
			}
			
			Attribute(::xmlNodePtr node, const Namespace& nameSpace, const ::std::string& name, const ::std::string& value) :
				attr(
					::xmlNewNsProp(
						node,
						nameSpace.get(),
						reinterpret_cast<const ::xmlChar*>(name.c_str()),
						reinterpret_cast<const ::xmlChar*>(value.c_str())
					)
				)
			{
			}
			
			~Attribute()
			{
				if (nullptr != this->attr && nullptr == this->attr->doc)
				{
					::xmlFreeProp(this->attr);
				}
			}
			
			::xmlAttrPtr get() const
			{
				return this->attr;
			}
			
			::std::string getName() const
			{
				return nullptr != this->attr->name ? reinterpret_cast<const char*>(this->attr->name) : ::std::string(); 
			}
			
			Attribute getNext() const
			{
				return Attribute(this->attr->next);
			}
			
			Attribute getPrevious() const
			{
				return Attribute(this->attr->prev);
			}
			
			::std::string getValue() const
			{
				if (nullptr != this->attr->ns)
				{
					::boost::shared_array< ::xmlChar> value(
						::xmlGetNsProp(
							this->attr->parent,
							this->attr->name,
							this->attr->ns->href
						),
						::xmlFree
					);
					
					return nullptr != value.get() ? reinterpret_cast<char*>(value.get()) : ::std::string();
				}
				else
				{
					::boost::shared_array< ::xmlChar> value(
						::xmlGetProp(
							this->attr->parent,
							this->attr->name
						),
						::xmlFree
					);
					
					return nullptr != value.get() ? reinterpret_cast<char*>(value.get()) : ::std::string();
				}
			}
			
			::xmlAttr& operator*() const
			{
				return *this->attr;
			}
			
			void remove()
			{
				xmlRemoveProp(this->attr);
			}
			
			void setValue(const ::std::string& value)
			{
				if (nullptr != this->attr->ns)
				{
					::xmlSetNsProp(
						this->attr->parent,
						this->attr->ns,
						this->attr->name,
						reinterpret_cast<const ::xmlChar*>(value.c_str())
					);
				}
				else
				{
					::xmlSetProp(
						this->attr->parent,
						this->attr->name,
						reinterpret_cast<const ::xmlChar*>(value.c_str())
					);
				}
			}
			
		protected:
			
		private:
			::xmlAttrPtr attr;
		};
	}
}

#endif // RL_XML_ATTRIBUTE_H
