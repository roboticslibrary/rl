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

#ifndef _RL_XML_ATTRIBUTE_H_
#define _RL_XML_ATTRIBUTE_H_

#include <string>
#include <boost/shared_array.hpp>
#include <libxml/parser.h>

namespace rl
{
	namespace xml
	{
		class Attribute
		{
		public:
			Attribute(xmlAttrPtr attr) :
				attr(attr)
			{
			}
			
			Attribute(xmlNodePtr node, const ::std::string& name, const ::std::string& value) :
				attr(
					xmlNewProp(
						node,
						reinterpret_cast< const xmlChar* >(name.c_str()),
						reinterpret_cast< const xmlChar* >(value.c_str())
					)
				)
			{
			}
			
			virtual ~Attribute()
			{
				if (NULL == this->attr->doc)
				{
					xmlFreeProp(this->attr);
				}
			}
			
			::std::string getValue() const
			{
				::boost::shared_array< xmlChar > value(
					xmlGetProp(
						this->attr->parent,
						this->attr->name
					),
					xmlFree
				);
				
				return reinterpret_cast< char* >(value.get());
			}
			
			xmlAttrPtr operator()() const
			{
				return this->attr;
			}
			
			void remove()
			{
				xmlRemoveProp(this->attr);
			}
			
			void setValue(const ::std::string& value)
			{
				xmlSetProp(
					this->attr->parent,
					this->attr->name,
					reinterpret_cast< const xmlChar* >(value.c_str())
				);
			}
			
		protected:
			
		private:
			xmlAttrPtr attr;
		};
	}
}

#endif // _RL_XML_ATTRIBUTE_H_
