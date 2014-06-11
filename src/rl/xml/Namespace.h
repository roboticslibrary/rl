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

#ifndef _RL_XML_NAMESPACE_H_
#define _RL_XML_NAMESPACE_H_

#include <string>
#include <boost/shared_ptr.hpp>
#include <libxml/parser.h>

namespace rl
{
	namespace xml
	{
		class Namespace
		{
		public:
			Namespace(xmlNsPtr ns) :
				ns(ns)
			{
			}
			
			Namespace(xmlNodePtr node, const ::std::string& href, const ::std::string& prefix) :
				ns(
					xmlNewNs(
						node,
						reinterpret_cast< const xmlChar* >(href.c_str()),
						reinterpret_cast< const xmlChar* >(prefix.c_str())
					)
				)
			{
				this->ns->_private = node;
			}
			
			virtual ~Namespace()
			{
				if (NULL == this->ns->_private)
				{
					xmlFreeNs(this->ns);
				}
			}
			
			::std::string getHref() const
			{
				return reinterpret_cast< const char* >(this->ns->href);
			}
			
			::std::string getPrefix() const
			{
				return reinterpret_cast< const char* >(this->ns->prefix);
			}
			
			xmlNsPtr operator()() const
			{
				return this->ns;
			}
			
		protected:
			
		private:
			xmlNsPtr ns;
		};
	}
}

#endif // _RL_XML_NAMESPACE_H_
