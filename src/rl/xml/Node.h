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

#ifndef _RL_XML_NODE_H_
#define _RL_XML_NODE_H_

#include <cstring>
#include <string>
#include <boost/shared_array.hpp>
#include <libxml/parser.h>
#include <libxml/uri.h>
#include <libxml/xinclude.h>

#include "Attribute.h"
#include "Exception.h"

namespace rl
{
	namespace xml
	{
		class Node
		{
		public:
			Node(xmlNodePtr node) :
				node(node)
			{
			}
			
			Node(const ::std::string& name) :
				node(
					xmlNewNode(
						NULL,
						reinterpret_cast< const xmlChar* >(name.c_str())
					)
				)
			{
			}
			
			virtual ~Node()
			{
				if (NULL == this->node->doc)
				{
					xmlFree(this->node);
				}
			}
			
			static Node Text(const ::std::string& content)
			{
				return xmlNewText(
					reinterpret_cast< const xmlChar* >(content.c_str())
				);
			}
			
			Node addChild(const Node& node)
			{
				return xmlAddChild(this->node, node());
			}
			
			Node addNextSibling(const Node& node)
			{
				return xmlAddNextSibling(this->node, node());
			}
			
			Node addPrevSibling(const Node& node)
			{
				return xmlAddPrevSibling(this->node, node());
			}
			
			Node addSibling(const Node& node)
			{
				return xmlAddSibling(this->node, node());
			}
			
			Attribute getAttribute(const ::std::string& name) const
			{
				return Attribute(
					xmlHasProp(
						this->node,
						reinterpret_cast< const xmlChar* >(name.c_str())
					)
				);
			}
			
			unsigned long getChildElementCount() const
			{
				return xmlChildElementCount(this->node);
			}
			
			::std::string getContent() const
			{
				::boost::shared_array< xmlChar > content(
					xmlNodeGetContent(this->node),
					xmlFree
				);
				
				return reinterpret_cast< char* >(content.get());
			}
			
			::std::string getLocalPath(const ::std::string& uri) const
			{
				::boost::shared_array< xmlChar > absolute(
					xmlBuildURI(
						reinterpret_cast< const xmlChar* >(uri.c_str()),
						xmlNodeGetBase(this->node->doc, this->node)
					),
					xmlFree
				);
				
				::boost::shared_array< char > unescaped(
					xmlURIUnescapeString(
						reinterpret_cast< char* >(absolute.get()),
						0,
						NULL
					),
					xmlFree
				);
				
				char* path;
				
				if (0 == strncmp(unescaped.get(), "file://localhost/", 17))
				{
#ifdef WIN32
					path = &unescaped.get()[17];
#else // WIN32
					path = &unescaped.get()[16];
#endif // WIN32
				}
				else if (0 == strncmp(unescaped.get(), "file:///", 8))
				{
#ifdef WIN32
					path = &unescaped.get()[8];
#else // WIN32
					path = &unescaped.get()[7];
#endif // WIN32
				}
				else
				{
					path = unescaped.get();
				}
				
				return path;
			}
			
			::std::string getName() const
			{
				return reinterpret_cast< const char* >(this->node->name);
			}
			
			::std::string getRelativeUri(const ::std::string& uri) const
			{
				::boost::shared_array<xmlChar> relative(
					xmlBuildRelativeURI(
						reinterpret_cast< const xmlChar* >(uri.c_str()),
						xmlNodeGetBase(this->node->doc, this->node)
					),
					xmlFree
				);
				
				return reinterpret_cast< char* >(relative.get());
			}
			
			::std::string getUri(const ::std::string& uri) const
			{
				::boost::shared_array< xmlChar > absolute(
					xmlBuildURI(
						reinterpret_cast< const xmlChar* >(uri.c_str()),
						xmlNodeGetBase(this->node->doc, this->node)
					),
					xmlFree
				);
				
				return reinterpret_cast< char* >(absolute.get());
			}
			
			bool hasAttribute(const ::std::string& name) const
			{
				return NULL != xmlHasProp(
					this->node,
					reinterpret_cast< const xmlChar* >(name.c_str())
				) ? true : false;
			}
			
			bool isText() const
			{
				return 1 == xmlNodeIsText(this->node) ? true : false;
			}
			
			xmlNodePtr operator()() const
			{
				return this->node;
			}
			
			Node replace(const Node& node)
			{
				return xmlReplaceNode(this->node, node());
			}
			
			void setContent(const ::std::string& content)
			{
				xmlNodeSetContent(
					this->node,
					reinterpret_cast< const xmlChar* >(content.c_str())
				);
			}
			
			void setName(const ::std::string& name)
			{
				xmlNodeSetName(this->node, reinterpret_cast< const xmlChar* >(name.c_str()));
			}
			
			int substitute(const int& flags = 0)
			{
				int substitutions = xmlXIncludeProcessTreeFlags(this->node, flags);
				
				if (-1 == substitutions)
				{
					throw Exception(xmlGetLastError()->message);
				}
				
				return substitutions;
			}
			
			void unlink()
			{
				xmlUnlinkNode(this->node);
			}
			
		protected:
			
		private:
			xmlNodePtr node;
		};
	}
}

#endif // _RL_XML_NODE_H_
