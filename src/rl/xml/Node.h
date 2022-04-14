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

#ifndef RL_XML_NODE_H
#define RL_XML_NODE_H

#include <cstring>
#include <memory>
#include <string>
#include <libxml/parser.h>
#include <libxml/uri.h>
#include <libxml/xinclude.h>

#include "Attribute.h"
#include "Exception.h"
#include "Namespace.h"

namespace rl
{
	namespace xml
	{
		class Node
		{
		public:
			explicit Node() :
				node(nullptr)
			{
			}
			
			explicit Node(::xmlNodePtr node) :
				node(node)
			{
			}
			
			explicit Node(const ::std::string& name) :
				node(
					::xmlNewNode(
						nullptr,
						reinterpret_cast<const ::xmlChar*>(name.c_str())
					)
				)
			{
			}
			
			~Node()
			{
				if (nullptr != this->node && nullptr == this->node->doc && nullptr == this->node->parent)
				{
					::xmlFreeNode(this->node);
				}
			}
			
			static Node Text(const ::std::string& content)
			{
				return Node(::xmlNewText(reinterpret_cast<const ::xmlChar*>(content.c_str())));
			}
			
			Node addChild(const Node& node)
			{
				return Node(::xmlAddChild(this->node, node.get()));
			}
			
			Node addNextSibling(const Node& node)
			{
				return Node(::xmlAddNextSibling(this->node, node.get()));
			}
			
			Node addPrevSibling(const Node& node)
			{
				return Node(::xmlAddPrevSibling(this->node, node.get()));
			}
			
			Node addSibling(const Node& node)
			{
				return Node(::xmlAddSibling(this->node, node.get()));
			}
			
			::xmlNodePtr get() const
			{
				return this->node;
			}
			
			unsigned long getChildElementCount() const
			{
				return ::xmlChildElementCount(this->node);
			}
			
			::std::string getContent() const
			{
				::std::unique_ptr<::xmlChar, decltype(::xmlFree)> content(
					::xmlNodeGetContent(this->node),
					::xmlFree
				);
				
				return nullptr != content.get() ? reinterpret_cast<char*>(content.get()) : ::std::string();
			}
			
			Node getFirstChild() const
			{
				return Node(this->node->children);
			}
			
			Attribute getFirstProperty() const
			{
				return Attribute(this->node->properties);
			}
			
			unsigned short int getLine() const
			{
				return this->node->line;
			}
			
			::std::string getLocalPath(const ::std::string& uri) const
			{
				::std::unique_ptr<::xmlChar, decltype(::xmlFree)> absolute(
					::xmlBuildURI(
						reinterpret_cast<const ::xmlChar*>(uri.c_str()),
						::xmlNodeGetBase(this->node->doc, this->node)
					),
					::xmlFree
				);
				
				::std::unique_ptr<char, decltype(::xmlFree)> unescaped(
					::xmlURIUnescapeString(
						reinterpret_cast<char*>(absolute.get()),
						0,
						nullptr
					),
					::xmlFree
				);
				
				char* path;
				
				if (0 == ::strncmp(unescaped.get(), "file://localhost/", 17))
				{
#ifdef WIN32
					path = &unescaped.get()[17];
#else // WIN32
					path = &unescaped.get()[16];
#endif // WIN32
				}
				else if (0 == ::strncmp(unescaped.get(), "file:///", 8))
				{
#ifdef WIN32
					path = &unescaped.get()[8];
#else // WIN32
					path = &unescaped.get()[7];
#endif // WIN32
				}
				else if (0 == ::strncmp(unescaped.get(), "file:/", 6))
				{
#ifdef WIN32
					path = &unescaped.get()[6];
#else // WIN32
					path = &unescaped.get()[5];
#endif // WIN32
				}
				else
				{
					path = unescaped.get();
				}
				
				return nullptr != path ? path : ::std::string();
			}
			
			::std::string getName() const
			{
				return nullptr != this->node->name ? reinterpret_cast<const char*>(this->node->name) : ::std::string();
			}
			
			Namespace getNamespace() const
			{
				return Namespace(this->node->ns);
			}
			
			Node getNext() const
			{
				return Node(this->node->next);
			}
			
			Node getParent() const
			{
				return Node(this->node->parent);
			}
			
			Node getPrevious() const
			{
				return Node(this->node->prev);
			}
			
			::std::string getProperty(const ::std::string& name) const
			{
				::std::unique_ptr<::xmlChar, decltype(::xmlFree)> prop(
					::xmlGetProp(
						this->node,
						reinterpret_cast<const ::xmlChar*>(name.c_str())
					),
					::xmlFree
				);
				
				return nullptr != prop.get() ? reinterpret_cast<char*>(prop.get()) : ::std::string();
			}
			
			::std::string getProperty(const ::std::string& name, const ::std::string& nameSpace) const
			{
				::std::unique_ptr<::xmlChar, decltype(::xmlFree)> prop(
					::xmlGetNsProp(
						this->node,
						reinterpret_cast<const ::xmlChar*>(name.c_str()),
						reinterpret_cast<const ::xmlChar*>(nameSpace.c_str())
					),
					::xmlFree
				);
				
				return nullptr != prop.get() ? reinterpret_cast<char*>(prop.get()) : ::std::string();
			}
			
			::std::string getRelativeUri(const ::std::string& uri) const
			{
				::std::unique_ptr<::xmlChar, decltype(::xmlFree)> relative(
					::xmlBuildRelativeURI(
						reinterpret_cast<const ::xmlChar*>(uri.c_str()),
						::xmlNodeGetBase(this->node->doc, this->node)
					),
					::xmlFree
				);
				
				return nullptr != relative.get() ? reinterpret_cast<char*>(relative.get()) : ::std::string();
			}
			
			::std::string getUri(const ::std::string& uri) const
			{
				::std::unique_ptr<::xmlChar, decltype(::xmlFree)> absolute(
					::xmlBuildURI(
						reinterpret_cast<const ::xmlChar*>(uri.c_str()),
						::xmlNodeGetBase(this->node->doc, this->node)
					),
					::xmlFree
				);
				
				return nullptr != absolute.get() ? reinterpret_cast<char*>(absolute.get()) : ::std::string();
			}
			
			bool hasChildren() const
			{
				return nullptr != this->node->children;
			}
			
			bool hasNamespace() const
			{
				return nullptr != this->node->ns;
			}
			
			bool hasParent() const
			{
				return nullptr != this->node->parent;
			}
			
			bool hasProperties() const
			{
				return nullptr != this->node->properties;
			}
			
			Attribute hasProperty(const ::std::string& name) const
			{
				return Attribute(
					::xmlHasProp(
						this->node,
						reinterpret_cast<const ::xmlChar*>(name.c_str())
					)
				);
			}
			
			Attribute hasProperty(const ::std::string& name, const ::std::string& nameSpace) const
			{
				return Attribute(
					::xmlHasNsProp(
						this->node,
						reinterpret_cast<const ::xmlChar*>(name.c_str()),
						reinterpret_cast<const ::xmlChar*>(nameSpace.c_str())
					)
				);
			}
			
			bool isBlank() const
			{
				return 1 == ::xmlIsBlankNode(this->node) ? true : false;
			}
			
			bool isText() const
			{
				return 1 == ::xmlNodeIsText(this->node) ? true : false;
			}
			
			::xmlNode& operator*() const
			{
				return *this->node;
			}
			
			bool removeAttribute(const Attribute& attribute)
			{
				return 0 == ::xmlRemoveProp(attribute.get()) ? true : false;
			}
			
			Node replace(const Node& node)
			{
				return Node(::xmlReplaceNode(this->node, node.get()));
			}
			
			void setContent(const ::std::string& content)
			{
				::xmlNodeSetContent(
					this->node,
					reinterpret_cast<const ::xmlChar*>(content.c_str())
				);
			}
			
			void setName(const ::std::string& name)
			{
				::xmlNodeSetName(this->node, reinterpret_cast<const ::xmlChar*>(name.c_str()));
			}
			
			Attribute setProperty(const ::std::string& name, const ::std::string& value)
			{
				return Attribute(
					::xmlSetProp(
						this->node,
						reinterpret_cast<const ::xmlChar*>(name.c_str()),
						reinterpret_cast<const ::xmlChar*>(value.c_str())
					)
				);
			}
			
			Attribute setProperty(const Namespace& nameSpace, const ::std::string& name, const ::std::string& value)
			{
				return Attribute(
					::xmlSetNsProp(
						this->node,
						nameSpace.get(),
						reinterpret_cast<const ::xmlChar*>(name.c_str()),
						reinterpret_cast<const ::xmlChar*>(value.c_str())
					)
				);
			}
			
			int substitute(const int& flags = 0)
			{
				int substitutions = ::xmlXIncludeProcessTreeFlags(this->node, flags);
				
				if (-1 == substitutions)
				{
					throw Exception(::xmlGetLastError()->message);
				}
				
				return substitutions;
			}
			
			void unlink()
			{
				::xmlUnlinkNode(this->node);
			}
			
		protected:
			
		private:
			::xmlNodePtr node;
		};
	}
}

#endif // RL_XML_NODE_H
