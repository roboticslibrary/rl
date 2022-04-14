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

#ifndef RL_XML_DOCUMENT_H
#define RL_XML_DOCUMENT_H

#include <memory>
#include <string>
#include <libxml/parser.h>
#include <libxml/xinclude.h>

#include "Exception.h"
#include "Node.h"

namespace rl
{
	/**
	 * XML abstraction layer.
	 */
	namespace xml
	{
		class Document
		{
		public:
			explicit Document(const ::std::string& version = "1.0") :
				doc(::xmlNewDoc(reinterpret_cast<const ::xmlChar*>(version.c_str())), ::xmlFreeDoc)
			{
			}
			
			explicit Document(::xmlDocPtr doc) :
				doc(doc, ::xmlFreeDoc)
			{
			}
			
			Document(Document&& document) :
				doc(::std::move(document.doc))
			{
			}
			
			~Document()
			{
			}
			
			::std::string dumpFormatMemory(const bool& format) const
			{
				::xmlChar* mem;
				int size;
				
				::xmlDocDumpFormatMemory(this->doc.get(), &mem, &size, format ? 1 : 0);
				::std::unique_ptr<::xmlChar, decltype(::xmlFree)> memory(mem, ::xmlFree);
				
				return nullptr != memory.get() ? reinterpret_cast<const char*>(memory.get()) : ::std::string();
			}
			
			::std::string dumpMemory() const
			{
				::xmlChar* mem;
				int size;
				
				::xmlDocDumpMemory(this->doc.get(), &mem, &size);
				::std::unique_ptr<::xmlChar, decltype(::xmlFree)> memory(mem, ::xmlFree);
				
				return nullptr != memory.get() ? reinterpret_cast<const char*>(memory.get()) : ::std::string();
			}
			
			::xmlDocPtr get() const
			{
				return this->doc.get();
			}
			
			int getCompression() const
			{
				return this->doc->compression;
			}
			
			::std::string getEncoding() const
			{
				return nullptr != this->doc->encoding ? reinterpret_cast<const char*>(this->doc->encoding) : ::std::string();
			}
			
			::std::string getName() const
			{
				return nullptr != this->doc->name ? this->doc->name : ::std::string();
			}
			
			int getProperties() const
			{
				return this->doc->properties;
			}
			
			Node getRootElement() const
			{
				return Node(::xmlDocGetRootElement(this->doc.get()));
			}
			
			::std::string getVersion() const
			{
				return nullptr != this->doc->version ? reinterpret_cast<const char*>(this->doc->version) : ::std::string();
			}
			
			bool isStandalone() const
			{
				return 1 == this->doc->standalone ? true : false;
			}
			
			Document& operator=(Document&& other)
			{
				this->doc = ::std::move(other.doc);
				return *this;
			}
			
			::xmlDoc& operator*() const
			{
				return *this->doc;
			}
			
			::xmlDocPtr release()
			{
				return this->doc.release();
			}
			
			void save(const ::std::string& filename, const bool& format = true) const
			{
				::xmlSaveFormatFile(filename.c_str(), this->doc.get(), format ? 1 : 0);
			}
			
			void save(const ::std::string& filename, const ::std::string& encoding, const bool& format = true) const
			{
				::xmlSaveFormatFileEnc(filename.c_str(), this->doc.get(), encoding.c_str(), format ? 1 : 0);
			}
			
			void setRootElement(const Node& node)
			{
				::xmlDocSetRootElement(this->doc.get(), node.get());
			}
			
			int substitute(const int& flags = 0)
			{
				int substitutions = ::xmlXIncludeProcessFlags(this->doc.get(), flags);
				
				if (-1 == substitutions)
				{
					throw Exception(::xmlGetLastError()->message);
				}
				
				return substitutions;
			}
			
		protected:
			
		private:
			::std::unique_ptr<::xmlDoc, decltype(&::xmlFreeDoc)> doc;
		};
	}
}

#endif // RL_XML_DOCUMENT_H
