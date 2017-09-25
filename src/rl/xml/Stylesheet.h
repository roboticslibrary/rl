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

#ifndef RL_XML_STYLESHEET_H
#define RL_XML_STYLESHEET_H

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <boost/shared_array.hpp>
#include <libxslt/transform.h>
#include <libxslt/xsltInternals.h>

#include "Document.h"
#include "Exception.h"

namespace rl
{
	namespace xml
	{
		class Stylesheet
		{
		public:
			explicit Stylesheet() :
				stylesheet(::xsltNewStylesheet(), ::xsltFreeStylesheet)
			{
			}
			
			explicit Stylesheet(::xsltStylesheetPtr stylesheet) :
				stylesheet(stylesheet, ::xsltFreeStylesheet)
			{
			}
			
			explicit Stylesheet(Document& document) :
				stylesheet(::xsltParseStylesheetDoc(document.get()), ::xsltFreeStylesheet)
			{
				document.release();
			}
			
			explicit Stylesheet(const ::std::string& filename) :
				stylesheet(::xsltParseStylesheetFile(reinterpret_cast<const ::xmlChar*>(filename.c_str())), ::xsltFreeStylesheet)
			{
			}
			
			~Stylesheet()
			{
			}
			
			Document apply()
			{
				return Document(::xsltApplyStylesheet(this->stylesheet.get(), this->stylesheet.get()->doc, nullptr));
			}
			
			Document apply(const ::std::map< ::std::string, ::std::string>& parameters)
			{
				::std::vector<const char*> params;
				
				for (::std::map< ::std::string, ::std::string>::const_iterator i = parameters.begin(); i != parameters.end(); ++i)
				{
					params.push_back(i->first.c_str());
					params.push_back(i->second.c_str());
				}
				
				params.push_back(nullptr);
				
				return Document(::xsltApplyStylesheet(this->stylesheet.get(), this->stylesheet.get()->doc, params.data()));
			}
			
			Document apply(const Document& document)
			{
				return Document(::xsltApplyStylesheet(this->stylesheet.get(), document.get(), nullptr));
			}
			
			Document apply(const Document& document, const ::std::map< ::std::string, ::std::string>& parameters)
			{
				::std::vector<const char*> params;
				
				for (::std::map< ::std::string, ::std::string>::const_iterator i = parameters.begin(); i != parameters.end(); ++i)
				{
					params.push_back(i->first.c_str());
					params.push_back(i->second.c_str());
				}
				
				params.push_back(nullptr);
				
				return Document(::xsltApplyStylesheet(this->stylesheet.get(), document.get(), params.data()));
			}
			
			::std::string dumpFormatMemory(const bool& format) const
			{
				::xmlChar* mem;
				int size;
				
				::xmlDocDumpFormatMemory(this->stylesheet.get()->doc, &mem, &size, format ? 1 : 0);
				::boost::shared_array< ::xmlChar> memory(mem, ::xmlFree);
				
				return nullptr != memory.get() ? reinterpret_cast<const char*>(memory.get()) : ::std::string();
			}
			
			::std::string dumpMemory() const
			{
				::xmlChar* mem;
				int size;
				
				::xmlDocDumpMemory(this->stylesheet.get()->doc, &mem, &size);
				::boost::shared_array< ::xmlChar> memory(mem, ::xmlFree);
				
				return nullptr != memory.get() ? reinterpret_cast<const char*>(memory.get()) : ::std::string();
			}
			
			::xsltStylesheetPtr get() const
			{
				return this->stylesheet.get();
			}
			
			::std::string getEncoding() const
			{
				return nullptr != this->stylesheet.get()->doc->encoding ? reinterpret_cast<const char*>(this->stylesheet.get()->doc->encoding) : ::std::string();
			}
			
			Node getRootElement() const
			{
				return Node(::xmlDocGetRootElement(this->stylesheet.get()->doc));
			}
			
			::std::string getVersion() const
			{
				return nullptr != this->stylesheet.get()->doc->version ? reinterpret_cast<const char*>(this->stylesheet.get()->doc->version) : ::std::string();
			}
			
			::xsltStylesheet& operator*() const
			{
				return *this->stylesheet;
			}
			
			void save(const ::std::string& filename, const bool& format = true) const
			{
				::xmlSaveFormatFile(filename.c_str(), this->stylesheet.get()->doc, format ? 1 : 0);
			}
			
			void save(const ::std::string& filename, const ::std::string& encoding, const bool& format = true) const
			{
				::xmlSaveFormatFileEnc(filename.c_str(), this->stylesheet.get()->doc, encoding.c_str(), format ? 1 : 0);
			}
			
			int substitute(const int& flags = 0)
			{
				int substitutions = ::xmlXIncludeProcessFlags(this->stylesheet.get()->doc, flags);
				
				if (-1 == substitutions)
				{
					throw Exception(::xmlGetLastError()->message);
				}
				
				return substitutions;
			}
			
		protected:
			
		private:
			::std::shared_ptr< ::xsltStylesheet> stylesheet;
		};
	}
}

#endif // RL_XML_STYLESHEET_H
