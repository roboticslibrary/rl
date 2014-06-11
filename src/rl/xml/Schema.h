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

#ifndef _RL_XML_SCHEMA_H_
#define _RL_XML_SCHEMA_H_

#include <string>
#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <libxml/xmlschemas.h>

#include "Document.h"

namespace rl
{
	namespace xml
	{
		class Schema
		{
		public:
			Schema(const ::std::string& url) :
				parser(xmlSchemaNewParserCtxt(url.c_str()), xmlSchemaFreeParserCtxt),
				schema(xmlSchemaParse(parser.get()), xmlSchemaFree),
				valid(xmlSchemaNewValidCtxt(schema.get()), xmlSchemaFreeValidCtxt)
			{
			}
			
			virtual ~Schema()
			{
			}
			
			bool validate(const Document& doc)
			{
				return 0 == xmlSchemaValidateDoc(this->valid.get(), doc()) ? true : false;
			}
			
		protected:
			
		private:
			::boost::shared_ptr< xmlSchemaParserCtxt > parser;
			
			::boost::shared_ptr< xmlSchema > schema;
			
			::boost::shared_ptr< xmlSchemaValidCtxt > valid;
		};
	}
}

#endif // _RL_XML_SCHEMA_H_
