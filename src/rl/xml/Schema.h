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

#ifndef RL_XML_SCHEMA_H
#define RL_XML_SCHEMA_H

#include <memory>
#include <string>
#include <boost/shared_array.hpp>
#include <libxml/xmlschemas.h>

#include "Document.h"

namespace rl
{
	namespace xml
	{
		class Schema
		{
		public:
			explicit Schema(const ::std::string& url) :
				parser(::xmlSchemaNewParserCtxt(url.c_str()), ::xmlSchemaFreeParserCtxt),
				schema(::xmlSchemaParse(parser.get()), ::xmlSchemaFree),
				valid(::xmlSchemaNewValidCtxt(schema.get()), ::xmlSchemaFreeValidCtxt)
			{
			}
			
			~Schema()
			{
			}
			
			::xmlSchemaPtr get() const
			{
				return this->schema.get();
			}
			
			::xmlSchema& operator*() const
			{
				return *this->schema;
			}
			
			bool validate(const Document& doc)
			{
				return 0 == ::xmlSchemaValidateDoc(this->valid.get(), doc.get()) ? true : false;
			}
			
		protected:
			
		private:
			::std::shared_ptr< ::xmlSchemaParserCtxt> parser;
			
			::std::shared_ptr< ::xmlSchema> schema;
			
			::std::shared_ptr< ::xmlSchemaValidCtxt> valid;
		};
	}
}

#endif // RL_XML_SCHEMA_H
