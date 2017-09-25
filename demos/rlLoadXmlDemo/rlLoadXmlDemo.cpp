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

#include <iostream>
#include <rl/xml/Document.h>
#include <rl/xml/DomParser.h>
#include <rl/xml/Stylesheet.h>

int
main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: rlLoadXmlDemo XMLFILE [PARAM1 VALUE1 ... PARAMN VALUEN]" << std::endl;
		return EXIT_FAILURE;
	}
	
	rl::xml::DomParser parser;
	rl::xml::Document document = parser.readFile(argv[1], "", XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
	document.substitute(XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
	
	if ("stylesheet" == document.getRootElement().getName() || "transform" == document.getRootElement().getName())
	{
		if ("1.0" == document.getRootElement().getProperty("version"))
		{
			if (document.getRootElement().hasNamespace() && "http://www.w3.org/1999/XSL/Transform" == document.getRootElement().getNamespace().getHref())
			{
				::std::map< ::std::string, ::std::string> parameters;
				
				for (int i = 2; i < argc - 1; i += 2)
				{
					parameters[argv[i]] = argv[i + 1];
					std::cout << "param['" << argv[i] << "'] = '" << argv[i + 1] << "'" << std::endl;
				}
				
				rl::xml::Stylesheet stylesheet(document);
				document = stylesheet.apply(parameters);
			}
		}
	}
	
	document.save("-");
	
	return EXIT_SUCCESS;
}
