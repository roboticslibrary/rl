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

#ifndef _RL_XML_OBJECT_H_
#define _RL_XML_OBJECT_H_

#include <string>
#include <boost/shared_ptr.hpp>
#include <libxml/xpath.h>

#include "Node.h"

namespace rl
{
	namespace xml
	{
		class Object
		{
		public:
			Object() :
				object()
			{
			}
			
			Object(xmlXPathObjectPtr object) :
				object(object, xmlXPathFreeObject)
			{
			}
			
			virtual ~Object()
			{
			}
			
			bool getBoolval() const
			{
				return (1 == this->object->boolval) ? true : false;
			}
			
			double getFloatval() const
			{
				return this->object->floatval;
			}
			
			double getFloatval(const double& val) const
			{
				return 0 == xmlXPathIsNaN(this->object->floatval) ? this->object->floatval : val;
			}
			
			int getNodeNr() const
			{
				return this->object->nodesetval->nodeNr;
			}
			
			Node getNodeTab(const int& i) const
			{
				return Node(this->object->nodesetval->nodeTab[i]);
			}
			
			::std::string getStringval() const
			{
				return reinterpret_cast< char* >(this->object->stringval);
			}
			
			xmlXPathObjectType getType() const
			{
				return this->object->type;
			}
			
			xmlXPathObjectPtr operator()() const
			{
				return this->object.get();
			}
			
		protected:
			
		private:
			::boost::shared_ptr< xmlXPathObject > object;
		};
	}
}

#endif // _RL_XML_PATH_H_
