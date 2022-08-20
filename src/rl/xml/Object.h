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

#ifndef RL_XML_OBJECT_H
#define RL_XML_OBJECT_H

#include <memory>
#include <string>
#include <boost/numeric/conversion/cast.hpp>
#include <libxml/xpath.h>
#include <libxml/xpointer.h>

#include "Node.h"
#include "NodeSet.h"

namespace rl
{
	namespace xml
	{
		class Object
		{
		public:
			explicit Object(::xmlXPathObjectPtr object) :
				object(object, ::xmlXPathFreeNodeSetList),
				nodeSet(nullptr != object ? object->nodesetval : nullptr, ::xmlXPathFreeNodeSet)
			{
			}
			
			~Object()
			{
				if (nullptr != this->object)
				{
					if (::XPATH_STRING == this->object->type && nullptr != this->object->stringval)
					{
						::xmlFree(this->object->stringval);
					}
#if LIBXML_VERSION < 21000 || defined(LIBXML_XPTR_LOCS_ENABLED)
					else if (::XPATH_LOCATIONSET == this->object->type && nullptr != this->object->user)
					{
						::xmlXPtrFreeLocationSet(static_cast<::xmlLocationSetPtr>(this->object->user));
					}
#endif
				}
			}
			
			xmlXPathObjectPtr get() const
			{
				return this->object.get();
			}
			
			xmlXPathObjectType getType() const
			{
				return this->object->type;
			}
			
			template<typename T>
			T getValue() const;
			
			template<typename T>
			T getValue(const T& val) const;
			
			xmlXPathObject& operator*() const
			{
				return *this->object;
			}
			
			const ::std::type_info& type() const
			{
				switch (this->object->type)
				{
				case ::XPATH_UNDEFINED:
					throw ::std::bad_typeid();
					break;
				case ::XPATH_NODESET:
					return typeid(NodeSet);
					break;
				case ::XPATH_BOOLEAN:
					return typeid(this->object->boolval);
					break;
				case ::XPATH_NUMBER:
					return typeid(this->object->floatval);
					break;
				case ::XPATH_STRING:
					return typeid(this->object->stringval);
					break;
#if LIBXML_VERSION < 21000 || defined(LIBXML_XPTR_LOCS_ENABLED)
				case ::XPATH_POINT:
				case ::XPATH_RANGE:
				case ::XPATH_LOCATIONSET:
#endif
				case ::XPATH_USERS:
				case ::XPATH_XSLT_TREE:
				default:
					throw ::std::bad_typeid();
					break;
				}
			}
			
		protected:
			
		private:
			::std::shared_ptr<::xmlXPathObject> object;
			
			::std::shared_ptr<::xmlNodeSet> nodeSet;
		};
		
		template<>
		inline
		double Object::getValue<double>(const double& val) const
		{
			return 0 == ::xmlXPathIsNaN(this->object->floatval) ? this->object->floatval : val;
		}
		
		template<>
		inline
		float Object::getValue<float>(const float& val) const
		{
			return 0 == ::xmlXPathIsNaN(this->object->floatval) ? ::boost::numeric_cast<float>(this->object->floatval) : val;
		}
		
		template<>
		inline
		short int Object::getValue<short int>(const short int& val) const
		{
			return 0 == ::xmlXPathIsNaN(this->object->floatval) ? ::boost::numeric_cast<short int>(this->object->floatval) : val;
		}
		
		template<>
		inline
		int Object::getValue<int>(const int& val) const
		{
			return 0 == ::xmlXPathIsNaN(this->object->floatval) ? ::boost::numeric_cast<int>(this->object->floatval) : val;
		}
		
		template<>
		inline
		long int Object::getValue<long int>(const long int& val) const
		{
			return 0 == ::xmlXPathIsNaN(this->object->floatval) ? ::boost::numeric_cast<long int>(this->object->floatval) : val;
		}
		
		template<>
		inline
		long long int Object::getValue<long long int>(const long long int& val) const
		{
			return 0 == ::xmlXPathIsNaN(this->object->floatval) ? ::boost::numeric_cast<long long int>(this->object->floatval) : val;
		}
		
		template<>
		inline
		unsigned short int Object::getValue<unsigned short int>(const unsigned short int& val) const
		{
			return 0 == ::xmlXPathIsNaN(this->object->floatval) ? ::boost::numeric_cast<unsigned short int>(this->object->floatval) : val;
		}
		
		template<>
		inline
		unsigned int Object::getValue<unsigned int>(const unsigned int& val) const
		{
			return 0 == ::xmlXPathIsNaN(this->object->floatval) ? ::boost::numeric_cast<unsigned int>(this->object->floatval) : val;
		}
		
		template<>
		inline
		unsigned long int Object::getValue<unsigned long int>(const unsigned long int& val) const
		{
			return 0 == ::xmlXPathIsNaN(this->object->floatval) ? ::boost::numeric_cast<unsigned long int>(this->object->floatval) : val;
		}
		
		template<>
		inline
		unsigned long long int Object::getValue<unsigned long long int>(const unsigned long long int& val) const
		{
			return 0 == ::xmlXPathIsNaN(this->object->floatval) ? ::boost::numeric_cast<unsigned long long int>(this->object->floatval) : val;
		}
		
		template<>
		inline
		bool Object::getValue<bool>() const
		{
			return 1 == this->object->boolval ? true : false;
		}
		
		template<>
		inline
		double Object::getValue<double>() const
		{
			return this->getValue<double>(::std::numeric_limits<double>::quiet_NaN());
		}
		
		template<>
		inline
		float Object::getValue<float>() const
		{
			return this->getValue<float>(::std::numeric_limits<float>::quiet_NaN());
		}
		
		template<>
		inline
		short int Object::getValue<short int>() const
		{
			return this->getValue<short int>(::std::numeric_limits<short int>::quiet_NaN());
		}
		
		template<>
		inline
		int Object::getValue<int>() const
		{
			return this->getValue<int>(::std::numeric_limits<int>::quiet_NaN());
		}
		
		template<>
		inline
		long int Object::getValue<long int>() const
		{
			return this->getValue<long int>(::std::numeric_limits<long int>::quiet_NaN());
		}
		
		template<>
		inline
		long long int Object::getValue<long long int>() const
		{
			return this->getValue<long long int>(::std::numeric_limits<long long int>::quiet_NaN());
		}
		
		template<>
		inline
		unsigned short int Object::getValue<unsigned short int>() const
		{
			return this->getValue<unsigned short int>(::std::numeric_limits<unsigned short int>::quiet_NaN());
		}
		
		template<>
		inline
		unsigned int Object::getValue<unsigned int>() const
		{
			return this->getValue<unsigned int>(::std::numeric_limits<unsigned int>::quiet_NaN());
		}
		
		template<>
		inline
		unsigned long int Object::getValue<unsigned long int>() const
		{
			return this->getValue<unsigned long int>(::std::numeric_limits<unsigned long int>::quiet_NaN());
		}
		
		template<>
		inline
		unsigned long long int Object::getValue<unsigned long long int>() const
		{
			return this->getValue<unsigned long long int>(::std::numeric_limits<unsigned long long int>::quiet_NaN());
		}
		
		template<>
		inline
		NodeSet Object::getValue<NodeSet>() const
		{
			return NodeSet(this->nodeSet);
		}
		
		template<>
		inline
		::std::string Object::getValue<::std::string>() const
		{
			return nullptr != this->object->stringval ? reinterpret_cast<char*>(this->object->stringval) : ::std::string();
		}
	}
}

#endif // RL_XML_PATH_H
