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

#ifndef RL_XML_NODESET_H
#define RL_XML_NODESET_H

#include <memory>
#include <stdexcept>
#include <libxml/xpath.h>

#include "Node.h"

namespace rl
{
	namespace xml
	{
		class NodeSet
		{
		public:
			explicit NodeSet(::xmlNodeSetPtr nodeSet) :
				nodeSet(nodeSet, ::xmlXPathFreeNodeSet)
			{
			}
			
			explicit NodeSet(const ::std::shared_ptr< ::xmlNodeSet>& nodeSet) :
				nodeSet(nodeSet)
			{
			}
			
			~NodeSet()
			{
			}
			
			Node at(const int& i) const
			{
				if (i < 0 || i >= this->size())
				{
					throw ::std::out_of_range("rl::xml::Node::at() out of range");
				}
				
				return Node(this->nodeSet->nodeTab[i]);
			}
			
			bool empty() const
			{
				return 0 == this->size();
			}
			
			::xmlNodeSetPtr get() const
			{
				return this->nodeSet.get();
			}
			
			int max_size() const
			{
				return this->nodeSet->nodeMax;
			}
			
			::xmlNodeSet& operator*() const
			{
				return *this->nodeSet;
			}
			
			Node operator[](const int& i) const
			{
				return Node(this->nodeSet->nodeTab[i]);
			}
			
			int size() const
			{
				return this->nodeSet->nodeNr;
			}
			
		protected:
			
		private:
			::std::shared_ptr< ::xmlNodeSet> nodeSet;
		};
	}
}

#endif // RL_XML_NODESET_H
