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

#ifndef RL_PLAN_ADDRRTCONCON_H
#define RL_PLAN_ADDRRTCONCON_H

#include "RrtConCon.h"

namespace rl
{
	namespace plan
	{
		/**
		 * Adaptive Dynamic-Domain RRT.
		 * 
		 * L&eacute;onard Jaillet, Anna Yershova, Steven M. LaValle, and Thiery
		 * Sim&eacute;on. Adaptive tuning of the sampling domain for dynamic-domain
		 * RRTs. In Proceedings of the IEEE/RSJ International Conference on Intelligent
		 * Robots and Systems, pages 2851-2856, August 2005.
		 * 
		 * http://dx.doi.org/10.1109/IROS.2005.1545607
		 */
		class AddRrtConCon : public RrtConCon
		{
		public:
			AddRrtConCon();
			
			virtual ~AddRrtConCon();
			
			virtual ::std::string getName() const;
			
			bool solve();
			
			/** Radius expansion factor. */
			::rl::math::Real alpha;
			
			/** Lower bound for radius. */
			::rl::math::Real lower;
			
			/** Initial vertex radius. */
			::rl::math::Real radius;
			
		protected:
			struct VertexBundle : Rrt::VertexBundle
			{
				::rl::math::Real radius;
			};
			
			Vertex addVertex(Tree& tree, const VectorPtr& q);
			
			static VertexBundle* get(const Tree& tree, const Vertex& v);
			
		private:
			
		};
	}
}

#endif //_RL_PLAN_ADDRRTCONCON_H
