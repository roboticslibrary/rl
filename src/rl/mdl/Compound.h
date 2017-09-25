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

#ifndef RL_MDL_COMPOUND_H
#define RL_MDL_COMPOUND_H

#include <boost/graph/filtered_graph.hpp>

#include "Model.h"

namespace rl
{
	namespace mdl
	{
		class Frame;
		class Transform;
		
		class Compound
		{
		public:
			Compound(Model* model);
			
			virtual ~Compound();
			
			void add(Frame* frame);
			
			void add(Transform* transform, const Frame* a, const Frame* b);
			
			void remove(Frame* frame);
			
			void remove(Transform* transform);
			
			Frame* inFrame;
			
			Transform* inTransform;
			
			Frame* outFrame;
			
			Transform* outTransform;
			
		protected:
			
		private:
			typedef ::boost::property_map< Model::Tree, ::boost::edge_weight_t>::type EdgeWeightMapType;
			
			typedef ::boost::property_map< Model::Tree, ::boost::vertex_color_t>::type VertexColorMapType;
			
			template<typename EdgeWeightMap>
			struct EdgePredicate
			{
				EdgePredicate()
				{
				}
				
				EdgePredicate(const EdgeWeightMap& weight) :
					weight(weight)
				{
				}
				
				template<typename Edge>
				bool operator()(const Edge& e) const
				{
					return this == get(weight, e);
				}
				
				EdgeWeightMap weight;
			};
			
			template<typename VertexColorMap>
			struct VertexPredicate
			{
				VertexPredicate()
				{
				}
				
				VertexPredicate(const VertexColorMap& color) :
					color(color)
				{
				}
				
				template<typename Vertex>
				bool operator()(const Vertex& v) const
				{
					return this == get(color, v);
				}
				
				VertexColorMap color;
			};
			
			typedef ::boost::filtered_graph<Model::Tree, EdgePredicate<EdgeWeightMapType>, VertexPredicate<VertexColorMapType>> Tree;
			
			Model* model;
			
			Tree tree;
		};
	}
}

#endif // RL_MDL_COMPOUND_H
