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

#ifndef RL_MATH_KDTREENEARESTNEIGHBORS_H
#define RL_MATH_KDTREENEARESTNEIGHBORS_H

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <memory>
#include <numeric>
#include <type_traits>
#include <utility>
#include <vector>
#include <boost/optional.hpp>

namespace rl
{
	namespace math
	{
		/**
		 * k-d tree.
		 * 
		 * Jon Louis Bentley. Multidimensional binary search trees used for
		 * associative searching. Communications of the ACM, 18(9):509-517,
		 * September 1975.
		 * 
		 * http://dx.doi.org/10.1145/361002.361007
		 */
		template<typename MetricT>
		class KdtreeNearestNeighbors
		{
		private:
			struct Node;
			
		public:
			typedef const typename MetricT::Value& const_reference;
			
			typedef ::std::ptrdiff_t difference_type;
			
			typedef typename MetricT::Value& reference;
			
			typedef ::std::size_t size_type;
			
			typedef typename MetricT::Value value_type;
			
			typedef typename MetricT::Distance Distance;
			
			typedef MetricT Metric;
			
			typedef typename MetricT::Size Size;
			
			typedef typename MetricT::Value Value;
			
			typedef ::std::pair<Distance, Value> Neighbor;
			
			explicit KdtreeNearestNeighbors(const Metric& metric) :
				checks(),
				mean(),
				metric(metric),
				root(),
				samples(100),
				values(0),
				var()
			{
			}
			
			explicit KdtreeNearestNeighbors(Metric&& metric = Metric()) :
				checks(),
				mean(),
				metric(::std::move(metric)),
				root(),
				samples(100),
				values(0),
				var()
			{
			}
			
			template<typename InputIterator>
			KdtreeNearestNeighbors(InputIterator first, InputIterator last, const Metric& metric) :
				checks(),
				mean(),
				metric(metric),
				root(),
				samples(100),
				values(0),
				var()
			{
				this->insert(first, last);
			}
			
			template<typename InputIterator>
			KdtreeNearestNeighbors(InputIterator first, InputIterator last, Metric&& metric = Metric()) :
				checks(),
				mean(),
				metric(::std::move(metric)),
				root(),
				samples(100),
				values(0),
				var()
			{
				this->insert(first, last);
			}
			
			~KdtreeNearestNeighbors()
			{
			}
			
			void clear()
			{
				this->root.children[0].reset(nullptr);
				this->root.children[1].reset(nullptr);
				this->root.data.reset();
				this->values = 0;
			}
			
			::std::vector<Value> data() const
			{
				::std::vector<Value> data;
				data.reserve(this->values);
				this->data(this->root, data);
				return data;
			}
			
			bool empty() const
			{
				return !this->root.data && nullptr == this->root.children[0] && nullptr == this->root.children[1];
			}
			
			::boost::optional< ::std::size_t> getChecks() const
			{
				return this->checks;
			}
			
			::std::size_t getSamples() const
			{
				return this->samples;
			}
			
			template<typename InputIterator>
			void insert(InputIterator first, InputIterator last)
			{
				if (this->empty())
				{
					::std::size_t size = ::std::distance(first, last);
					
					if (size > 1)
					{
						this->divide(this->root, first, last);
					}
					else
					{
						this->root.data = *first;
					}
					
					this->values += ::std::distance(first, last);
				}
				else
				{
					for (InputIterator i = first; i != last; ++i)
					{
						this->push(*i);
					}
				}
			}
			
			::std::vector<Neighbor> nearest(const Value& query, const ::std::size_t& k, const bool& sorted = true) const
			{
				return this->search(query, &k, nullptr, sorted);
			}
			
			void push(const Value& value)
			{
				this->push(this->root, value);
				++this->values;
			}
			
			::std::vector<Neighbor> radius(const Value& query, const Distance& radius, const bool& sorted = true) const
			{
				return this->search(query, nullptr, &radius, sorted);
			}
			
			void setChecks(const ::boost::optional< ::std::size_t>& checks)
			{
				this->checks = checks;
			}
			
			void setSamples(const ::std::size_t& samples)
			{
				this->samples = samples;
			}
			
			::std::size_t size() const
			{
				return this->values;
			}
			
			void swap(KdtreeNearestNeighbors& other)
			{
				using ::std::swap;
				swap(this->mean, other.mean);
				swap(this->metric, other.metric);
				swap(this->samples, other.samples);
				swap(this->root, other.root);
				swap(this->values, other.values);
				swap(this->var, other.var);
			}
			
			friend void swap(KdtreeNearestNeighbors& lhs, KdtreeNearestNeighbors& rhs)
			{
				lhs.swap(rhs);
			}
			
		protected:
			
		private:
			struct Branch
			{
				Branch(Distance& dist, ::std::vector<Distance>& sidedist, const Node* node) :
					dist(dist),
					node(node),
					sidedist(sidedist)
				{
				}
				
				Distance dist;
				
				const Node* node;
				
				::std::vector<Distance> sidedist;
			};
			
			struct BranchCompare
			{
				bool operator()(const Branch& lhs, const Branch& rhs) const
				{
					return lhs.dist > rhs.dist;
				}
			};
			
			struct Cut
			{
				bool operator()(const Value& other) const
				{
					using ::std::begin;
					return *(begin(other) + this->index) < this->value;
				}
				
				Size index;
				
				Distance value;
			};
			
			struct NeighborCompare
			{
				bool operator()(const Neighbor& lhs, const Neighbor& rhs) const
				{
					return lhs.first < rhs.first;
				}
			};
			
			struct Node
			{
				Node() :
					children(),
					cut(),
					data()
				{
				}
				
				~Node()
				{
				}
				
				void swap(Node& other)
				{
					using ::std::swap;
					swap(this->children, other.children);
					swap(this->cut, other.cut);
					swap(this->data, other.data);
				}
				
				friend void swap(Node& lhs, Node& rhs)
				{
					lhs.swap(rhs);
				}
				
				::std::array< ::std::unique_ptr<Node>, 2> children;
				
				Cut cut;
				
				::boost::optional<Value> data;
			};
			
			void data(const Node& node, ::std::vector<Value>& data) const
			{
				data.push_back(*node.data);
				
				for (::std::size_t i = 0; i < node.children.size(); ++i)
				{
					this->data(*node.children[i], data);
				}
			}
			
			template<typename InputIterator>
			void divide(Node& node, InputIterator first, InputIterator last)
			{
				node.cut = this->select(first, last);
				InputIterator split = ::std::partition(first, last, node.cut);
				
				for (::std::size_t i = 0; i < 2; ++i)
				{
#if __cplusplus > 201103L || _MSC_VER >= 1800
					node.children[i] = ::std::make_unique<Node>();
#else
					node.children[i].reset(new Node());
#endif
					
					InputIterator begin = 0 == i ? first : split;
					InputIterator end = 0 == i ? split : last;
					
					if (::std::distance(begin, end) > 1)
					{
						this->divide(*node.children[i], begin, end);
					}
					else
					{
						node.children[i]->data = *begin;
					}
				}
			}
			
			void push(Node& node, const Value& value)
			{
				using ::std::begin;
				using ::std::size;
				
				if (nullptr == node.children[0] && nullptr == node.children[1] && !node.data)
				{
					node.data = value;
				}
				else if (nullptr == node.children[0] && nullptr == node.children[1])
				{
					::std::size_t dim = size(value);
					Distance max = Distance();
					
					for (::std::size_t i = 0; i < dim; ++i)
					{
						Distance span = ::std::abs(*(begin(value) + i) - *(begin(*node.data) + i));
						
						if (span > max)
						{
							max = span;
							node.cut.index = i;
						}
					}
					
					for (::std::size_t i = 0; i < 2; ++i)
					{
#if __cplusplus > 201103L || _MSC_VER >= 1800
						node.children[i] = ::std::make_unique<Node>();
#else
						node.children[i].reset(new Node());
#endif
					}
					
					bool less = *(begin(value) + node.cut.index) < *(begin(*node.data) + node.cut.index);
					node.cut.value = (*(begin(value) + node.cut.index) + *(begin(*node.data) + node.cut.index)) / 2;
					
					node.children[0]->cut.index = 0;
					node.children[0]->data = less ? value : ::std::move(node.data);
					node.children[1]->cut.index = 0;
					node.children[1]->data = less ? ::std::move(node.data) : value;
					
					node.data.reset();
				}
				else
				{
					if (*(begin(value) + node.cut.index) < node.cut.value)
					{
						this->push(*node.children[0], value);
					}
					else
					{
						this->push(*node.children[1], value);
					}
				}
			}
			
			::std::vector<Neighbor> search(const Value& query, const ::std::size_t* k, const Distance* radius, const bool& sorted) const
			{
				using ::std::size;
				
				::std::vector<Neighbor> neighbors;
				neighbors.reserve(nullptr != k ? *k : this->values);
				
				::std::size_t checks = 0;
				
				::std::vector<Branch> branches;
				::std::vector<Distance> sidedist(size(query), Distance());
				this->search(this->root, query, k, radius, branches, neighbors, checks, Distance(), sidedist);
				
				while (!branches.empty() && (!this->checks || checks < this->checks))
				{
					Branch branch = branches.front();
					::std::pop_heap(branches.begin(), branches.end(), BranchCompare());
					branches.pop_back();
					this->search(*branch.node, query, k, radius, branches, neighbors, checks, branch.dist, branch.sidedist);
				}
				
				if (sorted)
				{
					::std::sort_heap(neighbors.begin(), neighbors.end(), NeighborCompare());
				}
				
				if (nullptr == k)
				{
					neighbors.shrink_to_fit();
				}
				
				return neighbors;
			}
			
			void search(const Node& node, const Value& query, const ::std::size_t* k, const Distance* radius, ::std::vector<Branch>& branches, ::std::vector<Neighbor>& neighbors, ::std::size_t& checks, const Distance& mindist, const ::std::vector<Distance>& sidedist) const
			{
				using ::std::begin;
				
				if (nullptr == node.children[0] && nullptr == node.children[1])
				{
					if (node.data)
					{
						Distance distance = this->metric(query, *node.data);
						
						if (nullptr == k || neighbors.size() < *k || distance < neighbors.front().first)
						{
							if (nullptr == radius || distance < *radius)
							{
								if (nullptr != k && *k == neighbors.size())
								{
									::std::pop_heap(neighbors.begin(), neighbors.end(), NeighborCompare());
									neighbors.pop_back();
								}
								
#if (defined(_MSC_VER) && _MSC_VER < 1800) || (defined(__GNUC__) && __GNUC__ == 4 && __GNUC_MINOR__ < 8)
								neighbors.push_back(::std::make_pair(distance, *node.data));
#else
								neighbors.emplace_back(::std::piecewise_construct, ::std::forward_as_tuple(distance), ::std::forward_as_tuple(*node.data));
#endif
								::std::push_heap(neighbors.begin(), neighbors.end(), NeighborCompare());
							}
						}
						
						++checks;
					}
				}
				else
				{
					Distance value = *(begin(query) + node.cut.index);
					Distance diff = value - node.cut.value;
					
					::std::size_t best = diff < 0 ? 0 : 1;
					::std::size_t worst = diff < 0 ? 1 : 0;
					
					this->search(*node.children[best], query, k, radius, branches, neighbors, checks, mindist, sidedist);
					
					Distance cutdist = this->metric(value, node.cut.value, node.cut.index);
					Distance newdist = mindist - sidedist[node.cut.index] + cutdist;
					
					if (nullptr == k || neighbors.size() < *k || newdist <= neighbors.front().first)
					{
						::std::vector<Distance> newsidedist(sidedist);
						newsidedist[node.cut.index] = cutdist;
						
						if (!this->checks)
						{
							this->search(*node.children[worst], query, k, radius, branches, neighbors, checks, newdist, newsidedist);
						}
						else
						{
#if defined(_MSC_VER) && _MSC_VER < 1800
							branches.push_back(Branch(newdist, newsidedist, node.children[worst].get()));
#else
							branches.emplace_back(newdist, newsidedist, node.children[worst].get());
#endif
							::std::push_heap(branches.begin(), branches.end(), BranchCompare());
						}
					}
				}
			}
			
			template<typename InputIterator>
			Cut select(InputIterator first, InputIterator last)
			{
				using ::std::begin;
				using ::std::size;
				
				::std::size_t distance = ::std::distance(first, last);
				assert(distance > 0 || "mean expects at least one element");
				::std::size_t samples = ::std::min(this->samples, distance);
				::std::size_t dim = size(*first);
				
				this->mean.resize(dim);
				::std::fill(this->mean.begin(), this->mean.end(), Distance());
				
				this->var.resize(dim);
				::std::fill(this->var.begin(), this->var.end(), Distance());
				
				for (InputIterator i = first; i < first + samples; ++i)
				{
					for (::std::size_t j = 0; j < dim; ++j)
					{
						this->mean[j] += *(begin(*i) + j);
					}
				}
				
				for (::std::size_t i = 0; i < dim; ++i)
				{
					this->mean[i] /= samples;
				}
				
				for (InputIterator i = first; i < first + samples; ++i)
				{
					for (::std::size_t j = 0; j < dim; ++j)
					{
						Distance d = *(begin(*i) + j) - this->mean[j];
						this->var[j] += d * d;
					}
				}
				
				typename ::std::vector<Distance>::iterator max = ::std::max_element(this->var.begin(), this->var.end());
				
				Cut cut;
				cut.index = ::std::distance(this->var.begin(), max);
				cut.value = this->mean[cut.index];
				return cut;
			}
			
			::boost::optional< ::std::size_t> checks;
			
			::std::vector<Distance> mean;
			
			Metric metric;
			
			Node root;
			
			::std::size_t samples;
			
			::std::size_t values;
			
			::std::vector<Distance> var;
		};
	}
}

#endif // RL_MATH_KDTREENEARESTNEIGHBORS_H
