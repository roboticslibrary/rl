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

#ifndef RL_MATH_KDTREEBOUNDINGBOXNEARESTNEIGHBORS_H
#define RL_MATH_KDTREEBOUNDINGBOXNEARESTNEIGHBORS_H

#include <algorithm>
#include <array>
#include <deque>
#include <fstream>
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
		class KdtreeBoundingBoxNearestNeighbors
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
			
			explicit KdtreeBoundingBoxNearestNeighbors(const Metric& metric) :
				boundingBox(),
				checks(),
				metric(metric),
				nodeDataMax(10),
				root(),
				values(0)
			{
			}
			
			explicit KdtreeBoundingBoxNearestNeighbors(Metric&& metric = Metric()) :
				boundingBox(),
				checks(),
				metric(::std::move(metric)),
				nodeDataMax(10),
				root(),
				values(0)
			{
			}
			
			template<typename InputIterator>
			KdtreeBoundingBoxNearestNeighbors(InputIterator first, InputIterator last, const Metric& metric) :
				boundingBox(),
				checks(),
				metric(metric),
				nodeDataMax(10),
				root(),
				values(0)
			{
				this->insert(first, last);
			}
			
			template<typename InputIterator>
			KdtreeBoundingBoxNearestNeighbors(InputIterator first, InputIterator last, Metric&& metric = Metric()) :
				boundingBox(),
				checks(),
				metric(::std::move(metric)),
				nodeDataMax(10),
				root(),
				values(0)
			{
				this->insert(first, last);
			}
			
			~KdtreeBoundingBoxNearestNeighbors()
			{
			}
			
			void clear()
			{
				this->boundingBox.clear();
				this->root.children[0].reset(nullptr);
				this->root.children[1].reset(nullptr);
				this->root.data.clear();
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
				return this->root.data.empty() && nullptr == this->root.children[0] && nullptr == this->root.children[1];
			}
			
			::boost::optional< ::std::size_t> getChecks() const
			{
				return this->checks;
			}
			
			::std::size_t getNodeDataMax() const
			{
				return this->nodeDataMax;
			}
			
			template<typename InputIterator>
			void insert(InputIterator first, InputIterator last)
			{
				using ::std::size;
				
				if (this->empty())
				{
					typename ::std::iterator_traits<InputIterator>::difference_type distance = ::std::distance(first, last);
					
					if (distance > 1)
					{
						this->boundingBox.resize(size(*first));
						this->computeBoundingBox(first, last, this->boundingBox);
						this->divide(this->root, this->boundingBox, first, last);
					}
					else
					{
						this->root.data.insert(this->root.data.end(), first, last);
					}
					
					this->values += distance;
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
				using ::std::begin;
				using ::std::size;
				
				if (this->boundingBox.empty())
				{
					this->boundingBox.resize(size(value));
					
					for (::std::size_t i = 0; i < this->boundingBox.size(); ++i)
					{
						Distance tmp = *(begin(value) + i);
						this->boundingBox[i].low = tmp;
						this->boundingBox[i].high = tmp;
					}
				}
				else
				{
					for (::std::size_t i = 0; i < this->boundingBox.size(); ++i)
					{
						Distance tmp = *(begin(value) + i);
						this->boundingBox[i].low = ::std::min(tmp, this->boundingBox[i].low);
						this->boundingBox[i].high = ::std::max(tmp, this->boundingBox[i].high);
					}
				}
				
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
			
			void setNodeDataMax(const ::std::size_t& nodeDataMax)
			{
				this->nodeDataMax = nodeDataMax;
			}
			
			::std::size_t size() const
			{
				return this->values;
			}
			
			void swap(KdtreeBoundingBoxNearestNeighbors& other)
			{
				using ::std::swap;
				swap(this->mean, other.mean);
				swap(this->metric, other.metric);
				swap(this->samples, other.samples);
				swap(this->root, other.root);
				swap(this->values, other.values);
				swap(this->var, other.var);
			}
			
			friend void swap(KdtreeBoundingBoxNearestNeighbors& lhs, KdtreeBoundingBoxNearestNeighbors& rhs)
			{
				lhs.swap(rhs);
			}
			
		protected:
			
		private:
			struct Interval;
			
			typedef ::std::vector<Interval> BoundingBox;
			
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
			
			struct IndexCompare
			{
				IndexCompare(const Size& index) :
					index(index)
				{
				}
				
				bool operator()(const Value& lhs, const Value& rhs) const
				{
					using ::std::begin;
					return *(begin(lhs) + this->index) < *(begin(rhs) + this->index);
				}
				
				Size index;
			};
			
			struct Interval
			{
				Distance high;
				
				Distance low;
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
					data(),
					index(),
					interval()
				{
				}
				
				~Node()
				{
				}
				
				void swap(Node& other)
				{
					using ::std::swap;
					swap(this->children, other.children);
					swap(this->data, other.data);
					swap(this->index, other.index);
					swap(this->interval, other.interval);
				}
				
				friend void swap(Node& lhs, Node& rhs)
				{
					lhs.swap(rhs);
				}
				
				::std::array< ::std::unique_ptr<Node>, 2> children;
				
				::std::vector<Value> data;
				
				Size index;
				
				Interval interval;
			};
			
			template<typename InputIterator>
			void computeBoundingBox(InputIterator first, InputIterator last, BoundingBox& boundingBox)
			{
				using ::std::begin;
				
				for (::std::size_t i = 0; i < boundingBox.size(); ++i)
				{
					Distance value = *(begin(*first) + i);
					boundingBox[i].low = value;
					boundingBox[i].high = value;
				}
				
				for (InputIterator i = first + 1; i != last; ++i)
				{
					auto start = begin(*i);
					
					for (::std::size_t j = 0; j < boundingBox.size(); ++j)
					{
						Distance value = *(start + j);
						boundingBox[j].low = ::std::min(value, boundingBox[j].low);
						boundingBox[j].high = ::std::max(value, boundingBox[j].high);
					}
				}
			}
			
			void data(const Node& node, ::std::vector<Value>& data) const
			{
				data.insert(data.end(), node.data.begin(), node.data.end());
				
				for (::std::size_t i = 0; i < node.children.size(); ++i)
				{
					this->data(*node.children[i], data);
				}
			}
			
			template<typename InputIterator>
			void divide(Node& node, BoundingBox& boundingBox, InputIterator first, InputIterator last)
			{
				Cut cut = this->select(first, last, boundingBox);
				node.index = cut.index;
				InputIterator split = ::std::partition(first, last, cut);
				
				::std::array<BoundingBox, 2> boundingBoxes = { boundingBox, boundingBox };
				
				for (::std::size_t i = 0; i < 2; ++i)
				{
					InputIterator begin = 0 == i ? first : split;
					InputIterator end = 0 == i ? split : last;
					
					switch (i)
					{
					case 0:
						boundingBoxes[i][cut.index].high = cut.value;
						break;
					case 1:
						boundingBoxes[i][cut.index].low = cut.value;
						break;
					default:
						break;
					}
					
#if __cplusplus > 201103L || _MSC_VER >= 1800
					node.children[i] = ::std::make_unique<Node>();
#else
					node.children[i].reset(new Node());
#endif
					
					if (::std::distance(begin, end) > this->nodeDataMax)
					{
						this->divide(*node.children[i], boundingBoxes[i], begin, end);
					}
					else
					{
						node.children[i]->data.insert(node.children[i]->data.end(), begin, end);
						this->computeBoundingBox(begin, end, boundingBoxes[i]);
					}
				}
				
				node.interval.low = boundingBoxes[0][cut.index].high;
				node.interval.high = boundingBoxes[1][cut.index].low;
				
				for (::std::size_t i = 0; i < boundingBox.size(); ++i)
				{
					boundingBox[i].low = ::std::min(boundingBoxes[0][i].low, boundingBoxes[1][i].low);
					boundingBox[i].high = ::std::max(boundingBoxes[0][i].high, boundingBoxes[1][i].high);
				}
			}
			
			void push(Node& node, const Value& value)
			{
				using ::std::begin;
				using ::std::size;
				
				if (nullptr == node.children[0] && nullptr == node.children[1])
				{
					node.data.push_back(value);
					
					if (node.data.size() > this->nodeDataMax)
					{
						BoundingBox boundingBox(size(value));
						this->computeBoundingBox(node.data.begin(), node.data.end(), boundingBox);
						this->divide(node, boundingBox, node.data.begin(), node.data.end());
						node.data.clear();
						node.data.shrink_to_fit();
					}
				}
				else
				{
					Distance tmp = *(begin(value) + node.index);
					Distance diff0 = tmp - node.interval.low;
					Distance diff1 = tmp - node.interval.high;
					Distance diff = diff0 + diff1;
					
					if (diff < 0)
					{
						this->push(*node.children[0], value);
						node.interval.low = ::std::max(node.interval.low, tmp);
					}
					else
					{
						this->push(*node.children[1], value);
						node.interval.high = ::std::min(node.interval.high, tmp);
					}
				}
			}
			
			::std::vector<Neighbor> search(const Value& query, const ::std::size_t* k, const Distance* radius, const bool& sorted) const
			{
				using ::std::begin;
				using ::std::size;
				
				::std::vector<Neighbor> neighbors;
				
				if (this->empty())
				{
					return neighbors;
				}
				
				neighbors.reserve(nullptr != k ? *k : this->values);
				
				::std::size_t checks = 0;
				
				Distance mindist = Distance();
				::std::vector<Distance> sidedist(size(query), Distance());
				
				for (::std::size_t i = 0; i < sidedist.size(); ++i)
				{
					Distance value = *(begin(query) + i);
					
					if (value < this->boundingBox[i].low)
					{
						sidedist[i] = this->metric(value, this->boundingBox[i].low, i);
						mindist += sidedist[i];
					}
					
					if (value > this->boundingBox[i].high)
					{
						sidedist[i] = this->metric(value, this->boundingBox[i].high, i);
						mindist += sidedist[i];
					}
				}
				
				::std::vector<Branch> branches;
				
				this->search(this->root, query, k, radius, branches, neighbors, checks, mindist, sidedist);
				
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
					for (::std::size_t i = 0; i < node.data.size(); ++i)
					{
						Distance distance = this->metric(query, node.data[i]);
						
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
								neighbors.push_back(::std::make_pair(distance, node.data[i]));
#else
								neighbors.emplace_back(::std::piecewise_construct, ::std::forward_as_tuple(distance), ::std::forward_as_tuple(node.data[i]));
#endif
								::std::push_heap(neighbors.begin(), neighbors.end(), NeighborCompare());
							}
						}
						
						if (this->checks && ++checks > this->checks)
						{
							return;
						}
					}
				}
				else
				{
					Distance value = *(begin(query) + node.index);
					Distance diff0 = value - node.interval.low;
					Distance diff1 = value - node.interval.high;
					Distance diff = diff0 + diff1;
					
					::std::size_t best = diff < 0 ? 0 : 1;
					::std::size_t worst = diff < 0 ? 1 : 0;
					
					this->search(*node.children[best], query, k, radius, branches, neighbors, checks, mindist, sidedist);
					
					Distance cutdist = this->metric(value, diff < 0 ? node.interval.high : node.interval.low, node.index);
					Distance newdist = mindist - sidedist[node.index] + cutdist;
					
					if (nullptr == k || neighbors.size() < *k || newdist <= neighbors.front().first)
					{
						::std::vector<Distance> newsidedist(sidedist);
						newsidedist[node.index] = cutdist;
						
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
			Cut select(InputIterator first, InputIterator last, BoundingBox& boundingBox)
			{
				using ::std::begin;
				
				Cut cut;
				cut.index = 0;
				cut.value = (boundingBox[0].high + boundingBox[0].low) / 2;
				
				Distance maxSpan = boundingBox[0].high - boundingBox[0].low;
				
				for (::std::size_t i = 1; i < boundingBox.size(); ++i)
				{
					Distance span = boundingBox[i].high - boundingBox[i].low;
					
					if (span > maxSpan)
					{
						maxSpan = span;
						cut.index = i;
						cut.value = (boundingBox[i].high + boundingBox[i].low) / 2;
					}
				}
				
				::std::pair<InputIterator, InputIterator> minmax = ::std::minmax_element(first, last, IndexCompare(cut.index));
				Distance min = *(begin(*minmax.first) + cut.index);
				Distance max = *(begin(*minmax.second) + cut.index);
				
				cut.value = (max + min) / 2;
				maxSpan = max - min;
				
				Size index = cut.index;
				
				for (::std::size_t i = 0; i < boundingBox.size(); ++i)
				{
					if (index != i)
					{
						Distance span = boundingBox[i].high - boundingBox[i].low;
						
						if (span > maxSpan)
						{
							minmax = ::std::minmax_element(first, last, IndexCompare(i));
							min = *(begin(*minmax.first) + i);
							max = *(begin(*minmax.second) + i);
							span = max - min;
							
							if (span > maxSpan)
							{
								maxSpan = span;
								cut.index = i;
								cut.value = (max + min) / 2;
							}
						}
					}
				}
				
				return cut;
			}
			
			BoundingBox boundingBox;
			
			::boost::optional< ::std::size_t> checks;
			
			Metric metric;
			
			::std::size_t nodeDataMax;
			
			Node root;
			
			::std::size_t values;
		};
	}
}

#endif // RL_MATH_KDTREEBOUNDINGBOXNEARESTNEIGHBORS_H
