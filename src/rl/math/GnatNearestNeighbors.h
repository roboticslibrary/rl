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

#ifndef RL_MATH_GNATNEARESTNEIGHBORS_H
#define RL_MATH_GNATNEARESTNEIGHBORS_H

#include <algorithm>
#include <iterator>
#include <limits>
#include <random>
#include <type_traits>
#include <utility>
#include <vector>
#include <boost/optional.hpp>

namespace rl
{
	namespace math
	{
		/**
		 * Geometric Near-Neighbor Access Tree (GNAT).
		 * 
		 * Sergey Brin. Near neighbor search in large metric spaces. In Proceedings of
		 * the International Conference on Very Large Data Bases, pages 574-584,
		 * Zurich, Switzerland, September, 1985.
		 * 
		 * http://www.vldb.org/conf/1995/P574.PDF
		 */
		template<typename MetricT>
		class GnatNearestNeighbors
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
			
			typedef typename MetricT::Value Value;
			
			typedef ::std::pair<Distance, Value> Neighbor;
			
			explicit GnatNearestNeighbors(const Metric& metric) :
				checks(),
				generator(::std::random_device()()),
				metric(metric),
				nodeDataMax(50),
				nodeDegree(8),
				nodeDegreeMax(12),
				nodeDegreeMin(4),
				root(0, 0, nodeDegree, nodeDataMax, true),
				values(0)
			{
			}
			
			explicit GnatNearestNeighbors(Metric&& metric = Metric()) :
				checks(),
				generator(::std::random_device()()),
				metric(::std::move(metric)),
				nodeDataMax(50),
				nodeDegree(8),
				nodeDegreeMax(12),
				nodeDegreeMin(4),
				root(0, 0, nodeDegree, nodeDataMax, true),
				values(0)
			{
			}
			
			template<typename InputIterator>
			GnatNearestNeighbors(InputIterator first, InputIterator last, const Metric& metric) :
				checks(),
				generator(::std::random_device()()),
				metric(metric),
				nodeDataMax(50),
				nodeDegree(8),
				nodeDegreeMax(12),
				nodeDegreeMin(4),
				root(first, last, 0, 0, nodeDegree, nodeDataMax, true),
				values(::std::distance(first, last))
			{
				if (this->root.data.size() > this->nodeDataMax && this->root.data.size() > this->root.degree)
				{
					this->split(this->root);
				}
			}
			
			template<typename InputIterator>
			GnatNearestNeighbors(InputIterator first, InputIterator last, Metric&& metric = Metric()) :
				checks(),
				generator(::std::random_device()()),
				metric(::std::move(metric)),
				nodeDataMax(50),
				nodeDegree(8),
				nodeDegreeMax(12),
				nodeDegreeMin(4),
				root(first, last, nullptr, 0, 0, nodeDegree, nodeDataMax, true),
				values(::std::distance(first, last))
			{
				if (this->root.data.size() > this->nodeDataMax && this->root.data.size() > this->root.degree)
				{
					this->split(this->root);
				}
			}
			
			~GnatNearestNeighbors()
			{
			}
			
			void clear()
			{
				this->root.children.clear();
				this->root.children.reserve(this->nodeDegree);
				this->root.data.clear();
				this->root.data.reserve(this->nodeDataMax + 1);
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
				return this->root.removed && this->root.data.empty() && this->root.children.empty();
			}
			
			::boost::optional< ::std::size_t> getChecks() const
			{
				return this->checks;
			}
			
			::std::size_t getNodeDataMax() const
			{
				return this->nodeDataMax;
			}
			
			::std::size_t getNodeDegree() const
			{
				return this->nodeDegree;
			}
			
			::std::size_t getNodeDegreeMax() const
			{
				return this->nodeDegreeMax;
			}
			
			::std::size_t getNodeDegreeMin() const
			{
				return this->nodeDegreeMin;
			}
			
			template<typename InputIterator>
			void insert(InputIterator first, InputIterator last)
			{
				if (this->empty())
				{
					this->root.data.insert(this->root.data.end(), first, last);
					
					if (this->root.data.size() > this->nodeDataMax && this->root.data.size() > this->root.degree)
					{
						this->split(this->root);
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
			
			void seed(const ::std::mt19937::result_type& value)
			{
				this->generator.seed(value);
			}
			
			void setChecks(const ::boost::optional< ::std::size_t>& checks)
			{
				this->checks = checks;
			}
			
			void setNodeDataMax(const ::std::size_t& nodeDataMax)
			{
				this->nodeDataMax = nodeDataMax;
			}
			
			void setNodeDegree(const ::std::size_t& nodeDegree)
			{
				this->nodeDegree = nodeDegree;
			}
			
			void setNodeDegreeMax(const ::std::size_t& nodeDegreeMax)
			{
				this->nodeDegreeMax = nodeDegreeMax;
			}
			
			void setNodeDegreeMin(const ::std::size_t& nodeDegreeMin)
			{
				this->nodeDegreeMin = nodeDegreeMin;
			}
			
			::std::size_t size() const
			{
				return this->values;
			}
			
			void swap(GnatNearestNeighbors& other)
			{
				using ::std::swap;
				swap(this->generator, other.generator);
				swap(this->metric, other.metric);
				swap(this->nodeDegree, other.nodeDegree);
				swap(this->nodeDegreeMax, other.nodeDegreeMax);
				swap(this->nodeDegreeMin, other.nodeDegreeMin);
				swap(this->nodeDataMax, other.nodeDataMax);
				swap(this->root, other.root);
				swap(this->values, other.values);
			}
			
			friend void swap(GnatNearestNeighbors& lhs, GnatNearestNeighbors& rhs)
			{
				lhs.swap(rhs);
			}
			
		protected:
			
		private:
			typedef ::std::pair<Distance, const Node*> Branch;
			
			struct BranchCompare
			{
				bool operator()(const Branch& lhs, const Branch& rhs) const
				{
					return lhs.first - lhs.second->max[lhs.second->index] > rhs.first - rhs.second->max[rhs.second->index];
				}
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
				Node(const ::std::size_t& index, const ::std::size_t& siblings, const ::std::size_t& degree, const ::std::size_t& capacity, const bool& removed = false) :
					children(),
					data(),
					degree(degree),
					index(index),
					max(siblings + 1, -::std::numeric_limits<Distance>::infinity()),
					min(siblings + 1, ::std::numeric_limits<Distance>::infinity()),
					pivot(),
					removed(removed)
				{
					this->children.reserve(degree);
					this->data.reserve(capacity + 1);
				}
				
				template<typename InputIterator>
				Node(InputIterator first, InputIterator last, const ::std::size_t& index, const ::std::size_t& siblings, const ::std::size_t& degree, const ::std::size_t& capacity, const bool& removed = false) :
					children(),
					data(first, last),
					degree(degree),
					index(index),
					max(siblings + 1, -::std::numeric_limits<Distance>::infinity()),
					min(siblings + 1, ::std::numeric_limits<Distance>::infinity()),
					pivot(),
					removed(removed)
				{
					this->children.reserve(degree);
					this->data.reserve(capacity + 1);
				}
				
				~Node()
				{
				}
				
				void swap(Node& other)
				{
					using ::std::swap;
					swap(this->children, other.children);
					swap(this->data, other.data);
					swap(this->degree, other.degree);
					swap(this->index, other.index);
					swap(this->max, other.max);
					swap(this->min, other.min);
					swap(this->pivot, other.pivot);
					swap(this->removed, other.removed);
				}
				
				friend void swap(Node& lhs, Node& rhs)
				{
					lhs.swap(rhs);
				}
				
				::std::vector<Node> children;
				
				::std::vector<Value> data;
				
				::std::size_t degree;
				
				::std::size_t index;
				
				::std::vector<Distance> max;
				
				::std::vector<Distance> min;
				
				Value pivot;
				
				bool removed;
			};
			
			void choose(const Node& node, ::std::vector< ::std::size_t>& centers, ::std::vector< ::std::vector<Distance>>& distances)
			{
				::std::size_t k = node.degree;
				::std::vector<Distance> min(node.data.size(), ::std::numeric_limits<Distance>::infinity());
				
				::std::uniform_int_distribution< ::std::size_t> distribution(0, node.data.size() - 1);
				centers[0] = distribution(this->generator);
				
				for (::std::size_t i = 0; i < k - 1; ++i)
				{
					Distance max = Distance();
					
					for (::std::size_t j = 0; j < node.data.size(); ++j)
					{
						distances[i][j] = j != centers[i] ? this->metric(node.data[j], node.data[centers[i]]) : 0;
						min[j] = ::std::min(min[j], distances[i][j]);
						
						if (min[j] > max)
						{
							max = min[j];
							centers[i + 1] = j;
						}
					}
				}
				
				for (::std::size_t j = 0; j < node.data.size(); ++j)
				{
					distances[k - 1][j] = this->metric(node.data[j], node.data[centers[k - 1]]);
				}
			}
			
			void data(const Node& node, ::std::vector<Value>& data) const
			{
				data.insert(data.end(), node.data.begin(), node.data.end());
				
				for (::std::size_t i = 0; i < node.children.size(); ++i)
				{
					data.push_back(node.children[i].pivot);
					this->data(node.children[i], data);
				}
			}
			
			void push(Node& node, const Value& value)
			{
				if (node.children.empty())
				{
					node.data.push_back(value);
					
					if (node.data.size() > this->nodeDataMax && node.data.size() > node.degree)
					{
						this->split(node);
					}
				}
				else
				{
					::std::vector<Distance> distances(node.children.size());
					::std::size_t index = 0;
					Distance min = ::std::numeric_limits<Distance>::infinity();
					
					for (::std::size_t i = 0; i < node.children.size(); ++i)
					{
						distances[i] = this->metric(value, node.children[i].pivot);
						
						if (distances[i] < min)
						{
							index = i;
							min = distances[i];
						}
					}
					
					for (::std::size_t i = 0; i < node.children.size(); ++i)
					{
						node.children[i].max[index] = ::std::max(node.children[i].max[index], distances[i]);
						node.children[i].min[index] = ::std::min(node.children[i].min[index], distances[i]);
					}
					
					this->push(node.children[index], value);
				}
			}
			
			::std::vector<Neighbor> search(const Value& query, const ::std::size_t* k, const Distance* radius, const bool& sorted) const
			{
				::std::vector<Neighbor> neighbors;
				neighbors.reserve(nullptr != k ? *k : this->values);
				
				::std::size_t checks = 0;
				
				::std::vector<Branch> branches;
				this->search(this->root, query, k, radius, branches, neighbors, checks);
				
				while (!branches.empty() && (!this->checks || checks < this->checks))
				{
					Branch branch = branches.front();
					::std::pop_heap(branches.begin(), branches.end(), BranchCompare());
					branches.pop_back();
					
					if (nullptr == k || *k == neighbors.size())
					{
						Distance distance = nullptr != radius ? *radius : neighbors.front().first;
						
						if (branch.first - distance > branch.second->max[branch.second->index] ||
							branch.first + distance < branch.second->min[branch.second->index])
						{
							continue;
						}
					}
					
					this->search(*branch.second, query, k, radius, branches, neighbors, checks);
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
			
			void search(const Node& node, const Value& query, const ::std::size_t* k, const Distance* radius, ::std::vector<Branch>& branches, ::std::vector<Neighbor>& neighbors, ::std::size_t& checks) const
			{
				if (node.children.empty())
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
					::std::vector<Distance> distances(node.children.size());
					::std::vector<bool> removed(node.children.size(), false);
					
					for (::std::size_t i = 0; i < node.children.size(); ++i)
					{
						if (!removed[i])
						{
							distances[i] = this->metric(query, node.children[i].pivot);
							
							if (!node.children[i].removed)
							{
								if (nullptr == k || neighbors.size() < *k || distances[i] < neighbors.front().first)
								{
									if (nullptr == radius || distances[i] < *radius)
									{
										if (nullptr != k && *k == neighbors.size())
										{
											::std::pop_heap(neighbors.begin(), neighbors.end(), NeighborCompare());
											neighbors.pop_back();
										}
										
#if (defined(_MSC_VER) && _MSC_VER < 1800) || (defined(__GNUC__) && __GNUC__ == 4 && __GNUC_MINOR__ < 8)
										neighbors.push_back(::std::make_pair(distances[i], node.children[i].pivot));
#else
										neighbors.emplace_back(::std::piecewise_construct, ::std::forward_as_tuple(distances[i]), ::std::forward_as_tuple(node.children[i].pivot));
#endif
										::std::push_heap(neighbors.begin(), neighbors.end(), NeighborCompare());
									}
								}
							}
							
							if (nullptr == k || *k == neighbors.size())
							{
								Distance distance = nullptr != radius ? *radius : neighbors.front().first;
								
								for (::std::size_t j = 0; j < node.children.size(); ++j)
								{
									if (i != j && !removed[j])
									{
										if (distances[i] - distance > node.children[i].max[j] ||
											distances[i] + distance < node.children[i].min[j])
										{
											removed[j] = true;
										}
									}
								}
							}
							
							if (this->checks && ++checks > this->checks)
							{
								return;
							}
						}
					}
					
					for (::std::size_t i = 0; i < node.children.size(); ++i)
					{
						if (!removed[i])
						{
							Distance distance = nullptr != radius ? *radius : neighbors.front().first;
							
							if (distances[i] - distance <= node.children[i].max[i] &&
								distances[i] + distance >= node.children[i].min[i])
							{
#if defined(_MSC_VER) && _MSC_VER < 1800
								branches.push_back(::std::make_pair(distances[i], &node.children[i]));
#else
								branches.emplace_back(distances[i], &node.children[i]);
#endif
								::std::push_heap(branches.begin(), branches.end(), BranchCompare());
							}
						}
					}
				}
			}
			
			void split(Node& node)
			{
				
				::std::vector< ::std::vector<Distance>> distances(node.degree, ::std::vector<Distance>(node.data.size()));
				::std::vector< ::std::size_t> centers(node.degree);
				this->choose(node, centers, distances);
				
				for (::std::size_t i = 0; i < centers.size(); ++i)
				{
#if defined(_MSC_VER) && _MSC_VER < 1800
					node.children.push_back(Node(i, node.degree - 1, this->nodeDegree, this->nodeDataMax));
#else
					node.children.emplace_back(i, node.degree - 1, this->nodeDegree, this->nodeDataMax);
#endif
					node.children[i].pivot = ::std::move(node.data[centers[i]]);
				}
				
				for (::std::size_t i = 0; i < node.data.size(); ++i)
				{
					::std::size_t index = 0;
					Distance min = ::std::numeric_limits<Distance>::infinity();
					
					for (::std::size_t j = 0; j < centers.size(); ++j)
					{
						Distance distance = distances[j][i];
						
						if (distance < min)
						{
							index = j;
							min = distance;
						}
					}
					
					for (::std::size_t j = 0; j < centers.size(); ++j)
					{
						if (i != centers[j])
						{
							node.children[j].max[index] = ::std::max(node.children[j].max[index], distances[j][i]);
							node.children[j].min[index] = ::std::min(node.children[j].min[index], distances[j][i]);
						}
					}
					
					if (i != centers[index])
					{
						node.children[index].data.push_back(::std::move(node.data[i]));
					}
				}
				
				for (::std::size_t i = 0; i < node.children.size(); ++i)
				{
					node.children[i].degree = ::std::min(::std::max(this->nodeDegree * node.children[i].data.size() / node.data.size(), this->nodeDegreeMin), this->nodeDegreeMax);
					
					if (node.children[i].data.empty())
					{
						node.children[i].max[i] = Distance();
						node.children[i].min[i] = Distance();
					}
				}
				
				::std::size_t size = node.data.size();
				
				node.data.clear();
				node.data.shrink_to_fit();
				
#pragma omp parallel for if (size > 2 * this->nodeDataMax)
#if defined(_OPENMP) && _OPENMP < 200805
				for (::std::ptrdiff_t i = 0; i < node.children.size(); ++i)
#else
				for (::std::size_t i = 0; i < node.children.size(); ++i)
#endif
				{
					if (node.children[i].data.size() > this->nodeDataMax && node.children[i].data.size() > node.children[i].degree)
					{
						this->split(node.children[i]);
					}
				}
			}
			
			::boost::optional< ::std::size_t> checks;
			
			::std::mt19937 generator;
			
			Metric metric;
			
			::std::size_t nodeDataMax;
			
			::std::size_t nodeDegree;
			
			::std::size_t nodeDegreeMax;
			
			::std::size_t nodeDegreeMin;
			
			Node root;
			
			::std::size_t values;
		};
	}
}

#endif // RL_MATH_GNATNEARESTNEIGHBORS_H
