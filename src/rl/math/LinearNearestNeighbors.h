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

#ifndef RL_MATH_LINEARNEARESTNEIGHBORS_H
#define RL_MATH_LINEARNEARESTNEIGHBORS_H

#include <algorithm>
#include <vector>

namespace rl
{
	namespace math
	{
		/**
		 * Linear nearest neighbor search.
		 */
		template<typename MetricT, typename ContainerT = ::std::vector<typename MetricT::Value>>
		class LinearNearestNeighbors
		{
		public:
			typedef typename ContainerT::const_iterator const_iterator;
			
			typedef const typename MetricT::Value& const_reference;
			
			typedef typename ContainerT::difference_type difference_type;
			
			typedef typename ContainerT::iterator iterator;
			
			typedef typename MetricT::Value& reference;
			
			typedef typename ContainerT::size_type size_type;
			
			typedef typename MetricT::Value value_type;
			
			typedef ContainerT Container;
			
			typedef typename MetricT::Distance Distance;
			
			typedef MetricT Metric;
			
			typedef typename MetricT::Value Value;
			
			typedef ::std::pair<Distance, Value> Neighbor;
			
			explicit LinearNearestNeighbors(const Metric& metric) :
				container(),
				metric(metric)
			{
			}
			
			explicit LinearNearestNeighbors(Metric&& metric = Metric()) :
				container(),
				metric(::std::move(metric))
			{
			}
			
			template<typename InputIterator>
			LinearNearestNeighbors(InputIterator first, InputIterator last, const Metric& metric) :
				container(first, last),
				metric(metric)
			{
			}
			
			template<typename InputIterator>
			LinearNearestNeighbors(InputIterator first, InputIterator last, Metric&& metric = Metric()) :
				container(first, last),
				metric(::std::move(metric))
			{
			}
			
			~LinearNearestNeighbors()
			{
			}
			
			Value& at(const ::std::size_t& i)
			{
				return this->container.at(i);
			}
			
			const Value& at(const ::std::size_t& i) const
			{
				return this->container.at(i);
			}
			
			Value& back()
			{
				return this->container.back();
			}
			
			const Value& back() const
			{
				return this->container.back();
			}
			
			iterator begin()
			{
				return this->container.begin();
			}
			
			const_iterator begin() const
			{
				return this->container.begin();
			}
			
			::std::size_t capacity() const
			{
				return this->container.capacity();
			}
			
			const_iterator cbegin() const
			{
				return this->container.cbegin();
			}
			
			const_iterator cend() const
			{
				return this->container.cend();
			}
			
			void clear()
			{
				this->container.clear();
			}
			
			bool empty() const
			{
				return this->container.empty();
			}
			
			iterator end()
			{
				return this->container.end();
			}
			
			const_iterator end() const
			{
				return this->container.end();
			}
			
			void erase(const_iterator pos)
			{
				this->container.erase(pos);
			}
			
			Value& front()
			{
				return this->container.front();
			}
			
			const Value& front() const
			{
				return this->container.front();
			}
			
			template<typename InputIterator>
			void insert(InputIterator first, InputIterator last)
			{
				this->container.insert(this->container.end(), first, last);
			}
			
			::std::size_t max_size() const
			{
				return this->container.max_size();
			}
			
			::std::vector<Neighbor> nearest(const Value& query, const ::std::size_t& k, const bool& sorted = true) const
			{
				return this->search(query, &k, nullptr, sorted);
			}
			
			Value& operator[](const ::std::size_t& i)
			{
				return this->container[i];
			}
			
			const Value& operator[](const ::std::size_t& i) const
			{
				return this->container[i];
			}
			
			void push(const Value& value)
			{
				this->container.push_back(value);
			}
			
			::std::vector<Neighbor> radius(const Value& query, const Distance& radius, const bool& sorted = true) const
			{
				return this->search(query, nullptr, &radius, sorted);
			}
			
			void reserve(const ::std::size_t& capacity)
			{
				this->container.reserve(capacity);
			}
			
			::std::size_t size() const
			{
				return this->container.size();
			}
			
			void swap(LinearNearestNeighbors& other)
			{
				using ::std::swap;
				swap(this->container, other.container);
				swap(this->metric, other.metric);
			}
			
			friend void swap(LinearNearestNeighbors& lhs, LinearNearestNeighbors& rhs)
			{
				lhs.swap(rhs);
			}
			
		protected:
			
		private:
			struct NeighborCompare
			{
				bool operator()(const Neighbor& lhs, const Neighbor& rhs) const
				{
					return lhs.first < rhs.first;
				}
			};
			
			::std::vector<Neighbor> search(const Value& query, const ::std::size_t* k, const Distance* radius, const bool& sorted) const
			{
				::std::vector<Neighbor> neighbors;
				neighbors.reserve(nullptr != k ? *k : this->size());
				
				::std::vector<Distance> distances(this->container.size());
				
#pragma omp parallel for
#if defined(_OPENMP) && _OPENMP < 200805
				for (::std::ptrdiff_t i = 0; i < this->container.size(); ++i)
#else
				for (::std::size_t i = 0; i < this->container.size(); ++i)
#endif
				{
					distances[i] = this->metric(query, this->container[i]);
				}
				
				for (::std::size_t i = 0; i < this->container.size(); ++i)
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
							
#if defined(_MSC_VER) && _MSC_VER < 1800
							neighbors.push_back(::std::make_pair(distances[i], this->container[i]));
#else
							neighbors.emplace_back(distances[i], this->container[i]);
#endif
							::std::push_heap(neighbors.begin(), neighbors.end(), NeighborCompare());
						}
					}
				}
				
				if (sorted)
				{
					::std::sort_heap(neighbors.begin(), neighbors.end(), NeighborCompare());
				}
				
				return neighbors;
			}
			
			Container container;
			
			Metric metric;
		};
	}
}

#endif // RL_MATH_LINEARNEARESTNEIGHBORS_H
