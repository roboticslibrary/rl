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
				
				if (this->empty())
				{
					return neighbors;
				}
				
				if (nullptr != k)
				{
					neighbors.reserve(::std::min(*k, this->size()));
				}
				
#ifdef _OPENMP
				::std::vector< ::std::vector<Neighbor>> neighbors2(::omp_get_max_threads());
				
				if (nullptr != k)
				{
					for (::std::size_t i = 0; i < neighbors2.size(); ++i)
					{
						neighbors2[i].reserve(::std::min(*k, this->size()) / ::omp_get_max_threads());
					}
				}
				
#pragma omp parallel for
#if _OPENMP < 200805
				for (::std::ptrdiff_t i = 0; i < this->container.size(); ++i)
#else
				for (::std::size_t i = 0; i < this->container.size(); ++i)
#endif
#else
				::std::vector<Neighbor>& neighbors3 = neighbors;
				
				for (::std::size_t i = 0; i < this->container.size(); ++i)
#endif
				{
#ifdef _OPENMP
					::std::vector<Neighbor>& neighbors3 = neighbors2[::omp_get_thread_num()];
#endif
					
					Distance distance = this->metric(query, this->container[i]);
					
					if (nullptr == k || neighbors3.size() < *k || distance < neighbors3.front().first)
					{
						if (nullptr == radius || distance < *radius)
						{
							if (nullptr != k && *k == neighbors3.size())
							{
								::std::pop_heap(neighbors3.begin(), neighbors3.end(), NeighborCompare());
								neighbors3.pop_back();
							}
							
							neighbors3.emplace_back(distance, this->container[i]);
							::std::push_heap(neighbors3.begin(), neighbors3.end(), NeighborCompare());
						}
					}
				}
				
#ifdef _OPENMP
				for (::std::size_t i = 0; i < neighbors2.size(); ++i)
				{
					for (::std::size_t j = 0; j < neighbors2[i].size(); ++j)
					{
						if (nullptr == k || neighbors.size() < *k || neighbors2[i][j].first < neighbors.front().first)
						{
							if (nullptr == radius || neighbors2[i][j].first < *radius)
							{
								if (nullptr != k && *k == neighbors.size())
								{
									::std::pop_heap(neighbors.begin(), neighbors.end(), NeighborCompare());
									neighbors.pop_back();
								}
								
								neighbors.push_back(::std::move(neighbors2[i][j]));
								::std::push_heap(neighbors.begin(), neighbors.end(), NeighborCompare());
							}
						}
					}
				}
#endif
				
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
