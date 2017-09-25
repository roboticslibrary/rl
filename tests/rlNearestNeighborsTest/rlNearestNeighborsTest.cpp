#include <chrono>
#include <iostream>
#include <vector>
#include <rl/math/GnatNearestNeighbors.h>
#include <rl/math/KdtreeBoundingBoxNearestNeighbors.h>
#include <rl/math/KdtreeNearestNeighbors.h>
#include <rl/math/LinearNearestNeighbors.h>
#include <rl/math/Vector.h>
#include <rl/math/metrics/L2.h>
#include <rl/math/metrics/L2Squared.h>

#include "iterator.h"

#define DIM 6
#define K 30
#define N 100000
#define QUERIES 100

template<typename NearestNeighbors>
std::vector<std::vector<typename NearestNeighbors::Neighbor>>
test(const std::vector<rl::math::Vector>& points, const std::vector<rl::math::Vector>& queries, const bool& iterative)
{
	std::vector<std::vector<typename NearestNeighbors::Neighbor>> results;
	
	std::vector<const rl::math::Vector*> points2;
	points2.reserve(points.size());
	
	for (std::size_t i = 0; i < points.size(); ++i)
	{
		points2.push_back(&points[i]);
	}
	
	NearestNeighbors nearestNeighbors;
	
	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	
	if (iterative)
	{
		for (std::size_t i = 0; i < points.size(); ++i)
		{
			nearestNeighbors.push(points2[i]);
		}
	}
	else
	{
		nearestNeighbors.insert(points2.begin(), points2.end());
	}
	
	std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now();
	
	std::cout << "build time " << std::chrono::duration_cast<std::chrono::duration<double>>(stop - start).count() * 1000 << " ms" << std::endl;
	
	double time = 0;
	
	for (std::size_t i = 0; i < queries.size(); ++i)
	{
		start = std::chrono::steady_clock::now();
		std::vector<typename NearestNeighbors::Neighbor> neighbors = nearestNeighbors.nearest(&queries[i], K);
		stop = std::chrono::steady_clock::now();
		
		results.push_back(neighbors);
		
		time += std::chrono::duration_cast<std::chrono::duration<double>>(stop - start).count() * 1000;
		
		if (0 == i)
		{
			std::cout << "searching for " << queries[i].transpose() << std::endl;
			
			for (std::size_t j = 0; j < neighbors.size(); ++j)
			{
				std::cout << "found " << neighbors[j].second->transpose() << " dist " << neighbors[j].first << std::endl;
				break;
			}
			
			std::cout << "search time " << std::chrono::duration_cast<std::chrono::duration<double>>(stop - start).count() * 1000 << " ms" << std::endl;
		}
	}
	
	if (queries.size() > 1)
	{
		std::cout << "average search time (" << queries.size() << ") " << time / queries.size() << " ms" << std::endl;
	}
	
	return results;
}

void
test(const std::vector<rl::math::Vector>& points, const std::vector<rl::math::Vector>& queries, const bool& iterative)
{
	typedef rl::math::metrics::L2<const rl::math::Vector*> Metric;
	typedef rl::math::metrics::L2Squared<const rl::math::Vector*> MetricSquared;
	
	std::cout << "** LinearNearestNeighbors<Metric> *********************************************" << std::endl;
	std::vector<std::vector<rl::math::LinearNearestNeighbors<Metric>::Neighbor>> linear = test<rl::math::LinearNearestNeighbors<Metric>>(points, queries, iterative);
	
	std::cout << "** LinearNearestNeighbors<MetricSquared> **************************************" << std::endl;
	std::vector<std::vector<rl::math::LinearNearestNeighbors<MetricSquared>::Neighbor>> linear2 = test<rl::math::LinearNearestNeighbors<MetricSquared>>(points, queries, iterative);
	
	std::cout << "** GnatNearestNeighbors<Metric> ***********************************************" << std::endl;
	std::vector<std::vector<rl::math::GnatNearestNeighbors<Metric>::Neighbor>> gnat = test<rl::math::GnatNearestNeighbors<Metric>>(points, queries, iterative);
	
	std::cout << "** KdtreeBoundingBoxNearestNeighbors<MetricSquared> ***************************" << std::endl;
	std::vector<std::vector<rl::math::KdtreeBoundingBoxNearestNeighbors<MetricSquared>::Neighbor>> kdtreeBoundingBox = test<rl::math::KdtreeBoundingBoxNearestNeighbors<MetricSquared>>(points, queries, iterative);
	
	std::cout << "** KdtreeNearestNeighbors<MetricSquared> **************************************" << std::endl;
	std::vector<std::vector<rl::math::KdtreeNearestNeighbors<MetricSquared>::Neighbor>> kdtree = test<rl::math::KdtreeNearestNeighbors<MetricSquared>>(points, queries, iterative);
	
	for (std::size_t i = 0; i < linear.size(); ++i)
	{
		for (std::size_t j = 0; j < linear[i].size(); ++j)
		{
			if (!linear[i][j].second->isApprox(*gnat[i][j].second))
			{
				std::cerr << "rlNearestNeighborsTest: LinearNearestNeighbors != GnatNearestNeighbors" << std::endl;
				std::cerr << "[" << i << "][" << j << "] " << linear[i][j].first << " LinearNearestNeighbors: " << linear[i][j].second->transpose() << std::endl;
				std::cerr << "[" << i << "][" << j << "] " << gnat[i][j].first << " GnatNearestNeighbors: " << gnat[i][j].second->transpose() << std::endl;
				exit(EXIT_FAILURE);
			}
			
			if (!linear[i][j].second->isApprox(*linear2[i][j].second))
			{
				std::cerr << "rlNearestNeighborsTest: LinearNearestNeighbors<Metric> != LinearNearestNeighbors<MetricSquared>" << std::endl;
				std::cerr << "[" << i << "][" << j << "] " << linear[i][j].first << " LinearNearestNeighbors<Metric>: " << linear[i][j].second->transpose() << std::endl;
				std::cerr << "[" << i << "][" << j << "] " << linear2[i][j].first << " LinearNearestNeighbors<MetricSquared>: " << linear2[i][j].second->transpose() << std::endl;
				exit(EXIT_FAILURE);
			}
			
			if (!linear[i][j].second->isApprox(*gnat[i][j].second))
			{
				std::cerr << "rlNearestNeighborsTest: LinearNearestNeighbors<Metric> != GnatNearestNeighbors<Metric>" << std::endl;
				std::cerr << "[" << i << "][" << j << "] " << linear[i][j].first << " LinearNearestNeighbors<Metric>: " << linear[i][j].second->transpose() << std::endl;
				std::cerr << "[" << i << "][" << j << "] " << gnat[i][j].first << " GnatNearestNeighbors<Metric>: " << gnat[i][j].second->transpose() << std::endl;
				exit(EXIT_FAILURE);
			}
			
			if (!linear[i][j].second->isApprox(*kdtreeBoundingBox[i][j].second))
			{
				std::cerr << "rlNearestNeighborsTest: LinearNearestNeighbors<Metric> != KdtreeBoundingBoxNearestNeighbors<MetricSquared>" << std::endl;
				std::cerr << "[" << i << "][" << j << "] " << linear[i][j].first << " LinearNearestNeighbors<Metric>: " << linear[i][j].second->transpose() << std::endl;
				std::cerr << "[" << i << "][" << j << "] " << kdtreeBoundingBox[i][j].first << " KdtreeBoundingBoxNearestNeighbors<MetricSquared>: " << kdtreeBoundingBox[i][j].second->transpose() << std::endl;
				exit(EXIT_FAILURE);
			}
			
			if (!linear[i][j].second->isApprox(*kdtree[i][j].second))
			{
				std::cerr << "rlNearestNeighborsTest: LinearNearestNeighbors<Metric> != KdtreeNearestNeighbors<MetricSquared>" << std::endl;
				std::cerr << "[" << i << "][" << j << "] " << linear[i][j].first << " LinearNearestNeighbors<Metric>: " << linear[i][j].second->transpose() << std::endl;
				std::cerr << "[" << i << "][" << j << "] " << kdtree[i][j].first << " KdtreeNearestNeighbors<MetricSquared>: " << kdtree[i][j].second->transpose() << std::endl;
				exit(EXIT_FAILURE);
			}
		}
	}
}

int
main(int argc, char** argv)
{
	std::vector<rl::math::Vector> points;
	std::vector<rl::math::Vector> queries;
	
	points.clear();
	points.push_back(rl::math::Vector(2));
	points.back() << 2, 3;
	points.push_back(rl::math::Vector(2));
	points.back() << 5, 4;
	points.push_back(rl::math::Vector(2));
	points.back() << 9, 6;
	points.push_back(rl::math::Vector(2));
	points.back() << 4, 8;
	points.push_back(rl::math::Vector(2));
	points.back() << 8, 1;
	points.push_back(rl::math::Vector(2));
	points.back() << 7, 2;
	
	std::cout << "===============================================================================" << std::endl << std::endl;
	
	queries.clear();
	queries.push_back(rl::math::Vector(2));
	queries.back() << 9, 2;
	
	test(points, queries, false);
	std::cout << std::endl << "-------------------------------------------------------------------------------" << std::endl << std::endl;
	test(points, queries, true);
	
	std::cout << std::endl;
	
	points.clear();
	points.reserve(N);
	
	for (std::size_t i = 0; i < N; ++i)
	{
		points.push_back(rl::math::Vector::Random(DIM));
	}
	
	std::cout << "===============================================================================" << std::endl << std::endl;
	
	queries.clear();
	
	for (std::size_t i = 0; i < QUERIES; ++i)
	{
		queries.push_back(rl::math::Vector::Random(DIM));
	}
	
	test(points, queries, false);
	std::cout << std::endl << "-------------------------------------------------------------------------------" << std::endl << std::endl;
	test(points, queries, true);
	
	return EXIT_SUCCESS;
}
