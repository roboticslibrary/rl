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

#include <fstream>
#include <iostream>
#include <Eigen/Eigenvalues>
#include <rl/math/Matrix.h>
#include <rl/math/Vector.h>

int
main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: rlPcaDemo FILE" << std::endl;
		return EXIT_FAILURE;
	}
	
	std::ifstream data;
	data.open(argv[1]);
	
	std::size_t rows;
	data >> rows;
	std::size_t cols;
	data >> cols;
	
	rl::math::Matrix src(rows, cols);
	
	for (std::size_t i = 0; i < rows; ++i)
	{
		for (std::size_t j = 0; j < cols; ++j)
		{
			data >> src(i, j);
		}
	}
	
	data.close();
	
	std::cout << "src = " << std::endl << src << std::endl;
	
	rl::math::Vector mean(cols);
	mean.setZero();
	
	for (std::size_t i = 0; i < rows; ++i)
	{
		mean += src.row(i);
	}
	
	mean /= static_cast<rl::math::Real>(rows);
	
	std::cout << "mean = " << mean.transpose() << std::endl;
	
	rl::math::Matrix covariance(cols, cols);
	covariance.setZero();
	
	for (std::size_t i = 0; i < rows; ++i)
	{
		rl::math::Vector delta = src.row(i).transpose() - mean;
		covariance += delta * delta.transpose();
	}
	
	covariance /= static_cast<rl::math::Real>(rows) - 1;
	
	std::cout << "covariance = " << std::endl << covariance << std::endl;
	
	Eigen::EigenSolver<rl::math::Matrix> eigen(covariance);
	
	std::cout << "eigenvectors = " << std::endl << eigen.eigenvectors() << std::endl;
	std::cout << "eigenvalues = " << std::endl << eigen.eigenvalues() << std::endl;
	
	return EXIT_SUCCESS;
}
