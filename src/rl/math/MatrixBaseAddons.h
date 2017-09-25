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

#ifndef RL_MATH_MATRIXBASEADDONS_H
#define RL_MATH_MATRIXBASEADDONS_H

Matrix<Scalar, 3, 1>
cross3() const
{
	EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3)
	Matrix<Scalar, 3, 1> res;
	res.x() = (this->derived()(2, 1) - this->derived()(1, 2)) / Scalar(2);
	res.y() = (this->derived()(0, 2) - this->derived()(2, 0)) / Scalar(2);
	res.z() = (this->derived()(1, 0) - this->derived()(0, 1)) / Scalar(2);
	return res;
}

Matrix<Scalar, 3, 3>
cross33() const
{
	EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3)
	Matrix<Scalar, 3, 3> res;
	res(0, 0) = 0;
	res(0, 1) = -this->derived().z();
	res(0, 2) = this->derived().y();
	res(1, 0) = this->derived().z();
	res(1, 1) = 0;
	res(1, 2) = -this->derived().x();
	res(2, 0) = -this->derived().y();
	res(2, 1) = this->derived().x();
	res(2, 2) = 0;
	return res;
}

Matrix<Scalar, 6, 1>
voigt6() const
{
	EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3)
	Matrix<Scalar, 6, 1> res;
	res(0) = this->derived()(0, 0);
	res(1) = this->derived()(1, 1);
	res(2) = this->derived()(2, 2);
	res(3) = (this->derived()(1, 2) + this->derived()(2, 1)) / Scalar(2);
	res(4) = (this->derived()(0, 2) + this->derived()(2, 0)) / Scalar(2);
	res(5) = (this->derived()(0, 1) + this->derived()(1, 0)) / Scalar(2);
	return res;
}

Matrix<Scalar, 3, 3>
voigt33() const
{
	EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 6)
	Matrix<Scalar, 3, 3> res;
	res(0, 0) = this->derived()(0);
	res(0, 1) = this->derived()(5);
	res(0, 2) = this->derived()(4);
	res(1, 0) = this->derived()(5);
	res(1, 1) = this->derived()(1);
	res(1, 2) = this->derived()(3);
	res(2, 0) = this->derived()(4);
	res(2, 1) = this->derived()(3);
	res(2, 2) = this->derived()(2);
	return res;
}

#endif // RL_MATH_MATRIXBASEADDONS_H
