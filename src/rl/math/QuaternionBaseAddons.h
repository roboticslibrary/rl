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

#ifndef RL_MATH_QUATERNIONBASEADDONS_H
#define RL_MATH_QUATERNIONBASEADDONS_H

template<typename OtherDerived>
Vector3
angularVelocity(const QuaternionBase<OtherDerived>& qd) const
{
	return ((qd * this->derived().conjugate()) * Scalar(2)).vec();
}

template<typename OtherDerived1, typename OtherDerived2>
Vector3
angularAcceleration(const QuaternionBase<OtherDerived1>& qd, const QuaternionBase<OtherDerived2>& qdd) const
{
	return ((qdd * this->derived().conjugate() + qd * qd.conjugate()) * Scalar(2)).vec();
}

Quaternion<Scalar>
exp() const
{
	Scalar theta = this->derived().vec().norm();
	Scalar sinTheta = ::sin(theta);
	Scalar cosTheta = ::cos(theta);
	
	Quaternion<Scalar> q;
	
	if (theta > Scalar(0))
	{
		q.vec() = sinTheta * this->derived().vec() / theta;
	}
	else
	{
		q.vec().setZero();
	}
	
	q.w() = cosTheta;
	
	return q;
}

Quaternion<Scalar>
firstDerivative(const Vector3& omega) const
{
	return (Quaternion<Scalar>(Scalar(0), omega.x(), omega.y(), omega.z()) * this->derived()) * Scalar(0.5);
}

template<typename OtherDerived>
Quaternion<Scalar> lerp(const Scalar& t, const QuaternionBase<OtherDerived>& other) const
{
	return Quaternion<Scalar>((Scalar(1) - t) * coeffs() + t * other.coeffs());
}

Quaternion<Scalar>
log() const
{
	Scalar theta = ::acos(this->derived().w());
	Scalar sinTheta = ::sin(theta);
	
	Quaternion<Scalar> q;
	
	if (sinTheta > Scalar(0))
	{
		q.vec() = theta * this->derived().vec() / sinTheta;
	}
	else
	{
		q.vec().setZero();
	}
	
	q.w() = Scalar(0);
	
	return q;
}

template<typename OtherDerived>
Quaternion<Scalar>
operator+(const QuaternionBase<OtherDerived>& other) const
{
	return Quaternion<Scalar>(this->derived().coeffs() + other.coeffs());
}

template<typename OtherDerived>
Quaternion<Scalar>
operator-(const QuaternionBase<OtherDerived>& other) const
{
	return Quaternion<Scalar>(this->derived().coeffs() - other.coeffs());
}

Quaternion<Scalar>
operator*(const Scalar& scalar) const
{
	return Quaternion<Scalar>(this->derived().coeffs() * scalar);
}

Quaternion<Scalar>
operator/(const Scalar& scalar) const
{
	return Quaternion<Scalar>(this->derived().coeffs() / scalar);
}

Quaternion<Scalar>
pow(const Scalar& t) const
{
	return (this->derived().log() * Scalar(t)).exp();
}

template<typename OtherDerived>
Quaternion<Scalar>
secondDerivative(const QuaternionBase<OtherDerived>& qd, const Vector3& omega, const Vector3& omegad) const
{
	return (Quaternion<Scalar>(Scalar(0), omegad.x(), omegad.y(), omegad.z()) * this->derived() + Quaternion<Scalar>(Scalar(0), omega.x(), omega.y(), omega.z()) * qd) * Scalar(0.5);
}

/**
 * QuTEM (Quaternion Tangent Ellipsoid at the Mean) sampling algorithm.
 * 
 * Michael Patrick Johnson. Exploiting Quaternions to Support Expressive
 * Interactive Character Motion. PhD Thesis, Massachusetts Institute of
 * Technology, Cambridge, MA, USA, February 2003.
 * 
 * http://characters.media.mit.edu/Theses/johnson_phd.pdf
 */
template<typename OtherDerived>
void
setFromGaussian(const Vector3& rand, const QuaternionBase<OtherDerived>& mean, const Vector3& sigma)
{
	eigen_assert(rand(0) >= Scalar(0));
	eigen_assert(rand(0) <= Scalar(1));
	eigen_assert(rand(1) >= Scalar(0));
	eigen_assert(rand(1) <= Scalar(1));
	eigen_assert(rand(2) >= Scalar(0));
	eigen_assert(rand(2) <= Scalar(1));
	
	Quaternion<Scalar> tmp;
	tmp.w() = rand.norm();
	tmp.vec() = sigma.cwiseProduct(rand);
	
	this->derived() = mean * tmp.exp();
}

/**
 * Generate uniformly-distributed random unit quaternions.
 * 
 * James J. Kuffner. Effective Sampling and Distance Metrics for 3D Rigid Body
 * Path Planning. Proceedings of the IEEE International Conference on Robotics
 * and Automation, pages 3993-3998. New Orleans, LA, USA, April 2004.
 * 
 * https://doi.org/10.1109/ROBOT.2004.1308895
 */
void
setFromUniform(const Vector3& rand)
{
	eigen_assert(rand(0) >= Scalar(0));
	eigen_assert(rand(0) <= Scalar(1));
	eigen_assert(rand(1) >= Scalar(0));
	eigen_assert(rand(1) <= Scalar(1));
	eigen_assert(rand(2) >= Scalar(0));
	eigen_assert(rand(2) <= Scalar(1));
	
	Scalar sigma1 = ::std::sqrt(Scalar(1) - rand(0));
	Scalar sigma2 = ::std::sqrt(rand(0));
	Scalar theta1 = Scalar(2) * static_cast<Scalar>(M_PI) * rand(1);
	Scalar theta2 = Scalar(2) * static_cast<Scalar>(M_PI) * rand(2);
	
	this->derived().w() = ::std::cos(theta2) * sigma2;
	this->derived().x() = ::std::sin(theta1) * sigma1;
	this->derived().y() = ::std::cos(theta1) * sigma1;
	this->derived().z() = ::std::sin(theta2) * sigma2;
}

template<typename OtherDerived>
Quaternion<Scalar>
slerpFirstDerivative(const Scalar& t, const QuaternionBase<OtherDerived>& other) const
{
	Quaternion<Scalar> tmp = this->derived().conjugate() * other;
	return this->derived() * tmp.pow(t) * tmp.log();
}

template<typename OtherDerived1, typename OtherDerived2, typename OtherDerived3>
Quaternion<Scalar>
squad(const Scalar& t, const QuaternionBase<OtherDerived1>& a, const QuaternionBase<OtherDerived2>& b, const QuaternionBase<OtherDerived3>& other) const
{
	return this->derived().slerp(t, other).slerp(Scalar(2) * t * (Scalar(1) - t), a.slerp(t, b));
}

template<typename OtherDerived1, typename OtherDerived2>
Quaternion<Scalar>
squadControlPoint(const QuaternionBase<OtherDerived1>& previous, const QuaternionBase<OtherDerived2>& next) const
{
	return this->derived() * (((this->derived().conjugate() * previous).log() + (this->derived().conjugate() * next).log()) / Scalar(-4)).exp();
}

template<typename OtherDerived1, typename OtherDerived2, typename OtherDerived3>
Quaternion<Scalar>
squadFirstDerivative(const Scalar& t, const QuaternionBase<OtherDerived1>& a, const QuaternionBase<OtherDerived2>& b, const QuaternionBase<OtherDerived3>& other) const
{
	Quaternion<Scalar> u = this->derived().slerp(t, other);
	Quaternion<Scalar> v = a.slerp(t, b);
	Quaternion<Scalar> w = u.inverse() * v;
	Quaternion<Scalar> ud = u * (this->derived().conjugate() * other).log();
	Quaternion<Scalar> vd = v * (a.conjugate() * b).log();
	Quaternion<Scalar> wd = u.conjugate() * vd - u.pow(Scalar(-2)) * ud * v;
	Scalar tmp = Scalar(2) * t * (Scalar(1) - t);
	return u * (w.pow(tmp) * (Scalar(2) - Scalar(4) * t) * w.log() + w.pow(tmp - Scalar(1)) * tmp * wd) + ud * w.pow(tmp);
}

#endif // RL_MATH_QUATERNIONBASEADDONS_H
