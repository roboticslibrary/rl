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

#include <iostream>
#include <rl/math/Matrix.h>
#include <rl/math/Quaternion.h>
#include <rl/math/Spatial.h>
#include <rl/math/Vector.h>

int
main(int argc, char** argv)
{
	rl::math::Vector3 a;
	a.setRandom();
	
	rl::math::Matrix33 m1a = a.cross33();
	rl::math::Matrix33 m1b = -a.cross33().transpose();
	
	if (!m1a.isApprox(m1b))
	{
		std::cerr << "ax != -ax^T" << std::endl;
		std::cerr << "ax = " << std::endl << m1a << std::endl;
		std::cerr << "-ax^T = " << std::endl << m1b << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector3 b;
	b.setRandom();
	
	rl::math::Matrix33 m2a = (a + b).cross33();
	rl::math::Matrix33 m2b = a.cross33() + b.cross33();
	
	if (!m2a.isApprox(m2b))
	{
		std::cerr << "(a + b)x != ax + bx" << std::endl;
		std::cerr << "(a + b)x = " << std::endl << m2a << std::endl;
		std::cerr << "ax + bx = " << std::endl << m2b << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Matrix33 r = rl::math::Matrix33(rl::math::Quaternion::Random());
	
	rl::math::Matrix33 m3a = (r * a).cross33();
	rl::math::Matrix33 m3b = r * a.cross33() * r.transpose();
	
	if (!m3a.isApprox(m3b))
	{
		std::cerr << "(R * a)x != R ax R^T" << std::endl;
		std::cerr << "(R * a)x = " << std::endl << m3a << std::endl;
		std::cerr << "R ax R^T = " << std::endl << m3b << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector3 v4a = a.cross33() * b;
	rl::math::Vector3 v4b = -b.cross33() * a;
	
	if (!v4a.isApprox(v4b))
	{
		std::cerr << "(ax) b != -(bx) a" << std::endl;
		std::cerr << "(ax) b = " << std::endl << v4a << std::endl;
		std::cerr << "-(bx) a = " << std::endl << v4b << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector3 v5a = a.cross(b).transpose();
	rl::math::Vector3 v5b = a.transpose() * b.cross33();
	
	if (!v5a.isApprox(v5b))
	{
		std::cerr << "(a x b)^T != a^T bx" << std::endl;
		std::cerr << "(a x b)^T = " << std::endl << v5a << std::endl;
		std::cerr << "a^T bx = " << std::endl << v5b << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Matrix33 m6a = a.cross(b).cross33();
	rl::math::Matrix33 m6b = a.cross33() * b.cross33() - b.cross33() * a.cross33();
	
	if (!m6a.isApprox(m6b))
	{
		std::cerr << "(a x b)x != ax bx - bx ax" << std::endl;
		std::cerr << "(a x b)x = " << std::endl << m6a << std::endl;
		std::cerr << "ax bx - bx ax = " << std::endl << m6b << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Matrix33 m7a = a.cross(b).cross33();
	rl::math::Matrix33 m7b = b * a.transpose() - a * b.transpose();
	
	if (!m7a.isApprox(m7b))
	{
		std::cerr << "(a x b)x != b a^T - a b^T" << std::endl;
		std::cerr << "(a x b)x = " << std::endl << m7a << std::endl;
		std::cerr << "b a^T - a b^T = " << std::endl << m7b << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Matrix33 m8a = a.cross33() * b.cross33();
	rl::math::Matrix33 m8b = b * a.transpose() - a.dot(b) * rl::math::Matrix33::Identity();
	
	if (!m8a.isApprox(m8b))
	{
		std::cerr << "ax bx != b a^T - (a * b) I" << std::endl;
		std::cerr << "ax bx = " << std::endl << m8a << std::endl;
		std::cerr << "b a^T - (a * b) I = " << std::endl << m8b << std::endl;
		exit(EXIT_FAILURE);
	}
	
	return EXIT_SUCCESS;
}
