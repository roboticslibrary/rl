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

#ifndef RL_MATH_SPATIAL_H
#define RL_MATH_SPATIAL_H

#define EIGEN_MATRIXBASE_PLUGIN <rl/math/MatrixBaseAddons.h>
#define EIGEN_QUATERNIONBASE_PLUGIN <rl/math/QuaternionBaseAddons.h>
#define EIGEN_TRANSFORM_PLUGIN <rl/math/TransformAddons.h>

#include "Real.h"

#include "spatial/ArticulatedBodyInertia.h"
#include "spatial/ForceVector.h"
#include "spatial/MotionVector.h"
#include "spatial/PlueckerTransform.h"
#include "spatial/RigidBodyInertia.h"

#include "spatial/ArticulatedBodyInertia.hxx"
#include "spatial/ForceVector.hxx"
#include "spatial/MotionVector.hxx"
#include "spatial/PlueckerTransform.hxx"
#include "spatial/RigidBodyInertia.hxx"

namespace rl
{
	namespace math
	{
		typedef spatial::ArticulatedBodyInertia<Real> ArticulatedBodyInertia;
		
		typedef spatial::ForceVector<Real> ForceVector;
		
		typedef spatial::MotionVector<Real> MotionVector;
		
		typedef spatial::PlueckerTransform<Real> PlueckerTransform;
		
		typedef spatial::RigidBodyInertia<Real> RigidBodyInertia;
	}
}

#endif // RL_MATH_SPATIAL_H
