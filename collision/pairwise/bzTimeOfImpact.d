/*
* Copyright (c) 2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

// Documentation comments modified by Brian Schott (SirAlaran)

module blaze.collision.pairwise.bzTimeOfImpact;

import blaze.common.bzMath;
import blaze.common.bzConstants;
import blaze.collision.bzCollision;
import blaze.collision.shapes.bzShape;
import blaze.collision.pairwise.bzDistance;

/**
 * This algorithm uses conservative advancement to compute the time of
 * impact (TOI) of two shapes.
 * See_also: Bullet, Young Kim
 * Params:
 *     shape1 = The first shape
 *     sweep1 = ?
 *     shape2 = The second shape
 *     sweep2 = ?
 * Returns: ?
 */
float timeOfImpact(bzShape shape1, bzSweep sweep1, bzShape shape2, bzSweep sweep2)
{
	float r1 = shape1.sweepRadius;
	float r2 = shape2.sweepRadius;

	assert(sweep1.t0 == sweep2.t0);
	assert(1.0f - sweep1.t0 > float.epsilon);

	float t0 = sweep1.t0;
	bzVec2 v1 = sweep1.c - sweep1.c0;
	bzVec2 v2 = sweep2.c - sweep2.c0;
	float omega1 = sweep1.a - sweep1.a0;
	float omega2 = sweep2.a - sweep2.a0;

	float alpha = 0.0f;

	bzVec2 p1, p2;
	const int k_maxIterations = 20;	// TODO_ERIN Settings
	int iter = 0;
	bzVec2 normal = bzVec2.zeroVect;
	float dist = 0.0f;
	float targetDistance = 0.0f;
	for(;;)
	{
		float t = (1.0f - alpha) * t0 + alpha;
		bzXForm xf1, xf2;
		sweep1.xForm(xf1, t);
		sweep2.xForm(xf2, t);

		// Get the distance between shapes.
		dist = distance(p1, p2, shape1, xf1, shape2, xf2);

		if (iter == 0)
		{
			// Compute a reasonable target distance to give some breathing room
			// for conservative advancement.
			if (dist > 2.0f * k_toiSlop)
			{
				targetDistance = 1.5f * k_toiSlop;
			}
			else
			{
				targetDistance = max(0.05f * k_toiSlop, dist - 0.5f * k_toiSlop);
			}
		}

		if (dist - targetDistance < 0.05f * k_toiSlop || iter == k_maxIterations)
		{
			break;
		}

		normal = p2 - p1;
		normal.normalize();

		// Compute upper bound on remaining movement.
		float approachVelocityBound = bzDot(normal, v1 - v2) + abs(omega1) * r1 + abs(omega2) * r2;
		if (abs(approachVelocityBound) < float.epsilon)
		{
			alpha = 1.0f;
			break;
		}

		// Get the conservative time increment. Don't advance all the way.
		float dAlpha = (dist - targetDistance) / approachVelocityBound;
		//float dt = (dist - 0.5f * k_linearSlop) / approachVelocityBound;
		float newAlpha = alpha + dAlpha;

		// The shapes may be moving apart or a safe distance apart.
		if (newAlpha < 0.0f || 1.0f < newAlpha)
		{
			alpha = 1.0f;
			break;
		}

		// Ensure significant advancement.
		if (newAlpha < (1.0f + 100.0f * float.epsilon) * alpha)
		{
			break;
		}

		alpha = newAlpha;

		++iter;
	}

	return alpha;
}
