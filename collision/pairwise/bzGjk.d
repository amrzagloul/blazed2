/*
 *  Copyright (c) 2008 Mason Green http://www.dsource.org/projects/blaze
 *
 *   This software is provided 'as-is', without any express or implied
 *   warranty. In no event will the authors be held liable for any damages
 *   arising from the use of this software.
 *
 *   Permission is granted to anyone to use this software for any purpose,
 *   including commercial applications, and to alter it and redistribute it
 *   freely, subject to the following restrictions:
 *
 *   1. The origin of this software must not be misrepresented; you must not
 *   claim that you wrote the original software. If you use this software
 *   in a product, an acknowledgment in the product documentation would be
 *   appreciated but is not required.
 *
 *   2. Altered source versions must be plainly marked as such, and must not be
 *   misrepresented as being the original software.
 *
 *   3. This notice may not be removed or altered from any source
 *   distribution.
 */
module blaze.collision.bzGjk;

import blaze.common.bzMath;
import blaze.collision.shapes.bzShape;
import blaze.dynamics.contact.bzContact;
/*
const SIMPLEX_EPSILON = 1f;

/// The Gilbert-Johnson-Keerthi algorithm
class Gjk
{
///
struct Entry {
	///
	bzVec2 y0, y1;
	///
	bzVec2 p0, p1;
	///
	bzVec2 q0, q1;
	/// Direction normal
	bzVec2 v;
	/// Separation distance
	float range;
	///
	float t;
	///
	float s;
}

Advance shape1, shape2;

bzVec2[] sA, sB, sAB;
Entry e;
bool penetrate;

this(bzShape shape1, bzShape shape2)
{
	this.shape1 = shape1;
	this.shape2 = shape2;
	penetrate = false;

	//sA ~= shape1.getColdGJKStart(shape2.position);
	//sB ~= shape2.getColdGJKStart(shape1.position);
	sAB ~= (sA[0] - sB[0]);

	constructEntry(sAB[0], sAB[0], sA[0], sB[0], sA[0], sB[0]);
}

void distance()
{
	int maxIterations;

	while (++maxIterations < 10) {     // Don't want to get caught in an infinite loop!
		bzVec2 v1 = shape1.support(-e.v);
		bzVec2 v2 = shape2.support(e.v);
		sA ~= v1; sB ~= v2;
		sAB ~= (v1 - v2);

		if ((e.v.bzDot(e.v) - e.v.bzDot(sAB[sAB.length - 1])) < SIMPLEX_EPSILON) return;

		// Line Test
		if (sAB.length == 2) {
			constructEntry(sAB[0], sAB[1], sA[0], sB[0], sA[1], sB[1]);
		}
		// Triangle Test
		else pointTriangle();

		if (penetrate) return;
	}
}

///
private void pointTriangle()
{
	int i = sAB.length - 1;
	bzVec2 ab = sAB[i - 1] - sAB[sAB.length - 1];
	bzVec2 ac = sAB[i - 2] - sAB[sAB.length - 1];
	bzVec2 ao = -sAB[sAB.length - 1];

	// Origin in vertex region outside A
	float d1 = ab.bzDot(ao);
	float d2 = ac.bzDot(ao);
	if (d1 <= 0.0f && d2 <= 0.0f) {
		constructEntry(sAB[i], sAB[sAB.length - 1], sA[i], sB[i], sA[sA.length - 1], sB[sB.length - 1]);
		return;
	}

	// Origin in edge region outside AB
	bzVec2 bo = -sAB[i - 1];
	float d3 = ab.bzDot(bo);
	float d4 = ac.bzDot(bo);
	float vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
		constructEntry(sAB[i - 1], sAB[sAB.length - 1], sA[i - 1], sB[i - 1], sA[sA.length - 1], sB[sB.length - 1]);
		return;
	}

	// Origin in edge region ouside AC
	bzVec2 co = -sAB[i - 2];
	float d5 = ab.bzDot(co);
	float d6 = ac.bzDot(co);
	float vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
		return constructEntry(sAB[i - 2], sAB[sAB.length - 1], sA[i - 2], sB[i - 2], sA[sA.length - 1], sB[sB.length - 1]);
		return;
	}

	// Origin inside face region. Penetration!!!
	penetrate = true;
	return;
}

/// bzContact infromation container
private void constructEntry(bzVec2 A, bzVec2 B, bzVec2 p0, bzVec2 q0, bzVec2 p1, bzVec2 q1)
{
	e.y0 = A;
	e.y1 = B;

	e.p0 = p0;
	e.p1 = p1;

	e.q0 = q0;
	e.q1 = q1;

	bzVec2 ab = B - A;
	float t = -A.bzDot(ab);

	if (t <= 0.0f) {
		t = 0.0f;
		e.v = A;
	} else {
		float denom = ab.bzDot(ab);
		if (t >= denom) {
			e.v = B;
			t = 1.0f;
		} else {
			t /= denom;
			e.v = A + t * ab;
		}
	}

	e.s = 1 - t;
	e.t = t;
	e.range = e.v.magnitude;
}
}
*/
