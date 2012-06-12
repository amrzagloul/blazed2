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
module blaze.collision.pairwise.bzMpr;

import blaze.common.bzMath;
import blaze.collision.shapes.bzPolygon;
import blaze.dynamics.contact.bzContact;
import blaze.collision.shapes.bzShape;

// Simplex tollerance
const SIMPLEX_EPSILON = 0.01f;

/**
 * Unknown
 */
bzVec2 insidePortal(bzVec2 v1, bzVec2 v2)
{
	// Perp-bzDot product
	float dir = v1.x * v2.y - v1.y * v2.x;

	if (dir > float.epsilon) return bzVec2(v1.x - v2.x, v1.y - v2.y).rotateLeft90;
	else return bzVec2(v1.x - v2.x, v1.y - v2.y).rotateRight90;
}

/**
 * Unknown
 */
bzVec2 outsidePortal(bzVec2 v1, bzVec2 v2)
{
	// Perp-bzDot product
	float dir = v1.x * v2.y - v1.y * v2.x;

	if (dir < float.epsilon) return bzVec2(v1.x - v2.x, v1.y - v2.y).rotateLeft90;
	else return bzVec2(v1.x - v2.x, v1.y - v2.y).rotateRight90;
}

/**
 * Params:
 *     a = the first point
 *     b = the second point
 *     c = the third point
 * Returns: true if the origin (0, 0) lies in the specified triangle
 */
bool originInTriangle(bzVec2 a, bzVec2 b, bzVec2 c)
{
	bzVec2 ab = b - a;
	bzVec2 bc = c - b;
	bzVec2 ca = a - c;

	float pab = (-a).bzCross(ab);
	float pbc = (-b).bzCross(bc);
	bool sameSign = (((pab > 0) - (pab < 0)) == ((pbc > 0) - (pbc < 0)));
	if (!sameSign) return false;

	float pca = (-c).bzCross(ca);
	sameSign = (((pab > 0) - (pab < 0)) == ((pca > 0) - (pca < 0)));
	if (!sameSign) return false;

	return true;
}

/**
 * Unknown
 */
bool intersectPortal(bzVec2 v0, bzVec2 v1, bzVec2 v2)
{
	bzVec2 a = bzVec2(0, 0);
	bzVec2 b = v0;
	bzVec2 c = v1;
	bzVec2 d = v2;

	float a1 = (a.x - d.x) * (b.y - d.y) - (a.y - d.y) * (b.x - d.x);
	float a2 = (a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x);

	if (a1 != 0.0f && a2 != 0.0f && a1 * a2 < 0.0f) {
		float a3 = (c.x - a.x) * (d.y - a.y) - (c.y - a.y) * (d.x - a.x);
		float a4 = a3 + a2 - a1;
		if (a3 != 0.0f && a4 != 0.0f && a3 * a4 < 0.0f) return true;
	}

	// Segments not intersecting (or collinear)
	return false;
}

/**
 * Unknown
 */
bool collideAndFindPoint(bzShape shape1, bzShape shape2, ref bzContact[] contacts)
{
	// Phase one: Portal discovery

	bzVec2 center1 = shape1.worldCenter();
	bzVec2 center2 = shape2.worldCenter();

    bzXForm x1 = shape1.rBody.xf;
    bzXForm x2 = shape2.rBody.xf;

	// v0 = center of Minkowski sum
	bzVec2 v01 = center1;
	bzVec2 v02 = center2;
	bzVec2 v0 = v02 - v01;

	// Avoid case where centers overlap -- any direction is fine in this case
	if (v0.equalsZero()) v0 = bzVec2(0.00001f, 0);

	// v1 = support in direction of origin
	bzVec2 n = -v0;
	bzVec2 v11 = shape1.support(x1, -n);
	bzVec2 v12 = shape2.support(x2, n);
	bzVec2 v1 = v12 - v11;

	// origin outside v1 support plane ==> miss
	if (v1.bzDot(n) <= 0) return false;

	// Find a candidate portal
	n = outsidePortal(v1, v0);
	bzVec2 v21 = shape1.support(x1, -n);
	bzVec2 v22 = shape2.support(x2, n);
	bzVec2 v2 = v22 - v21;

	// origin outside v2 support plane ==> miss
	if (v2.bzDot(n) <= 0) return false;

	// Phase two: Portal refinement

	int maxIterations;
	while (1) {
		// Find normal direction
		if (!intersectPortal(v0, v2, v1)) {
			// Origin lies inside the portal
			n = insidePortal(v2, v1);
		} else {
			// Origin lies outside the portal
			n = outsidePortal(v2, v1);
		}

		// Obtain the next support point

		bzVec2 v31 = shape1.support(x1, -n);
		bzVec2 v32 = shape2.support(x2, n);
		bzVec2 v3 = v32 - v31;

		if (v3.bzDot(n) <= 0) return false;

		// Finished searching
		if ((v3 - v2).bzDot(n) <= SIMPLEX_EPSILON || maxIterations++ > 10) {
			bzVec2 normal;
			bzVec2 ab = v2 - v1;
			float t = -v1.bzDot(ab);

			if (t <= 0.0f) {
				t = 0.0f;
				normal = v1;
			} else {
				float denom = ab.bzDot(ab);
				if (t >= denom) {
					normal = v2;
					t = 1.0f;
				} else {
					t /= denom;
					normal = v1 + t * ab;
				}
			}

			float s = 1 - t;

			bzVec2 point1 = s * v11 + t * v21;
			bzVec2 point2 = s * v12 + t * v22;

            // Penetration depth
			float pDepth = normal.normalize();

			return true;
		}

		// If origin is inside (v1,v0,v3), We have a hit!
		if (originInTriangle(v0, v1, v3)) {
			v2 = v3;
			v21 = v31;
			v22 = v32;
			continue;
		}
		// If origin is inside (v3,v0,v2), we have a hit!
		else if (originInTriangle(v0, v2, v3)) {
			v1 = v3;
			v11 = v31;
			v12 = v32;
			continue;
		}

		return false;
	}
}

