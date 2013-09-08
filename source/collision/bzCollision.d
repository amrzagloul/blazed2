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

// Documentation modified by Brian Schott (Sir Alaran)

module blaze.collision.bzCollision;

import blaze.common.bzMath;
import blaze.common.bzConstants;

const k_nullFeature = uint.max;

/**
 * bzContact point ID
 */
union bzContactID {
    /**
	 *bzContact features
	 */
    struct Features {
        int referenceEdge;
        int incidentEdge;
        int incidentVertex;
        int flip;
    }

    Features features;

    int key;
}

/**
 * A manifold point is a contact point belonging to a contact
 * manifold. It holds details related to the geometry and dynamics
 * of the contact points.
 * The point is stored in local coordinates because CCD
 * requires sub-stepping in which the separation is stale.
 */
struct bzManifoldPoint {
    /// local position of the contact point in body1
    bzVec2 localPoint1;
	/// local position of the contact point in body2
    bzVec2 localPoint2;
	/// the separation of the shapes along the normal vector
    float separation = 0;
	/// the non-penetration impulse
    float normalImpulse = 0;
    /// the friction impulse
	float tangentImpulse = 0;
    /// uniquely identifies a contact point between two shapes
    bzContactID id;
}

/**
 * A manifold for two touching convex shapes.
 */
struct bzManifold {
    /// the points of contact
    bzManifoldPoint points[k_maxManifoldPoints];
	/// the shared unit normal vector
    bzVec2 normal;
	/// the number of manifold points
    int pointCount;
}

/**
 * An axis aligned bounding box.
 */
struct bzAABB {
	/// the lower vertex (lower left)
    bzVec2 lowerBound;
	/// the upper vertex (upper right)
    bzVec2 upperBound;

    bool isValid() {
        bzVec2 d = upperBound - lowerBound;
        bool valid = d.x >= 0.0f && d.y >= 0.0f;
        valid = valid && lowerBound.isValid() && upperBound.isValid();
        return valid;
    }

}

/**
 * An oriented bounding box.
 */
struct bzOBB {
    /// the rotation matrix
    bzMat22 R;
	/// the local centroid
    bzVec2 center;
	/// the half-widths.
    bzVec2 extents;
}

/**
 * Test if two bzAABB's overlap
 * Returns: true if there is an overlap. false otherwise
 */
bool testOverlap(bzAABB a, bzAABB b)
{
	bzVec2 d1, d2;
	d1 = b.lowerBound - a.upperBound;
	d2 = a.lowerBound - b.upperBound;

	if (d1.x > 0.0f || d1.y > 0.0f)
		return false;

	if (d2.x > 0.0f || d2.y > 0.0f)
		return false;

	return true;
}

/**
 * Test if bzAABB contains the given point
 * Params:
 *     a = the box
 *     point = the point
 * Returns: true if there is an intersection, false otherwise
 */
bool testOverlap(bzAABB a, bzVec2 point) {
    // Using epsilon to try and gaurd against float rounding errors.
    if ((point.x < (a.upperBound.x + float.epsilon) && point.x > (a.lowerBound.x - float.epsilon)
            && (point.y < (a.upperBound.y + float.epsilon) && point.y > (a.lowerBound.y - float.epsilon)))) {
        return true;
    } else {
        return false;
    }

}

/**
 * A line segment.
 */
struct bzSegment {

    /// the starting point
    bzVec2 p1;
	/// the ending point
    bzVec2 p2;

    // Ray cast against this segment with another segment.
    // Collision Detection in Interactive 3D Environments by Gino van den Bergen
    // From Section 3.4.1
    // x = mu1 * p1 + mu2 * p2
    // mu1 + mu2 = 1 && mu1 >= 0 && mu2 >= 0
    // mu1 = 1 - mu2;
    // x = (1 - mu2) * p1 + mu2 * p2
    //   = p1 + mu2 * (p2 - p1)
    // x = s + a * r (s := start, r := end - start)
    // s + a * r = p1 + mu2 * d (d := p2 - p1)
    // -a * r + mu2 * d = b (b := s - p1)
    // [-r d] * [a; mu2] = b
    // Cramer's rule:
    // denom = det[-r d]
    // a = det[b d] / denom
    // mu2 = det[-r b] / denom

	/**
	 * Tests to determine if a ray intersects the given segment
	 * Params:
	 *     lambda = unknown
	 *     normal = the ray
	 *     segment = the segment
	 *     maxLambda = unknown
	 * Returns: true if the ray intersects the segment, false otherwise
	 */
    bool testSegment(ref float lambda, ref bzVec2 normal, bzSegment segment,
		float maxLambda)
	{
        bzVec2 s = segment.p1;
        bzVec2 r = segment.p2 - s;
        bzVec2 d = p2 - p1;
        bzVec2 n = bzCross(d, 1.0f);

        float k_slop = 100.0f * float.epsilon;
        float denom = -bzDot(r, n);

        // Cull back facing collision and ignore parallel segments.
        if (denom > k_slop) {
            // Does the segment intersect the infinite line associated with this
			// segment?
            bzVec2 b = s - p1;
            float a = bzDot(b, n);

            if (0.0f <= a && a <= maxLambda * denom) {
                float mu2 = -r.x * b.y + r.y * b.x;

                // Does the segment intersect this segment?
                if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0f + k_slop)) {
                    a /= denom;
                    n.normalize();
                    lambda = a;
                    normal = n;
                    return true;
                }
            }
        }

        return false;
    }
}

