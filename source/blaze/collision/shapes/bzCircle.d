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

// Documentation comments modified by Brian Schott (SirAlaran)

module blaze.collision.shapes.bzCircle;

import blaze.common.bzMath;
import blaze.common.bzConstants;
import blaze.collision.shapes.bzShape;
import blaze.collision.shapes.bzShapeType;
import blaze.collision.bzCollision;

/**
 * This is used to build circle shapes.
 */
class bzCircleDef : bzShapeDef
{
	this(float density = 0.0f, float radius = 0.0f)
	{
		type = bzShapeType.CIRCLE;
		localPosition.zero();
		this.radius = radius;
		super.density = density;
	}

	bzVec2 localPosition;
	float radius = 0.0f;
}

/**
 * A circle shape.
 */
class bzCircle : bzShape {

	/**
	 * Constructs a circle from a shape definition
	 */
    this (bzShapeDef def) {
        assert(def.type == bzShapeType.CIRCLE);
        super(def);
        bzCircleDef circleDef = cast(bzCircleDef) def;
        m_type = bzShapeType.CIRCLE;
        m_localPosition = circleDef.localPosition;
        m_radius = circleDef.radius;
		area = PI * m_radius * m_radius;
    }

    /**
     * Test a point for containment in this shape. This only works for convex
	 * shapes.
     * Params:
	 *     xf = the shape world transform.
     *     p = a point in world coordinates.
	 * Returns: true if the point lies within the shape, false otherwise.
     */
    override bool testPoint(bzXForm xf, bzVec2 p) {
        bzVec2 center = xf.position + bzMul(xf.R, m_localPosition);
        bzVec2 d = p - center;
        return bzDot(d, d) <= m_radius * m_radius;
    }

	/**
	 * Returns: the shape's center in world coordinates
	 */
	override bzVec2 worldCenter() {
		bzXForm xf = rBody.xf;
		return bzMul(xf, m_localPosition);
	}

	/**
     * Perform a ray cast against this shape. Collision Detection in Interactive
	 * 3D Environments by Gino van den Bergen From Section 3.1.2 x = s + a * r
     * norm(x) = radius
	 * Params:
     *     xf = the shape world transform.
     *     lambda = returns the hit fraction. You can use this to compute the
	 *         contact point p = (1 - lambda) * segment.p1 + lambda * segment.p2
     *     normal = returns the normal at the contact point. If there is no
	 *         intersection, the normal is not set.
     *     segment = defines the begin and end point of the ray cast.
     *     maxLambda = a number typically in the range [0,1].
	 * Returns: a bzSegmentCollide representing the collision between this shape
	 * and the segment
     */
    override bzSegmentCollide testSegment(bzXForm xf, ref float lambda, ref bzVec2 normal, bzSegment segment, float maxLambda)  {
        bzVec2 position = xf.position + bzMul(xf.R, m_localPosition);
        bzVec2 s = segment.p1 - position;
        float b = bzDot(s, s) - m_radius * m_radius;

        // Does the segment start inside the circle?
        if (b < 0.0f) {
            lambda = 0;
            return bzSegmentCollide.STARTS_INSIDE;
        }

        // Solve quadratic equation.
        bzVec2 r = segment.p2 - segment.p1;
        float c =  bzDot(s, r);
        float rr = bzDot(r, r);
        float sigma = c * c - rr * b;

        // Check for negative discriminant and short segment.
        if (sigma < 0.0f || rr < float.epsilon) {
            return bzSegmentCollide.MISS;
        }

        // Find the point of intersection of the line with the circle.
        float a = -(c + sqrt(sigma));

        // Is the intersection point on the segment?
        if (0.0f <= a && a <= maxLambda * rr) {
            a /= rr;
            lambda = a;
            normal = s + a * r;
            normal.normalize();
            return bzSegmentCollide.HIT;
        }

        return bzSegmentCollide.MISS;
    }

	/**
	 * Given a transform, compute the associated axis aligned bounding box for
	 *     this shape.
	 * This does not modify the shape's bzAABB
     * Params:
	 *     aabb = returns the axis aligned box.
     *     xf = the world transform of the shape.
	 */
    override void computeAABB(ref bzAABB aabb, bzXForm xf)
    {
        bzVec2 p = xf.position + bzMul(xf.R, m_localPosition);
        aabb.lowerBound.set(p.x - m_radius, p.y - m_radius);
        aabb.upperBound.set(p.x + m_radius, p.y + m_radius);

        xmax = aabb.upperBound.x;
        xmin = aabb.lowerBound.x;
        ymax = aabb.upperBound.y;
        ymin = aabb.lowerBound.y;
    }

	/**
	 * Given two transforms, compute the associated swept axis aligned bounding
	 *     box for this shape. This DOES modify the shape's bzAABB (see bugs)
     * Params:
     *     xf1 = the world transform of the shape.
	 *     xf2 = the velocity of the shape
	 */
    override void computeSweptAABB(ref bzAABB aabb, bzXForm xf1, bzXForm xf2)
    {
        bzVec2 p1 = xf1.position + bzMul(xf1.R, m_localPosition);
        bzVec2 p2 = xf2.position + bzMul(xf2.R, m_localPosition);
        bzVec2 lower = bzMin(p1, p2);
        bzVec2 upper = bzMax(p1, p2);
        aabb.lowerBound.set(lower.x - m_radius, lower.y - m_radius);
        aabb.upperBound.set(upper.x + m_radius, upper.y + m_radius);

        xmax = aabb.upperBound.x;
        xmin = aabb.lowerBound.x;
        ymax = aabb.upperBound.y;
        ymin = aabb.lowerBound.y;
    }

	/**
	 * Compute the mass properties of this shape using its dimensions and density.
     * The inertia tensor is computed about the local origin, not the centroid.
     * Params: massData = returns the mass data for this shape.
	 */
    override void computeMass(ref bzMassData massData)
    {
        massData.mass = m_density * PI * m_radius * m_radius;
        massData.center = m_localPosition;
        // inertia about the local origin
        massData.I = massData.mass * (0.5f * m_radius * m_radius + bzDot(m_localPosition, m_localPosition));
    }

    /**
	 * Returns: the local position of this circle in its parent body.
	 */
    bzVec2 localPosition() {
        return m_localPosition;
    }

    /**
	 * Returns: the radius of this circle.
	 */
    float radius() @property {
        return m_radius;
    }

	/**
	 * Returns: The shape's support point (for MPR & GJK)
	 */
    override bzVec2 support(bzXForm xf, bzVec2 d) {
        d.normalize();
        bzVec2 r = m_radius * d;
        r += worldCenter();
        return r;
    }

protected:

    override void updateSweepRadius(bzVec2 center) {
        // Update the sweep radius (maximum radius) as measured from
        // a local center point.
        bzVec2 d = m_localPosition - center;
        m_sweepRadius = d.length() + m_radius - k_toiSlop;
    }

    // Local position in parent body
    bzVec2 m_localPosition;
    float m_radius = 0.0f;
};
