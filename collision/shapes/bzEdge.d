/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
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
module blaze.collision.shapes.bzEdge;

import blaze.common.bzMath;
import blaze.common.bzConstants;
import blaze.collision.shapes.bzShape;
import blaze.collision.shapes.bzShapeType;
import blaze.collision.bzCollision;

/// This class is used to build edge shapes.
class bzEdgeDef : bzShapeDef
{
	this()
	{
		type = bzShapeType.EDGE;
		isALoop = true;
	}
	/// The vertices in local coordinates. You must manage the memory
	/// of this array on your own, outside of Box2D.
	bzVec2[] vertices;

	/// Whether to create an extra edge between the first and last vertices:
	bool isALoop;
}

/// An edge shape.
class bzEdge : bzShape
{
public:

    this(bzVec2 v1, bzVec2 v2, bzShapeDef def) {
	
		super();
        assert(def.type == bzShapeType.EDGE);

        m_type = bzShapeType.EDGE;

        m_prevEdge = null;
        m_nextEdge = null;

        m_v1 = v1;
        m_v2 = v2;

        m_direction = m_v2 - m_v1;
        m_length = m_direction.normalize();
        m_normal.set(m_direction.y, -m_direction.x);

        m_coreV1 = -k_toiSlop * (m_normal - m_direction) + m_v1;
        m_coreV2 = -k_toiSlop * (m_normal + m_direction) + m_v2;

        m_cornerDir1 = m_normal;
        m_cornerDir2 = -1.0f * m_normal;
    }

	/// @see b2Shape::TestPoint
	override bool testPoint(bzXForm transform, bzVec2 p) {
        //B2_NOT_USED(transform);
        //B2_NOT_USED(p);
        return false;
    }

	/// @see b2Shape::TestSegment
	override bzSegmentCollide testSegment(bzXForm transform, ref float lambda, ref bzVec2 normal, bzSegment segment,
							   float maxLambda) {

        bzVec2 r = segment.p2 - segment.p1;
        bzVec2 v1 = bzMul(transform, m_v1);
        bzVec2 d = bzMul(transform, m_v2) - v1;
        bzVec2 n = bzCross(d, 1.0f);

        const float k_slop = 100.0f * float.epsilon;
        float denom = -bzDot(r, n);

        // Cull back facing collision and ignore parallel segments.
        if (denom > k_slop)
        {
            // Does the segment intersect the infinite line associated with this segment?
            bzVec2 b = segment.p1 - v1;
            float a = bzDot(b, n);

            if (0.0f <= a && a <= maxLambda * denom)
            {
                float mu2 = -r.x * b.y + r.y * b.x;

                // Does the segment intersect this segment?
                if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0f + k_slop))
                {
                    a /= denom;
                    n.normalize();
                    lambda = a;
                    normal = n;
                    return bzSegmentCollide.HIT;
                }
            }
        }

        return bzSegmentCollide.MISS;
    }

	override void computeAABB(ref bzAABB aabb, bzXForm transform) {
        bzVec2 v1 = bzMul(transform, m_v1);
        bzVec2 v2 = bzMul(transform, m_v2);
        aabb.lowerBound = bzMin(v1, v2);
        aabb.upperBound = bzMax(v1, v2);
    }

    override void computeSweptAABB(ref bzAABB aabb, bzXForm transform1, bzXForm transform2) {
        bzVec2 v1 = bzMul(transform1, m_v1);
        bzVec2 v2 = bzMul(transform1, m_v2);
        bzVec2 v3 = bzMul(transform2, m_v1);
        bzVec2 v4 = bzMul(transform2, m_v2);
        aabb.lowerBound = bzMin(bzMin(bzMin(v1, v2), v3), v4);
        aabb.upperBound = bzMax(bzMax(bzMax(v1, v2), v3), v4);
    }

    override void computeMass(ref bzMassData massData) {
        massData.mass = 0;
        massData.center = m_v1;
        // inertia about the local origin
        massData.I = 0;
    }

    override bzVec2 worldCenter() {
        return bzVec2(0,0);
    }

	/// @warning This only gives a consistent and sensible answer when when summed over a body only contains loops of edges
	/// @see b2Shape::ComputeSubmergedArea
	float computeSubmergedArea(bzVec2 normal, float offset, bzXForm xf, ref bzVec2 c) {
        //Note that v0 is independant of any details of the specific edge
        //We are relying on v0 being consistent between multiple edges of the same body
        bzVec2 v0 = offset * normal;
        //bzVec2 v0 = xf.position + (offset - bzDot(normal, xf.position)) * normal;

        bzVec2 v1 = bzMul(xf, m_v1);
        bzVec2 v2 = bzMul(xf, m_v2);

        float d1 = bzDot(normal, v1) - offset;
        float d2 = bzDot(normal, v2) - offset;

        if(d1>0)
        {
            if(d2>0)
            {
                return 0;
            }
            else
            {
                v1 = -d2 / (d1 - d2) * v1 + d1 / (d1 - d2) * v2;
            }
        }
        else
        {
            if(d2>0)
            {
                v2 = -d2 / (d1 - d2) * v1 + d1 / (d1 - d2) * v2;
            }
            else
            {
                //Nothing
            }
        }

        // v0,v1,v2 represents a fully submerged triangle
        float k_inv3 = 1.0f / 3.0f;

        // Area weighted centroid
        c = k_inv3 * (v0 + v1 + v2);

        bzVec2 e1 = v1 - v0;
        bzVec2 e2 = v2 - v0;

        return 0.5f * bzCross(e1, e2);
    }

    /// Linear distance from vertex1 to vertex2:
	float length() {
        return m_length;
    }

    /// Local position of vertex in parent body
    bzVec2 vertex1() @property {
        return m_v1;
    }

    /// Local position of vertex in parent body
    bzVec2 vertex2() @property {
        return m_v2;
    }

    /// "Core" vertex with TOI slop for b2Distance functions:
    bzVec2 coreVertex1() @property {
        return m_coreV1;
    }

    /// "Core" vertex with TOI slop for b2Distance functions:
    bzVec2 coreVertex2() @property {
        return m_coreV2;
    }

    /// Perpendicular unit vector point, pointing from the solid side to the empty side:
    bzVec2 normalVector() @property {
        return m_normal;
    }

    /// Parallel unit vector, pointing from vertex1 to vertex2:
    bzVec2 directionVector() @property {
        return m_direction;
    }

    bzVec2 corner1Vector() @property {
        return m_cornerDir1;
    }

    bzVec2 corner2Vector() @property {
        return m_cornerDir2;
    }

	bool corner1IsConvex() @property {
        return m_cornerConvex1;
    }

	bool corner2IsConvex() @property {
        return m_cornerConvex2;
    }

	bzVec2 firstVertex(bzXForm xf) {
        return bzMul(xf, m_coreV1);
    }

	override bzVec2 support(bzXForm xf, bzVec2 d) {
        bzVec2 v1 = bzMul(xf, m_coreV1);
        bzVec2 v2 = bzMul(xf, m_coreV2);
        return bzDot(v1, d) > bzDot(v2, d) ? v1 : v2;
    }

    /// Get the next edge in the chain.
    bzEdge nextEdge() {
        return m_nextEdge;
    }

    /// Get the previous edge in the chain.
    bzEdge prevEdge() {
        return m_prevEdge;
    }

	void setPrevEdge(bzEdge edge, bzVec2 core, bzVec2 cornerDir, bool convex) {
        m_prevEdge = edge;
        m_coreV1 = core;
        m_cornerDir1 = cornerDir;
        m_cornerConvex1 = convex;
    }

	void setNextEdge(bzEdge edge, bzVec2 core, bzVec2 cornerDir, bool convex) {
        m_nextEdge = edge;
        m_coreV2 = core;
        m_cornerDir2 = cornerDir;
        m_cornerConvex2 = convex;
    }

    // Update the sweep radius (maximum radius) as measured from
    // a local center point.
    override void updateSweepRadius(bzVec2 center) {
        bzVec2 d = m_coreV1 - center;
        float d1 = bzDot(d,d);
        d = m_coreV2 - center;
        float d2 = bzDot(d,d);
        m_sweepRadius = sqrt(d1 > d2 ? d1 : d2);
    }

private:

	bzVec2 m_v1;
	bzVec2 m_v2;

	bzVec2 m_coreV1;
	bzVec2 m_coreV2;

	float m_length = 0;

	bzVec2 m_normal;

	bzVec2 m_direction;

	// Unit vector halfway between m_direction and m_prevEdge.m_direction:
	bzVec2 m_cornerDir1;

	// Unit vector halfway between m_direction and m_nextEdge.m_direction:
	bzVec2 m_cornerDir2;

	bool m_cornerConvex1;
	bool m_cornerConvex2;

	bzEdge m_nextEdge;
	bzEdge m_prevEdge;
}
