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

// Documentation comments altered quite a bit by Brian Schott (SirAlaran)

module blaze.collision.shapes.bzPolygon;

import blaze.common.bzMath;
import blaze.common.bzConstants;
import blaze.collision.shapes.bzShape;
import blaze.collision.shapes.bzShapeType;
import blaze.collision.bzCollision;

/**
 * Convex polygon. The vertices must be in CCW order for a right-handed
 * coordinate system with the z-axis coming out of the screen.
 * Implements: bzShapeDef
 */
class bzPolyDef : bzShapeDef {

    /** The polygon vertices in local coordinates. */
    bzVec2[] vertices;

    /**
	 * Params:
	 *     density = the density of the polygon
	 *     verticies = the verticies that make up the polygon
	 */
	this(float density = 0.0f, bzVec2[] vertices = null) {
        super.density = density;
        this.vertices = vertices.dup;
        type = bzShapeType.POLYGON;
    }

    /**
     * Build vertices to represent an axis-aligned box.
     * Params:
	 *     hx = the half-width.
     *     hy = the half-height.
     */
    override void setAsBox(float hx, float hy) {
        vertices.length = 4;
        vertices[0].set(-hx, -hy);
        vertices[1].set( hx, -hy);
        vertices[2].set( hx,  hy);
        vertices[3].set(-hx,  hy);
    }

    /**
     * Build vertices to represent an oriented box.
     * Params:
	 *     hx = the half-width.
	 *     hy the = half-height.
	 *     center = the center of the box in local coordinates.
	 *     angle = the rotation of the box in local coordinates.
     */
    void setAsBox(float hx, float hy, bzVec2 center, float angle) {
        setAsBox(hx, hy);
        bzXForm xf;
        xf.position = center;
        xf.R.set(angle);

        for (int i = 0; i < vertices.length; ++i) {
            vertices[i] = bzMul(xf, vertices[i]);
        }
    }
}

/**
 * A polygon
 * Implements: bzShape
 */
class bzPolygon : bzShape {

    this (bzShapeDef def) {
        assert(def.type == bzShapeType.POLYGON);
        super(def);
        m_type = bzShapeType.POLYGON;
        bzPolyDef poly = cast (bzPolyDef) def;

        // Get the vertices transformed into the body frame.
        m_vertexCount = poly.vertices.length;
        assert(3 <= m_vertexCount && m_vertexCount <= k_maxPolygonVertices);

        // Copy vertices.
        m_vertices = poly.vertices.dup;
        m_normals.length = m_vertexCount;
        m_coreVertices.length = m_vertexCount;

        // Compute normals. Ensure the edges have non-zero length.
        for (int i = 0; i < m_vertexCount; ++i) {
            int i1 = i;
            int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
            bzVec2 edge = m_vertices[i2] - m_vertices[i1];
            assert(edge.lengthSquared() > float.epsilon * float.epsilon);
            m_normals[i] = bzCross(edge, 1.0f);
            m_normals[i].normalize();
        }

        if(m_debug)
        {
            // Ensure the polygon is convex.
	    	for (int i = 0; i < m_vertexCount; ++i) {
	    		for (int j = 0; j < m_vertexCount; ++j) {
	    			// Don't check vertices on the current edge.
	    			if (j == i || j == (i + 1) % m_vertexCount)
	    			{
	    				continue;
	    			}

	    			// Your polygon is non-convex (it has an indentation).
	    			// Or your polygon is too skinny.
	    			float s = bzDot(m_normals[i], m_vertices[j] - m_vertices[i]);
	    			if(s > -k_linearSlop) {
	    			    string e = "Polygon is non-conved, or too skinny";
	    			    throw new Exception(e);
	    			}
	    		}
	    	}

	    	// Ensure the polygon is counter-clockwise.
	    	for (int i = 1; i < m_vertexCount; ++i) {
	    		float cross = bzCross(m_normals[i-1], m_normals[i]);

	    		// Keep asinf happy.
	    		cross = bzClamp(cross, -1.0f, 1.0f);

	    		// You have consecutive edges that are almost parallel on your polygon.
	    		// Or the polygon is not counter-clockwise.
	    		float angle = asin(cross);
	    		if(angle < k_angularSlop) {
	    		    string e = "Polygon edges are almost parallel, or vertices are not counter-clockwise";
                    throw new Exception(e);
	    		}
	    	}
        }

        // Compute the polygon centroid.
        m_centroid = computeCentroid(poly.vertices);

        // Compute the oriented bounding box.
        computeOBB(m_obb, m_vertices);

        // Create core polygon shape by shifting edges inward.
        // Also compute the min/max radius for CCD.
        for (int i = 0; i < m_vertexCount; ++i) {
            int i1 = i - 1 >= 0 ? i - 1 : m_vertexCount - 1;
            int i2 = i;

            bzVec2 n1 = m_normals[i1];
            bzVec2 n2 = m_normals[i2];
            bzVec2 v = m_vertices[i] - m_centroid;

            bzVec2 d;
            d.x = bzDot(n1, v) - k_toiSlop;
            d.y = bzDot(n2, v) - k_toiSlop;

            // Shifting the edge inward by _toiSlop should
            // not cause the plane to pass the centroid.

            // Your shape has a radius/extent less than _toiSlop.
            assert(d.x >= 0.0f);
            assert(d.y >= 0.0f);
            bzMat22 A;
            A.col1.x = n1.x;
            A.col2.x = n1.y;
            A.col1.y = n2.x;
            A.col2.y = n2.y;
            m_coreVertices[i] = A.solve(d) + m_centroid;
        }
    }

    /**
	 * Returns: the first vertex and apply the supplied transform.
	 */
    bzVec2 firstVertex(bzXForm xf) {
        return bzMul(xf, m_coreVertices[0]);
    }

    /**
	 * Returns: the oriented bounding box relative to the parent body.
	 */
    bzOBB obb() {
        return m_obb;
    }

    /**
	 * Returns: the local centroid relative to the parent body.
	 */
    bzVec2 centroid() {
        return m_centroid;
    }

    /**
	 * Returns: the centroid and apply the supplied transform.
	 */
    bzVec2 centroid(bzXForm xf) {
        return bzMul(xf, m_centroid);
    }

    /**
	 * Returns: the vertex count
	 */
    int vertexCount() {
        return m_vertexCount;
    }

    /**
	 * Returns: the vertices in local coordinates.
	 */
    bzVec2[] vertices() @property {
        return m_vertices;
    }

    /**
	 * Returns: the vertices in world coordinates
	 */
    bzVec2[] worldVertices() {
        bzVec2[] worldVertices;
        worldVertices.length = m_vertices.length;
        bzXForm xf = rBody.xf;
        for (int i = 0; i < m_vertices.length; i++) {
            worldVertices[i] = bzMul(xf, m_vertices[i]);
        }
        return worldVertices;
    }

	/**
	 * Returns: The polygon's centroid in world coordinates
	 */
	override bzVec2 worldCenter() {
		bzXForm xf = rBody.xf;
		return bzMul(xf, m_centroid);
	}

    /**
     * Returns: The core vertices in local coordinates. These vertices
     * represent a smaller polygon that is used for time of impact
     * computations.
     */
    bzVec2[] coreVertices() {
        return m_coreVertices;
    }

    /**
	 * Returns: The edge normal vectors. There is one for each vertex.
	 */
    bzVec2[] normals() @property {
        return m_normals;
    }

	/**
     * Update the sweep radius (maximum radius) as measured from
     * a local center point.
	 * Params: center = the center point to measure from
	 */
    override void updateSweepRadius(bzVec2 center) {
        m_sweepRadius = 0.0f;
        for (int i = 0; i < m_vertexCount; ++i) {
            bzVec2 d = m_coreVertices[i] - center;
            m_sweepRadius = max(m_sweepRadius, d.length());
        }
    }

	/**
     * Test a point for containment in this shape. This only works for convex
	 * shapes.
     * Params:
	 *     xf = the shape world transform.
     *     p = a point in world coordinates.
	 * Returns: true if the point lies within the shape, false otherwise.
	 * Implements: blaze.collision.shapes.bzShape.bzShape.testPoint
     */
    override bool testPoint(bzXForm xf, bzVec2 p) {

        bzVec2 pLocal = bzMulT(xf.R, p - xf.position);
        for (int i = 0; i < m_vertexCount; ++i) {
            float bzDot = bzDot(m_normals[i], pLocal - m_vertices[i]);
            if (bzDot > 0.0f) {
                return false;
            }
        }
        return true;
    }

	/**
     * Perform a ray cast against this shape.
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
	 * Implements: blaze.collision.shapes.bzShape.bzShape.testSegment
     */
    override bzSegmentCollide testSegment(bzXForm xf, ref float lambda,
		ref bzVec2 normal, bzSegment segment, float maxLambda) {

        float lower = 0.0f;
        float upper = maxLambda;

        bzVec2 p1 = bzMulT(xf.R, segment.p1 - xf.position);
        bzVec2 p2 = bzMulT(xf.R, segment.p2 - xf.position);
        bzVec2 d = p2 - p1;
        int index = -1;

        for (int i = 0; i < m_vertexCount; ++i) {
            // p = p1 + a * d
            // bzDot(normal, p - v) = 0
            // bzDot(normal, p1 - v) + a * bzDot(normal, d) = 0
            float numerator = bzDot(m_normals[i], m_vertices[i] - p1);
            float denominator = bzDot(m_normals[i], d);

            if (denominator == 0.0f) {
                if (numerator < 0.0f) {
                    return bzSegmentCollide.MISS;
                }
            } else {
                // Note: we want this predicate without division:
                // lower < numerator / denominator, where denominator < 0
                // Since denominator < 0, we have to flip the inequality:
                // lower < numerator / denominator <==> denominator * lower > numerator.
                if (denominator < 0.0f && numerator < lower * denominator) {
                    // Increase lower.
                    // The segment enters this half-space.
                    lower = numerator / denominator;
                    index = i;
                } else if (denominator > 0.0f && numerator < upper * denominator) {
                    // Decrease upper.
                    // The segment exits this half-space.
                    upper = numerator / denominator;
                }
            }

            if (upper < lower) {
                return bzSegmentCollide.MISS;
            }
        }

        assert(0.0f <= lower && lower <= maxLambda);

        if (index >= 0) {
            lambda = lower;
            normal = bzMul(xf.R, m_normals[index]);
            return bzSegmentCollide.HIT;
        }

        lambda = 0;
        return bzSegmentCollide.STARTS_INSIDE;
    }

	/**
	 * Given a transform, compute the associated axis aligned bounding box for
	 *     this shape.
	 * This does not modify the shape's bzAABB
     * Params:
	 *     aabb = returns the axis aligned box.
     *     xf = the world transform of the shape.
	 * Implements: blaze.collision.shapes.bzShape.bzShape.computeAABB
	 */
    override void computeAABB(ref bzAABB aabb, bzXForm xf) {
        bzMat22 R = bzMul(xf.R, m_obb.R);
        bzMat22 absR = bzAbs(R);
        bzVec2 h = bzMul(absR, m_obb.extents);
        bzVec2 position = xf.position + bzMul(xf.R, m_obb.center);
        aabb.lowerBound = position - h;
        aabb.upperBound = position + h;
    }

	/**
	 * Given two transforms, compute the associated swept axis aligned bounding
	 *     box for this shape. This DOES modify the shape's bzAABB (see bugs)
     * Params:
     *     xf1 = the world transform of the shape.
	 *     xf2 = the velocity of the shape
	 * Implements: blaze.collision.shapes.bzShape.bzShape.computeSweptAABB
	 */
    override void computeSweptAABB(ref bzAABB aabb, bzXForm xf1, bzXForm xf2) {
        bzAABB aabb1, aabb2;
        computeAABB(aabb1, xf1);
        computeAABB(aabb2, xf2);
        aabb.lowerBound = bzMin(aabb1.lowerBound, aabb2.lowerBound);
        aabb.upperBound = bzMax(aabb1.upperBound, aabb2.upperBound);
    }

	/**
	 * Compute the mass properties of this shape using its dimensions and density.
     * The inertia tensor is computed about the local origin, not the centroid.
     * Params: massData = returns the mass data for this shape.
	 * Implements: blaze.collision.shapes.bzShape.bzShape.computeMass
	 */
    override void computeMass(ref bzMassData massData) {

        // bzPolygon mass, centroid, and inertia.
        // Let rho be the polygon density in mass per unit area.
        // Then:
        // mass = rho * int(dA)
        // centroid.x = (1/mass) * rho * int(x * dA)
        // centroid.y = (1/mass) * rho * int(y * dA)
        // I = rho * int((x*x + y*y) * dA)
        //
        // We can compute these integrals by summing all the integrals
        // for each triangle of the polygon. To evaluate the integral
        // for a single triangle, we make a change of variables to
        // the (u,v) coordinates of the triangle:
        // x = x0 + e1x * u + e2x * v
        // y = y0 + e1y * u + e2y * v
        // where 0 <= u && 0 <= v && u + v <= 1.
        //
        // We integrate u from [0,1-v] and then v from [0,1].
        // We also need to use the Jacobian of the transformation:
        // D = bzCross(e1, e2)
        //
        // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
        //
        // The rest of the derivation is handled by computer algebra.

        assert(m_vertexCount >= 3);

        bzVec2 center;
		area = 0.0f;
        float I = 0.0f;

        // pRef is the reference point for forming triangles.
        // It's location doesn't change the result (except for rounding error).
        bzVec2 pRef;

        const k_inv3 = 1.0f / 3.0f;

        for (int i = 0; i < m_vertexCount; ++i) {
            // Triangle vertices.
            bzVec2 p1 = pRef;
            bzVec2 p2 = m_vertices[i];
            bzVec2 p3 = i + 1 < m_vertexCount ? m_vertices[i+1] : m_vertices[0];

            bzVec2 e1 = p2 - p1;
            bzVec2 e2 = p3 - p1;

            float D = bzCross(e1, e2);

            float triangleArea = 0.5f * D;
            area += triangleArea;

            // Area weighted centroid
            center += triangleArea * k_inv3 * (p1 + p2 + p3);

            float px = p1.x, py = p1.y;
            float ex1 = e1.x, ey1 = e1.y;
            float ex2 = e2.x, ey2 = e2.y;

            float intx2 = k_inv3 * (0.25f * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5f*px*px;
            float inty2 = k_inv3 * (0.25f * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5f*py*py;

            I += D * (intx2 + inty2);
        }

        // Total mass
        massData.mass = m_density * area;

        // Center of mass
        assert(area > float.epsilon);
        center *= 1.0f / area;
        massData.center = center;

        // Inertia tensor relative to the local origin.
        massData.I = m_density * I;
    }

	/**
	 * Returns: The shape's support point (for MPR & GJK)
	 */
    override bzVec2 support(bzXForm xf, bzVec2 d) {

        bzVec2 dLocal = bzMulT(xf.R, d);
        int bestIndex = 0;
        float bestValue = bzDot(m_coreVertices[0], dLocal);
        for (int i = 1; i < m_vertexCount; ++i) {
            float value = bzDot(m_coreVertices[i], dLocal);
            if (value > bestValue) {
                bestIndex = i;
                bestValue = value;
            }
        }

        return bzMul(xf, m_coreVertices[bestIndex]);
    }

    /**
	 * Triangulate the shape in world coordinates.
	 * Currently this is only used by the buoyancy solver.
	 */
	override void triangulate() {
		m_triangleList.length = 0;
        //triangulate by center-point
        bzXForm xf = rBody.xf;
		bzVec2 p1 = bzMul(xf, m_centroid);
		bzVec2[] v = worldVertices();
		int count = v.length;
		for (int i = 0; i < count; ++i) {
			// Triangle vertices.
			bzVec2 p2 = v[i];
			bzVec2 p3 = i + 1 < count ? v[i+1] : v[0];
			m_triangleList ~= bzTri2(p1, p2, p3);
		}
    }

private:

    /// Local position of the polygon centroid.
    bzVec2 m_centroid;

	/// bzPolygon's object bounding box
    bzOBB m_obb;

	/// Vertices
    bzVec2[] m_vertices;

	/// bzEdge normals
    bzVec2[] m_normals;

	/// Core verticies
    bzVec2[] m_coreVertices;

	/// Bugs: Why don't we just use m_verticies.length?
    int m_vertexCount;

    /// Turn this on the debug your shapes
    bool m_debug = true;
}

/**
 * Determines the centroid of a polygon
 * Params: vs = the verticies that define the polygon
 * Returns: the location of the centroid (probably in local coordinates)
 */
bzVec2 computeCentroid(bzVec2[] vs) {

    int count = vs.length;
    assert(count >= 3);

    bzVec2 c;
    float area = 0.0f;

    // pRef is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    bzVec2 pRef;

    const inv3 = 1.0f / 3.0f;

    for (int i = 0; i < count; ++i) {
        // Triangle vertices.
        bzVec2 p1 = pRef;
        bzVec2 p2 = vs[i];
        bzVec2 p3 = i + 1 < count ? vs[i+1] : vs[0];

        bzVec2 e1 = p2 - p1;
        bzVec2 e2 = p3 - p1;

        float D = bzCross(e1, e2);

        float triangleArea = 0.5f * D;
        area += triangleArea;

        // Area weighted centroid
        c += triangleArea * inv3 * (p1 + p2 + p3);
    }

    // Centroid
    assert(area > float.epsilon);
    c *= 1.0f / area;
    return c;
}

/**
 * Computes the object bounding box for a polygon
 * Params:
 *     obb = the bounding box that the result is stored in
 *     vs = the verticies that define the polygon
 * See_also: www.geometrictools.com/Documentation/minNumimumAreaRectangle.pdf
 */
void computeOBB(ref bzOBB obb, bzVec2[] vs) {

    bzVec2[] p;
    p = vs.dup;
    p ~= p[0];

    float minArea = float.max;

    for (int i = 1; i <= vs.length; ++i) {
        bzVec2 root = p[i-1];
        bzVec2 ux = p[i] - root;
        float length = ux.normalize();
        assert(length > float.epsilon);
        bzVec2 uy = bzVec2(-ux.y, ux.x);
        bzVec2 lower = bzVec2(float.max, float.max);
        bzVec2 upper = bzVec2(-float.max, -float.max);

        for (int j = 0; j < p.length; ++j) {
            bzVec2 d = p[j] - root;
            bzVec2 r;
            r.x = bzDot(ux, d);
            r.y = bzDot(uy, d);
            lower = bzMin(lower, r);
            upper = bzMax(upper, r);
        }

        float area = (upper.x - lower.x) * (upper.y - lower.y);
        if (area < 0.95f * minArea) {
            minArea = area;
            obb.R.col1 = ux;
            obb.R.col2 = uy;
            bzVec2 center = 0.5f * (lower + upper);
            obb.center = root + bzMul(obb.R, center);
            obb.extents = 0.5f * (upper - lower);
        }
    }

    assert(minArea < float.max);
}
