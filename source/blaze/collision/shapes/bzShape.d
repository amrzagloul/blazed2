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

// Documentation comments altered quite a bit by Brian Schott (SirAlaran)

module blaze.collision.shapes.bzShape;

import blaze.common.bzMath;
import blaze.dynamics.bzBody;
import blaze.dynamics.joints.bzJoint;
import blaze.collision.bzCollision;
import blaze.collision.shapes.bzPolygon;
import blaze.collision.shapes.bzCircle;
import blaze.collision.shapes.bzFluidParticle;
import blaze.collision.shapes.bzShapeType;
import blaze.collision.nbody.bzBroadPhase;
import blaze.collision.nbody.bzPairManager;

/**
 * This holds the mass data computed for a shape.
 */
struct bzMassData {
    /**
	 * The mass of the shape, usually in kilograms.
	 */
    float mass = 0.0f;

    /**
     * The position of the shape's centroid relative to the shape's origin.
	 */
    bzVec2 center;

    /**
     * The rotational inertia of the shape.
     */
    float I = 0.0f;
}

/**
 * This holds contact filtering data.
 * Use bzFilterData to fine-tune which objects collide against which others
 */
struct bzFilterData {
    /**
     * The collision category bits. Normally you would just set one bit.
	 */
    int categoryBits;

    /**
     * The collision mask bits. This states the categories that this
     * shape would accept for collision.
	 */
    int maskBits;

    /**
     * Collision groups allow a certain group of objects to never collide
	 * (negative) or always collide (positive). Zero means no collision group.
	 * Non-zero group filtering always wins against the mask bits.
	 *
	 * An example of this may be a set of snowflake particles. It would be
	 * wasteful to try to detect collisions between snowflakes.
	 */
    int groupIndex;
}

/**
 * Return codes from TestSegment
 */
enum bzSegmentCollide {
    /// The segment starts inside the shape being tested
    STARTS_INSIDE = -1,
	/// The segment missed the shape
    MISS = 0,
	/// The segment hit the shape
    HIT = 1
}

/**
 * A shape definition is used to construct a shape. This class defines an
 * abstract shape definition. You can reuse shape definitions safely.
 */
class bzShapeDef {
    /**
	 * The constructor sets the default shape definition values.
	 */
    this () {
        type = bzShapeType.UNKNOWN;
        userData = null;
        friction = 1.0f;
        restitution = 0.0f;
        density = 0.0f;
        filter.categoryBits = 0x0001;
        filter.maskBits = 0xFFFF;
        filter.groupIndex = 0;
        isSensor = false;
    }

    /**
	 * The bzBody this shape belongs to
	 */
    bzBody rBody;

    /**
	 * Holds the shape type for down-casting.
	 */
    bzShapeType type;

    /**
	 * Use this to store application specify shape data.
	 */
    Object userData;

    /**
	 * The shape's friction coefficient, usually in the range [0,1].
	 */
    float friction = 0.0f;

    /**
	 * The shape's restitution (elasticity) usually in the range [0,1].
	 * Set this to greater than 1 for some silly results.
	 */
    float restitution = 0.0f;

    /**
	 * The shape's density, usually in kilograms per square meter.
	 */
    float density = 0.0f;

    /**
     * A sensor shape collects contact information but never generates a
	 * collision response. Examples of this sort of shape would be a goal,
	 * a trap sensor, a teleporter, or something similar.
     */
    bool isSensor;

    /**
	 * bzContact filtering data.
	 */
    bzFilterData filter;

	/**
	 * Bugs: Not implemented
	 */
    void setAsBox(float hx, float hy) {
    }
}

/**
 * A shape is used for collision detection. Shapes are created in bzWorld.
 * You can use shape for collision detection before they are attached to the world.
 * Warning: you cannot reuse shapes.
 */
class bzShape {

    /**
	 * Returns: the maximum radius about the parent body's center of mass.
	 */
    float sweepRadius() @property {
        return m_sweepRadius;
    }

    /**
     * Get the type of this shape. You can use this to down cast to the concrete
	 * shape.
     * Returns: the shape type.
	 * Bugs: Is this really necessary in D?
     */
    bzShapeType type() @property {
        return m_type;
    }

    /**
     * Get the shape's triangle list. Vertices are in world coordinates
     * Returns: the triangle list.
     */
    bzTri2[] triangleList() @property {
        return m_triangleList;
    }

    /**
     * Is this shape a sensor (non-solid)?
     * Returns: true if the shape is a sensor, false otherwise
     */
    bool isSensor() @property {
        return m_isSensor;
    }

    /**
	 * bzShape definition Constructor
	 * Params: def = the shape definition
	 */
    this (bzShapeDef def) {
        userData = def.userData;
        friction = def.friction;
        restitution = def.restitution;
        m_density = def.density;
        rBody = def.rBody;
        m_sweepRadius = 0.0f;
        next = null;
        filter = def.filter;
        m_proxyId = bz_nullProxy;
        m_isSensor = def.isSensor;
    }

    /**
	 * Generic constructor (Used by FluidParticles). Does nothing
	 */
    this() {
		m_proxyId = bz_nullProxy;
	}

    /** Internal */
	public bool synchronize(bzBroadPhase broadPhase, bzXForm transform1, bzXForm transform2) {

    	if (m_proxyId == bz_nullProxy) {
    		return false;
    	}

    	// Compute an bzAABB that covers the swept shape (may miss some rotation effect).
    	bzAABB aabb;
    	computeSweptAABB(aabb, transform1, transform2);

    	if (broadPhase.inRange(aabb)) {
    		broadPhase.moveProxy(m_proxyId, aabb);
    		return true;
    	} else {
    		return false;
    	}
    }

    /** Internal */
    public void refilterProxy(bzBroadPhase broadPhase, bzXForm transform){
    	if (m_proxyId == bz_nullProxy){
    		return;
    	}

    	broadPhase.destroyProxy(m_proxyId);

    	bzAABB aabb;
    	computeAABB(aabb, transform);

    	bool inRange = broadPhase.inRange(aabb);

    	if (inRange) {
    		m_proxyId = broadPhase.createProxy(aabb, this);
    	} else {
    		m_proxyId = bz_nullProxy;
    	}
    }

    /**
     * Test a point for containment in this shape. This only works for convex
	 * shapes.
     * Params:
	 *     xf = the shape world transform.
     *     p = a point in world coordinates.
	 * Returns: true if the point lies within the shape, false otherwise.
     */
    abstract bool testPoint(bzXForm xf, bzVec2 p);

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
     */
    abstract bzSegmentCollide testSegment(bzXForm xf, ref float lambda,
        ref bzVec2 normal, bzSegment segment, float maxLambda);

    /**
	 * Given a transform, compute the associated axis aligned bounding box for
	 *     this shape.
	 * This does not modify the shape's bzAABB
     * Params:
	 *     aabb = returns the axis aligned box.
     *     xf = the world transform of the shape.
	 */
    abstract void computeAABB(ref bzAABB aabb, bzXForm xf);

	/**
     * Update the sweep radius (maximum radius) as measured from
     * a local center point.
	 * Params: center = the center point to measure from
	 */
    abstract void updateSweepRadius(bzVec2 center);

    /**
	 * Given two transforms, compute the associated swept axis aligned bounding
	 *     box for this shape. This DOES modify the shape's bzAABB (see bugs)
     * Params:
     *     xf1 = the world transform of the shape.
	 *     xf2 = the velocity of the shape
	 * Bugs: Why is this different from the way Box2d does it? And why does it
	 *     behave differently from computeAABB?
	 */
    abstract void computeSweptAABB(ref bzAABB aabb, bzXForm xf1, bzXForm xf2);

    /**
	 * Compute the mass properties of this shape using its dimensions and density.
     * The inertia tensor is computed about the local origin, not the centroid.
     * Params: massData = returns the mass data for this shape.
	 */
    abstract void computeMass(ref bzMassData massData);

    /**
	 * Triangulate the shape in world coordinates.
	 * Currently this is only used by the buoyancy solver.
	 * Bugs: Shouldn't this be made abstract?
	 */
	void triangulate() {
		assert(0);
	}

	/**
	 * The shape's world center
	 */
	abstract bzVec2 worldCenter();

    /**
	 * Returns: The shape's support point (for MPR & GJK)
	 */
    abstract bzVec2 support(bzXForm xf, bzVec2 d);
	
	/**
	 * Returns: The shape's density
	 */
	float density() {
		return m_density;
	}

	/**
	 * Creates a shape from a definition. Automatically calls the correct
	 * constructor for the shape specified in the definition
	 * Params def = the shape definition
	 */
    static bzShape create(bzShapeDef def) {
        switch (def.type) {
        case bzShapeType.CIRCLE: {
            return new bzCircle(def);
        }
        case bzShapeType.POLYGON: {
            return new bzPolygon(def);
        }
        default:
            throw new Exception ("Unknown shape type!");
        }
    }

    /** Internal */
    public void destructor() {
        assert(m_proxyId == bz_nullProxy);
    }

    /** Internal */
    public void createProxy(bzBroadPhase broadPhase, bzXForm transform) {
    	assert(m_proxyId == bz_nullProxy);

    	bzAABB aabb;
    	computeAABB(aabb, transform);

    	bool inRange = broadPhase.inRange(aabb);

    	// You are creating a shape outside the world box.
    	assert(inRange);

    	if (inRange){
    		m_proxyId = broadPhase.createProxy(aabb, this);
    	} else {
    		m_proxyId = bz_nullProxy;
    	}
    }

    /** Internal */
    public void destroyProxy(bzBroadPhase broadPhase) {
        if (m_proxyId != bz_nullProxy) {
            broadPhase.destroyProxy(m_proxyId);
            m_proxyId = bz_nullProxy;
        }
    }

	/**
     * User data. This can be anything that you want to have connected to the
	 * shape. Often this is a reference back to the game object associated with
	 * the shape. Use this inside collision callbacks to refer to your game
	 * object.
	 */
    Object userData;

	/// The shape's area (Usually square meters)
	float area = 0.0f;

	/// The shape's friction
    float friction = 0.0f;

	/// The shape's coefficient of restitution
    float restitution = 0.0f;

	/// The next shape in the shape list
    bzShape next;

    /// The body this shape is attached to
    bzBody rBody;

	/// Information for contact filtering
    bzFilterData filter;

    int m_proxyId;

    //bzAABB, e=extends (halfwidths)
    float xmax = 0;
    float xmin = 0;
    float ymax = 0;
    float ymin = 0;


protected:

    // The type of shape that this is
    bzShapeType m_type;

    // Sweep radius relative to the parent body's center of mass.
    float m_sweepRadius = 0;

    // The shape's density
    float m_density = 0;

    // True if the shape is a sensor
    bool m_isSensor;

    // The triangle list
    bzTri2[] m_triangleList;
}
