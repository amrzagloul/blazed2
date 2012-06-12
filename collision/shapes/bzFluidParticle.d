/*
 *  Copyright (c) 2008 Rene Schulte. http://www.rene-schulte.info/
 *  Ported to D by Mason Green. http:/www.dsource.org/projects/blaze
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

module blaze.collision.shapes.bzFluidParticle;

import blaze.common.bzMath;
import blaze.common.bzConstants;
import blaze.dynamics.bzBody;
import blaze.dynamics.bzBodyDef;
import blaze.bzWorld;
import blaze.collision.shapes.bzShape;
import blaze.collision.shapes.bzShapeType;
import blaze.collision.bzCollision;

/**
 * A fluid particle
 */
public class bzFluidParticle : bzShape {

    public bzVec2 velocity;
    public bzVec2 position;
    public bzVec2 positionOld;
    public float mass;
    public float massInv;
    public float density;
    public bzVec2 force;
    public float pressure;
    public bzFluidParticle[uint] neighbors;
    private float gridSize;
    public static float margin;
	/// This shapes identifier
    ushort ID;

    /**
     * Constructor
     */
    this(bzVec2 position, float restitution, float friction, bzWorld world) {
        super.m_type = bzShapeType.FLUID;
        super.restitution = restitution;
        super.friction = friction;
        this.setMass(1.25);
        this.gridSize = CELL_SPACE * 2f;
        this.margin = 0.01f;
        this.position = position;
        this.positionOld = position;
        this.density = DENSITY_OFFSET;

		// Create proxy
		bzXForm dummy;
		createProxy(world.broadPhase, dummy);
        updatePressure();
    }

    /**
     * Updates the pressure using a modified ideal gas state equation
     * (see the paper "Smoothed particles: A new paradigm for animating highly
	 * deformable bodies." by Desbrun)
     */
    public void updatePressure() {
        pressure = GAS_CONSTANT * (density - DENSITY_OFFSET);
    }

    /**
     * Updates the particle.
     */
    public void update(bzTimeStep step, bzVec2 gravity) {

        bzVec2 acceleration = force * massInv;
        acceleration += gravity;

        // Vertlet integration
        bzVec2 t;
        float damping = 0.01f;
        bzVec2 oldPos = position;
        // Position = Position + (1.0f - Damping) * (Position - PositionOld) + dt * dt * a;
        acceleration *= (step.dt * step.dt);
        t = position - positionOld;
        t *= (1.0f - damping);
        t += acceleration;
        position += t;
        positionOld = oldPos;

        // calculate velocity
        // Velocity = (Position - PositionOld) / dt;
        t = position - positionOld;
        velocity = t * 1.0 * step.inv_dt;
    }

	/**
	 * Returns: The particles position in world coordinates
	 */
	override bzVec2 worldCenter() {
		return position;
	}

	/**
	 * Params: mass = the new mass of the particle
	 */
    public void setMass(float mass) {
        this.mass = mass;
        massInv = mass > 0.0f ? 1.0f / mass : 0.0f;
    }

	/**
	 * Adds a neighbor to the neighbor list
	 */
    public void addNeighbor(bzFluidParticle p) {
        neighbors[p.ID] = p;
    }

	/**
	 * Adds a neighbor to the neighbor list
	 */
    public void removeNeighbor(bzFluidParticle p) {
		uint key = p.ID;
        neighbors.remove(key);
    }

	/**
	 * Applies a force to the particle
	 * Params: force = the force to apply
	 */
    public void addForce(bzVec2 force) {
        this.force += force;
    }

	/**
	 * Returns: false. A particle can't contain a point
	 */
    bool containsPoint(bzVec2 v) {
        return false;
    }

	/**
	 * Calculates the inertia
	 * Params:
	 *     m = ignored
	 *     offset = ignored
	 * Returns: 0.0f
	 */
    float calculateInertia(float m, bzVec2 offset) {
        return 0.0f;
    }

	/**
	 * Applies an impulse to the particle
	 * Params:
	 *     penetration = the distance and direction of penetration
	 *     penetrationNormal = penetration, but with a length of 1.0
	 *     rest = the coefficient of restitution
	 *     fric = the coefficient of friction
	 */
    void applyImpulse(bzVec2 penetration, bzVec2 penetrationNormal, float rest, float fric) {

        // Move this method to bzFluidParticle

        // Handle collision
        // Calc new velocity using elastic collision with friction
        // -> Split oldVelocity in normal and tangential component, revert normal component and add it afterwards
        // v = pos - oldPos;
        //vn = n * Vector2.Dot(v, n) * -Bounciness;
        //vt = t * Vector2.Dot(v, t) * (1.0f - Friction);
        //v = vn + vt;
        //oldPos = pos - v;

        bzVec2 v, vn, vt;

        v = position - positionOld;
        bzVec2 tangent = penetrationNormal.rotateRight90();
        float dp = v.bzDot(penetrationNormal);
        vn = penetrationNormal * dp * -rest;
        dp = v.bzDot(tangent);
        vt = tangent * dp * (1.0f - fric);
        v = vn + vt;
        position -= penetration;
        positionOld = position - v;
    }

	/**
	 * Updates swept bzAABB
	 */
    override void computeSweptAABB(ref bzAABB aabb, bzXForm xf1, bzXForm xf2) {
        aabb.lowerBound.x = position.x - gridSize;
        aabb.lowerBound.y = position.y - gridSize;
        aabb.upperBound.x = position.x + gridSize;
        aabb.upperBound.y = position.y + gridSize;

		xmax = aabb.upperBound.x;
        xmin = aabb.lowerBound.x;
        ymax = aabb.upperBound.y;
        ymin = aabb.lowerBound.y;

    }

	/**
	 * Does nothing
	 */
	override void triangulate() {}

	/**
	 * Does nothing
	 */
    override void updateSweepRadius(bzVec2 center) {}

	/**
	 * Does nothing
	 */
    override void computeMass(ref bzMassData massData) {}

	/**
	 * Update bzAABB
	 */
    override void computeAABB(ref bzAABB aabb, bzXForm xf) {
		aabb.lowerBound.x = position.x - gridSize;
        aabb.lowerBound.y = position.y - gridSize;
        aabb.upperBound.x = position.x + gridSize;
        aabb.upperBound.y = position.y + gridSize;
	}

	/**
	 * Returns: bzSegmentCollide.MISS
	 */
    override bzSegmentCollide testSegment(bzXForm xf, ref float lambda, ref bzVec2 normal,
		bzSegment segment, float maxLambda){
		return bzSegmentCollide.MISS;
	}

	/**
	 * Returns: false
	 */
    override bool testPoint(bzXForm xf, bzVec2 p) { return false; }

	/**
	 * Returns: a zero vector
	 */
    override bzVec2 support(bzXForm xf, bzVec2 d) { return bzVec2(0,0); }
}

