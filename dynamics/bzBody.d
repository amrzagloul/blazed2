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

module blaze.dynamics.bzBody;

import blaze.bzWorld;
import blaze.common.bzMath;
import blaze.common.bzConstants;
import blaze.collision.shapes.bzShape;
import blaze.collision.shapes.bzFluidParticle;
import blaze.collision.shapes.bzEdge;
import blaze.collision.shapes.bzShapeType;
import blaze.dynamics.bzBodyDef;
import blaze.dynamics.joints.bzJoint;
import blaze.dynamics.contact.bzContact;
import blaze.dynamics.forces.bzForceGenerator;

enum
{
    e_frozenFlag		= 0x0002,
    e_islandFlag		= 0x0004,
    e_sleepFlag			= 0x0008,
    e_allowSleepFlag	= 0x0010,
    e_bulletFlag		= 0x0020,
    e_fixedRotationFlag	= 0x0040,
}

/**
 * A rigid body
 */
class bzBody
{

    /// The body's origin transform
    bzXForm xf;

    /// The body's linear velocity
    bzVec2 linearVelocity;

    /// the body's angular velocity
    float angularVelocity = 0;

    /// The force exerted on this body
    bzVec2 force;

    /// The torque exerted on this body
    float torque = 0;

    /// Is this body static (immovable)?
    bool isStatic;

    /// Next body in the world linked body list
    bzBody prev;

    /// Previous body in the world linked body list
    bzBody next;

    /**
     * Linear damping is use to reduce the linear velocity. The damping parameter
     * can be larger than 1.0f but the damping effect becomes sensitive to the
     * time step when the damping parameter is large.
     * An example of the use of this is drag from friction with the air.
     */
    float linearDamping = 0;

    /**
     * Angular damping is use to reduce the angular velocity. The damping parameter
     * can be larger than 1.0f but the damping effect becomes sensitive to the
     * time step when the damping parameter is large.
     */
    float angularDamping = 0;

    /**
     * A reference to whatever data you want attached to the body. This may be
     * redundant because of the userData member of
     * blaze.collision.shapes.bzShape.bzShape
     */
    Object userData;

    /**
     * Creates a shape and attach it to this body.
     * Params: shapeDef = the shape definition.
     * Warning: This function is locked during callbacks.
     */
    bzShape createShape(bzShapeDef def)
    {

        assert(m_world.lock == false);

        if (m_world.lock == true)
        {
            return null;
        }


        // TODO: Decide on a better place to initialize edgeShapes. (b2Shape::Create() can't
        //       return more than one shape to add to parent body... maybe it should add
        //       shapes directly to the body instead of returning them?)
        if (def.type == bzShapeType.EDGE)
        {

            bzEdgeDef edgeDef = cast(bzEdgeDef) def;
            bzVec2 v1;
            bzVec2 v2;
            int i;

            if (edgeDef.isALoop)
            {
                v1 = edgeDef.vertices[edgeDef.vertices.length-1];
                i = 0;
            }
            else
            {
                v1 = edgeDef.vertices[0];
                i = 1;
            }

            bzEdge s0;
            bzEdge s1;
            bzEdge s2;
            float angle = 0.0f;

            for (; i < edgeDef.vertices.length; i++)
            {
                v2 = edgeDef.vertices[i];
                s2 = new bzEdge(v1, v2, def);
                s2.next = m_shapeList;
                m_shapeList = s2;
                ++m_shapeCount;
                s2.rBody = this;
                s2.createProxy(m_world.broadPhase, xf);
                s2.updateSweepRadius(sweep.localCenter);

                if (!s1)
                {
                    s0 = s2;
                    angle = atan2(s2.directionVector.y, s2.directionVector.x);
                }
                else
                {
                    angle = connectEdges(s1, s2, angle);
                }
                s1 = s2;
                v1 = v2;
            }
            if (edgeDef.isALoop) connectEdges(s1, s0, angle);
            return s0;
        }

        bzShape s = bzShape.create(def);

        s.next = m_shapeList;
        m_shapeList = s;
        ++m_shapeCount;

        s.rBody = this;

        // Add the shape to the world's broad-phase.
        s.createProxy(m_world.broadPhase, xf);

        // Compute the sweep radius for CCD.
        s.updateSweepRadius(sweep.localCenter);

        return s;
    }

    /**
     * Destroy a shape. This removes the shape from the broad-phase and
     * therefore destroys any contacts associated with this shape. All shapes
     * attached to a body are implicitly destroyed when the body is destroyed.
     * Params: shape = the shape to be removed.
     * Warning: This function is locked during callbacks.
     */
    void destroyShape(bzShape s)
    {

        assert(!m_world.lock);
        if (m_world.lock)
        {
            return;
        }

        assert(s.rBody is this);
        s.destroyProxy(m_world.broadPhase);

        assert(m_shapeCount > 0);
        bzShape node = m_shapeList;
        bool found = false;
        while (node !is null)
        {
            if (node is s)
            {
                node = s.next;
                found = true;
                break;
            }

            node = node.next;
        }

        // You tried to remove a shape that is not attached to this body.
        assert(found);

        s.rBody = null;
        s.next = null;

        --m_shapeCount;
        delete s;
    }

    /**
     * Set the mass properties. Note that this changes the center of mass
     * position. If you are not sure how to compute mass properties, use
     * SetMassFromShapes. The inertia tensor is assumed to be relative to the
     * center of mass.
     * Params: massData = the mass properties.
     */
    void setMass(bzMassData massData)
    {

        assert(!m_world.lock);
        if (m_world.lock)
        {
            return;
        }

        m_invMass = 0.0f;
        m_I = 0.0f;
        m_invI = 0.0f;

        m_mass = massData.mass;

        if (m_mass > 0.0f)
        {
            m_invMass = 1.0f / m_mass;
        }

        if ((flags & e_fixedRotationFlag) == 0)
        {
            m_I = massData.I;
        }

        if (m_I > 0.0f)
        {
            m_invI = 1.0f / m_I;
        }

        // Move center of mass.
        sweep.localCenter = massData.center;
        sweep.c0 = sweep.c = bzMul(xf, sweep.localCenter);

        // Update the sweep radii of all child shapes.
        for (bzShape s = m_shapeList; s; s = s.next)
        {
            s.updateSweepRadius(sweep.localCenter);
        }

        bool oldType = isStatic;
        if (m_invMass == 0.0f && m_invI == 0.0f)
        {
            isStatic = true;
        }
        else
        {
            isStatic = false;
        }

        // If the body type changed, we need to refilter the broad-phase proxies.
        if (oldType != isStatic)
        {
            for (bzShape s = m_shapeList; s; s = s.next)
            {
                s.refilterProxy(m_world.broadPhase, xf);
            }
        }

    }

    /**
     * Compute the mass properties from the attached shapes. You typically call
     * this after adding all the shapes. If you add or remove shapes later, you
     * may want to call this again. Note that this changes the center of mass
     * position.
     */
    void setMassFromShapes()
    {

        assert(!m_world.lock);
        if (m_world.lock)
        {
            return;
        }

        // Compute mass data from shapes. Each shape has its own density.
        m_mass = 0.0f;
        m_invMass = 0.0f;
        m_I = 0.0f;
        m_invI = 0.0f;

        bzVec2 center;
        for (bzShape s = m_shapeList; s; s = s.next)
        {
            bzMassData massData;
            s.computeMass(massData);
            m_mass += massData.mass;
            center += massData.mass * massData.center;
            m_I += massData.I;
        }

        // Compute center of mass, and shift the origin to the COM.
        if (m_mass > 0.0f)
        {
            m_invMass = 1.0f / m_mass;
            center *= m_invMass;
        }

        if (m_I > 0.0f && (flags & e_fixedRotationFlag) == 0)
        {
            // Center the inertia about the center of mass.
            m_I -= m_mass * bzDot(center, center);
            assert(m_I > 0.0f);
            m_invI = 1.0f / m_I;
        }
        else
        {
            m_I = 0.0f;
            m_invI = 0.0f;
        }

        // Move center of mass.
        sweep.localCenter = center;
        sweep.c0 = sweep.c = bzMul(xf, sweep.localCenter);

        // Update the sweep radii of all child shapes.
        for (bzShape s = m_shapeList; s; s = s.next)
        {
            s.updateSweepRadius(sweep.localCenter);
        }

        bool oldType = isStatic;
        if (m_invMass == 0.0f && m_invI == 0.0f)
        {
            isStatic = true;
        }
        else
        {
            isStatic = false;
        }

        // If the body type changed, we need to refilter the broad-phase proxies.
        if (oldType != isStatic)
        {
            for (bzShape s = m_shapeList; s; s = s.next)
            {
                s.refilterProxy(m_world.broadPhase, xf);
            }
        }

    }

    /**
     * Set the position of the body's origin and rotation (radians).
     * This breaks any contacts and wakes the other bodies.
     * Params:
     *     position = the new world position of the body's origin (not
     *         necessarily the center of mass).
     *     angle = the new world rotation angle of the body in radians.
     * Returns: false if the movement put a shape outside the world. In this
     * case the body is automatically frozen.
     * Bugs: See bzShape.synchronize
     */
    bool setXForm(bzVec2 position, float angle)
    {

        assert(!m_world.lock);

        if (m_world.lock)
        {
            return true;
        }

        if (isFrozen)
        {
            return false;
        }

        xf.R.set(angle);
        xf.position = position;

        sweep.c0 = sweep.c = bzMul(xf, sweep.localCenter);
        sweep.a0 = sweep.a = angle;

        bool freeze = false;
        for (bzShape s = m_shapeList; s; s = s.next)
        {
            bool inRange = s.synchronize(m_world.broadPhase, xf, xf);
            if (inRange == false)
            {
                freeze = true;
                break;
            }
        }

        if (freeze == true)
        {
            flags |= e_frozenFlag;
            linearVelocity.zero();
            angularVelocity = 0.0f;
            for (bzShape s = m_shapeList; s; s = s.next)
            {
                s.destroyProxy(m_world.broadPhase);
            }

            // Failure
            return false;
        }

        // Success
        m_world.broadPhase.commit();
        return true;
    }

    /**
     * Returns: the world position of the body's origin.
     */
    bzVec2 position() @property
    {
        return xf.position;
    }
    
     /**
     * Set the body's position in world coordinates
     */
    void position(bzVec2 p) @property {
        sweep.c = p;
        synchronizeTransform();
    }

    /**
     * Returns: the current world rotation angle in radians.
     */
    float angle() @property
    {
        return sweep.a;
    }

    /**
     * Set the body's angle
     * Params: a = the new angle (in radians)
     */
    void angle(float a) @property
    {
        sweep.a = a;
        synchronizeTransform();
    }

    /**
     * Returns: the world position of the center of mass.
     */
    bzVec2 worldCenter() @property
    {
        return sweep.c;
    }

    /**
     * Returns: the local position of the center of mass.
     */
    bzVec2 localCenter() @property
    {
        return sweep.localCenter;
    }

    /**
     * Apply a force at a world point. If the force is not
     * applied at the center of mass, it will generate a torque and
     * affect the angular velocity. This wakes up the body.
     * Params:
     *     force = the world force vector, usually in Newtons (N).
     *     point = the world position of the point of application.
     */
    void applyForce(bzVec2 force, bzVec2 point)
    {
        if (isSleeping())
        {
            wakeup();
        }
        this.force += force;
        this.torque += bzCross(point - sweep.c, force);
    }

    /**
     * Applies a bouyancy force caused by fluid particles. This wakes up the
     * body.
     * Params: particle = the fluid particle that will apply the force
     */
    void applyBuoyancyForce(bzFluidParticle particle)
    {
        if (isSleeping())
        {
            wakeup();
        }
        // Apply buoyancy force to body
        float angDrag = 0.05f;
        float linDrag = 0.5f;
        bzVec2 f = particle.force * linDrag;
        bzVec2 r = particle.position - position;
        this.force += f;
        this.torque += r.x * f.y * angDrag - r.y * f.x * angDrag;
    }

    /**
     * Apply a torque. This affects the angular velocity
     * without affecting the linear velocity of the center of mass.
     * This wakes up the body.
     * Params: torque = torque about the z-axis (out of the screen), usually
     * measured in Newton-meters
     */
    void applyTorque(float torque)
    {
        if (isSleeping())
        {
            wakeup();
        }
        this.torque += torque;
    }

    /**
     * Apply an impulse at a point. This immediately modifies the velocity.
     * It also modifies the angular velocity if the point of application
     * is not at the center of mass. This wakes up the body.
     * Params:
     *     impulse = the world impulse vector, usually in Newton-seconds or
     *         kg-m/s.
     *     point = the world position of the point of application.
     */
    void applyImpulse(bzVec2 impulse, bzVec2 point)
    {
        if (isSleeping())
        {
            wakeup();
        }
        linearVelocity += m_invMass * impulse;
        angularVelocity += m_invI * bzCross(point - sweep.c, impulse);
    }

    /**
     * Returns: the total mass of the body, usually in kilograms.
     */
    float mass() @property
    {
        return m_mass;
    }

    /**
     * Returns: 1 divided by the total mass of the body. Units are 1/kg
     */
    float invMass() @property
    {
        return m_invMass;
    }

    /**
     * Get the central rotational inertia of the body.
     * Returns: the rotational inertia, usually in kg-m^2.
     */
    float inertia()
    {
        return m_I;
    }

    /**
     * Returns: the inverse rotational inertia of the body.
     */
    float invI() @property
    {
        return m_invI;
    }

    /**
     * Get the world coordinates of a point given the local coordinates.
     * Params:
     *     localPoint = a point on the body measured relative the the body's
     *         origin.
     * Returns: the same point expressed in world coordinates.
     */
    bzVec2 worldPoint(bzVec2 localPoint)
    {
        return bzMul(xf, localPoint);
    }

    /**
     * Get the world coordinates of a vector given the local coordinates.
     * Params: localVector = a vector fixed in the body.
     * Params: the same vector expressed in world coordinates.
     */
    bzVec2 worldVector(bzVec2 localVector)
    {
        return bzMul(xf.R, localVector);
    }

    /**
     * Gets a local point relative to the body's origin given a world point.
     * Params: worldPoint = a point in world coordinates.
     * Returns: the corresponding local point relative to the body's origin.
     */
    bzVec2 localPoint(bzVec2 worldPoint)
    {
        return bzMulT(xf, worldPoint);
    }

    /**
     * Gets a local vector given a world vector.
     * Params: worldVector a vector in world coordinates.
     * Returns: the corresponding local vector.
     */
    bzVec2 localVector(bzVec2 worldVector)
    {
        return bzMulT(xf.R, worldVector);
    }

    /**
     * Get the world linear velocity of a world point attached to this body.
     * Params: worldPoint = a point in world coordinates.
     * Returns: the world velocity of a point.
     */
    bzVec2 linearVelocityFromWorldPoint(bzVec2 worldPoint)
    {
        return linearVelocity + bzCross(angularVelocity, worldPoint - sweep.c);
    }

    /**
     * Get the world velocity of a local point.
     * Params: localPoint = a point in local coordinates.
     * Returns: the world velocity of a point.
     */
    bzVec2 linearVelocityFromLocalPoint(bzVec2 localPoint)
    {
        return linearVelocityFromWorldPoint(worldPoint(localPoint));
    }

    /**
     * Returns: true if this body treated like a bullet for continuous collision
     * detection, false otherwise.
     */
    int bullet() @property
    {
        return (flags & e_bulletFlag);
    }

    /**
     * Sets whether or not this body be treated like a bullet for continuous
     * collision detection. This will cause the collision detection to be more
     * accurate, but less efficient. Use this only on bodies that will have
     * high speeds.
     * Params: flag = true if the body should be treated as a bullet, false
     *     otherwise.
     */
    void bullet(bool flag) @property
    {
        if (flag)
        {
            flags |= e_bulletFlag;
        }
        else
        {
            flags &= ~e_bulletFlag;
        }
    }

    /**
     * Returns: true if the body is frozen, false otherwise.
     */
    int isFrozen() @property
    {
        return (flags & e_frozenFlag);
    }

    /**
     * Returns: true if the body is sleeping, false otherwise.
     */
    int isSleeping() @property
    {
        return (flags & e_sleepFlag);
    }

    /**
     * Sets wheter or not sleeping is allowed for this body.
     * Params: flag = true if the body can go to sleep when it stops moving,
     *     false if it cannot.
     */
    void allowSleeping(bool flag)
    {
        if (flag)
        {
            flags |= e_allowSleepFlag;
        }
        else
        {
            flags &= ~e_allowSleepFlag;
            wakeup();
        }
    }
    
    /**
     * Sets wheter or not freezing is allowed for this body.
     * Params: flag = true if the body can freeze, false if it cannot.
     */
    void allowFreeze(bool flag)
    {
        m_allowFreeze = flag;
        if (!flag) {
            unFreeze();
        }
    }

    /**
     * Wake up this body so it will begin simulating.
     */
    void wakeup() {
        flags &= ~e_sleepFlag;
        m_sleepTime = 0.0f;
    }
    
    /**
     * Unfreeze the body
     */
    void unFreeze() {
        flags &= ~e_frozenFlag;
    }

    /**
     * Put this body to sleep so it will stop simulating.
     * Note: This also sets the velocity to zero.
     */
    void putToSleep()
    {
        flags |= e_sleepFlag;
        m_sleepTime = 0.0f;
        linearVelocity.zero();
        angularVelocity = 0.0f;
        force.zero();
        torque = 0.0f;
    }

    /**
     * Get the firsh shape attached to this body. Access to the other shapes
     * is accomplished through bzShape.next
     * Returns: the first shape attached to this body.
     */
    bzShape shapeList()
    {
        return m_shapeList;
    }

    void shapeList(bzShape list)
    {
        m_shapeList = list;
    }

    /**
     * Returns: the number of shapes attached to this body
     */
    int shapeCount() @property
    {
        return m_shapeCount;
    }

    void shapeCount(int count) @property
    {
        m_shapeCount = count;
    }

    /**
     * Returns: the parent world of this body.
     */
    bzWorld world()
    {
        return m_world;
    }

    //--------------- Internals Below -------------------
    // TODO: maybe make these private/protected?

    /// The swept motion for CCD
    bzSweep sweep;

    /// The body's status flags
    uint flags;

    /**
     * Constructor
     * Params:
     *     bd = the body definition to base this body off of
     *     world = the world that this body will be a part of
     * Warning: This function assert()'s that the world is not locked
     */
    this (bzBodyDef bd, bzWorld world)
    {

        assert(!world.lock);

        m_world = world;

        if (bd.isBullet)
        {
            flags |= e_bulletFlag;
        }
        if (bd.fixedRotation)
        {
            flags |= e_fixedRotationFlag;
        }
        if (bd.allowSleep)
        {
            flags |= e_allowSleepFlag;
        }
        if (bd.isSleeping)
        {
            flags |= e_sleepFlag;
        }

        m_allowFreeze = bd.allowFreeze;

        xf.position = bd.position;
        xf.R.set(bd.angle);

        sweep.localCenter = bd.massData.center;
        sweep.t0 = 1.0f;
        sweep.a0 = sweep.a = bd.angle;
        sweep.c0 = sweep.c = bzMul(xf, sweep.localCenter);

        prev = null;
        next = null;
        jointList = null;
        contactList = null;

        linearDamping = bd.linearDamping;
        angularDamping = bd.angularDamping;

        force.zero();
        torque = 0.0f;

        linearVelocity.zero();
        angularVelocity = 0.0f;

        m_sleepTime = 0.0f;

        m_invMass = 0.0f;
        m_I = 0.0f;
        m_invI = 0.0f;

        m_mass = bd.massData.mass;

        if (m_mass > 0.0f)
        {
            m_invMass = 1.0f / m_mass;
        }

        if ((flags & e_fixedRotationFlag) == 0)
        {
            m_I = bd.massData.I;
        }

        if (m_I > 0.0f)
        {
            m_invI = 1.0f / m_I;
        }

        if (m_invMass == 0.0f && m_invI == 0.0f)
        {
            isStatic = true;
        }
        else
        {
            isStatic = false;
        }

        userData = bd.userData;

        m_shapeList = null;
        m_shapeCount = 0;
    }

    this()
    {
    }

    /**
     * Synchronizes all the shapes in the body.
     * For internal use only.
     */
    bool synchronizeShapes()
    {

        bzXForm xf1;
        xf1.R.set(sweep.a0);
        xf1.position = sweep.c0 - bzMul(xf1.R, sweep.localCenter);

        bool inRange;
        for (bzShape s = m_shapeList; s; s = s.next) {
            inRange = s.synchronize(m_world.broadPhase, xf1, xf);
            if (!inRange) {
                break;
            }
        }

        if (!inRange) {
            if (m_allowFreeze) {
                flags |= e_frozenFlag;
                linearVelocity = bzVec2(0,0);
                angularVelocity = 0.0f;
                for (bzShape s = m_shapeList; s; s = s.next) {
                    s.destroyProxy(m_world.broadPhase);
                }
            }
            // Failure
            return false;
        }

        // Success
        return true;
    }

    /**
     * Update rotation and position of the body
     */
    void synchronizeTransform()
    {
        xf.R.set(sweep.a);
        xf.position = sweep.c - bzMul(xf.R, sweep.localCenter);
    }

    /**
     * This is used to prevent connected bodies from colliding.
     * It may lie, depending on the collideConnected flag.
     *
     */
    bool isConnected(bzBody other)
    {
        for (bzJointEdge jn = jointList; jn; jn = jn.next)
        {
            if (jn.other == other)
                return jn.joint.collideConnected() == false;
        }

        return false;
    }

    /**
     * Advance to the new safe time.
     */
    void advance(float t)
    {

        sweep.advance(t);
        sweep.c = sweep.c0;
        sweep.a = sweep.a0;
        synchronizeTransform();
    }

    /**
     * Returns: the island index of the body
     */
    int islandIndex()
    {
        return m_islandIndex;
    }

    /**
     * Sets the body's island index
     * Params: i = the new island index
     */
    void islandIndex(int i)
    {
        m_islandIndex = i;
    }

    /**
     * Returns: the body's sleep time
     */
    float sleepTime()
    {
        return m_sleepTime;
    }

    /**
     * Sets the body's sleep time
     * Params: time = the sleep time
     */
    void sleepTime(float time)
    {
        m_sleepTime = time;
    }

    /**
     * The list of joints connected to the body
     */
    bzJointEdge jointList;

    /**
     * The list of contacts associated with the body
     */
    bzContactEdge contactList;

    // Force generator pointer
    bzForceGenerator forceGen;

private:
    /// bzWorld that the body is in
    bzWorld m_world;

    /// Sleep timer
    float m_sleepTime = 0;

    /// Mass of the body
    float m_mass = 0;

    /// 1 / Mass of the body
    float m_invMass = 0;

    /// Rotational inerta
    float m_I = 0;

    /// 1 / rotational inertia
    float m_invI = 0;

    /// First of the shapes attached to the body
    bzShape m_shapeList;

    /// The number of shapes in the body
    int m_shapeCount;

    /// The island index
    int m_islandIndex;
    
    bool m_allowFreeze;
}

float connectEdges(ref bzEdge s1, ref bzEdge s2, float angle1)
{
    float angle2 = atan2(s2.directionVector.y, s2.directionVector.x);
    bzVec2 core = tan((angle2 - angle1) * 0.5f) * s2.directionVector;
    core = k_toiSlop * (core - s2.normalVector) + s2.vertex1;
    bzVec2 cornerDir = s1.directionVector() + s2.directionVector;
    cornerDir.normalize();
    bool convex = bzDot(s1.directionVector, s2.normalVector) > 0.0f;
    s1.setNextEdge(s2, core, cornerDir, convex);
    s2.setPrevEdge(s1, core, cornerDir, convex);
    return angle2;
}

