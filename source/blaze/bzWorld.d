/*
* Copyright (c) 2009, Mason Green (zzzzrrr)
* Based on Box2D by Erin Catto, http://www.box2d.org
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
module blaze.bzWorld;

import blaze.common.bzMath;
import blaze.common.bzConstants;
import blaze.dynamics.bzIsland;
import blaze.dynamics.bzBody;
import blaze.dynamics.bzBodyDef;
import blaze.dynamics.contact.bzContact;
import blaze.dynamics.contact.bzContactManager;
import blaze.dynamics.forces.bzForceGenerator;
import blaze.dynamics.forces.bzForceRegistry;
import blaze.dynamics.forces.bzBuoyancy;
import blaze.dynamics.joints.bzJoint;
import blaze.dynamics.joints.bzPulleyJoint;
import blaze.dynamics.bzWorldCallbacks;
import blaze.collision.shapes.bzShapeType;
import blaze.collision.shapes.bzShape;
import blaze.collision.shapes.bzPolygon;
import blaze.collision.shapes.bzCircle;
import blaze.collision.shapes.bzEdge;
import blaze.collision.shapes.bzFluidParticle;
import blaze.collision.nbody.bzBroadPhase;
import blaze.collision.nbody.bzPairManager;
import blaze.collision.nbody.bzPair;
import blaze.collision.pairwise.bzTimeOfImpact;
import blaze.collision.bzCollision;
import blaze.dynamics.fluid.bzSPHSimulation;

struct bzTimeStep {
    float dt = 0;			// time step
    float inv_dt = 0;		// inverse time step (0 if dt == 0).
    float dtRatio = 0;	// dt * inv_dt0
    int velocityIterations;
    int positionIterations;
    bool warmStarting;
}

/**
 * The world class manages all physics entities, dynamic simulation,
 * and asynchronous queries. The world also contains efficient memory
 * management facilities.
 */
class bzWorld {

    /**
     * Constructs a world object.
     * Params:
     *     gravity = the world gravity vector.
     *     doSleep = improve performance by not simulating inactive bodies.
     */
    this (bzAABB worldAABB, bzVec2 gravity, bool doSleep) {

        m_bodyCount = 0;
        contactCount = 0;
        m_jointCount = 0;

        m_warmStarting = true;
        m_continuousPhysics = true;

        m_allowSleep = doSleep;
        m_gravity = gravity;

        m_lock = false;

        m_inv_dt0 = 0.0f;

        m_forceRegistry = new bzForceRegistry();
        m_contactManager = new bzContactManager(this);
        broadPhase = new bzBroadPhase(worldAABB, m_contactManager);

        auto bd = new bzBodyDef(bzVec2.zeroVect, 0.0f);
        m_groundBody = createBody(bd);
    }

    /// Allow or disallow sleeping
    void allowSleep(bool sleep) {
        m_allowSleep = sleep;
    }

    /// Register a destruction listener.
    void destructionListener(bzDestructionListener listener) {
        m_destructionListener = listener;
    }

    /// Register a broad-phase boundary listener.
    void boundaryListener(bzBoundaryListener listener) {
        m_boundaryListener = listener;
    }

    /// Register a contact filter to provide specific control over collision.
    /// Otherwise the default filter is used (_defaultFilter).
    void contactFilter(bzContactFilter filter) {
        m_contactFilter = filter;
    }

    bzContactFilter contactFilter() {
        return m_contactFilter;
    }

    /**
     * Registers a contact event listener
     */
    void contactListener(bzContactListener listener) @property {
        m_contactListener = listener;
    }

    /**
     * Returns: the current contact listener
     */
    bzContactListener contactListener() @property {
        return m_contactListener;
    }

    /**
     * Create a rigid body given a definition. No reference to the definition
     * is retained.
     * Params: def = the body definition to base the new body off of.
     * Warning: This function is locked during callbacks.
     */
    bzBody createBody(bzBodyDef def) {
        assert(!m_lock);
        if (m_lock) {
            return null;
        }

        bzBody b = new bzBody(def, this);

        // Add to world doubly linked list.
        b.prev = null;
        b.next = m_bodyList;
        if (m_bodyList) {
            m_bodyList.prev = b;
        }
        m_bodyList = b;
        ++m_bodyCount;

        return b;
    }

    /**
     * Destroy a rigid body given a definition. No reference to the definition
     * is retained. This function is locked during callbacks.
     * Params: b = the body to destroy
     * Warning: This automatically deletes all associated shapes and joints.
     * Warning: This function is locked during callbacks.
     */
    void destroyBody(bzBody b) {


        assert(m_bodyCount > 0);
        assert(!m_lock);

        // Remove force generator attached to this body
        if(b.forceGen) {
            m_forceRegistry.remove(b.forceGen);
        }

        // Delete the attached joints.
        bzJointEdge jn = b.jointList;
        while (jn) {
            bzJointEdge jn0 = jn;
            jn = jn.next;

            if (m_destructionListener) {
                m_destructionListener.sayGoodbye(jn0.joint);
            }

            destroyJoint(jn0.joint);
            delete jn0;
        }

        // Delete the attached shapes. This destroys broad-phase
        // proxies and pairs, leading to the destruction of contacts.
        bzShape s = b.shapeList();
        while (s) {
            bzShape s0 = s;
            s = s.next;

            if (m_destructionListener) {
                m_destructionListener.sayGoodbye(s0);
            }

            s0.destroyProxy(broadPhase);
            delete s0;
        }

        // Remove world body list.
        if (b.prev) {
            b.prev.next = b.next;
        }

        if (b.next) {
            b.next.prev = b.prev;
        }

        if (b is m_bodyList) {
            m_bodyList = b.next;
        }

        --m_bodyCount;
        delete b;
    }

    /**
     * Create a joint to constrain bodies together. No reference to the definition
     * is retained. This may cause the connected bodies to cease colliding.
     * Warning: This function is locked during callbacks.
     */
    bzJoint createJoint(bzJointDef def) {

        assert(!m_lock);

        bzJoint j = bzJoint.create(def);

        // Connect to the world list.
        j.prev = null;
        j.next = m_jointList;
        if (m_jointList) {
            m_jointList.prev = j;
        }
        m_jointList = j;
        ++m_jointCount;

        // Connect to the bodies' doubly linked lists.
        j.node1.joint = j;
        j.node1.other = j.rBody2;
        j.node1.prev = null;
        j.node1.next = j.rBody1.jointList;
        if (j.rBody1.jointList) j.rBody1.jointList.prev = j.node1;
        j.rBody1.jointList = j.node1;

        j.node2.joint = j;
        j.node2.other = j.rBody1;
        j.node2.prev = null;
        j.node2.next = j.rBody2.jointList;
        if (j.rBody2.jointList) j.rBody2.jointList.prev = j.node2;
        j.rBody2.jointList = j.node2;


        // If the joint prevents collisions, then reset collision filtering.
        if (!def.collideConnected) {
            // Reset the proxies on the body with the minimum number of shapes.
            bzBody b = j.rBody1.shapeCount < j.rBody2.shapeCount ? j.rBody1 : j.rBody2;
            for (bzShape s = b.shapeList(); s; s = s.next) {
                s.refilterProxy(broadPhase, b.xf);
            }
        }

        return j;
    }

    /**
     * Destroy a joint. This may cause the connected bodies to begin colliding.
     * Params: j = the joint to destroy
     * Warning This function is locked during callbacks.
     */
    void destroyJoint(bzJoint j) {

        assert(!m_lock);

        bool collideConnected = j.collideConnected();

        // Remove from the doubly linked list.
        if (j.prev !is null) {
            j.prev.next = j.next;
        }

        if (j.next) {
            j.next.prev = j.prev;
        }

        if (j == m_jointList) {
            m_jointList = j.next;
        }

        // Disconnect from island graph.
        bzBody body1 = j.rBody1;
        bzBody body2 = j.rBody2;

        // Wake up connected bodies.
        body1.wakeup();
        body2.wakeup();

        // Remove from body 1.
        if (j.node1.prev) {
            j.node1.prev.next = j.node1.next;
        }

        if (j.node1.next) {
            j.node1.next.prev = j.node1.prev;
        }

        if (j.node1 == body1.jointList) {
            body1.jointList = j.node1.next;
        }

        j.node1.prev = null;
        j.node1.next = null;

        // Remove from body 2
        if (j.node2.prev) {
            j.node2.prev.next = j.node2.next;
        }

        if (j.node2.next) {
            j.node2.next.prev = j.node2.prev;
        }

        if (j.node2 == body2.jointList) {
            body2.jointList = j.node2.next;
        }

        j.node2.prev = null;
        j.node2.next = null;

        delete j;

        assert(m_jointCount > 0);
        --m_jointCount;

        //If the joint prevents collisions, then reset collision filtering.
        if (collideConnected == false) {
            // Reset the proxies on the body with the minimum number of shapes.
            bzBody b = body1.shapeCount < body2.shapeCount ? body1 : body2;
            for (bzShape s = b.shapeList(); s; s = s.next) {
                s.refilterProxy(broadPhase, b.xf);
            }
        }

    }

    /**
     * Applies a force to a rigid body.
     */
    bool addForce(bzForceGenerator fg) {
        //TODO check for duplicates
        m_forceRegistry.add(fg);
        return true;
    }

    /**
     * Removes a force from a rigid body.
     */
    bool removeForce(bzForceGenerator fg) {
        return m_forceRegistry.remove(fg);
    }

    /**
     * Return a list of world forces
     */
    bzForceGenerator[] forces() {
        return m_forceRegistry.forces();
    }

    /**
     * The world provides a single static ground body with no collision shapes.
     * You can use this to simplify the creation of joints and static shapes.
     */
    bzBody groundBody() {
        return m_groundBody;
    }

    /**
     * Take a time step. This performs collision detection, integration,
     * and constraint solution.
     * Params:
     *     timeStep = the amount of time to simulate, this should not vary.
     *     velocityIterations = for the velocity constraint solver.
     *     positionIterations = for the position constraint solver.
     */
    void step(float dt, int velocityIterations, int positionIterations) {

        m_lock = true;
        bzTimeStep step;
        step.dt = dt;
        step.velocityIterations	= velocityIterations;
        step.positionIterations = positionIterations;

        if (dt > 0.0f) {
            step.inv_dt = 1.0f / dt;
        } else {
            step.inv_dt = 0.0f;
        }

        step.dtRatio = m_inv_dt0 * dt;
        step.warmStarting = m_warmStarting;

        // Update narrow-phase collision detection
        m_contactManager.collide();

        // evaluate all attached forces
        m_forceRegistry.evaluate();

        // Integrate velocities, solve velocity constraints, and integrate
        // positions.
        if (step.dt > 0.0f) {
            solve(step);
        }

        // Handle TOI events.
        if (m_continuousPhysics && step.dt > 0.0f) {
            solveTOI(step);
        }

        m_inv_dt0 = step.inv_dt;
        m_lock = false;
    }

    /**
     * Query the world for all shapes that potentially overlap the
     * provided bzAABB. You provide a shape pointer buffer of specified
     * size. The number of shapes found is returned.
     * Params:
     *     aabb = the query box.
     *     results = an array in which the results are stored.
     */
    bzShape[] query(bzAABB aabb, int maxCount) {
        Object[] objs = broadPhase.query(aabb, maxCount);
        bzShape[] ret;
        ret.length = objs.length;
        for (int i=0; i<ret.length; ++i) {
            ret[i] = cast(bzShape)(objs[i]);
        }
        return ret;
    }

    /**
     * Query the world for all shapes that intersect a given segment.
     * You provide a shape pointer buffer of specified size. The number of
     * shapes found is returned, and the buffer is filled in order of
     * intersection
     * Params:
     *     segment = defines the begin and end point of the ray cast, from p1
     *         to p2. Use bzSegment.Extend to create (semi-)infinite rays
     *     shapes = an array in which all intersecting shapes are stored
     *     solidShapes = determines if shapes that the ray starts in are counted
     *         as hits.
     *     userData = passed through the worlds contact filter, with method
     *         RayCollide. This can be used to filter valid shapes
     * Returns: the number of shapes found
     * Bugs: This doesn't work. At all.
     */
    int raycast(bzSegment segment, bzShape[] shapes, bool solidShapes,
                Object userData) {
        m_raycastSegment = segment;
        m_raycastUserData = userData;
        m_raycastSolidShape = solidShapes;
        bzShape[] results;
        // TODO: Add later
        int count; // = broadPhase.querySegment(segment,results,maxCount, RaycastSortKey);
        shapes = results.dup;
        delete results;
        return count;
    }

    /**
     * Performs a raycast as with Raycast, finding the first intersecting shape.
     * Params:
     *     segment = defines the begin and end point of the ray cast, from p1 to
     *         p2. Use bzSegment.Extend to create (semi-)infinite rays.
     *     lambda = returns the hit fraction. You can use this to compute the
     *         contact point p = (1 - lambda) * segment.p1 + lambda * segment.p2
     *     normal = returns the normal at the contact point. If there is no
     *         intersection, the normal is not set.
     *     solidShapes = determines if shapes that the ray starts in are counted
     *         as hits.
     * Returns: the colliding shape shape, or null if not found
     * Bugs: Because raycast doesn't work, this won't either.
     */
    bzShape raycastOne(bzSegment segment, ref float lambda, ref bzVec2 normal, bool solidShapes, Object userData) {
        int maxCount = 1;
        bzShape[] shape;
        shape.length = 1;
        int count = raycast(segment, shape, solidShapes, userData);
        if (count==0) return null;
        assert(count==1);
        // Redundantly do TestSegment a second time, as the previous one's
        // results are inaccessible
        bzXForm xf = shape[0].rBody.xf;
        shape[0].testSegment(xf, lambda, normal,segment,1);
        //We already know it returns true
        return shape[0];
    }

    /**
     * Add fluid particles to the world
     * Params: shape = the fluid shape to add.
     * See_also: blaze.collision.shapes.bzFluidParticle
     */
    void addFluidParticle(bzShape shape) {
        if (sph is null) {
            sph = new bzSPHSimulation(this);
        }
        if (shape.type != bzShapeType.FLUID) {
            throw new Exception("Invalid shape type");
        }
        sph.addParticle(cast(bzFluidParticle)shape);
    }

    /**
     * Returns: the Fluid Particle list (or null)
     */
    bzFluidParticle[] particles() {
        if (sph) {
            return sph.particles();
        }
        return null;
    }

    /**
     * Get the world body list. With the returned body, use bzBody::GetNext to get
     * the next body in the world list. A null body indicates the end of the list.
     * Returns: the head of the world body list.
     */
    bzBody bodyList() {
        return m_bodyList;
    }

    /**
     * Get the world joint list. With the returned joint, use bzJoint::GetNext to
     * get the next joint in the world list. A null joint indicates the end of
     * the list.
     * Returns: the head of the world joint list.
     */
    bzJoint jointList() {
        return m_jointList;
    }

    /// Enable/disable warm starting. For testing.
    void warmStarting(bool flag) {
        m_warmStarting = flag;
    }

    /// Enable/disable continuous physics. For testing.
    void continuousPhysics(bool flag) {
        m_continuousPhysics = flag;
    }

    /**
     * Perform validation of internal data structures.
     * Bugs: Currently does nothing.
     */
    void validate() {
        broadPhase.validate();
    }

    /**
     * Returns: the number of bodies.
     */
    int bodyCount() {
        // There's a crime scene joke in here somewhere...
        return m_bodyCount;
    }

    /**
     * Returns: the number joints.
     */
    int jointCount() {
        return m_jointCount;
    }

    /**
     * Returns: the number of proxies.
     */
    int proxyCount() {
        return broadPhase.m_proxyCount;
    }

    /**
     * Returns: the number of contact pairs.
     */
    int pairCount() {
        return broadPhase.m_pairManager.m_pairCount;
    }

    /**
     * Change the global gravity vector.
     * Params: gravity = the new gravity vector
     */
    void gravity(bzVec2 gravity) {
        m_gravity = gravity;
        // Wake all the bodies up
        for (bzBody b = m_bodyList; b; b = b.next) {
            b.wakeup();
        }
    }

    /**
     * Returns: the global gravity vector.
     */
    bzVec2 gravity() {
        return m_gravity;
    }

    /**
     * TODO: change the name of this to "locked"
     * Returns: true if the world is locked, false otherwise
     */
    bool lock() @property {
        return m_lock;
    }

    bzBroadPhase broadPhase;

    // TODO: Why not make these private if they shouldn't be accessed?
    // Do not access
    bzContact contactList;

    int contactCount;
    bzSPHSimulation sph;

    bzContact[int] SPHContacts;

private:

    // Find islands, integrate and solve constraints, solve position constraints
    void solve(bzTimeStep step) {

        bzIsland island = new bzIsland(m_bodyCount, contactCount, m_jointCount, m_contactListener);

        // Clear all the island flags.
        for (bzBody b = m_bodyList; b; b = b.next) {
            b.flags &= ~e_islandFlag;
        }
        for (bzContact c = contactList; c; c = c.next) {
            c.flags &= ~bzContact.ISLAND;
        }
        for (bzJoint j = m_jointList; j; j = j.next) {
            j.islandFlag = false;
        }

        // Build and simulate all awake islands.
        int stackSize = m_bodyCount;
        bzBody[] stack;
        stack.length = stackSize;

        for (bzBody seed = m_bodyList; seed; seed = seed.next) {

            if (seed.flags & (e_islandFlag | e_sleepFlag | e_frozenFlag)) {
                continue;
            }

            if (seed.isStatic) {
                continue;
            }

            // Reset island and stack.
            island.clear();
            stack[0] = seed;
            int stackCount = 1;

            seed.flags |= e_islandFlag;

            // Perform a depth first search (DFS) on the constraint graph.
            while (stackCount > 0) {
                // Grab the next body off the stack and add it to the island.
                bzBody b = stack[--stackCount];
                island.add(b);

                // Make sure the body is awake.
                b.flags &= ~e_sleepFlag;

                // To keep islands as small as possible, we don't
                // propagate islands across static bodies.
                if (b.isStatic) {
                    continue;
                }

                // Search all contacts connected to this body.
                for (bzContactEdge cn = b.contactList; cn; cn = cn.next) {
                    // Has this contact already been added to an island?
                    if (cn.contact.flags & (bzContact.ISLAND | bzContact.NON_SOLID)) {
                        continue;
                    }

                    // Is this contact touching?
                    if (cn.contact.manifoldCount() == 0) {
                        continue;
                    }

                    island.add(cn.contact);
                    cn.contact.flags |= bzContact.ISLAND;

                    bzBody other = cn.other;

                    // Was the other body already added to this island?
                    if (other.flags & e_islandFlag) {
                        continue;
                    }

                    assert(stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.flags |= e_islandFlag;
                }

                // Search all joints connect to this body.
                for (bzJointEdge jn = b.jointList; jn; jn = jn.next) {
                    if (jn.joint.islandFlag) {
                        continue;
                    }

                    island.add(jn.joint);
                    jn.joint.islandFlag = true;

                    bzBody other = jn.other;
                    if (other.flags & e_islandFlag) {
                        continue;
                    }

                    assert(stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.flags |= e_islandFlag;
                }
            }

            island.solve(step, m_gravity, m_allowSleep);

            // Post solve cleanup.
            for (int i = 0; i < island.numBodies; ++i) {
                // Allow static bodies to participate in other islands.
                bzBody b = island.bodies[i];
                if (b.isStatic) {
                    b.flags &= ~e_islandFlag;
                }
            }
        }

        // Synchronize shapes, check for out of range bodies.
        for (bzBody b = m_bodyList; b; b = b.next) {

            if ((b.flags & (e_sleepFlag | e_frozenFlag)) > 0) {
                continue;
            }

            if (b.isStatic) {
                continue;
            }

            // Update shapes (for broad-phase). If the shapes go out of
            // the world bzAABB then shapes and contacts may be destroyed,
            // including contacts that are
            bool inRange = b.synchronizeShapes();

            // Did the body's shapes leave the world?
            if (!inRange && m_boundaryListener) {
                m_boundaryListener.violation(b);
            }
        }

        // Update fluid dynamics
        if (sph !is null) {
            sph.update(m_gravity, step);
            foreach(c; SPHContacts) {
                c.update(null);
            }
        }

        // Reset fluid forces
        if (sph !is null) {
            sph.resetForce();
        }

        // Commit shape proxy movements to the broad-phase so that new contacts are created.
        // Also, some contacts can be destroyed.
        broadPhase.commit();

    }

    // Find TOI contacts and solve them.
    void solveTOI(bzTimeStep step) {

        auto island = new bzIsland(m_bodyCount, k_maxTOIContactsPerIsland, k_maxTOIJointsPerIsland, m_contactListener);

        //Simple one pass queue
        //Relies on the fact that we're only making one pass
        //through and each body can only be pushed/popped once.
        //To push:
        //  queue[queueStart+queueSize++] = newElement;
        //To pop:
        //	poppedElement = queue[queueStart++];
        //  --queueSize;
        int queueCapacity = m_bodyCount;
        bzBody[] queue;
        queue.length = queueCapacity;

        for (bzBody b = m_bodyList; b; b = b.next) {
            b.flags &= ~e_islandFlag;
            b.sweep.t0 = 0.0f;
        }

        for (bzContact c = contactList; c; c = c.next) {
            // Invalidate TOI
            c.flags &= ~(bzContact.TOI | bzContact.ISLAND);
        }

        for (bzJoint j = m_jointList; j; j = j.next) {
            j.islandFlag = false;
        }

        // Find TOI events and solve them.
        for (;;) {
            // Find the first TOI.
            bzContact minContact = null;
            float minTOI = 1.0f;

            for (bzContact c = contactList; c; c = c.next) {

                if (c.flags & (bzContact.SLOW | bzContact.NON_SOLID)) {
                    continue;
                }

                // TODO_ERIN keep a counter on the contact, only respond to M TOIs per contact.

                float toi = 1.0f;
                if (c.flags & bzContact.TOI) {
                    // This contact has a valid cached TOI.
                    toi = c.toi;
                } else {
                    // Compute the TOI for this contact.
                    bzShape s1 = c.shape1;
                    bzShape s2 = c.shape2;
                    bzBody b1 = s1.rBody;
                    bzBody b2 = s2.rBody;

                    if ((b1.isStatic || b1.isSleeping()) && (b2.isStatic || b2.isSleeping())) {
                        continue;
                    }

                    // Put the sweeps onto the same time interval.
                    float t0 = b1.sweep.t0;

                    if (b1.sweep.t0 < b2.sweep.t0) {
                        t0 = b2.sweep.t0;
                        b1.sweep.advance(t0);
                    } else if (b2.sweep.t0 < b1.sweep.t0) {
                        t0 = b1.sweep.t0;
                        b2.sweep.advance(t0);
                    }

                    assert(t0 < 1.0f);

                    // Compute the time of impact.
                    toi = timeOfImpact(c.shape1, b1.sweep, c.shape2, b2.sweep);

                    assert(0.0f <= toi && toi <= 1.0f);

                    if (toi > 0.0f && toi < 1.0f) {
                        toi = min((1.0f - toi) * t0 + toi, 1.0f);
                    }

                    c.toi = toi;
                    c.flags |= bzContact.TOI;
                }

                if (float.epsilon < toi && toi < minTOI) {
                    // This is the minimum TOI found so far.
                    minContact = c;
                    minTOI = toi;
                }
            }

            if (minContact is null || 1.0f - 100.0f * float.epsilon < minTOI) {
                // No more TOI events. Done!
                break;
            }

            // Advance the bodies to the TOI.
            bzShape s1 = minContact.shape1;
            bzShape s2 = minContact.shape2;
            bzBody b1 = s1.rBody;
            bzBody b2 = s2.rBody;
            b1.advance(minTOI);
            b2.advance(minTOI);

            // The TOI contact likely has some new contact points.
            minContact.update(m_contactListener);
            minContact.flags &= ~bzContact.TOI;

            if (minContact.manifoldCount == 0) {
                // This shouldn't happen. Numerical error?
                //assert(false);
                continue;
            }

            // Build the TOI island. We need a dynamic seed.
            bzBody seed = b1;
            if (seed.isStatic) {
                seed = b2;
            }

            // Reset island and queue.
            island.clear();

            int queueStart = 0; //starting index for queue
            int queueSize = 0;  //elements in queue
            queue[queueStart + queueSize++] = seed;
            seed.flags |= e_islandFlag;

            // Perform a breadth first search (BFS) on the contact/joint graph.
            while (queueSize > 0) {
                // Grab the next body off the stack and add it to the island.
                bzBody b = queue[queueStart++];
                --queueSize;

                island.add(b);

                // Make sure the body is awake.
                b.flags &= ~e_sleepFlag;

                // To keep islands as small as possible, we don't
                // propagate islands across static bodies.
                if (b.isStatic) {
                    continue;
                }

                // Search all contacts connected to this body.
                for (bzContactEdge cn = b.contactList; cn; cn = cn.next) {

                    // Has this contact already been added to an island? Skip slow or non-solid contacts.
                    if (cn.contact.flags & (bzContact.ISLAND | bzContact.SLOW | bzContact.NON_SOLID)) {
                        continue;
                    }

                    // Is this contact touching? For performance we are not updating this contact.
                    if (cn.contact.manifoldCount == 0) {
                        continue;
                    }

                    island.add(cn.contact);
                    cn.contact.flags |= bzContact.ISLAND;

                    // Update other body.
                    bzBody other = cn.other;

                    // Was the other body already added to this island?
                    if (other.flags & e_islandFlag) {
                        continue;
                    }

                    // March forward, this can do no harm since this is the min TOI.
                    if (other.isStatic == false) {
                        other.advance(minTOI);
                        other.wakeup();
                    }

                    assert(queueStart + queueSize < queueCapacity);
                    queue[queueStart + queueSize++] = other;
                    other.flags |= e_islandFlag;
                }

                for (bzJointEdge jn = b.jointList; jn; jn = jn.next) {

                    if (jn.joint.islandFlag) {
                        continue;
                    }

                    island.add(jn.joint);

                    jn.joint.islandFlag = true;

                    bzBody other = jn.other;

                    if (other.flags & e_islandFlag) {
                        continue;
                    }

                    if (!other.isStatic) {
                        other.advance(minTOI);
                        other.wakeup();
                    }

                    assert(queueStart + queueSize < queueCapacity);
                    queue[queueStart + queueSize++] = other;
                    other.flags |= e_islandFlag;
                }
            }

            bzTimeStep subStep;
            subStep.warmStarting = false;
            subStep.dt = (1.0f - minTOI) * step.dt;
            assert(subStep.dt > float.epsilon);
            subStep.inv_dt = 1.0f / subStep.dt;
            subStep.dtRatio = 0.0f;
            subStep.velocityIterations = step.velocityIterations;
            subStep.positionIterations = step.positionIterations;

            island.solveTOI(subStep);

            // Post solve cleanup.
            for (int i = 0; i < island.numBodies; ++i) {
                // Allow bodies to participate in future TOI islands.
                bzBody b = island.bodies[i];
                b.flags &= ~e_islandFlag;

                if (b.flags & (e_sleepFlag | e_frozenFlag)) {
                    continue;
                }

                if (b.isStatic) {
                    continue;
                }

                // Update shapes (for broad-phase). If the shapes go out of
                // the world bzAABB then shapes and contacts may be destroyed,
                // including contacts that are
                bool inRange = b.synchronizeShapes();

                // Did the body's shapes leave the world?
                if (!inRange && m_boundaryListener) {
                    m_boundaryListener.violation(b);
                }

                // Invalidate all contact TOIs associated with this body. Some of these
                // may not be in the island because they were not touching.
                for (bzContactEdge cn = b.contactList; cn; cn = cn.next) {
                    cn.contact.flags &= ~bzContact.TOI;
                }
            }

            for (int i = 0; i < island.numContacts; ++i) {
                // Allow contacts to participate in future TOI islands.
                bzContact c = island.contacts[i];
                c.flags &= ~(bzContact.TOI | bzContact.ISLAND);
            }

            for (int i = 0; i < island.numJoints; ++i) {
                // Allow joints to participate in future TOI islands.
                bzJoint j = island.joints[i];
                j.islandFlag = false;
            }

            // Commit shape proxy movements to the broad-phase so that new contacts are created.
            // Also, some contacts can be destroyed.
            broadPhase.commit();
        }
    }

    bool m_lock;

    bzIsland island;
    bzContactManager m_contactManager;

    bzBody m_bodyList;
    bzJoint m_jointList;

    bzVec2 m_raycastNormal;
    Object m_raycastUserData;
    bzSegment m_raycastSegment;
    bool m_raycastSolidShape;

    int m_bodyCount;
    int m_jointCount;

    bzVec2 m_gravity;
    bool m_allowSleep;

    bzBody m_groundBody;

    bzForceRegistry m_forceRegistry;

    bzDestructionListener m_destructionListener;
    bzBoundaryListener m_boundaryListener;
    bzContactFilter m_contactFilter;
    bzContactListener m_contactListener;

    // This is used to compute the time step ratio to
    // support a variable time step.
    float m_inv_dt0 = 0;

    // This is for debugging the solver.
    bool m_warmStarting;

    // This is for debugging the solver.
    bool m_continuousPhysics;
}
