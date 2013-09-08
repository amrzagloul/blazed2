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
module blaze.dynamics.contact.bzContactManager;

import blaze.common.bzMath;
import blaze.common.bzConstants;
import blaze.collision.shapes.bzShape;
import blaze.collision.shapes.bzFluidParticle;
import blaze.collision.shapes.bzShapeType;
import blaze.collision.bzCollision;
import blaze.collision.nbody.bzPairManager;
import blaze.dynamics.bzBody;
import blaze.dynamics.contact.bzContactFactory;
import blaze.dynamics.contact.bzContact;
import blaze.dynamics.contact.generator.bzNullContact;
import blaze.dynamics.contact.generator.bzFluidContact;
import blaze.bzWorld;

// Delegate of b2World.
class bzContactManager : bzPairCallback
{

    bzWorld m_world;
    bzContactFactory factory;

    /// This lets us provide broadphase proxy pair user data for contacts that shouldn't exist.
	bzNullContact m_nullContact;

    bool m_destroyImmediate;

    this (bzWorld world) {
        m_world = world;
        factory = new bzContactFactory();
    }

    // This is a callback from the broadphase when two bzAABB proxies begin
    // to overlap. We create a bzContact to manage the narrow phase.
    override Object pairAdded(Object proxyUserData1, Object proxyUserData2) {

        bzShape shape1 = cast(bzShape) proxyUserData1;
        bzShape shape2 = cast(bzShape) proxyUserData2;

		if (shape1.type == bzShapeType.FLUID && shape2.type == bzShapeType.FLUID) {
			bzContact c = new bzFluidContact(shape1, shape2);
			bzFluidParticle p1 = cast(bzFluidParticle) shape1;
	        bzFluidParticle p2 = cast(bzFluidParticle) shape2;
 	        p1.addNeighbor(p2);
	        p2.addNeighbor(p1);
			return c;
		}

		if(shape1.type == bzShapeType.FLUID || shape2.type == bzShapeType.FLUID) {
			bzContact c = factory.create(shape1, shape2);
	        m_world.SPHContacts[c.hash] = c;
			return c;
		}

        bzBody body1 = shape1.rBody;
        bzBody body2 = shape2.rBody;

        if (body1.isStatic && body2.isStatic) {
            return m_nullContact;
        }

        if (shape1.rBody is shape2.rBody) {
            return m_nullContact;
        }

        if (body2.isConnected(body1)) {
            return m_nullContact;
        }

        if (m_world.contactFilter() && !m_world.contactFilter().shouldCollide(shape1, shape2)) {
            return m_nullContact;
        }

        // Call the factory.
        bzContact c = factory.create(shape1, shape2);

        if (c is null) {
            return m_nullContact;
        }

        // bzContact creation may swap shapes.
        shape1 = c.shape1;
        shape2 = c.shape2;
        body1 = shape1.rBody;
        body2 = shape2.rBody;

        // Insert into the world.
        c.prev = null;
        c.next = m_world.contactList;
        if (m_world.contactList) {
            m_world.contactList.prev = c;
        }
        m_world.contactList = c;

        // Connect to island graph.

        // Connect to body 1
        c.node1.contact = c;
        c.node1.other = body2;

        c.node1.prev = null;
        c.node1.next = body1.contactList;
        if (body1.contactList) {
            body1.contactList.prev = c.node1;
        }
        body1.contactList = c.node1;

        // Connect to body 2
        c.node2.contact = c;
        c.node2.other = body1;

        c.node2.prev = null;
        c.node2.next = body2.contactList;
        if (body2.contactList) {
            body2.contactList.prev = c.node2;
        }
        body2.contactList = c.node2;

        m_world.contactCount++;
        return c;
    }

    // This is a callback from the broadphase when two bzAABB proxies cease
    // to overlap. We retire the bzContact.
    override void pairRemoved(Object proxyUserData1, Object proxyUserData2, Object pairUserData) {

        if (pairUserData is null) {
            return;
        }

        bzContact c = cast(bzContact) pairUserData;
        if (c == m_nullContact)
        {
            return;
        }

        // An attached body is being destroyed, we must destroy this contact
        // immediately to avoid orphaned shape pointers.
        destroy(c);
    }

    void destroy(bzContact c) {

        bzShape shape1 = c.shape1;
        bzShape shape2 = c.shape2;

		if (shape1.type == bzShapeType.FLUID && shape2.type == bzShapeType.FLUID) {
			bzFluidParticle p1 = cast(bzFluidParticle) shape1;
	        bzFluidParticle p2 = cast(bzFluidParticle) shape2;
 	        p1.removeNeighbor(p2);
	        p2.removeNeighbor(p1);
			delete c;
			return;
		}

		if(shape1.type == bzShapeType.FLUID || shape2.type == bzShapeType.FLUID) {
			int key = c.hash;
	        m_world.SPHContacts.remove(key);
			delete c;
			return;
		}

        // Inform the user that this contact is ending.
        int manifoldCount = c.manifoldCount;
        if (manifoldCount > 0 && m_world.contactListener) {

            bzBody b1 = shape1.rBody;
            bzBody b2 = shape2.rBody;

            bzManifold manifold = c.manifold;
            bzContactPoint cp;
            cp.shape1 = c.shape1;
            cp.shape2 = c.shape2;
            cp.friction = mixFriction(shape1.friction, shape2.friction);
            cp.restitution = mixRestitution(shape1.restitution, shape2.restitution);

            for (int i = 0; i < manifoldCount; ++i) {
                cp.normal = manifold.normal;
                for (int j = 0; j < manifold.pointCount; ++j) {
                    bzManifoldPoint *mp = &manifold.points[j];
                    cp.position = b1.worldPoint(mp.localPoint1);
                    bzVec2 v1 = b1.linearVelocityFromLocalPoint(mp.localPoint1);
                    bzVec2 v2 = b2.linearVelocityFromLocalPoint(mp.localPoint2);
                    cp.velocity = v2 - v1;
                    cp.separation = mp.separation;
                    cp.id = mp.id;
                    m_world.contactListener.remove(cp);
                }
            }
        }

        // Remove from the world.
        if (c.prev) {
            c.prev.next = c.next;
        }

        if (c.next) {
            c.next.prev = c.prev;
        }

        if (c == m_world.contactList) {
            m_world.contactList = c.next;
        }

        bzBody body1 = shape1.rBody;
        bzBody body2 = shape2.rBody;

        // Remove from body 1
        if (c.node1.prev) {
            c.node1.prev.next = c.node1.next;
        }

        if (c.node1.next) {
            c.node1.next.prev = c.node1.prev;
        }

        if (c.node1 == body1.contactList) {
            body1.contactList = c.node1.next;
        }

        // Remove from body 2
        if (c.node2.prev) {
            c.node2.prev.next = c.node2.next;
        }

        if (c.node2.next) {
            c.node2.next.prev = c.node2.prev;
        }

        if (c.node2 == body2.contactList) {
            body2.contactList = c.node2.next;
        }

        // Call the factory.
        //factory.destroy(c);
        delete c;
        m_world.contactCount--;
    }

// This is the top level collision call for the time step. Here
// all the narrow phase collision is processed for the world
// contact list.
    void collide() {
        // Update awake contacts.
        for (bzContact c = m_world.contactList; c; c = c.next) {
            bzBody body1 = c.shape1.rBody;
            bzBody body2 = c.shape2.rBody;
            if ((body1.flags & body2.flags) & e_sleepFlag) {
                continue;
            }
            c.update(m_world.contactListener);
        }
    }
}
