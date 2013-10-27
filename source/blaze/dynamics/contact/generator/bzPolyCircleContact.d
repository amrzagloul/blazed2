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

module blaze.dynamics.contact.generator.bzPolyCircleContact;

import blaze.common.bzConstants;
import blaze.common.bzMath;
import blaze.collision.shapes.bzShape;
import blaze.collision.shapes.bzShapeType;
import blaze.collision.shapes.bzPolygon;
import blaze.collision.shapes.bzCircle;
import blaze.collision.pairwise.bzCollideCircle;
import blaze.dynamics.bzWorldCallbacks;
import blaze.collision.bzCollision;
import blaze.dynamics.contact.bzContact;
import blaze.dynamics.bzWorldCallbacks;
import blaze.dynamics.bzBody;

/** Generate contacts from the narrow phase colliion detection
 * for bzPolygon-bzCircle contacts.
 */
class bzPolyCircleContact : bzContact {

    this (bzShape s1, bzShape s2) {
        assert(s1.type == bzShapeType.POLYGON);
        assert(s2.type == bzShapeType.CIRCLE);
        super(s1, s2);
    }

    static bzContact create(bzShape s1, bzShape s2) {
        return new bzPolyCircleContact(s1, s2);
    }

    override void evaluate(bzContactListener listener) {

        bzBody b1 = m_shape1.rBody;
        bzBody b2 = m_shape2.rBody;

        bzManifold m0 = manifold;

        collidePolygonCircle(manifold, cast(bzPolygon) m_shape1, cast(bzCircle) m_shape2);

        bool persisted[k_maxManifoldPoints] = [false, false];

        bzContactPoint cp;
        cp.shape1 = m_shape1;
        cp.shape2 = m_shape2;
        cp.friction = mixFriction(m_shape1.friction, m_shape2.friction);
        cp.restitution = mixRestitution(m_shape1.restitution, m_shape2.restitution);

        // Match contact ids to facilitate warm starting.
        if (manifold.pointCount > 0) {
            // Match old contact ids to new contact ids and copy the
            // stored impulses to warm start the solver.
            for (int i = 0; i < manifold.pointCount; ++i) {
                bzManifoldPoint *mp = &manifold.points[i];
                mp.normalImpulse = 0.0f;
                mp.tangentImpulse = 0.0f;
                bool found = false;
                bzContactID id = mp.id;

                for (int j = 0; j < m0.pointCount; ++j) {
                    if (persisted[j]) {
                        continue;
                    }

                    bzManifoldPoint *mp0 = &m0.points[j];

                    if (mp0.id.key == id.key) {
                        persisted[j] = true;
                        mp.normalImpulse = mp0.normalImpulse;
                        mp.tangentImpulse = mp0.tangentImpulse;

                        // A persistent point.
                        found = true;

                        // Report persistent point.
                        if (listener !is null) {
                            cp.position = b1.worldPoint(mp.localPoint1);
                            bzVec2 v1 = b1.linearVelocityFromLocalPoint(mp.localPoint1);
                            bzVec2 v2 = b2.linearVelocityFromLocalPoint(mp.localPoint2);
                            cp.velocity = v2 - v1;
                            cp.normal = manifold.normal;
                            cp.separation = mp.separation;
                            cp.id = id;
                            listener.persist(cp);
                        }
                        break;
                    }
                }

                // Report added point.
                if (found is false && listener !is null) {
                    cp.position = b1.worldPoint(mp.localPoint1);
                    bzVec2 v1 = b1.linearVelocityFromLocalPoint(mp.localPoint1);
                    bzVec2 v2 = b2.linearVelocityFromLocalPoint(mp.localPoint2);
                    cp.velocity = v2 - v1;
                    cp.normal = manifold.normal;
                    cp.separation = mp.separation;
                    cp.id = id;
                    listener.add(cp);
                }
            }

            m_manifoldCount = 1;
        } else {
            m_manifoldCount = 0;
        }

        if (listener is null) {
            return;
        }

        // Report removed points.
        for (int i = 0; i < m0.pointCount; ++i) {
            if (persisted[i]) {
                continue;
            }

            bzManifoldPoint *mp0 = &m0.points[i];
            cp.position = b1.worldPoint(mp0.localPoint1);
            bzVec2 v1 = b1.linearVelocityFromLocalPoint(mp0.localPoint1);
            bzVec2 v2 = b2.linearVelocityFromLocalPoint(mp0.localPoint2);
            cp.velocity = v2 - v1;
            cp.normal = m0.normal;
            cp.separation = mp0.separation;
            cp.id = mp0.id;
            listener.remove(cp);
        }
    }
}
