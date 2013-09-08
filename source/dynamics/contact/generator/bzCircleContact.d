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
module blaze.dynamics.contact.generator.bzCircleContact;

import blaze.common.bzConstants;
import blaze.common.bzMath;
import blaze.collision.shapes.bzShape;
import blaze.collision.shapes.bzCircle;
import blaze.collision.shapes.bzShapeType;
import blaze.collision.bzCollision;
import blaze.collision.pairwise.bzCollideCircle;
import blaze.dynamics.bzWorldCallbacks;
import blaze.dynamics.contact.bzContact;
import blaze.dynamics.bzWorldCallbacks;
import blaze.dynamics.bzBody;

/** Generate contacts from the narrow phase colliion detection
 * for bzCircle-bzCircle contacts.
 */
class bzCircleContact : bzContact {

    this(bzShape s1, bzShape s2) {
        assert(s1.type == bzShapeType.CIRCLE);
        assert(s2.type == bzShapeType.CIRCLE);
        super(s1, s2);
    }

    static bzContact create(bzShape s1, bzShape s2) {
        return new bzCircleContact(s1, s2);
    }

    override void evaluate(bzContactListener listener) {

        bzBody b1 = m_shape1.rBody;
        bzBody b2 = m_shape2.rBody;

        // Make a copy
        bzManifold m0 = manifold;

        collideCircles(manifold, cast (bzCircle) m_shape1, cast (bzCircle) m_shape2);

        bzContactPoint cp;
        cp.shape1 = m_shape1;
        cp.shape2 = m_shape2;
        cp.friction = mixFriction(m_shape1.friction, m_shape2.friction);
        cp.restitution = mixRestitution(m_shape1.restitution, m_shape2.restitution);

        if (manifold.pointCount > 0) {
            bzManifoldPoint *mp = &manifold.points[0];
            if (m0.pointCount == 0) {
                mp.normalImpulse = 0.0f;
                mp.tangentImpulse = 0.0f;

                if (listener) {
                    cp.position = b1.worldPoint(mp.localPoint1);
                    bzVec2 v1 = b1.linearVelocityFromLocalPoint(mp.localPoint1);
                    bzVec2 v2 = b2.linearVelocityFromLocalPoint(mp.localPoint2);
                    cp.velocity = v2 - v1;
                    cp.normal = manifold.normal;
                    cp.separation = mp.separation;
                    cp.id = mp.id;
                    listener.add(cp);
                }
            } else {
                bzManifoldPoint mp0 = m0.points[0];
                mp.normalImpulse = mp0.normalImpulse;
                mp.tangentImpulse = mp0.tangentImpulse;

                if (listener) {
                    cp.position = b1.worldPoint(mp.localPoint1);
                    bzVec2 v1 = b1.linearVelocityFromLocalPoint(mp.localPoint1);
                    bzVec2 v2 = b2.linearVelocityFromLocalPoint(mp.localPoint2);
                    cp.velocity = v2 - v1;
                    cp.normal = manifold.normal;
                    cp.separation = mp.separation;
                    cp.id = mp.id;
                    listener.persist(cp);
                }
            }
            m_manifoldCount = 1;
        } else {
            m_manifoldCount = 0;
            if (m0.pointCount > 0 && listener) {
                bzManifoldPoint mp0 = m0.points[0];
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
}
