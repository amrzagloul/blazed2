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
module blaze.dynamics.contact.generator.bzEdgeCircleContact;

import blaze.common.bzConstants;
import blaze.common.bzMath;
import blaze.collision.shapes.bzShape;
import blaze.collision.shapes.bzShapeType;
import blaze.collision.shapes.bzCircle;
import blaze.collision.shapes.bzEdge;
import blaze.collision.bzCollision;
import blaze.dynamics.contact.bzContact;
import blaze.dynamics.bzWorldCallbacks;
import blaze.dynamics.bzBody;

class bzEdgeCircleContact : bzContact {
public:

    this(bzShape s1, bzShape s2) {
        assert(s1.type == bzShapeType.EDGE);
        assert(s2.type == bzShapeType.CIRCLE);
        super(s1, s2);
    }

    static bzContact create(bzShape shape1, bzShape shape2) {
        return new bzEdgeCircleContact(shape1, shape2);
    }

    override void evaluate(bzContactListener listener) {

        bzBody b1 = m_shape1.rBody;
        bzBody b2 = m_shape2.rBody;

        bzManifold m0 = manifold;

        collideEdgeAndCircle(manifold, cast(bzEdge) m_shape1, b1.xf, cast(bzCircle) m_shape2, b2.xf);

        bzContactPoint cp;
        cp.shape1 = m_shape1;
        cp.shape2 = m_shape2;
        cp.friction = mixFriction(m_shape1.friction, m_shape2.friction);
        cp.restitution = mixRestitution(m_shape1.restitution, m_shape2.restitution);

        if (manifold.pointCount > 0) {
            m_manifoldCount = 1;
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

    void collideEdgeAndCircle(ref bzManifold manifold, bzEdge edge,
        bzXForm xf1, bzCircle circle, bzXForm xf2) {

        manifold.pointCount = 0;
        bzVec2 d;
        bzVec2 c = bzMul(xf2, circle.localPosition());
        bzVec2 cLocal = bzMulT(xf1, c);
        bzVec2 n = edge.normalVector;
        bzVec2 v1 = edge.vertex1;
        bzVec2 v2 = edge.vertex2;
        float radius = circle.radius;
        float separation = 0;

        float dirDist = bzDot((cLocal - v1), edge.directionVector);
        if (dirDist <= 0) {
            d = cLocal - v1;
            if (bzDot(d, edge.corner1Vector) < 0) {
                return;
            }
            d = c - bzMul(xf1, v1);
        } else if (dirDist >= edge.length()) {
            d = cLocal - v2;
            if (bzDot(d, edge.corner2Vector) > 0) {
                return;
            }
            d = c - bzMul(xf1, v2);
        } else {
            separation = bzDot(cLocal - v1, n);
            if (separation > radius || separation < -radius) {
                return;
            }
            separation -= radius;
            manifold.normal = bzMul(xf1.R, n);
            manifold.pointCount = 1;
            manifold.points[0].id.key = 0;
            manifold.points[0].separation = separation;
            c = c - radius * manifold.normal;
            manifold.points[0].localPoint1 = bzMulT(xf1, c);
            manifold.points[0].localPoint2 = bzMulT(xf2, c);
            return;
        }

        float distSqr = bzDot(d,d);
        if (distSqr > radius * radius) {
            return;
        }

        if (distSqr < float.epsilon) {
            separation = -radius;
            manifold.normal = bzMul(xf1.R, n);
        } else {
            separation = d.normalize() - radius;
            manifold.normal = d;
        }

        manifold.pointCount = 1;
        manifold.points[0].id.key = 0;
        manifold.points[0].separation = separation;
        c = c - radius * manifold.normal;
        manifold.points[0].localPoint1 = bzMulT(xf1, c);
        manifold.points[0].localPoint2 = bzMulT(xf2, c);
    }
}
