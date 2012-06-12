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
module blaze.dynamics.contact.generator.bzPolyFluidContact;

import blaze.common.bzConstants;
import blaze.common.bzMath;
import blaze.collision.shapes.bzShape;
import blaze.collision.shapes.bzShapeType;
import blaze.collision.shapes.bzPolygon;
import blaze.collision.shapes.bzFluidParticle;
import blaze.collision.pairwise.bzCollidePoly;
import blaze.collision.bzCollision;
import blaze.dynamics.contact.bzContact;
import blaze.dynamics.bzWorldCallbacks;
import blaze.dynamics.bzBody;

/** Generate contacts from the narrow phase colliion detection
 * for bzPolygon-bzCircle contacts.
 */
class bzPolyFluidContact : bzContact {

    this(bzShape s1, bzShape s2) {
        assert(s1.type == bzShapeType.POLYGON);
        assert(s2.type == bzShapeType.FLUID);
        super(s1, s2);
    }

    ///
    static bzContact create(bzShape s1, bzShape s2) {
        return new bzPolyFluidContact(s1, s2);
    }

    /** Narrow phase collision detection */
    override void evaluate(bzContactListener listener) {

        auto poly = cast(bzPolygon) m_shape1;
        auto particle = cast(bzFluidParticle) m_shape2;

        float restitution = particle.restitution * poly.restitution;
        float friction = particle.friction * poly.friction;
        bzVec2 penetration;
        bzVec2 penetrationNormal;

        bool collide;
        collide = collidePolyFluid(poly, particle, penetration, penetrationNormal);
        if(!collide) return;

        particle.applyImpulse(penetration, penetrationNormal, restitution, friction);
        poly.rBody.applyBuoyancyForce(particle);

    }
}


