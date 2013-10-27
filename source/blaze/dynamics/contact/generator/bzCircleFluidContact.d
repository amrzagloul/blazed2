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
module blaze.dynamics.contact.generator.bzCircleFluidContact;

import blaze.common.bzConstants;
import blaze.common.bzMath;
import blaze.collision.shapes.bzShape;
import blaze.collision.shapes.bzShapeType;
import blaze.collision.shapes.bzCircle;
import blaze.collision.shapes.bzFluidParticle;
import blaze.collision.bzCollision;
import blaze.collision.pairwise.bzCollideCircle;
import blaze.dynamics.contact.bzContact;
import blaze.dynamics.bzWorldCallbacks;
import blaze.dynamics.bzBody;

/** Generate contacts from the narrow phase colliion detection
 * for bzCircle-bzCircle contacts.
 */
class bzCircleFluidContact : bzContact {

    /** Constructor */
    this(bzShape s1, bzShape s2) {
        assert(s1.type == bzShapeType.CIRCLE);
        assert(s2.type == bzShapeType.FLUID);
        super(s1, s2);
    }

    /** Create a new contact */
    static bzContact create(bzShape s1, bzShape s2) {
        return new bzCircleFluidContact(s1, s2);
    }

    /** Narrow phase collision detection */
    override void evaluate(bzContactListener listener) {

        auto circle = cast(bzCircle) m_shape1;
        auto particle = cast(bzFluidParticle) m_shape2;

        float restitution = particle.restitution * circle.restitution;
        float friction = particle.friction * circle.friction;
        bzVec2 penetration;
        bzVec2 penetrationNormal;

        bool collide;
        collide = collideCircleFluid(circle, particle, penetration, penetrationNormal);
        if(!collide) return;

        particle.applyImpulse(penetration, penetrationNormal, restitution, friction);
        circle.rBody.applyBuoyancyForce(particle);
    }
}
