/*
 * Copyright (c) 2007-2008, Michael Baczynski
 * Based on Box2D by Erin Catto, http://www.box2d.org
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the polygonal nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
module blaze.dynamics.contact.bzContact;

import blaze.common.bzMath;
import blaze.common.bzConstants;
import blaze.collision.shapes.bzShape;
import blaze.collision.shapes.bzShapeType;
import blaze.collision.bzCollision;
import blaze.dynamics.bzWorldCallbacks;
import blaze.dynamics.bzBody;

/// This structure is used to report contact points.
struct bzContactPoint
{
    bzShape shape1;		    ///< the first shape
    bzShape shape2;		    ///< the second shape
    bzVec2 position;		    ///< position in world coordinates
    bzVec2 velocity;		    ///< velocity of point on body2 relative to point on body1 (pre-solver)
    bzVec2 normal;		    ///< points from shape1 to shape2
    float separation = 0;	///< the separation is negative when shapes are touching
    float friction = 0;		///< the combined friction coefficient
    float restitution = 0;	///< the combined restitution coefficient
    bzContactID id;		    ///< the contact id identifies the features in contact
}

/// This structure is used to report contact point results.
struct bzContactResult {
    bzShape shape1;		        ///< the first shape
    bzShape shape2;		        ///< the second shape
    bzVec2 position;		        ///< position in world coordinates
    bzVec2 normal;			    ///< points from shape1 to shape2
    float normalImpulse = 0;	///< the normal impulse applied to body2
    float tangentImpulse = 0;	///< the tangent impulse applied to body2
    bzContactID id;			    ///< the contact id identifies the features in contact
}

/// A contact edge is used to connect bodies and contacts together
/// in a contact graph where each body is a node and each contact
/// is an edge. A contact edge belongs to a doubly linked list
/// maintained in each attached body. Each contact has two contact
/// nodes, one for each attached body.
class bzContactEdge {
    bzBody other;			///< provides quick access to the other body attached.
    bzContact contact;	///< the contact
    bzContactEdge prev;	///< the previous contact edge in the body's contact list
    bzContactEdge next;	///< the next contact edge in the body's contact list
}

/// The class manages contact between two shapes. A contact exists for each overlapping
/// bzAABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
class bzContact {

    /** bzContact manifold */
    bzManifold manifold;

    /// Get the number of manifolds. This is 0 or 1 between convex shapes.
    /// This may be greater than 1 for convex-vs-concave shapes. Each
    /// manifold holds up to two contact points with a shared contact normal.
    int manifoldCount() @property {
        return m_manifoldCount;
    }

    /// Is this contact solid?
    /// @return true if this contact should generate a response.
    bool isSolid() @property {
        return (flags & NON_SOLID) == 0;
    }

    /// Get the first shape in this contact.
    bzShape shape1() @property {
        return m_shape1;
    }

    /// Get the second shape in this contact.
    bzShape shape2() @property {
        return m_shape2;
    }

    //--------------- Internals Below -------------------

    void toi(float toi) @property {
        m_toi = toi;
    }

    float toi() @property {
        return m_toi;
    }

    this (bzShape s1, bzShape s2) {
        flags = 0;

        // work-around for bzNullContact + optimizer complaning about null dereference
        if ((s1 !is null) && (s2 !is null)) {
            if (s1.isSensor || s2.isSensor) {
                flags |= NON_SOLID;
            }
        }

        m_shape1 = s1;
        m_shape2 = s2;
        m_manifoldCount = 0;
        node1 = new bzContactEdge();
        node2 = new bzContactEdge();
		hash = this.toHash();
    }

    // flags
    static enum {
        NON_SOLID	= 0x0001,
        SLOW		= 0x0002,
        ISLAND	    = 0x0004,
        TOI		    = 0x0008,
    }

    void update(bzContactListener listener) {

        int oldCount = m_manifoldCount;

        evaluate(listener);

        // Fluid contacts are a special case
        if(m_shape2.type is bzShapeType.FLUID) return;

        int newCount = m_manifoldCount;

        auto body1 = m_shape1.rBody;
        auto body2 = m_shape2.rBody;

        if (newCount == 0 && oldCount > 0) {
            body1.wakeup();
            body2.wakeup();
        }

        // Bullets generate TOI events.
        if (body1.bullet || body1.isStatic || body2.isStatic || body2.bullet) {
            flags &= ~SLOW;
        } else {
            flags |= SLOW;
        }

    }

    abstract void evaluate(bzContactListener listener);

    int flags;

    // bzWorld pool and list pointers.
    bzContact prev;
    bzContact next;

    // Nodes for connecting bodies.
    bzContactEdge node1;
    bzContactEdge node2;

    int stamp;
    uint hash;
    bool flag;

protected:

    int m_manifoldCount;
    bzShape m_shape1;
    bzShape m_shape2;

    float m_toi;
}
