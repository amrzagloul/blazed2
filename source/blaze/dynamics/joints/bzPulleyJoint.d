/*
* Copyright (c) 2008-2009, Mason Green (zzzzrrr)
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
module blaze.dynamics.joints.bzPulleyJoint;

import blaze.common.bzMath;
import blaze.common.bzConstants;
import blaze.dynamics.bzBody;
import blaze.bzWorld;
import blaze.dynamics.joints.bzJoint;

const k_minPulleyLength = 2.0f;

/// Pulley joint definition. This requires two ground anchors,
/// two dynamic body anchor points, max lengths for each side,
/// and a pulley ratio.
class bzPulleyJointDef : bzJointDef {

    /// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
    this (bzBody b1, bzBody b2, bzVec2 ga1 = bzVec2(0,0), bzVec2 ga2 = bzVec2(0,0),
          bzVec2 anchor1 = bzVec2(0,0), bzVec2 anchor2 = bzVec2(0,0), float r = 0.0f) {
        super(b1, b2);
        type = bzJointType.PULLEY;
        groundAnchor1 = ga1;
        groundAnchor2 = ga2;
        localAnchor1 = body1.localPoint(anchor1);
        localAnchor2 = body2.localPoint(anchor2);
        bzVec2 d1 = anchor1 - ga1;
        length1 = d1.length();
        bzVec2 d2 = anchor2 - ga2;
        length2 = d2.length();
        ratio = r;
        assert(ratio > float.epsilon);
        float C = length1 + ratio * length2;
        maxLength1 = C - ratio * k_minPulleyLength;
        maxLength2 = (C - k_minPulleyLength) / ratio;
        collideConnected = true;
    }

    /// The first ground anchor in world coordinates. This point never moves.
    bzVec2 groundAnchor1;

    /// The second ground anchor in world coordinates. This point never moves.
    bzVec2 groundAnchor2;

    /// The local anchor point relative to body1's origin.
    bzVec2 localAnchor1;

    /// The local anchor point relative to body2's origin.
    bzVec2 localAnchor2;

    /// The a reference length for the segment attached to body1.
    float length1;

    /// The maximum length of the segment attached to body1.
    float maxLength1;

    /// The a reference length for the segment attached to body2.
    float length2;

    /// The maximum length of the segment attached to body2.
    float maxLength2;

    /// The pulley ratio, used to simulate a block-and-tackle.
    float ratio;
}

/// The pulley joint is connected to two bodies and two fixed ground points.
/// The pulley supports a ratio such that:
/// length1 + ratio * length2 <= constant
/// Yes, the force transmitted is scaled by the ratio.
/// The pulley also enforces a maximum length limit on both sides. This is
/// useful to prevent one side of the pulley hitting the top.
class bzPulleyJoint : bzJoint {

    override bzVec2 anchor1() {
        return m_body1.worldPoint(m_localAnchor1);
    }

    override bzVec2 anchor2() {
        return m_body2.worldPoint(m_localAnchor2);
    }

    override bzVec2 reactionForce(float inv_dt) {
        bzVec2 P = m_impulse * m_u2;
        return inv_dt * P;
    }

    override float reactionTorque(float inv_dt) {
        return 0.0f;
    }

    /// Get the first ground anchor.
    bzVec2 groundAnchor1() {
        return m_ground.xf.position + m_groundAnchor1;
    }

    /// Get the second ground anchor.
    bzVec2 groundAnchor2() {
        return m_ground.xf.position + m_groundAnchor2;
    }

    /// Get the current length of the segment attached to body1.
    float length1() {
        bzVec2 p = m_body1.worldPoint(m_localAnchor1);
        bzVec2 s = m_ground.xf.position + m_groundAnchor1;
        bzVec2 d = p - s;
        return d.length();
    }

    /// Get the current length of the segment attached to body2.
    float length2() {
        bzVec2 p = m_body2.worldPoint(m_localAnchor2);
        bzVec2 s = m_ground.xf.position + m_groundAnchor2;
        bzVec2 d = p - s;
        return d.length();
    }

    /// Get the pulley ratio.
    float ratio() {
        return m_ratio;
    }

    //--------------- Internals Below -------------------
    // Pulley:
    // length1 = norm(p1 - s1)
    // length2 = norm(p2 - s2)
    // C0 = (length1 + ratio * length2)_initial
    // C = C0 - (length1 + ratio * length2) >= 0
    // u1 = (p1 - s1) / norm(p1 - s1)
    // u2 = (p2 - s2) / norm(p2 - s2)
    // Cdot = -bzDot(u1, v1 + bzCross(w1, r1)) - ratio * bzDot(u2, v2 + bzCross(w2, r2))
    // J = -[u1 bzCross(r1, u1) ratio * u2  ratio * bzCross(r2, u2)]
    // K = J * invM * JT
    //   = invMass1 + invI1 * bzCross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * bzCross(r2, u2)^2)
    //
    // Limit:
    // C = maxLength - length
    // u = (p - s) / norm(p - s)
    // Cdot = -bzDot(u, v + bzCross(w, r))
    // K = invMass + invI * bzCross(r, u)^2
    // 0 <= impulse

    this(bzPulleyJointDef def) {
        super(def);
        m_ground = m_body1.world().groundBody();
        m_groundAnchor1 = def.groundAnchor1 - m_ground.xf.position;
        m_groundAnchor2 = def.groundAnchor2 - m_ground.xf.position;
        m_localAnchor1 = def.localAnchor1;
        m_localAnchor2 = def.localAnchor2;

        assert(def.ratio != 0.0f);
        m_ratio = def.ratio;

        m_constant = def.length1 + m_ratio * def.length2;

        m_maxLength1 = min(def.maxLength1, m_constant - m_ratio * k_minPulleyLength);
        m_maxLength2 = min(def.maxLength2, (m_constant - k_minPulleyLength) / m_ratio);

        m_impulse = 0.0f;
        m_limitImpulse1 = 0.0f;
        m_limitImpulse2 = 0.0f;
    }

    override void initVelocityConstraints(bzTimeStep step) {

        bzBody b1 = m_body1;
        bzBody b2 = m_body2;

        bzVec2 r1 = bzMul(b1.xf.R, m_localAnchor1 - b1.localCenter);
        bzVec2 r2 = bzMul(b2.xf.R, m_localAnchor2 - b2.localCenter);

        bzVec2 p1 = b1.sweep.c + r1;
        bzVec2 p2 = b2.sweep.c + r2;

        bzVec2 s1 = m_ground.xf.position + m_groundAnchor1;
        bzVec2 s2 = m_ground.xf.position + m_groundAnchor2;

        // Get the pulley axes.
        m_u1 = p1 - s1;
        m_u2 = p2 - s2;

        float length1 = m_u1.length();
        float length2 = m_u2.length();

        if (length1 > k_linearSlop) {
            m_u1 *= 1.0f / length1;
        } else {
            m_u1.zero();
        }

        if (length2 > k_linearSlop) {
            m_u2 *= 1.0f / length2;
        } else {
            m_u2.zero();
        }

        float C = m_constant - length1 - m_ratio * length2;
        if (C > 0.0f) {
            m_state = bzLimitState.INACTIVE;
            m_impulse = 0.0f;
        } else {
            m_state = bzLimitState.UPPER;
        }

        if (length1 < m_maxLength1) {
            m_limitState1 = bzLimitState.INACTIVE;
            m_limitImpulse1 = 0.0f;
        } else {
            m_limitState1 = bzLimitState.UPPER;
        }

        if (length2 < m_maxLength2) {
            m_limitState2 = bzLimitState.INACTIVE;
            m_limitImpulse2 = 0.0f;
        } else {
            m_limitState2 = bzLimitState.UPPER;
        }

        // Compute effective mass.
        float cr1u1 = bzCross(r1, m_u1);
        float cr2u2 = bzCross(r2, m_u2);

        m_limitMass1 = b1.invMass + b1.invI * cr1u1 * cr1u1;
        m_limitMass2 = b2.invMass + b2.invI * cr2u2 * cr2u2;
        m_pulleyMass = m_limitMass1 + m_ratio * m_ratio * m_limitMass2;
        assert(m_limitMass1 > float.epsilon);
        assert(m_limitMass2 > float.epsilon);
        assert(m_pulleyMass > float.epsilon);
        m_limitMass1 = 1.0f / m_limitMass1;
        m_limitMass2 = 1.0f / m_limitMass2;
        m_pulleyMass = 1.0f / m_pulleyMass;

        if (step.warmStarting) {
            // Scale impulses to support variable time steps.
            m_impulse *= step.dtRatio;
            m_limitImpulse1 *= step.dtRatio;
            m_limitImpulse2 *= step.dtRatio;

            // Warm starting.
            bzVec2 P1 = -(m_impulse + m_limitImpulse1) * m_u1;
            bzVec2 P2 = (-m_ratio * m_impulse - m_limitImpulse2) * m_u2;
            b1.linearVelocity = b1.linearVelocity + b1.invMass * P1;
            b1.angularVelocity = b1.angularVelocity + b1.invI * bzCross(r1, P1);
            b2.linearVelocity = b2.linearVelocity + b2.invMass * P2;
            b2.angularVelocity = b2.angularVelocity + b2.invI * bzCross(r2, P2);
        } else {
            m_impulse = 0.0f;
            m_limitImpulse1 = 0.0f;
            m_limitImpulse2 = 0.0f;
        }
    }

    override void solveVelocityConstraints(bzTimeStep step) {

        bzBody b1 = m_body1;
        bzBody b2 = m_body2;

        bzVec2 r1 = bzMul(b1.xf.R, m_localAnchor1 - b1.localCenter);
        bzVec2 r2 = bzMul(b2.xf.R, m_localAnchor2 - b2.localCenter);

        if (m_state == bzLimitState.UPPER) {
            bzVec2 v1 = b1.linearVelocity + bzCross(b1.angularVelocity, r1);
            bzVec2 v2 = b2.linearVelocity + bzCross(b2.angularVelocity, r2);

            float Cdot = -bzDot(m_u1, v1) - m_ratio * bzDot(m_u2, v2);
            float impulse = m_pulleyMass * (-Cdot);
            float oldImpulse = m_impulse;
            m_impulse = max(0.0f, m_impulse + impulse);
            impulse = m_impulse - oldImpulse;

            bzVec2 P1 = -impulse * m_u1;
            bzVec2 P2 = -m_ratio * impulse * m_u2;
            b1.linearVelocity = b1.linearVelocity + b1.invMass * P1;
            b1.angularVelocity = b1.angularVelocity + b1.invI * bzCross(r1, P1);
            b2.linearVelocity = b2.linearVelocity + b2.invMass * P2;
            b2.angularVelocity = b2.angularVelocity + b2.invI * bzCross(r2, P2);
        }

        if (m_limitState1 == bzLimitState.UPPER) {
            bzVec2 v1 = b1.linearVelocity + bzCross(b1.angularVelocity, r1);

            float Cdot = -bzDot(m_u1, v1);
            float impulse = -m_limitMass1 * Cdot;
            float oldImpulse = m_limitImpulse1;
            m_limitImpulse1 = max(0.0f, m_limitImpulse1 + impulse);
            impulse = m_limitImpulse1 - oldImpulse;

            bzVec2 P1 = -impulse * m_u1;
            b1.linearVelocity = b1.linearVelocity + b1.invMass * P1;
            b1.angularVelocity = b1.angularVelocity + b1.invI * bzCross(r1, P1);
        }

        if (m_limitState2 == bzLimitState.UPPER) {
            bzVec2 v2 = b2.linearVelocity + bzCross(b2.angularVelocity, r2);

            float Cdot = -bzDot(m_u2, v2);
            float impulse = -m_limitMass2 * Cdot;
            float oldImpulse = m_limitImpulse2;
            m_limitImpulse2 = max(0.0f, m_limitImpulse2 + impulse);
            impulse = m_limitImpulse2 - oldImpulse;

            bzVec2 P2 = -impulse * m_u2;
            b2.linearVelocity = b2.linearVelocity + b2.invMass * P2;
            b2.angularVelocity = b2.angularVelocity + b2.invI * bzCross(r2, P2);
        }
    }

    override bool solvePositionConstraints(float baumgarte) {

        bzBody b1 = m_body1;
        bzBody b2 = m_body2;

        bzVec2 s1 = m_ground.xf.position + m_groundAnchor1;
        bzVec2 s2 = m_ground.xf.position + m_groundAnchor2;

        float linearError = 0.0f;

        if (m_state == bzLimitState.UPPER) {
            bzVec2 r1 = bzMul(b1.xf.R, m_localAnchor1 - b1.localCenter);
            bzVec2 r2 = bzMul(b2.xf.R, m_localAnchor2 - b2.localCenter);

            bzVec2 p1 = b1.sweep.c + r1;
            bzVec2 p2 = b2.sweep.c + r2;

            // Get the pulley axes.
            m_u1 = p1 - s1;
            m_u2 = p2 - s2;

            float length1 = m_u1.length();
            float length2 = m_u2.length();

            if (length1 > k_linearSlop) {
                m_u1 *= 1.0f / length1;
            } else {
                m_u1.zero();
            }

            if (length2 > k_linearSlop) {
                m_u2 *= 1.0f / length2;
            } else {
                m_u2.zero();
            }

            float C = m_constant - length1 - m_ratio * length2;
            linearError = max(linearError, -C);

            C = bzClamp(C + k_linearSlop, -k_maxLinearCorrection, 0.0f);
            float impulse = -m_pulleyMass * C;

            bzVec2 P1 = -impulse * m_u1;
            bzVec2 P2 = -m_ratio * impulse * m_u2;

            b1.sweep.c = b1.sweep.c + b1.invMass * P1;
            b1.sweep.a = b1.sweep.a + b1.invI * bzCross(r1, P1);
            b2.sweep.c = b2.sweep.c + b2.invMass * P2;
            b2.sweep.a = b2.sweep.a  + b2.invI * bzCross(r2, P2);

            b1.synchronizeTransform();
            b2.synchronizeTransform();
        }

        if (m_limitState1 == bzLimitState.UPPER) {
            bzVec2 r1 = bzMul(b1.xf.R, m_localAnchor1 - b1.localCenter);
            bzVec2 p1 = b1.sweep.c + r1;

            m_u1 = p1 - s1;
            float length1 = m_u1.length();

            if (length1 > k_linearSlop) {
                m_u1 *= 1.0f / length1;
            } else {
                m_u1.zero();
            }

            float C = m_maxLength1 - length1;
            linearError = max(linearError, -C);
            C = bzClamp(C + k_linearSlop, -k_maxLinearCorrection, 0.0f);
            float impulse = -m_limitMass1 * C;

            bzVec2 P1 = -impulse * m_u1;
            b1.sweep.c = b1.sweep.c + b1.invMass * P1;
            b1.sweep.a = b1.sweep.a + b1.invI * bzCross(r1, P1);

            b1.synchronizeTransform();
        }

        if (m_limitState2 == bzLimitState.UPPER) {
            bzVec2 r2 = bzMul(b2.xf.R, m_localAnchor2 - b2.localCenter);
            bzVec2 p2 = b2.sweep.c + r2;

            m_u2 = p2 - s2;
            float length2 = m_u2.length();

            if (length2 > k_linearSlop) {
                m_u2 *= 1.0f / length2;
            } else {
                m_u2.zero();
            }

            float C = m_maxLength2 - length2;
            linearError = max(linearError, -C);
            C = bzClamp(C + k_linearSlop, -k_maxLinearCorrection, 0.0f);
            float impulse = -m_limitMass2 * C;

            bzVec2 P2 = -impulse * m_u2;
            b2.sweep.c = b2.sweep.c + b2.invMass * P2;
            b2.sweep.a = b2.sweep.a  + b2.invI * bzCross(r2, P2);

            b2.synchronizeTransform();
        }

        return linearError < k_linearSlop;
    }

    bzVec2 localAnchor1() {
        return m_localAnchor1;
    }

    bzVec2 localAnchor2() {
        return m_localAnchor2;
    }

    private:

    bzBody m_ground;
    bzVec2 m_groundAnchor1;
    bzVec2 m_groundAnchor2;
    bzVec2 m_localAnchor1;
    bzVec2 m_localAnchor2;

    bzVec2 m_u1;
    bzVec2 m_u2;

    float m_constant;
    float m_ratio;

    float m_maxLength1;
    float m_maxLength2;

    // Effective masses
    float m_pulleyMass;
    float m_limitMass1;
    float m_limitMass2;

    // Impulses for accumulation/warm starting.
    float m_impulse;
    float m_limitImpulse1;
    float m_limitImpulse2;

    bzLimitState m_state;
    bzLimitState m_limitState1;
    bzLimitState m_limitState2;
}
