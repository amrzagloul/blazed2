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
module blaze.dynamics.joints.bzPrismaticJoint;

import blaze.common.bzMath;
import blaze.common.bzConstants;
import blaze.dynamics.bzBody;
import blaze.bzWorld;
import blaze.dynamics.joints.bzJoint;

/**
 * Prismatic joint definition. This requires defining a line of
 * motion using an axis and an anchor point. The definition uses local
 * anchor points and a local axis so that the initial configuration
 * can violate the constraint slightly. The joint translation is zero
 * when the local anchor points coincide in world space. Using local
 * anchors and a local axis helps when saving and loading a game.
 */
class bzPrismaticJointDef : bzJointDef {

    /**
     * Initialize the bodies, anchors, axis, and reference angle using the world
     * anchor and world axis.
     */
    this (bzBody body1, bzBody body2, bzVec2 anchor = bzVec2(0,0), bzVec2 axis = bzVec2(0,0)) {
        super(body1, body2);
        type = bzJointType.PRISMATIC;
        localAnchor1 = body1.localPoint(anchor);
        localAnchor2 = body2.localPoint(anchor);
        localAxis1 = body1.localVector(axis);
        referenceAngle = body2.angle - body1.angle;
        lowerTranslation = 0.0f;
        upperTranslation = 0.0f;
        maxMotorForce = 0.0f;
        motorSpeed = 0.0f;
    }

    /** The local anchor point relative to body1's origin. */
    bzVec2 localAnchor1;

    /** The local anchor point relative to body2's origin. */
    bzVec2 localAnchor2;

    /** The local translation axis in body1. */
    bzVec2 localAxis1;

    /** The constrained angle between the bodies: body2_angle - body1_angle. */
    float referenceAngle;

    /** Enable/disable the joint limit. */
    bool enableLimit;

    /** The lower translation limit, usually in meters. */
    float lowerTranslation;

    /** The upper translation limit, usually in meters. */
    float upperTranslation;

    /** Enable/disable the joint motor. */
    bool enableMotor;

    /** The maximum motor torque, usually in N-m. */
    float maxMotorForce;

    /** The desired motor speed in radians per second. */
    float motorSpeed;
}

/**
 * A prismatic joint. This joint provides one degree of freedom: translation
 * along an axis fixed in body1. Relative rotation is prevented. You can
 * use a joint limit to restrict the range of motion and a joint motor to
 * drive the motion or to model joint friction.
 */
class bzPrismaticJoint : bzJoint {

    override bzVec2 anchor1() {
        return m_body1.worldPoint(m_localAnchor1);
    }

    override bzVec2 anchor2() {
        return m_body2.worldPoint(m_localAnchor2);
    }

    override bzVec2 reactionForce(float inv_dt) {
        return inv_dt * (m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis);
    }

    override float reactionTorque(float inv_dt) {
        return inv_dt * m_impulse.y;
    }

    /** Get the current joint translation, usually in meters. */
    float jointTranslation() {
        bzBody b1 = m_body1;
        bzBody b2 = m_body2;

        bzVec2 p1 = b1.worldPoint(m_localAnchor1);
        bzVec2 p2 = b2.worldPoint(m_localAnchor2);
        bzVec2 d = p2 - p1;
        bzVec2 axis = b1.worldVector(m_localXAxis1);

        float translation = bzDot(d, axis);
        return translation;
    }

    /** Get the current joint translation speed, usually in meters per second. */
    float jointSpeed() {
        bzBody b1 = m_body1;
        bzBody b2 = m_body2;

        bzVec2 r1 = bzMul(b1.xf.R, m_localAnchor1 - b1.localCenter);
        bzVec2 r2 = bzMul(b2.xf.R, m_localAnchor2 - b2.localCenter);
        bzVec2 p1 = b1.sweep.c + r1;
        bzVec2 p2 = b2.sweep.c + r2;
        bzVec2 d = p2 - p1;
        bzVec2 axis = b1.worldVector(m_localXAxis1);

        bzVec2 v1 = b1.linearVelocity;
        bzVec2 v2 = b2.linearVelocity;
        float w1 = b1.angularVelocity;
        float w2 = b2.angularVelocity;

        float speed = bzDot(d, bzCross(w1, axis)) + bzDot(axis, v2 + bzCross(w2, r2) - v1 - bzCross(w1, r1));
        return speed;
    }

    /** Is the joint limit enabled? */
    bool isLimitEnabled() {
        return m_enableLimit;
    }

    /** Enable/disable the joint limit. */
    void enableLimit(bool flag) {
        m_body1.wakeup();
        m_body2.wakeup();
        m_enableLimit = flag;
    }

    /** Get the lower joint limit, usually in meters. */
    float lowerLimit() {
        return m_lowerTranslation;
    }

    /** Get the upper joint limit, usually in meters. */
    float upperLimit() {
        return m_upperTranslation;
    }

    /** Set the joint limits, usually in meters. */
    void limits(float lower, float upper) {
        assert(lower <= upper);
        m_body1.wakeup();
        m_body2.wakeup();
        m_lowerTranslation = lower;
        m_upperTranslation = upper;
    }

    /** Is the joint motor enabled? */
    bool isMotorEnabled() {
        return m_enableMotor;
    }

    /** Enable/disable the joint motor. */
    void enableMotor(bool flag) {
        m_body1.wakeup();
        m_body2.wakeup();
        m_enableMotor = flag;
    }

    /** Set the motor speed, usually in meters per second. */
    void motorSpeed(float speed) {
        m_body1.wakeup();
        m_body2.wakeup();
        m_motorSpeed = speed;
    }

    /** Get the motor speed, usually in meters per second. */
    float motorSpeed() {
        return m_motorSpeed;
    }

    /** Set the maximum motor force, usually in N. */
    void maxMotorForce(float force) {
        m_body1.wakeup();
        m_body2.wakeup();
        m_maxMotorForce = FORCE_SCALE(1)*force;
    }

    /** Get the current motor force, usually in N. */
    float motorForce() {
        return m_motorImpulse;
    }

    //--------------- Internals Below -------------------
    // Linear constraint (point-to-line)
    // d = p2 - p1 = x2 + r2 - x1 - r1
    // C = bzDot(perp, d)
    // Cdot = bzDot(d, bzCross(w1, perp)) + bzDot(perp, v2 + bzCross(w2, r2) - v1 - bzCross(w1, r1))
    //      = -bzDot(perp, v1) - bzDot(bzCross(d + r1, perp), w1) + bzDot(perp, v2) + bzDot(bzCross(r2, perp), v2)
    // J = [-perp, -bzCross(d + r1, perp), perp, bzCross(r2,perp)]
    //
    // Angular constraint
    // C = a2 - a1 + a_initial
    // Cdot = w2 - w1
    // J = [0 0 -1 0 0 1]
    //
    // K = J * invM * JT
    //
    // J = [-a -s1 a s2]
    //     [0  -1  0  1]
    // a = perp
    // s1 = bzCross(d + r1, a) = bzCross(p2 - x1, a)
    // s2 = bzCross(r2, a) = bzCross(p2 - x2, a)

    // Motor/Limit linear constraint
    // C = bzDot(ax1, d)
    // Cdot = = -bzDot(ax1, v1) - bzDot(bzCross(d + r1, ax1), w1) + bzDot(ax1, v2) + bzDot(bzCross(r2, ax1), v2)
    // J = [-ax1 -bzCross(d+r1,ax1) ax1 bzCross(r2,ax1)]

    // Block Solver
    // We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
    // when the mass has poor distribution (leading to large torques about the joint anchor points).
    //
    // The Jacobian has 3 rows:
    // J = [-uT -s1 uT s2] // linear
    //     [0   -1   0  1] // angular
    //     [-vT -a1 vT a2] // limit
    //
    // u = perp
    // v = axis
    // s1 = bzCross(d + r1, u), s2 = bzCross(r2, u)
    // a1 = bzCross(d + r1, v), a2 = bzCross(r2, v)

    // M * (v2 - v1) = JT * df
    // J * v2 = bias
    //
    // v2 = v1 + invM * JT * df
    // J * (v1 + invM * JT * df) = bias
    // K * df = bias - J * v1 = -Cdot
    // K = J * invM * JT
    // Cdot = J * v1 - bias
    //
    // Now solve for f2.
    // df = f2 - f1
    // K * (f2 - f1) = -Cdot
    // f2 = invK * (-Cdot) + f1
    //
    // Clamp accumulated limit impulse.
    // lower: f2(3) = max(f2(3), 0)
    // upper: f2(3) = min(f2(3), 0)
    //
    // Solve for correct f2(1:2)
    // K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
    //                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
    // K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
    // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
    //
    // Now compute impulse to be applied:
    // df = f2 - f1

    this (bzPrismaticJointDef def) {
        super(def);
        m_localAnchor1 = def.localAnchor1;
        m_localAnchor2 = def.localAnchor2;
        m_localXAxis1 = def.localAxis1;
        m_localYAxis1 = bzCross(1.0f, m_localXAxis1);
        m_refAngle = def.referenceAngle;

        m_impulse.zero();
        m_motorMass = 0.0;
        m_motorImpulse = 0.0f;

        m_lowerTranslation = def.lowerTranslation;
        m_upperTranslation = def.upperTranslation;
        m_maxMotorForce = FORCE_INV_SCALE(def.maxMotorForce);
        m_motorSpeed = def.motorSpeed;
        m_enableLimit = def.enableLimit;
        m_enableMotor = def.enableMotor;

        m_axis.zero();
        m_perp.zero();
    }

    override void initVelocityConstraints(bzTimeStep step) {

        bzBody b1 = m_body1;
        bzBody b2 = m_body2;

        // You cannot create a prismatic joint between bodies that
        // both have fixed rotation.
        assert(b1.invI > 0.0f || b2.invI > 0.0f);

        m_localCenter1 = b1.localCenter;
        m_localCenter2 = b2.localCenter;

        bzXForm xf1 = b1.xf;
        bzXForm xf2 = b2.xf;

        // Compute the effective masses.
        bzVec2 r1 = bzMul(xf1.R, m_localAnchor1 - m_localCenter1);
        bzVec2 r2 = bzMul(xf2.R, m_localAnchor2 - m_localCenter2);
        bzVec2 d = b2.sweep.c + r2 - b1.sweep.c - r1;

        m_invMass1 = b1.invMass;
        m_invI1 = b1.invI;
        m_invMass2 = b2.invMass;
        m_invI2 = b2.invI;

        // Compute motor Jacobian and effective mass.
        {
            m_axis = bzMul(xf1.R, m_localXAxis1);
            m_a1 = bzCross(d + r1, m_axis);
            m_a2 = bzCross(r2, m_axis);

            m_motorMass = m_invMass1 + m_invMass2 + m_invI1 * m_a1 * m_a1 + m_invI2 * m_a2 * m_a2;
            assert(m_motorMass > float.epsilon);
            m_motorMass = 1.0f / m_motorMass;
        }

        // Prismatic constraint.
        {
            m_perp = bzMul(xf1.R, m_localYAxis1);

            m_s1 = bzCross(d + r1, m_perp);
            m_s2 = bzCross(r2, m_perp);

            float m1 = m_invMass1, m2 = m_invMass2;
            float i1 = m_invI1, i2 = m_invI2;

            float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
            float k12 = i1 * m_s1 + i2 * m_s2;
            float k13 = i1 * m_s1 * m_a1 + i2 * m_s2 * m_a2;
            float k22 = i1 + i2;
            float k23 = i1 * m_a1 + i2 * m_a2;
            float k33 = m1 + m2 + i1 * m_a1 * m_a1 + i2 * m_a2 * m_a2;

            m_K.col1.set(k11, k12, k13);
            m_K.col2.set(k12, k22, k23);
            m_K.col3.set(k13, k23, k33);
        }

        // Compute motor and limit terms.
        if (m_enableLimit) {
            float jointTranslation = bzDot(m_axis, d);
            if (abs(m_upperTranslation - m_lowerTranslation) < 2.0f * k_linearSlop) {
                m_limitState = bzLimitState.EQUAL;
            } else if (jointTranslation <= m_lowerTranslation) {
                if (m_limitState != bzLimitState.LOWER) {
                    m_limitState = bzLimitState.LOWER;
                    m_impulse.z = 0.0f;
                }
            } else if (jointTranslation >= m_upperTranslation) {
                if (m_limitState != bzLimitState.UPPER) {
                    m_limitState = bzLimitState.UPPER;
                    m_impulse.z = 0.0f;
                }
            } else {
                m_limitState = bzLimitState.INACTIVE;
                m_impulse.z = 0.0f;
            }
        }

        if (m_enableMotor == false) {
            m_motorImpulse = 0.0f;
        }

        if (step.warmStarting) {
            // Account for variable time step.
            m_impulse *= step.dtRatio;
            m_motorImpulse *= step.dtRatio;

            bzVec2 P = m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis;
            float L1 = m_impulse.x * m_s1 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a1;
            float L2 = m_impulse.x * m_s2 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a2;

            b1.linearVelocity = b1.linearVelocity - m_invMass1 * P;
            b1.angularVelocity = b1.angularVelocity - m_invI1 * L1;

            b2.linearVelocity = b2.linearVelocity + m_invMass2 * P;
            b2.angularVelocity = b2.angularVelocity + m_invI2 * L2;
        } else {
            m_impulse.zero();
            m_motorImpulse = 0.0f;
        }
    }

    override void solveVelocityConstraints(bzTimeStep step) {

        bzBody b1 = m_body1;
        bzBody b2 = m_body2;

        bzVec2 v1 = b1.linearVelocity;
        float w1 = b1.angularVelocity;
        bzVec2 v2 = b2.linearVelocity;
        float w2 = b2.angularVelocity;

        // Solve linear motor constraint.
        if (m_enableMotor && m_limitState != bzLimitState.EQUAL) {
            float Cdot = bzDot(m_axis, v2 - v1) + m_a2 * w2 - m_a1 * w1;
            float impulse = m_motorMass * (m_motorSpeed - Cdot);
            float oldImpulse = m_motorImpulse;
            float maxImpulse = step.dt * m_maxMotorForce;
            m_motorImpulse = bzClamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = m_motorImpulse - oldImpulse;

            bzVec2 P = impulse * m_axis;
            float L1 = impulse * m_a1;
            float L2 = impulse * m_a2;

            v1 -= m_invMass1 * P;
            w1 -= m_invI1 * L1;

            v2 += m_invMass2 * P;
            w2 += m_invI2 * L2;
        }

        bzVec2 Cdot1;
        Cdot1.x = bzDot(m_perp, v2 - v1) + m_s2 * w2 - m_s1 * w1;
        Cdot1.y = w2 - w1;

        if (m_enableLimit && m_limitState != bzLimitState.INACTIVE) {
            // Solve prismatic and limit constraint in block form.
            float Cdot2;
            Cdot2 = bzDot(m_axis, v2 - v1) + m_a2 * w2 - m_a1 * w1;
            bzVec3 Cdot = bzVec3(Cdot1.x, Cdot1.y, Cdot2);

            bzVec3 f1 = m_impulse;
            bzVec3 df =  m_K.solve33(-Cdot);
            m_impulse += df;

            if (m_limitState == bzLimitState.LOWER) {
                m_impulse.z = max(m_impulse.z, 0.0f);
            } else if (m_limitState == bzLimitState.UPPER) {
                m_impulse.z = min(m_impulse.z, 0.0f);
            }

            // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
            bzVec2 b = -Cdot1 - (m_impulse.z - f1.z) * bzVec2(m_K.col3.x, m_K.col3.y);
            bzVec2 f2r = m_K.solve22(b) + bzVec2(f1.x, f1.y);
            m_impulse.x = f2r.x;
            m_impulse.y = f2r.y;

            df = m_impulse - f1;

            bzVec2 P = df.x * m_perp + df.z * m_axis;
            float L1 = df.x * m_s1 + df.y + df.z * m_a1;
            float L2 = df.x * m_s2 + df.y + df.z * m_a2;

            v1 -= m_invMass1 * P;
            w1 -= m_invI1 * L1;

            v2 += m_invMass2 * P;
            w2 += m_invI2 * L2;
        } else {
            // Limit is inactive, just solve the prismatic constraint in block form.
            bzVec2 df = m_K.solve22(-Cdot1);
            m_impulse.x += df.x;
            m_impulse.y += df.y;

            bzVec2 P = df.x * m_perp;
            float L1 = df.x * m_s1 + df.y;
            float L2 = df.x * m_s2 + df.y;

            v1 -= m_invMass1 * P;
            w1 -= m_invI1 * L1;

            v2 += m_invMass2 * P;
            w2 += m_invI2 * L2;
        }

        b1.linearVelocity = v1;
        b1.angularVelocity = w1;
        b2.linearVelocity = v2;
        b2.angularVelocity = w2;
    }

    override bool solvePositionConstraints(float baumgarte) {

        bzBody b1 = m_body1;
        bzBody b2 = m_body2;

        bzVec2 c1 = b1.sweep.c;
        float a1 = b1.sweep.a;

        bzVec2 c2 = b2.sweep.c;
        float a2 = b2.sweep.a;

        // Solve linear limit constraint.
        float linearError = 0.0f, angularError = 0.0f;
        bool active = false;
        float C2 = 0.0f;

        bzMat22 R1 = bzMat22(a1);
        bzMat22 R2 = bzMat22(a2);

        bzVec2 r1 = bzMul(R1, m_localAnchor1 - m_localCenter1);
        bzVec2 r2 = bzMul(R2, m_localAnchor2 - m_localCenter2);
        bzVec2 d = c2 + r2 - c1 - r1;

        if (m_enableLimit) {
            m_axis = bzMul(R1, m_localXAxis1);

            m_a1 = bzCross(d + r1, m_axis);
            m_a2 = bzCross(r2, m_axis);

            float translation = bzDot(m_axis, d);
            if (abs(m_upperTranslation - m_lowerTranslation) < 2.0f * k_linearSlop) {
                // Prevent large angular corrections
                C2 = bzClamp(translation, -k_maxLinearCorrection, k_maxLinearCorrection);
                linearError = abs(translation);
                active = true;
            } else if (translation <= m_lowerTranslation) {
                // Prevent large linear corrections and allow some slop.
                C2 = bzClamp(translation - m_lowerTranslation + k_linearSlop, -k_maxLinearCorrection, 0.0f);
                linearError = m_lowerTranslation - translation;
                active = true;
            } else if (translation >= m_upperTranslation) {
                // Prevent large linear corrections and allow some slop.
                C2 = bzClamp(translation - m_upperTranslation - k_linearSlop, 0.0f, k_maxLinearCorrection);
                linearError = translation - m_upperTranslation;
                active = true;
            }
        }

        m_perp = bzMul(R1, m_localYAxis1);

        m_s1 = bzCross(d + r1, m_perp);
        m_s2 = bzCross(r2, m_perp);

        bzVec3 impulse;
        bzVec2 C1;
        C1.x = bzDot(m_perp, d);
        C1.y = a2 - a1 - m_refAngle;

        linearError = max(linearError, cast(float)abs(C1.x));
        angularError = abs(C1.y);

        if (active) {
            float m1 = m_invMass1, m2 = m_invMass2;
            float i1 = m_invI1, i2 = m_invI2;

            float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
            float k12 = i1 * m_s1 + i2 * m_s2;
            float k13 = i1 * m_s1 * m_a1 + i2 * m_s2 * m_a2;
            float k22 = i1 + i2;
            float k23 = i1 * m_a1 + i2 * m_a2;
            float k33 = m1 + m2 + i1 * m_a1 * m_a1 + i2 * m_a2 * m_a2;

            m_K.col1.set(k11, k12, k13);
            m_K.col2.set(k12, k22, k23);
            m_K.col3.set(k13, k23, k33);

            bzVec3 C;
            C.x = C1.x;
            C.y = C1.y;
            C.z = C2;

            impulse = m_K.solve33(-C);
        } else {
            float m1 = m_invMass1, m2 = m_invMass2;
            float i1 = m_invI1, i2 = m_invI2;

            float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
            float k12 = i1 * m_s1 + i2 * m_s2;
            float k22 = i1 + i2;

            m_K.col1.set(k11, k12, 0.0f);
            m_K.col2.set(k12, k22, 0.0f);

            bzVec2 impulse1 = m_K.solve22(-C1);
            impulse.x = impulse1.x;
            impulse.y = impulse1.y;
            impulse.z = 0.0f;
        }

        bzVec2 P = impulse.x * m_perp + impulse.z * m_axis;
        float L1 = impulse.x * m_s1 + impulse.y + impulse.z * m_a1;
        float L2 = impulse.x * m_s2 + impulse.y + impulse.z * m_a2;

        c1 -= m_invMass1 * P;
        a1 -= m_invI1 * L1;
        c2 += m_invMass2 * P;
        a2 += m_invI2 * L2;

        // TODO_ERIN remove need for this.
        b1.sweep.c = c1;
        b1.sweep.a = a1;
        b2.sweep.c = c2;
        b2.sweep.a = a2;
        b1.synchronizeTransform();
        b2.synchronizeTransform();

        return linearError <= k_linearSlop && angularError <= k_angularSlop;
    }

    bzVec2 localXAxis1() {
        return m_localXAxis1;
    }

    bzVec2 localYAxis1() {
        return m_localYAxis1;
    }

    bzVec2 localAnchor1() {
        return m_localAnchor1;
    }

    bzVec2 localAnchor2() {
        return m_localAnchor2;
    }

    private:

    bzVec2 m_localAnchor1;
    bzVec2 m_localAnchor2;
    bzVec2 m_localXAxis1;
    bzVec2 m_localYAxis1;
    float m_refAngle;

    bzVec2 m_axis, m_perp;
    float m_s1, m_s2;
    float m_a1, m_a2;

    bzMat33 m_K;
    bzVec3 m_impulse;

    float m_motorMass;			// effective mass for motor/limit translational constraint.
    float m_motorImpulse;

    float m_lowerTranslation;
    float m_upperTranslation;
    float m_maxMotorForce;
    float m_motorSpeed;

    bool m_enableLimit;
    bool m_enableMotor;
    bzLimitState m_limitState;
}
