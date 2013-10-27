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
module blaze.dynamics.joints.bzRevoluteJoint;

import blaze.common.bzMath;
import blaze.common.bzConstants;
import blaze.dynamics.bzBody;
import blaze.bzWorld;
import blaze.dynamics.joints.bzJoint;

/// Revolute joint definition. This requires defining an
/// anchor point where the bodies are joined. The definition
/// uses local anchor points so that the initial configuration
/// can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This
/// helps when saving and loading a game.
/// The local anchor points are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass,
///    the joints will be broken.
class bzRevoluteJointDef : bzJointDef {

    /// Initialize the bodies, anchors, and reference angle using the world
    /// anchor.
    this (bzBody body1 = null, bzBody body2 = null, bzVec2 anchor = bzVec2(0,0)) {
        super(body1, body2);
        type = bzJointType.REVOLUTE;
        lowerAngle = 0.0f;
        upperAngle = 0.0f;
        maxMotorTorque = 0.0f;
        motorSpeed = 0.0f;

        if(body1 && body2) {
            localAnchor1 = body1.localPoint(anchor);
            localAnchor2 = body2.localPoint(anchor);
            referenceAngle = body2.angle - body1.angle;
        }
    }

    /// The local anchor point relative to body1's origin.
    bzVec2 localAnchor1;

    /// The local anchor point relative to body2's origin.
    bzVec2 localAnchor2;

    /// The body2 angle minus body1 angle in the reference state (radians).
    float referenceAngle = 0;

    /// A flag to enable joint limits.
    bool enableLimit;

    /// The lower angle for the joint limit (radians).
    float lowerAngle;

    /// The upper angle for the joint limit (radians).
    float upperAngle;

    /// A flag to enable the joint motor.
    bool enableMotor;

    /// The desired motor speed. Usually in radians per second.
    float motorSpeed;

    /// The maximum motor torque used to achieve the desired motor speed.
    /// Usually in N-m.
    float maxMotorTorque;
}

/// A revolute joint constrains to bodies to share a common point while they
/// are free to rotate about the point. The relative rotation about the shared
/// point is the joint angle. You can limit the relative rotation with
/// a joint limit that specifies a lower and upper angle. You can use a motor
/// to drive the relative rotation about the shared point. A maximum motor torque
/// is provided so that infinite forces are not generated.
class bzRevoluteJoint : bzJoint {

    override bzVec2 anchor1() {
        return m_body1.worldPoint(m_localAnchor1);
    }

    override bzVec2 anchor2() {
        return m_body2.worldPoint(m_localAnchor2);
    }

    override bzVec2 reactionForce(float inv_dt) {
        bzVec2 P = bzVec2(m_impulse.x, m_impulse.y);
        return inv_dt * P;
    }

    override float reactionTorque(float inv_dt) {
        return inv_dt * m_impulse.z;
    }

    /// Get the current joint angle in radians.
    float jointAngle() @property {
        bzBody b1 = m_body1;
        bzBody b2 = m_body2;
        return b2.sweep.a - b1.sweep.a - m_referenceAngle;
    }

    /// Get the current joint angle speed in radians per second.
    float jointSpeed() {
        bzBody b1 = m_body1;
        bzBody b2 = m_body2;
        return b2.angularVelocity - b1.angularVelocity;
    }

    /// Is the joint limit enabled?
    bool isLimitEnabled() {
        return m_enableLimit;
    }

    /// Enable/disable the joint limit.
    void enableLimit(bool flag) {
        m_body1.wakeup();
        m_body2.wakeup();
        m_enableLimit = flag;
    }

    /// Get the lower joint limit in radians.
    float lowerLimit() {
        return m_lowerAngle;
    }

    /// Get the upper joint limit in radians.
    float upperLimit() {
        return m_upperAngle;
    }

    /// Set the joint limits in radians.
    void limits(float lower, float upper) {
        assert(lower <= upper);
        m_body1.wakeup();
        m_body2.wakeup();
        m_lowerAngle = lower;
        m_upperAngle = upper;
    }

    /// Is the joint motor enabled?
    bool isMotorEnabled() {
        return m_enableMotor;
    }

    /// Enable/disable the joint motor.
    void enableMotor(bool flag) {
        m_body1.wakeup();
        m_body2.wakeup();
        m_enableMotor = flag;
    }

    /// Set the motor speed in radians per second.
    void motorSpeed(float speed) {
        m_body1.wakeup();
        m_body2.wakeup();
        m_motorSpeed = speed;
    }

    /// Get the motor speed in radians per second.
    float motorSpeed() {
        return m_motorSpeed;
    }

    /// Set the maximum motor torque, usually in N-m.
    void maxMotorTorque(float torque) {
        m_body1.wakeup();
        m_body2.wakeup();
        m_maxMotorTorque = torque;
    }

    /// Get the current motor torque, usually in N-m.
    float motorTorque() {
        return m_motorImpulse;
    }

    //--------------- Internals Below -------------------
    // Point-to-point constraint
    // C = p2 - p1
    // Cdot = v2 - v1
    //      = v2 + bzCross(w2, r2) - v1 - bzCross(w1, r1)
    // J = [-I -r1_skew I r2_skew ]
    // Identity used:
    // w k % (rx i + ry j) = w * (-ry i + rx j)

    // Motor constraint
    // Cdot = w2 - w1
    // J = [0 0 -1 0 0 1]
    // K = invI1 + invI2

    this(bzRevoluteJointDef def) {
        super(def);
        m_localAnchor1 = def.localAnchor1;
        m_localAnchor2 = def.localAnchor2;
        m_referenceAngle = def.referenceAngle;

        m_impulse.zero();
        m_motorImpulse = 0.0f;

        m_lowerAngle = def.lowerAngle;
        m_upperAngle = def.upperAngle;
        m_maxMotorTorque = def.maxMotorTorque;
        m_motorSpeed = def.motorSpeed;
        m_enableLimit = def.enableLimit;
        m_enableMotor = def.enableMotor;
    }

    override void initVelocityConstraints(bzTimeStep step) {

        bzBody b1 = m_body1;
        bzBody b2 = m_body2;

        if (m_enableMotor || m_enableLimit) {
            // You cannot create a rotation limit between bodies that
            // both have fixed rotation.
            assert(b1.invI > 0.0f || b2.invI > 0.0f);
        }

        // Compute the effective mass matrix.
        bzVec2 r1 = bzMul(b1.xf.R, m_localAnchor1 - b1.localCenter);
        bzVec2 r2 = bzMul(b2.xf.R, m_localAnchor2 - b2.localCenter);

        // J = [-I -r1_skew I r2_skew]
        //     [ 0       -1 0       1]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ m1+r1y^2*i1+m2+r2y^2*i2,  -r1y*i1*r1x-r2y*i2*r2x,          -r1y*i1-r2y*i2]
        //     [  -r1y*i1*r1x-r2y*i2*r2x, m1+r1x^2*i1+m2+r2x^2*i2,           r1x*i1+r2x*i2]
        //     [          -r1y*i1-r2y*i2,           r1x*i1+r2x*i2,                   i1+i2]

        float m1 = b1.invMass, m2 = b2.invMass;
        float i1 = b1.invI, i2 = b2.invI;

        m_mass.col1.x = m1 + m2 + r1.y * r1.y * i1 + r2.y * r2.y * i2;
        m_mass.col2.x = -r1.y * r1.x * i1 - r2.y * r2.x * i2;
        m_mass.col3.x = -r1.y * i1 - r2.y * i2;
        m_mass.col1.y = m_mass.col2.x;
        m_mass.col2.y = m1 + m2 + r1.x * r1.x * i1 + r2.x * r2.x * i2;
        m_mass.col3.y = r1.x * i1 + r2.x * i2;
        m_mass.col1.z = m_mass.col3.x;
        m_mass.col2.z = m_mass.col3.y;
        m_mass.col3.z = i1 + i2;

        m_motorMass = 1.0f / (i1 + i2);

        if (m_enableMotor == false) {
            m_motorImpulse = 0.0f;
        }

        if (m_enableLimit) {
            float jointAngle = b2.sweep.a - b1.sweep.a - m_referenceAngle;
            if (abs(m_upperAngle - m_lowerAngle) < 2.0f * k_angularSlop) {
                m_limitState = bzLimitState.EQUAL;
            } else if (jointAngle <= m_lowerAngle) {
                if (m_limitState != bzLimitState.LOWER) {
                    m_impulse.z = 0.0f;
                }
                m_limitState = bzLimitState.LOWER;
            } else if (jointAngle >= m_upperAngle) {
                if (m_limitState != bzLimitState.UPPER) {
                    m_impulse.z = 0.0f;
                }
                m_limitState = bzLimitState.UPPER;
            } else {
                m_limitState = bzLimitState.INACTIVE;
                m_impulse.z = 0.0f;
            }
        }

        if (step.warmStarting) {
            // Scale impulses to support a variable time step.
            m_impulse *= step.dtRatio;
            m_motorImpulse *= step.dtRatio;

            bzVec2 P = bzVec2(m_impulse.x, m_impulse.y);

            b1.linearVelocity = b1.linearVelocity - m1 * P;
            b1.angularVelocity = b1.angularVelocity - i1 * (bzCross(r1, P) + m_motorImpulse + m_impulse.z);

            b2.linearVelocity = b2.linearVelocity + m2 * P;
            b2.angularVelocity = b2.angularVelocity + i2 * (bzCross(r2, P) + m_motorImpulse + m_impulse.z);
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

        float m1 = b1.invMass, m2 = b2.invMass;
        float i1 = b1.invI, i2 = b2.invI;

        if (m_enableMotor && m_limitState != bzLimitState.EQUAL) {
            float Cdot = w2 - w1 - m_motorSpeed;
            float impulse = m_motorMass * (-Cdot);
            float oldImpulse = m_motorImpulse;
            float maxImpulse = step.dt * m_maxMotorTorque;
            m_motorImpulse = bzClamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = m_motorImpulse - oldImpulse;

            w1 -= i1 * impulse;
            w2 += i2 * impulse;
        }

        if (m_enableLimit && m_limitState != bzLimitState.INACTIVE) {
            bzVec2 r1 = bzMul(b1.xf.R, m_localAnchor1 - b1.localCenter);
            bzVec2 r2 = bzMul(b2.xf.R, m_localAnchor2 - b2.localCenter);

            // Solve point-to-point constraint
            bzVec2 Cdot1 = v2 + bzCross(w2, r2) - v1 - bzCross(w1, r1);
            float Cdot2 = w2 - w1;
            bzVec3 Cdot = bzVec3(Cdot1.x, Cdot1.y, Cdot2);

            bzVec3 impulse = m_mass.solve33(-Cdot);

            if (m_limitState == bzLimitState.EQUAL) {
                m_impulse += impulse;
            } else if (m_limitState == bzLimitState.LOWER) {
                float newImpulse = m_impulse.z + impulse.z;
                if (newImpulse < 0.0f) {
                    bzVec2 reduced = m_mass.solve22(-Cdot1);
                    impulse.x = reduced.x;
                    impulse.y = reduced.y;
                    impulse.z = -m_impulse.z;
                    m_impulse.x += reduced.x;
                    m_impulse.y += reduced.y;
                    m_impulse.z = 0.0f;
                }
            } else if (m_limitState == bzLimitState.UPPER) {
                float newImpulse = m_impulse.z + impulse.z;
                if (newImpulse > 0.0f) {
                    bzVec2 reduced = m_mass.solve22(-Cdot1);
                    impulse.x = reduced.x;
                    impulse.y = reduced.y;
                    impulse.z = -m_impulse.z;
                    m_impulse.x += reduced.x;
                    m_impulse.y += reduced.y;
                    m_impulse.z = 0.0f;
                }
            }

            bzVec2 P = bzVec2(impulse.x, impulse.y);

            v1 -= m1 * P;
            w1 -= i1 * (bzCross(r1, P) + impulse.z);

            v2 += m2 * P;
            w2 += i2 * (bzCross(r2, P) + impulse.z);
        } else {
            bzVec2 r1 = bzMul(b1.xf.R, m_localAnchor1 - b1.localCenter);
            bzVec2 r2 = bzMul(b2.xf.R, m_localAnchor2 - b2.localCenter);

            // Solve point-to-point constraint
            bzVec2 Cdot = v2 + bzCross(w2, r2) - v1 - bzCross(w1, r1);
            bzVec2 impulse = m_mass.solve22(-Cdot);

            m_impulse.x += impulse.x;
            m_impulse.y += impulse.y;

            v1 -= m1 * impulse;
            w1 -= i1 * bzCross(r1, impulse);

            v2 += m2 * impulse;
            w2 += i2 * bzCross(r2, impulse);
        }

        b1.linearVelocity = v1;
        b1.angularVelocity = w1;
        b2.linearVelocity = v2;
        b2.angularVelocity = w2;
    }

    override bool solvePositionConstraints(float baumgarte) {
        // TODO_ERIN block solve with limit.

        bzBody b1 = m_body1;
        bzBody b2 = m_body2;

        float angularError = 0.0f;
        float positionError = 0.0f;

        // Solve angular limit constraint.
        if (m_enableLimit && m_limitState != bzLimitState.INACTIVE) {
            float angle = b2.sweep.a - b1.sweep.a - m_referenceAngle;
            float limitImpulse = 0.0f;

            if (m_limitState == bzLimitState.EQUAL) {
                // Prevent large angular corrections
                float C = bzClamp(angle, -k_maxAngularCorrection, k_maxAngularCorrection);
                limitImpulse = -m_motorMass * C;
                angularError = abs(C);
            } else if (m_limitState == bzLimitState.LOWER) {
                float C = angle - m_lowerAngle;
                angularError = -C;

                // Prevent large angular corrections and allow some slop.
                C = bzClamp(C + k_angularSlop, -k_maxAngularCorrection, 0.0f);
                limitImpulse = -m_motorMass * C;
            } else if (m_limitState == bzLimitState.UPPER) {
                float C = angle - m_upperAngle;
                angularError = C;

                // Prevent large angular corrections and allow some slop.
                C = bzClamp(C - k_angularSlop, 0.0f, k_maxAngularCorrection);
                limitImpulse = -m_motorMass * C;
            }

            b1.sweep.a = b1.sweep.a - b1.invI * limitImpulse;
            b2.sweep.a = b2.sweep.a  + b2.invI * limitImpulse;

            b1.synchronizeTransform();
            b2.synchronizeTransform();
        }

        // Solve point-to-point constraint.
        {
            bzVec2 r1 = bzMul(b1.xf.R, m_localAnchor1 - b1.localCenter);
            bzVec2 r2 = bzMul(b2.xf.R, m_localAnchor2 - b2.localCenter);

            bzVec2 C = b2.sweep.c + r2 - b1.sweep.c - r1;
            positionError = C.length();

            float invMass1 = b1.invMass, invMass2 = b2.invMass;
            float invI1 = b1.invI, invI2 = b2.invI;

            // Handle large detachment.
            const float k_allowedStretch = 10.0f * k_linearSlop;
            if (C.lengthSquared > k_allowedStretch * k_allowedStretch) {
                // Use a particle solution (no rotation).
                bzVec2 u = C;
                u.normalize;
                float k = invMass1 + invMass2;
                assert(k > float.epsilon);
                float m = 1.0f / k;
                bzVec2 impulse = m * (-C);
                const float k_beta = 0.5f;
                b1.sweep.c = b1.sweep.c - k_beta * invMass1 * impulse;
                b2.sweep.c = b2.sweep.c + k_beta * invMass2 * impulse;

                C = b2.sweep.c + r2 - b1.sweep.c - r1;
            }

            bzMat22 K1;
            K1.col1.x = invMass1 + invMass2;
            K1.col2.x = 0.0f;
            K1.col1.y = 0.0f;
            K1.col2.y = invMass1 + invMass2;

            bzMat22 K2;
            K2.col1.x =  invI1 * r1.y * r1.y;
            K2.col2.x = -invI1 * r1.x * r1.y;
            K2.col1.y = -invI1 * r1.x * r1.y;
            K2.col2.y =  invI1 * r1.x * r1.x;

            bzMat22 K3;
            K3.col1.x =  invI2 * r2.y * r2.y;
            K3.col2.x = -invI2 * r2.x * r2.y;
            K3.col1.y = -invI2 * r2.x * r2.y;
            K3.col2.y =  invI2 * r2.x * r2.x;

            bzMat22 K = K1 + K2 + K3;
            bzVec2 impulse = K.solve(-C);

            b1.sweep.c = b1.sweep.c - b1.invMass * impulse;
            b1.sweep.a = b1.sweep.a - b1.invI * bzCross(r1, impulse);

            b2.sweep.c = b2.sweep.c + b2.invMass * impulse;
            b2.sweep.a = b2.sweep.a + b2.invI * bzCross(r2, impulse);

            b1.synchronizeTransform();
            b2.synchronizeTransform();
        }

        return positionError <= k_linearSlop && angularError <= k_angularSlop;
    }

    bzVec2 localAnchor1() {
        return m_localAnchor1;
    }

    bzVec2 localAnchor2() {
        return m_localAnchor2;
    }

    private:

    bzVec2 m_localAnchor1;	// relative
    bzVec2 m_localAnchor2;
    bzVec3 m_impulse;
    float m_motorImpulse;

    bzMat33 m_mass;		// effective mass for point-to-point constraint.
    float m_motorMass;	// effective mass for motor/limit angular constraint.

    bool m_enableMotor;
    float m_maxMotorTorque;
    float m_motorSpeed;

    bool m_enableLimit;
    float m_referenceAngle;
    float m_lowerAngle;
    float m_upperAngle;
    bzLimitState m_limitState;
}
