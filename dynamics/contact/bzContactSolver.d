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
module blaze.dynamics.contact.bzContactSolver;

import blaze.common.bzMath;
import blaze.common.bzConstants;
import blaze.dynamics.bzBody;
import blaze.collision.shapes.bzShape;
import blaze.collision.bzCollision;
import blaze.dynamics.contact.bzContact;
import blaze.bzWorld;

struct bzContactConstraintPoint {
    bzVec2 localAnchor1;
    bzVec2 localAnchor2;
    bzVec2 r1;
    bzVec2 r2;
    float normalImpulse = 0;
    float tangentImpulse = 0;
    float normalMass = 0;
    float tangentMass = 0;
    float equalizedMass = 0;
    float separation = 0;
    float velocityBias = 0;
}

struct bzContactConstraint {
    bzContactConstraintPoint[k_maxManifoldPoints] points;
    bzVec2 normal;
    bzMat22 normalMass;
    bzMat22 K;
    bzManifold* manifold;
    bzBody rBody1;
    bzBody rBody2;
    float friction = 0;
    float restitution = 0;
    int pointCount;
}

class bzContactSolver {

    bzTimeStep m_step;
    bzContactConstraint[] m_constraints;
    int m_constraintCount;

    bzContactConstraint[] constraints() {
        return m_constraints;
    }

    int constrainCount() {
        return m_constraintCount;
    }

    this(bzTimeStep step, bzContact[] contacts, int numContacts) {

        m_step = step;

        for (int i = 0; i < numContacts; ++i) {
            assert(contacts[i].isSolid);
            m_constraintCount += contacts[i].manifoldCount;
        }

        m_constraints.length = m_constraintCount;

        int count = 0;
        for (int i = 0; i < numContacts; ++i) {

            bzContact contact = contacts[i];

            bzShape shape1 = contact.shape1;
            bzShape shape2 = contact.shape2;
            bzBody b1 = contact.shape1.rBody;
            bzBody b2 = contact.shape2.rBody;

            float friction = mixFriction(shape1.friction, shape2.friction);
            float restitution = mixRestitution(shape1.restitution, shape2.restitution);

            bzVec2 v1 = b1.linearVelocity;
            bzVec2 v2 = b2.linearVelocity;
            float w1 = b1.angularVelocity;
            float w2 = b2.angularVelocity;

            for (int j = 0; j < contact.manifoldCount; ++j) {

                bzManifold* manifold = &contact.manifold;

                assert(manifold.pointCount > 0);
                bzVec2 normal = manifold.normal;

                assert(count < m_constraintCount);

                bzContactConstraint *cc = &m_constraints[count];
                cc.rBody1 = b1;
                cc.rBody2 = b2;
                cc.manifold = manifold;
                cc.normal = normal;
                cc.pointCount = manifold.pointCount;
                cc.friction = friction;
                cc.restitution = restitution;

                for (int k = 0; k < cc.pointCount; ++k) {
                    bzManifoldPoint cp = manifold.points[k];
                    bzContactConstraintPoint *ccp = &cc.points[k];

                    ccp.normalImpulse = cp.normalImpulse;
                    ccp.tangentImpulse = cp.tangentImpulse;
                    ccp.separation = cp.separation;

                    ccp.localAnchor1 = cp.localPoint1;
                    ccp.localAnchor2 = cp.localPoint2;
                    ccp.r1 = bzMul(b1.xf.R, cp.localPoint1 - b1.localCenter);
                    ccp.r2 = bzMul(b2.xf.R, cp.localPoint2 - b2.localCenter);

                    float rn1 = bzCross(ccp.r1, normal);
                    float rn2 = bzCross(ccp.r2, normal);
                    rn1 *= rn1;
                    rn2 *= rn2;

                    float kNormal = b1.invMass + b2.invMass + b1.invI * rn1 + b2.invI * rn2;

                    assert(kNormal > float.epsilon);
                    ccp.normalMass = 1.0f / kNormal;

                    float kEqualized = b1.mass * b1.invMass + b2.mass * b2.invMass;
                    kEqualized += b1.mass * b1.invI * rn1 + b2.mass * b2.invI * rn2;

                    assert(kEqualized > float.epsilon);
                    ccp.equalizedMass = 1.0f / kEqualized;

                    bzVec2 tangent = bzCross(normal, 1.0f);

                    float rt1 = bzCross(ccp.r1, tangent);
                    float rt2 = bzCross(ccp.r2, tangent);
                    rt1 *= rt1;
                    rt2 *= rt2;

                    float kTangent = b1.invMass + b2.invMass + b1.invI * rt1 + b2.invI * rt2;

                    assert(kTangent > float.epsilon);
                    ccp.tangentMass = 1.0f /  kTangent;

                    // Setup a velocity bias for restitution.
                    ccp.velocityBias = 0.0f;
                    if (ccp.separation > 0.0f) {
                        ccp.velocityBias = -step.inv_dt * ccp.separation; // TODO_ERIN b2TimeStep
                    } else {
                        float vRel = bzDot(cc.normal, v2 + bzCross(w2, ccp.r2) - v1 - bzCross(w1, ccp.r1));
                        if (vRel < -k_velocityThreshold) {
                            ccp.velocityBias = -cc.restitution * vRel;
                        }
                    }
                }

                // If we have two points, then prepare the block solver.
                if (cc.pointCount == 2) {
                    bzContactConstraintPoint *ccp1 = &cc.points[0];
                    bzContactConstraintPoint *ccp2 = &cc.points[1];

                    float invMass1 = b1.invMass;
                    float invI1 = b1.invI;
                    float invMass2 = b2.invMass;
                    float invI2 = b2.invI;

                    float rn11 = bzCross(ccp1.r1, normal);
                    float rn12 = bzCross(ccp1.r2, normal);
                    float rn21 = bzCross(ccp2.r1, normal);
                    float rn22 = bzCross(ccp2.r2, normal);

                    float k11 = invMass1 + invMass2 + invI1 * rn11 * rn11 + invI2 * rn12 * rn12;
                    float k22 = invMass1 + invMass2 + invI1 * rn21 * rn21 + invI2 * rn22 * rn22;
                    float k12 = invMass1 + invMass2 + invI1 * rn11 * rn21 + invI2 * rn12 * rn22;

                    // Ensure a reasonable condition number.
                    const float k_maxConditionNumber = 100.0f;
                    if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
                        // K is safe to invert.
                        cc.K.col1.set(k11, k12);
                        cc.K.col2.set(k12, k22);
                        cc.normalMass = cc.K.inverse();
                    } else {
                        // The constraints are redundant, just use one.
                        // TODO_ERIN use deepest?
                        cc.pointCount = 1;
                    }
                }
                ++count;
            }
        }

        assert(count == m_constraintCount);
    }

    void initVelocityConstraints(bzTimeStep step) {
        // Warm start.
        for (int i = 0; i < m_constraintCount; ++i) {
            bzContactConstraint *c = &m_constraints[i];

            bzBody b1 = c.rBody1;
            bzBody b2 = c.rBody2;
            float invMass1 = b1.invMass;
            float invI1 = b1.invI;
            float invMass2 = b2.invMass;
            float invI2 = b2.invI;
            bzVec2 normal = c.normal;
            bzVec2 tangent = bzCross(normal, 1.0f);

            if (step.warmStarting) {
                for (int j = 0; j < c.pointCount; ++j) {
                    bzContactConstraintPoint *ccp = &c.points[j];
                    ccp.normalImpulse *= step.dtRatio;
                    ccp.tangentImpulse *= step.dtRatio;
                    bzVec2 P = ccp.normalImpulse * normal + ccp.tangentImpulse * tangent;
                    b1.angularVelocity = b1.angularVelocity - invI1 * bzCross(ccp.r1, P);
                    b1.linearVelocity = b1.linearVelocity - invMass1 * P;
                    b2.angularVelocity = b2.angularVelocity + invI2 * bzCross(ccp.r2, P);
                    b2.linearVelocity = b2.linearVelocity + invMass2 * P;
                }
            } else {
                for (int j = 0; j < c.pointCount; ++j) {
                    bzContactConstraintPoint* ccp = &c.points[j];
                    ccp.normalImpulse = 0.0f;
                    ccp.tangentImpulse = 0.0f;
                }
            }
        }
    }

    void solveVelocityConstraints() {

        for (int i = 0; i < m_constraintCount; ++i) {
            bzContactConstraint *c = &m_constraints[i];
            bzBody b1 = c.rBody1;
            bzBody b2 = c.rBody2;
            float w1 = b1.angularVelocity;
            float w2 = b2.angularVelocity;
            bzVec2 v1 = b1.linearVelocity;
            bzVec2 v2 = b2.linearVelocity;
            float invMass1 = b1.invMass;
            float invI1 = b1.invI;
            float invMass2 = b2.invMass;
            float invI2 = b2.invI;
            bzVec2 normal = c.normal;
            bzVec2 tangent = bzCross(normal, 1.0f);
            float friction = c.friction;

            assert(c.pointCount == 1 || c.pointCount == 2);

            // Solve normal constraints
            if (c.pointCount == 1) {
                bzContactConstraintPoint *ccp = &c.points[0];

                // Relative velocity at contact
                bzVec2 dv = v2 + bzCross(w2, ccp.r2) - v1 - bzCross(w1, ccp.r1);

                // Compute normal impulse
                float vn = bzDot(dv, normal);
                float lambda = -ccp.normalMass * (vn - ccp.velocityBias);

                // clamp the accumulated impulse
                float newImpulse = max(ccp.normalImpulse + lambda, 0.0f);
                lambda = newImpulse - ccp.normalImpulse;

                // Apply contact impulse
                bzVec2 P = lambda * normal;
                v1 -= invMass1 * P;
                w1 -= invI1 * bzCross(ccp.r1, P);

                v2 += invMass2 * P;
                w2 += invI2 * bzCross(ccp.r2, P);
                ccp.normalImpulse = newImpulse;
            } else {
                // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
                // Build the mini LCP for this contact patch
                //
                // vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
                //
                // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
                // b = vn_0 - velocityBias
                //
                // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
                // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
                // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
                // solution that satisfies the problem is chosen.
                //
                // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
                // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
                //
                // Substitute:
                //
                // x = x' - a
                //
                // Plug into above equation:
                //
                // vn = A * x + b
                //    = A * (x' - a) + b
                //    = A * x' + b - A * a
                //    = A * x' + b'
                // b' = b - A * a;

                bzContactConstraintPoint *cp1 = &c.points[0];
                bzContactConstraintPoint *cp2 = &c.points[1];

                bzVec2 a = bzVec2(cp1.normalImpulse, cp2.normalImpulse);
                assert(a.x >= 0.0f && a.y >= 0.0f);

                // Relative velocity at contact
                bzVec2 dv1 = v2 + bzCross(w2, cp1.r2) - v1 - bzCross(w1, cp1.r1);
                bzVec2 dv2 = v2 + bzCross(w2, cp2.r2) - v1 - bzCross(w1, cp2.r1);

                // Compute normal velocity
                float vn1 = bzDot(dv1, normal);
                float vn2 = bzDot(dv2, normal);

                bzVec2 b;
                b.x = vn1 - cp1.velocityBias;
                b.y = vn2 - cp2.velocityBias;
                b -= bzMul(c.K, a);

                k_errorTol = 1e-3f;

                for (;;) {
                    //
                    // Case 1: vn = 0
                    //
                    // 0 = A * x' + b'
                    //
                    // Solve for x':
                    //
                    // x' = - inv(A) * b'
                    //
                    bzVec2 x = - bzMul(c.normalMass, b);

                    if (x.x >= 0.0f && x.y >= 0.0f) {
                        // Resubstitute for the incremental impulse
                        bzVec2 d = x - a;

                        // Apply incremental impulse
                        bzVec2 P1 = d.x * normal;
                        bzVec2 P2 = d.y * normal;
                        v1 -= invMass1 * (P1 + P2);
                        w1 -= invI1 * (bzCross(cp1.r1, P1) + bzCross(cp2.r1, P2));

                        v2 += invMass2 * (P1 + P2);
                        w2 += invI2 * (bzCross(cp1.r2, P1) + bzCross(cp2.r2, P2));

                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;

                        break;
                    }

                    //
                    // Case 2: vn1 = 0 and x2 = 0
                    //
                    //   0 = a11 * x1' + a12 * 0 + b1'
                    // vn2 = a21 * x1' + a22 * 0 + b2'
                    //
                    x.x = - cp1.normalMass * b.x;
                    x.y = 0.0f;
                    vn1 = 0.0f;
                    vn2 = c.K.col1.y * x.x + b.y;

                    if (x.x >= 0.0f && vn2 >= 0.0f) {
                        // Resubstitute for the incremental impulse
                        bzVec2 d = x - a;

                        // Apply incremental impulse
                        bzVec2 P1 = d.x * normal;
                        bzVec2 P2 = d.y * normal;
                        v1 -= invMass1 * (P1 + P2);
                        w1 -= invI1 * (bzCross(cp1.r1, P1) + bzCross(cp2.r1, P2));

                        v2 += invMass2 * (P1 + P2);
                        w2 += invI2 * (bzCross(cp1.r2, P1) + bzCross(cp2.r2, P2));

                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;

                        break;
                    }


                    //
                    // Case 3: w2 = 0 and x1 = 0
                    //
                    // vn1 = a11 * 0 + a12 * x2' + b1'
                    //   0 = a21 * 0 + a22 * x2' + b2'
                    //
                    x.x = 0.0f;
                    x.y = - cp2.normalMass * b.y;
                    vn1 = c.K.col2.x * x.y + b.x;
                    vn2 = 0.0f;

                    if (x.y >= 0.0f && vn1 >= 0.0f) {
                        // Resubstitute for the incremental impulse
                        bzVec2 d = x - a;

                        // Apply incremental impulse
                        bzVec2 P1 = d.x * normal;
                        bzVec2 P2 = d.y * normal;
                        v1 -= invMass1 * (P1 + P2);
                        w1 -= invI1 * (bzCross(cp1.r1, P1) + bzCross(cp2.r1, P2));

                        v2 += invMass2 * (P1 + P2);
                        w2 += invI2 * (bzCross(cp1.r2, P1) + bzCross(cp2.r2, P2));

                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;

                        break;
                    }

                    //
                    // Case 4: x1 = 0 and x2 = 0
                    //
                    // vn1 = b1
                    // vn2 = b2;
                    x.x = 0.0f;
                    x.y = 0.0f;
                    vn1 = b.x;
                    vn2 = b.y;

                    if (vn1 >= 0.0f && vn2 >= 0.0f ) {
                        // Resubstitute for the incremental impulse
                        bzVec2 d = x - a;

                        // Apply incremental impulse
                        bzVec2 P1 = d.x * normal;
                        bzVec2 P2 = d.y * normal;
                        v1 -= invMass1 * (P1 + P2);
                        w1 -= invI1 * (bzCross(cp1.r1, P1) + bzCross(cp2.r1, P2));

                        v2 += invMass2 * (P1 + P2);
                        w2 += invI2 * (bzCross(cp1.r2, P1) + bzCross(cp2.r2, P2));

                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;

                        break;
                    }

                    // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                    break;
                }
            }

            // Solve tangent constraints
            for (int j = 0; j < c.pointCount; ++j) {
                bzContactConstraintPoint *ccp = &c.points[j];

                // Relative velocity at contact
                bzVec2 dv = v2 + bzCross(w2, ccp.r2) - v1 - bzCross(w1, ccp.r1);

                // Compute tangent force
                float vt = bzDot(dv, tangent);
                float lambda = ccp.tangentMass * (-vt);

                // clamp the accumulated force
                float maxFriction = friction * ccp.normalImpulse;
                float newImpulse = bzClamp(ccp.tangentImpulse + lambda, -maxFriction, maxFriction);
                lambda = newImpulse - ccp.tangentImpulse;

                // Apply contact impulse
                bzVec2 P = lambda * tangent;

                v1 -= invMass1 * P;
                w1 -= invI1 * bzCross(ccp.r1, P);

                v2 += invMass2 * P;
                w2 += invI2 * bzCross(ccp.r2, P);

                ccp.tangentImpulse = newImpulse;
            }

            b1.linearVelocity = v1;
            b1.angularVelocity = w1;
            b2.linearVelocity = v2;
            b2.angularVelocity = w2;
        }
    }

    void finalizeVelocityConstraints() {
        for (int i = 0; i < m_constraintCount; ++i) {
            bzContactConstraint *c = &m_constraints[i];
            bzManifold* m = c.manifold;
            for (int j = 0; j < c.pointCount; ++j) {
                m.points[j].normalImpulse = c.points[j].normalImpulse;
                m.points[j].tangentImpulse = c.points[j].tangentImpulse;
            }
        }
    }

    // Sequential solver.
    bool solvePositionConstraints(float baumgarte) {
        float minSeparation = 0.0f;

        for (int i = 0; i < m_constraintCount; ++i) {
            bzContactConstraint *c = &m_constraints[i];
            bzBody b1 = c.rBody1;
            bzBody b2 = c.rBody2;
            float invMass1 = b1.mass * b1.invMass;
            float invI1 = b1.mass * b1.invI;
            float invMass2 = b2.mass * b2.invMass;
            float invI2 = b2.mass * b2.invI;

            bzVec2 normal = c.normal;

            // Solver normal constraints
            for (int j = 0; j < c.pointCount; ++j) {
                bzContactConstraintPoint *ccp = &c.points[j];

                bzVec2 r1 = bzMul(b1.xf.R, ccp.localAnchor1 - b1.localCenter);
                bzVec2 r2 = bzMul(b2.xf.R, ccp.localAnchor2 - b2.localCenter);

                bzVec2 p1 = b1.sweep.c + r1;
                bzVec2 p2 = b2.sweep.c + r2;
                bzVec2 dp = p2 - p1;

                // Approximate the current separation.
                float separation = bzDot(dp, normal) + ccp.separation;

                // Track max constraint error.
                minSeparation = min(minSeparation, separation);

                // Prevent large corrections and allow slop.
                float C = baumgarte * bzClamp(separation + k_linearSlop, -k_maxLinearCorrection, 0.0f);

                // Compute normal impulse
                float impulse = -ccp.equalizedMass * C;

                bzVec2 P = impulse * normal;

                b1.sweep.c = b1.sweep.c - invMass1 * P;
                b1.sweep.a = b1.sweep.a - invI1 * bzCross(r1, P);
                b1.synchronizeTransform();

                b2.sweep.c = b2.sweep.c + invMass2 * P;
                b2.sweep.a = b2.sweep.a + invI2 * bzCross(r2, P);
                b2.synchronizeTransform();
            }
        }

        // We can't expect minSpeparation >= -k_linearSlop because we don't
        // push the separation above -k_linearSlop.
        return minSeparation >= -1.5f * k_linearSlop;
    }


}
