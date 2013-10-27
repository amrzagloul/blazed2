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
module blaze.dynamics.joints.bzJoint;

import blaze.bzWorld;
import blaze.common.bzMath;
import blaze.dynamics.bzBody;
import blaze.dynamics.joints.bzDistanceJoint;
import blaze.dynamics.joints.bzGearJoint;
import blaze.dynamics.joints.bzPrismaticJoint;
import blaze.dynamics.joints.bzPulleyJoint;
import blaze.dynamics.joints.bzRevoluteJoint;
import blaze.dynamics.joints.bzMouseJoint;
import blaze.dynamics.joints.bzLineJoint;

enum bzJointType {
    UNKNOWN,
    REVOLUTE,
    PRISMATIC,
    DISTANCE,
    PULLEY,
    MOUSE,
    GEAR,
	LINE
}

enum bzLimitState {
    INACTIVE,
    LOWER,
    UPPER,
    EQUAL
}

/// bzJoint definitions are used to construct joints.
class bzJointDef {

    this(bzBody body1, bzBody body2) {
        this.body1 = body1;
        this.body2 = body2;
        type = bzJointType.UNKNOWN;
    }

    /// The joint type is set automatically for concrete joint types.
    bzJointType type;

    /// Use this to attach application specific data to your joints.
    Object userData;

    /// The first attached body.
    bzBody body1;

    /// The second attached body.
    bzBody body2;

    /// Set this flag to true if the attached bodies should collide.
    bool collideConnected;

}

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
class bzJoint {

    this(bzJointDef def) {
        m_type = def.type;
        prev = null;
        next = null;
        m_body1 = def.body1;
        m_body2 = def.body2;
        m_collideConnected = def.collideConnected;
        m_islandFlag = false;
        userData = def.userData;
        m_node1 = new bzJointEdge();
        m_node2 = new bzJointEdge();
    }

    /// Get the type of the concrete joint.
    bzJointType type() @property {
        return m_type;
    }

    /// Get the first body attached to this joint.
    bzBody rBody1() @property {
        return m_body1;
    }

    /// Get the second body attached to this joint.
    bzBody rBody2() @property {
        return m_body2;
    }

    bzJointEdge node1() @property {
        return m_node1;
    }

    bzJointEdge node2() @property {
        return m_node2;
    }

    void node1(bzJointEdge edge) @property {
        m_node1 = edge;
    }

    Object userData;

    bool islandFlag() @property {
        return m_islandFlag;
    }

    void islandFlag(bool flag) @property {
        m_islandFlag = flag;
    }

    bool collideConnected() {
        return m_collideConnected;
    }

    /// Get the anchor point on body1 in world coordinates.
    abstract bzVec2 anchor1();

    /// Get the anchor point on body2 in world coordinates.
    abstract bzVec2 anchor2();

    /// Get the reaction force on body2 at the joint anchor.
    abstract bzVec2 reactionForce(float inv_dt);

    /// Get the reaction torque on body2.
    abstract float reactionTorque(float inv_dt);

    //--------------- Internals Below -------------------

    static bzJoint create(bzJointDef def) {

        bzJoint joint;

        switch (def.type) {
        case bzJointType.DISTANCE: {
            return new bzDistanceJoint(cast(bzDistanceJointDef) def);
        }
        case bzJointType.MOUSE: {
            return new bzMouseJoint(cast(bzMouseJointDef) def);
        }
        case bzJointType.PRISMATIC: {
            return new bzPrismaticJoint(cast(bzPrismaticJointDef) def);
        }
        case bzJointType.REVOLUTE: {
            return new bzRevoluteJoint(cast(bzRevoluteJointDef) def);
        }
        case bzJointType.PULLEY: {
            return new bzPulleyJoint(cast(bzPulleyJointDef) def);
        }
        case bzJointType.GEAR: {
            return new bzGearJoint(cast(bzGearJointDef) def);
        }
        case bzJointType.LINE: {
            return new bzLineJoint(cast(bzLineJointDef) def);
        }
        default:
            throw new Exception("Unknown joint type.");
        }
    }

    void computeXForm(bzXForm xf, bzVec2 center, bzVec2 localCenter, float angle) {
        xf.R.set(angle);
        xf.position = center - bzMul(xf.R, localCenter);
    }

    abstract void initVelocityConstraints(bzTimeStep step);
    abstract void solveVelocityConstraints(bzTimeStep step);
    // This returns true if the position errors are within tolerance.
    abstract bool solvePositionConstraints(float baumgarte);

    bzJoint prev;
    bzJoint next;

protected:

    bzJointType m_type;
    bzJointEdge m_node1;
    bzJointEdge m_node2;
    bzBody m_body1;
    bzBody m_body2;

    bool m_islandFlag;
    bool m_collideConnected;

    // Cache here per time step to reduce cache misses.
    bzVec2 m_localCenter1, m_localCenter2;
    float m_invMass1, m_invI1;
    float m_invMass2, m_invI2;
}

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
class bzJointEdge {
    bzBody other;			///< provides quick access to the other body attached.
    bzJoint joint;			///< the joint
    bzJointEdge prev;		///< the previous joint edge in the body's joint list
    bzJointEdge next;		///< the next joint edge in the body's joint list
}
