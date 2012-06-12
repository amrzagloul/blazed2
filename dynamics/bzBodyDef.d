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

// Documentation comments modified by Brian Schott (SirAlaran)

module blaze.dynamics.bzBodyDef;

import blaze.collision.shapes.bzShape;
import blaze.common.bzMath;

/**
 * A body definition holds all the data needed to construct a rigid body.
 * You can safely re-use body definitions.
 */
class bzBodyDef
{
	/**
	 * This constructor sets the body definition default values.
	 * Params:
	 *     positon = the position of the body
	 *     angle = the angle of rotation of the body
	 */
	this(bzVec2 position = bzVec2(0,0), float angle = 0.0f)
	{
	    this.position = position;
	    this.angle =  angle;
		massData.center.zero();
		massData.mass = 0.0f;
		massData.I = 0.0f;
		position.set(0.0f, 0.0f);
		angle = 0.0f;
		linearDamping = 0.0f;
		angularDamping = 0.0f;
		allowSleep = true;
		isSleeping = false;
        allowFreeze = true;
		fixedRotation = false;
		isBullet = false;
	}

	/**
	 * You can use this to initialized the mass properties of the body.
	 * If you prefer, you can set the mass properties after the shapes
	 * have been added using bzBody::SetMassFromShapes.
	 */
	bzMassData massData;

	/**
	 * Use this to store application specific body data.
	 */
	Object userData;

	/**
	 * The world position of the body. Avoid creating bodies at the origin
	 * since this can lead to many overlapping shapes.
	 */
	bzVec2 position;

	/**
	 * The world angle of the body in radians.
	 */
	float angle;

	/**
	 * Linear damping is use to reduce the linear velocity. The damping parameter
	 * can be larger than 1.0f but the damping effect becomes sensitive to the
	 * time step when the damping parameter is large.
	 */
	float linearDamping;

	/**
	 * Angular damping is use to reduce the angular velocity. The damping parameter
	 * can be larger than 1.0f but the damping effect becomes sensitive to the
	 * time step when the damping parameter is large.
	 */
	float angularDamping;

	/**
	 * Set this flag to false if this body should never fall asleep. Note that
	 * this increases CPU usage.
	 */
	bool allowSleep;
    
    /**
	 * Set this flag to false if this body should never freeze. 
	 * Useful for world boundary voilation handeling
	 */
    bool allowFreeze;

	/**
	 * Is this body initially sleeping?
	 */
	bool isSleeping;

	/**
	 * Should this body be prevented from rotating? Useful for characters.
	 */
	bool fixedRotation;

	/**
	 * Is this a fast moving body that should be prevented from tunneling through
	 * other moving bodies? Note that all bodies are prevented from tunneling through
	 * static bodies.
	 * Warning: You should use this flag sparingly since it increases processing time.
	 */
	bool isBullet;
}
