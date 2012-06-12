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

// Documentation comments modified by Brian Schott (SirAlaran)

module blaze.common.bzConstants;

import blaze.common.bzMath;

/**
 * Unknown
 */
const COLLISION_TOLLERANCE = 0.1;

/** SPH Constants. These are used for fluid simulation */
const DENSITY_OFFSET = 100f;
const GAS_CONSTANT = 0.1f;
const VISCOSCITY = 0.002f;
const CELL_SPACE = 15.0f / 64.0f;

/** Global tuning constants based on meters-kilograms-seconds (MKS) units. */

// Collision
const k_maxManifoldPoints = 2;
const k_maxPolygonVertices = 8;
const k_maxProxies = 2048;				// this must be a power of two
const k_maxPairs = 8 * k_maxProxies;	// this must be a power of two

// Dynamics

/**
 * A small length used as a collision and constraint tolerance. Usually it is
 * chosen to be numerically significant, but visually insignificant.
 * Currently 0.5 centimeters
 */
const k_linearSlop = 0.005f;

/**
 * A small angle used as a collision and constraint tolerance. Usually it is
 * chosen to be numerically significant, but visually insignificant.
 * Currently 2 degrees
 */
const k_angularSlop = 2.0f / 180.0f * PI;

/**
 * Continuous collision detection (CCD) works with core, shrunken shapes.
 * This is the amount by which shapes are automatically shrunk to work with CCD.
 * This must be larger than k_linearSlop.
 * Currently 8 * k_linearSlop
 */
const k_toiSlop = 8.0f * k_linearSlop;

/**
 * Maximum number of contacts to be handled to solve a TOI island.
 * Note: It is likely that you shouldn't change this unless you really
 * understand what it does.
 */
const k_maxTOIContactsPerIsland = 32;

/**
 * Maximum number of joints to be handled to solve a TOI island.
 * Note: It is likely that you shouldn't change this unless you really
 * understand what it does.
 */
const k_maxTOIJointsPerIsland = 32;

/**
 * A velocity threshold for elastic collisions. Any collision with a relative
 * linear velocity below this threshold will be treated as inelastic.
 * Currently 1 meter per second
 */
const k_velocityThreshold = 1.0f;

/**
 * The maximum linear position correction used when solving constraints. This
 * helps to prevent overshoot.
 * Currently 20 cm
 */
const k_maxLinearCorrection = 0.2f;

/**
 * The maximum angular position correction used when solving constraints.
 * This helps to prevent overshoot.
 * Currently 8 degrees
 */
const k_maxAngularCorrection = 8.0f / 180.0f * PI;

/**
 * The maximum linear velocity of a body. This limit is very large and is used
 * to prevent numerical problems. You shouldn't need to adjust this.
 * Currently set at 200 meters per second
 */
const k_maxLinearVelocity = 200.0f;

/**
 * As the name suggests, this is k_maxLinearVelocity squared.
 * Likely pre-calculated to increase efficiency
 */
const k_maxLinearVelocitySquared = k_maxLinearVelocity * k_maxLinearVelocity;

/**
 * The maximum angular velocity of a body. This limit is very large and is used
 * to prevent numerical problems. You shouldn't need to adjust this.
 * Currently set at 250 radians / second
 */
const k_maxAngularVelocity = 250.0f;

/**
 * As the name suggests, this is k_maxAngularVelocity squared.
 * Likely pre-calculated to increase efficiency
 */
const k_maxAngularVelocitySquared = k_maxAngularVelocity * k_maxAngularVelocity;

/**
 * This scale factor controls how fast overlap is resolved.
 * Ideally this would be 1 so that overlap is removed in one time step. However
 * using values close to 1 often leads to overshoot.
 */
const k_contactBaumgarte = 0.2f;

/**
 * The time that a body must be still before it will go to sleep.
 * Currently half of a second
 */
const k_timeToSleep = 0.5f;

/**
 * A body cannot sleep if its linear velocity is above this tolerance.
 * Currently 1 centimeter per second
 */
const k_linearSleepTolerance = 0.01f;

/**
 * A body cannot sleep if its angular velocity is above this tolerance.
 * Currently  2 degrees per second
 */
const k_angularSleepTolerance = 2.0f / 180.0f;

/**
 * bzVersion numbering scheme.
 * See_also: http://en.wikipedia.org/wiki/Software_versioning
 */
struct bzVersion
{
	int major;		///< significant changes
	int minor;		///< incremental changes
	int revision;	///< bug fixes
}

/// Current version.
extern bzVersion k_version;

/// Unknown
float k_errorTol;

/// Magic?
float FORCE_SCALE(float x) { return cast(uint)(x) << 7; }

/// Magic?
float FORCE_INV_SCALE(float x) { return cast(uint) (x) >> 7; }

/**
 * Friction mixing law. Feel free to customize this.
 * Params:
 *     friction1 = the first friction force
 *     friction2 = the second friction force
 * Returns: The square root of the product of the two friction forces
 */
float mixFriction(float friction1, float friction2)
{
	return sqrt(friction1 * friction2);
}

/**
 * Restitution mixing law. Feel free to customize this.
 * Params:
 *     restitution1 = the first force
 *     restitution2 = the other force
 * Returns: The greater of the two restitution forces
 */
float mixRestitution(float restitution1, float restitution2)
{
	return restitution1 > restitution2 ? restitution1 : restitution2;
}
