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

// Documentation comments modified by Brian Schott (SirAlaran)
module blaze.dynamics.bzWorldCallbacks;

import blaze.common.bzMath;
import blaze.dynamics.contact.bzContact;
import blaze.dynamics.joints.bzJoint;
import blaze.collision.shapes.bzShape;
import blaze.dynamics.bzBody;

enum
{
    e_shapeBit				= 0x0001, ///< draw shapes
    e_jointBit				= 0x0002, ///< draw joint connections
    e_coreShapeBit			= 0x0004, ///< draw core (TOI) shapes
    e_aabbBit				= 0x0008, ///< draw axis aligned bounding boxes
    e_obbBit				= 0x0010, ///< draw oriented bounding boxes
    e_pairBit				= 0x0020, ///< draw broad-phase pairs
    e_centerOfMassBit		= 0x0040, ///< draw center of mass frame
    e_controllerBit			= 0x0080, ///< draw controllers
}

/**
 * Implement this class to get collision results. You can use these results for
 * things like sounds and game logic. You can also get contact results by
 * traversing the contact lists after the time step. However, you might miss
 * some contacts because continuous physics leads to sub-stepping.
 * Additionally you may receive multiple callbacks for the same contact in a
 * single time step.
 * You should strive to make your callbacks efficient because there may be
 * many callbacks per time step.
 * Warning: The contact separation is the last computed value.
 * Warning: You cannot create/destroy Box2D entities inside these callbacks.
 */
abstract class bzContactListener
{
    /**
     * Called when a contact point is added. This includes the geometry
     * and the forces.
     */
    void add(bzContactPoint point);

    /**
     * Called when a contact point persists. This includes the geometry
     * and the forces.
     */
    void persist(bzContactPoint point);

    /**
     * Called when a contact point is removed. This includes the last
     * computed geometry and forces.
     */
    void remove(bzContactPoint point);

    /**
     * Called after a contact point is solved.
     */
    void result(bzContactResult point);
}

/**
 * Joints and shapes are destroyed when their associated
 * body is destroyed. Implement this listener so that you
 * may nullify references to these joints and shapes.
 */
abstract class bzDestructionListener
{
    /**
     * Called when any joint is about to be destroyed due
     * to the destruction of one of its attached bodies.
     */
    abstract void sayGoodbye(bzJoint joint);

    /**
     * Called when any shape is about to be destroyed due
     * to the destruction of its parent body.
     */
    abstract void sayGoodbye(bzShape shape);
}

/**
 * This is called when a body's shape passes outside of the world boundary.
 */
abstract class bzBoundaryListener
{
    /**
     * This is called for each body that leaves the world boundary.
     * Warning: you can't modify the world inside this callback.
     */
    abstract void violation(bzBody rBody);
}

/**
 * Implement this class to provide collision filtering. In other words, you can implement
 * this class if you want finer control over contact creation.
 */
class bzContactFilter
{
    /**
     * Returns: true if contact calculations should be performed between these two shapes.
     * Warning: for performance reasons this is only called when the AABBs begin to overlap.
     * Note: If you implement your own collision filter you may want to build from this implementation.
     */
    bool shouldCollide(bzShape shape1, bzShape shape2)
    {

        bzFilterData filter1 = shape1.filter;
        bzFilterData filter2 = shape2.filter;

        if (filter1.groupIndex == filter2.groupIndex && filter1.groupIndex != 0)
        {
            return filter1.groupIndex > 0;
        }

        bool collide = (filter1.maskBits & filter2.categoryBits) != 0 && (filter1.categoryBits & filter2.maskBits) != 0;
        return collide;
    }

    /**
     * Returns: true if the given shape should be considered for ray intersection
     */
    bool rayCollide(Object userData, bzShape shape)
    {
        //By default, cast userData as a shape, and then collide if the shapes would collide
        if (!userData)
            return true;
        return shouldCollide(cast(bzShape) userData, shape);
    }
}
