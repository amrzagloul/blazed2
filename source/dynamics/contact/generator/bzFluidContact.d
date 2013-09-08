
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
module blaze.dynamics.contact.generator.bzFluidContact;

import blaze.collision.bzCollision;
import blaze.collision.shapes.bzShape;
import blaze.dynamics.bzWorldCallbacks;
import blaze.dynamics.contact.bzContact;

class bzFluidContact : bzContact
{
	this(bzShape s1, bzShape s2) {
	    super(s1, s2);
    }

    override void evaluate(bzContactListener listener) {}

}
