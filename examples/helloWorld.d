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
module helloWorld;

import blaze.all;

import std.stdio;

// This is a simple example of building and running a simulation
// using Box2D. Here we create a large ground box and a small dynamic
// box.
void main()
{
	// Define the gravity vector.
	bzVec2 gravity = bzVec2(0.0f, -10.0f);

	// Do we want to let bodies sleep?
	bool doSleep = true;

	// Define world boundaries
	bzAABB worldAABB;
	worldAABB.lowerBound.set(-200.0f, -100.0f);
	worldAABB.upperBound.set(200.0f, 200.0f);

	// Construct a world object, which will hold and simulate the rigid bodies.
	auto world = new bzWorld(worldAABB, gravity, doSleep);

	// Define the ground body.
	auto groundBodyDef = new bzBodyDef(bzVec2(0.0f, -10.0f), 0);

	// Call the body factory which allocates memory for the ground body
	// from a pool and creates the ground box shape (also from a pool).
	// The body is also added to the world.
	auto groundBody = world.createBody(groundBodyDef);

	// Define the ground box shape.
	auto groundShapeDef = new bzPolyDef();

	// The extents are the half-widths of the box.
	groundShapeDef.setAsBox(50.0f, 10.0f);

	// Add the ground shape to the ground body.
	groundBody.createShape(groundShapeDef);

	// Define the dynamic body. We set its position and call the body factory.
    auto bodyDef = new bzBodyDef(bzVec2(0.0f, 4.0f), 0);
	auto rBody = world.createBody(bodyDef);

	// Define another box shape for our dynamic body.
	auto shapeDef = new bzPolyDef();
	shapeDef.setAsBox(1.0f, 1.0f);

	// Set the box density to be non-zero, so it will be dynamic.
	shapeDef.density = 1.0f;

	// Override the default friction.
	shapeDef.friction = 0.3f;

	// Add the shape to the body.
	rBody.createShape(shapeDef);

	// Now tell the dynamic body to compute it's mass properties base
	// on its shape.
	rBody.setMassFromShapes();

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 10 iterations. This provides a high quality simulation
	// in most game scenarios.
	float timeStep = 1.0f / 60.0f;
	int velocityIterations = 8;
	int positionIterations = 1;

	// This is our little game loop.
	for (int i = 0; i < 60; ++i) {
	    // Instruct the world to perform a single step of simulation. It is
	    // generally best to keep the time step and iterations fixed.
	    world.step(timeStep, velocityIterations, positionIterations);

	    // Now print the position and angle of the body.
	    bzVec2 position = rBody.position;
	    float angle = rBody.angle;

	    writef("%.4f", position.x);
	    writef(",");
	    writef("%.4f", position.y);
	    writefln(" %.4f", angle);
	}

	// When the world destructor is called, all bodies and joints are freed. This can
	// create orphaned pointers, so be careful about your world management.

	return;
}
