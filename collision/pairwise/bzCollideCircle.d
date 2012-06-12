/*
* Copyright (c) 2007 Erin Catto http://www.gphysics.com
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

// Documentation modified by Brian Schott (Sir Alaran)

module blaze.collision.pairwise.bzCollideCircle;

import blaze.collision.bzCollision;
import blaze.collision.shapes.bzCircle;
import blaze.collision.shapes.bzPolygon;
import blaze.collision.shapes.bzFluidParticle;
import blaze.common.bzMath;


/**
 * Collides two circles against each other
 * Params:
 *     manifold = the collision manifold that results
 *     circle1 = the first circle
 *     circle2 = the second circle
 */
void collideCircles(ref bzManifold manifold, bzCircle circle1, bzCircle circle2) {

    bzXForm xf1 = circle1.rBody.xf;
    bzXForm xf2 = circle2.rBody.xf;

    manifold.pointCount = 0;

    bzVec2 p1 = bzMul(xf1, circle1.localPosition());
    bzVec2 p2 = bzMul(xf2, circle2.localPosition());

    bzVec2 d = p2 - p1;
    float distSqr = bzDot(d, d);
    float r1 = circle1.radius();
    float r2 = circle2.radius();
    float radiusSum = r1 + r2;
    if (distSqr > radiusSum * radiusSum) {
        return;
    }

    float separation;
    if (distSqr < float.epsilon) {
        separation = -radiusSum;
        manifold.normal.set(0.0f, 1.0f);
    } else {
        float dist = sqrt(distSqr);
        separation = dist - radiusSum;
        float a = 1.0f / dist;
        manifold.normal.x = a * d.x;
        manifold.normal.y = a * d.y;
    }

    manifold.pointCount = 1;
    manifold.points[0].id.key = 0;
    manifold.points[0].separation = separation;

    p1 += r1 * manifold.normal;
    p2 -= r2 * manifold.normal;

    bzVec2 p = 0.5f * (p1 + p2);

    manifold.points[0].localPoint1 = bzMulT(xf1, p);
    manifold.points[0].localPoint2 = bzMulT(xf2, p);
}

/**
 * Collides a circle against a polygon
 * Params:
 *     manifold = the collision manifold that results
 *     polygon = the polygon
 *     circle = the circle
 */
void collidePolygonCircle(ref bzManifold manifold, bzPolygon polygon, bzCircle circle) {

    bzXForm xf1 = polygon.rBody.xf;
    bzXForm xf2 = circle.rBody.xf;

    manifold.pointCount = 0;

    // Compute circle position in the frame of the polygon.
    bzVec2 c = bzMul(xf2, circle.localPosition());
    bzVec2 cLocal = bzMulT(xf1, c);

    // Find the min separating edge.
    int normalIndex = 0;
    float separation = -float.max;
    float radius = circle.radius();
    int vertexCount = polygon.vertices.length;
    bzVec2[] vertices = polygon.vertices;
    bzVec2[] normals = polygon.normals;

    for (int i = 0; i < vertexCount; ++i) {
        float s = bzDot(normals[i], cLocal - vertices[i]);

        if (s > radius) {
            // Early out.
            return;
        }

        if (s > separation) {
            separation = s;
            normalIndex = i;
        }
    }

    // If the center is inside the polygon ...
    if (separation < float.epsilon) {
        manifold.pointCount = 1;
        manifold.normal = bzMul(xf1.R, normals[normalIndex]);
        manifold.points[0].id.features.incidentEdge = cast(ubyte) normalIndex;
        manifold.points[0].id.features.incidentVertex = ubyte.max;
        manifold.points[0].id.features.referenceEdge = 0;
        manifold.points[0].id.features.flip = 0;
        bzVec2 position = c - radius * manifold.normal;
        manifold.points[0].localPoint1 = bzMulT(xf1, position);
        manifold.points[0].localPoint2 = bzMulT(xf2, position);
        manifold.points[0].separation = separation - radius;
        return;
    }

    // Project the circle center onto the edge segment.
    int vertIndex1 = normalIndex;
    int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
    bzVec2 e = vertices[vertIndex2] - vertices[vertIndex1];

    float length = e.normalize;
    assert(length > float.epsilon);

    // Project the center onto the edge.
    float u = bzDot(cLocal - vertices[vertIndex1], e);
    bzVec2 p;
    if (u <= 0.0f) {
        p = vertices[vertIndex1];
        manifold.points[0].id.features.incidentEdge = ubyte.max;
        manifold.points[0].id.features.incidentVertex = cast(ubyte) vertIndex1;
    } else if (u >= length) {
        p = vertices[vertIndex2];
        manifold.points[0].id.features.incidentEdge = ubyte.max;
        manifold.points[0].id.features.incidentVertex = cast(ubyte) vertIndex2;
    } else {
        p = vertices[vertIndex1] + u * e;
        manifold.points[0].id.features.incidentEdge = cast(ubyte) normalIndex;
        manifold.points[0].id.features.incidentVertex = ubyte.max;
    }

    bzVec2 d = cLocal - p;
    float dist = d.normalize;
    if (dist > radius) {
        return;
    }

    manifold.pointCount = 1;
    manifold.normal = bzMul(xf1.R, d);
    bzVec2 position = c - radius * manifold.normal;
    manifold.points[0].localPoint1 = bzMulT(xf1, position);
    manifold.points[0].localPoint2 = bzMulT(xf2, position);
    manifold.points[0].separation = dist - radius;
    manifold.points[0].id.features.referenceEdge = 0;
    manifold.points[0].id.features.flip = 0;
}

/**
 * Collides a circle against a fluid particle
 * Params:
 *     circle = the circle
 *     particle = the fluid particle
 *     penetration = the vector representing how far the particle has penetrated
 *         into the circle
 *     penetrationNormal = a normalized version of penetration
 */
bool collideCircleFluid(bzCircle circle, bzFluidParticle particle,
	ref bzVec2 penetration, ref bzVec2 penetrationNormal) {

    bzXForm xf1 = circle.rBody.xf;

    bzVec2 p1 = bzMul(xf1, circle.localPosition());
    bzVec2 p2 = particle.position;
    bzVec2 normal;

    bzVec2 d = p2 - p1;
    float distSqr = bzDot(d, d);
    float r1 = circle.radius;
    float r2 = 0.0f;
    float radiusSum = r1 + r2;
    if (distSqr > radiusSum * radiusSum) {
        return false;
    }

    float separation;
    if (distSqr < float.epsilon) {
        separation = -radiusSum;
        normal.set(0.0f, 1.0f);
    } else {
        float dist = sqrt(distSqr);
        separation = dist - radiusSum;
        float a = 1.0f / dist;
        normal.x = a * d.x;
        normal.y = a * d.y;
    }

    penetration = normal * separation;
    penetrationNormal = normal;
    return true;
}
