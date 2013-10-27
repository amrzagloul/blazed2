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
module blaze.dynamics.contact.generator.bzPolyEdgeContact;

import blaze.common.bzConstants;
import blaze.common.bzMath;
import blaze.collision.shapes.bzShape;
import blaze.collision.shapes.bzShapeType;
import blaze.collision.shapes.bzPolygon;
import blaze.collision.shapes.bzEdge;
import blaze.collision.bzCollision;
import blaze.dynamics.contact.bzContact;
import blaze.dynamics.bzWorldCallbacks;
import blaze.dynamics.bzBody;

class bzPolyEdgeContact : bzContact {
public:

    this(bzShape s1, bzShape s2) {

        assert(s1.type == bzShapeType.POLYGON);
        assert(s2.type == bzShapeType.EDGE);
        super(s1, s2);
    }

    static bzContact create(bzShape shape1, bzShape shape2) {

        return new bzPolyEdgeContact(shape1, shape2);
    }

    override void evaluate(bzContactListener listener) {

        bzBody b1 = m_shape1.rBody;
        bzBody b2 = m_shape2.rBody;

        bzManifold m0 = manifold;

        collidePolyAndEdge(manifold, cast (bzPolygon) m_shape1, b1.xf, cast (bzEdge) m_shape2, b2.xf);

        bool persisted[k_maxManifoldPoints] = [false, false];

        bzContactPoint cp;
        cp.shape1 = m_shape1;
        cp.shape2 = m_shape2;
        cp.friction = mixFriction(m_shape1.friction, m_shape2.friction);
        cp.restitution = mixRestitution(m_shape1.restitution, m_shape2.restitution);

        // Match contact ids to facilitate warm starting.
        if (manifold.pointCount > 0) {
            // Match old contact ids to new contact ids and copy the
            // stored impulses to warm start the solver.
            for (int i = 0; i < manifold.pointCount; ++i) {
                bzManifoldPoint *mp = &manifold.points[i];
                mp.normalImpulse = 0.0f;
                mp.tangentImpulse = 0.0f;
                bool found = false;
                bzContactID id = mp.id;

                for (int j = 0; j < m0.pointCount; ++j) {
                    if (persisted[j] == true) {
                        continue;
                    }

                    bzManifoldPoint mp0 = m0.points[j];

                    if (mp0.id.key == id.key) {
                        persisted[j] = true;
                        mp.normalImpulse = mp0.normalImpulse;
                        mp.tangentImpulse = mp0.tangentImpulse;

                        // A persistent point.
                        found = true;

                        // Report persistent point.
                        if (listener) {
                            cp.position = b1.worldPoint(mp.localPoint1);
                            bzVec2 v1 = b1.linearVelocityFromLocalPoint(mp.localPoint1);
                            bzVec2 v2 = b2.linearVelocityFromLocalPoint(mp.localPoint2);
                            cp.velocity = v2 - v1;
                            cp.normal = manifold.normal;
                            cp.separation = mp.separation;
                            cp.id = id;
                            listener.persist(cp);
                        }
                        break;
                    }
                }

                // Report added point.
                if (!found && listener) {
                    cp.position = b1.worldPoint(mp.localPoint1);
                    bzVec2 v1 = b1.linearVelocityFromLocalPoint(mp.localPoint1);
                    bzVec2 v2 = b2.linearVelocityFromLocalPoint(mp.localPoint2);
                    cp.velocity = v2 - v1;
                    cp.normal = manifold.normal;
                    cp.separation = mp.separation;
                    cp.id = id;
                    listener.add(cp);
                }
            }

            m_manifoldCount = 1;
        } else {
            m_manifoldCount = 0;
        }

        if (listener is null) {
            return;
        }

        // Report removed points.
        for (int i = 0; i < m0.pointCount; ++i) {
            if (persisted[i]) {
                continue;
            }

            bzManifoldPoint mp0 = m0.points[i];
            cp.position = b1.worldPoint(mp0.localPoint1);
            bzVec2 v1 = b1.linearVelocityFromLocalPoint(mp0.localPoint1);
            bzVec2 v2 = b2.linearVelocityFromLocalPoint(mp0.localPoint2);
            cp.velocity = v2 - v1;
            cp.normal = m0.normal;
            cp.separation = mp0.separation;
            cp.id = mp0.id;
            listener.remove(cp);
        }
    }

    void collidePolyAndEdge(ref bzManifold manifold, bzPolygon polygon, bzXForm xf1, bzEdge edge, bzXForm xf2) {

        manifold.pointCount = 0;
        bzVec2 v1 = bzMul(xf2, edge.vertex1());
        bzVec2 v2 = bzMul(xf2, edge.vertex2());
        bzVec2 n = bzMul(xf2.R, edge.normalVector());
        bzVec2 v1Local = bzMulT(xf1, v1);
        bzVec2 v2Local = bzMulT(xf1, v2);
        bzVec2 nLocal = bzMulT(xf1.R, n);

        float separation1 = 0;
        int separationIndex1 = -1; // which normal on the poly found the shallowest depth?
        float separationMax1 = -float.max; // the shallowest depth of edge in poly
        float separation2 = 0;
        int separationIndex2 = -1; // which normal on the poly found the shallowest depth?
        float separationMax2 = -float.max; // the shallowest depth of edge in poly
        float separationMax = -float.max; // the shallowest depth of edge in poly
        bool separationV1 = false; // is the shallowest depth from edge's v1 or v2 vertex?
        int separationIndex = -1; // which normal on the poly found the shallowest depth?

        int vertexCount = polygon.vertexCount();
        bzVec2[] vertices = polygon.vertices[];
        bzVec2[] normals = polygon.normals[];

        int enterStartIndex = -1; // the last poly vertex above the edge
        int enterEndIndex = -1; // the first poly vertex below the edge
        int exitStartIndex = -1; // the last poly vertex below the edge
        int exitEndIndex = -1; // the first poly vertex above the edge
        //int deepestIndex;

        // the "N" in the following variables refers to the edge's normal.
        // these are projections of poly vertices along the edge's normal,
        // a.k.a. they are the separation of the poly from the edge.
        float prevSepN = 0.0f;
        float nextSepN = 0.0f;
        float enterSepN = 0.0f; // the depth of enterEndIndex under the edge (stored as a separation, so it's negative)
        float exitSepN = 0.0f; // the depth of exitStartIndex under the edge (stored as a separation, so it's negative)
        float deepestSepN = float.max; // the depth of the deepest poly vertex under the end (stored as a separation, so it's negative)


        // for each poly normal, get the edge's depth into the poly.
        // for each poly vertex, get the vertex's depth into the edge.
        // use these calculations to define the remaining variables declared above.
        prevSepN = bzDot(vertices[vertexCount-1] - v1Local, nLocal);
        for (int i = 0; i < vertexCount; i++) {
            separation1 = bzDot(v1Local - vertices[i], normals[i]);
            separation2 = bzDot(v2Local - vertices[i], normals[i]);
            if (separation2 < separation1) {
                if (separation2 > separationMax) {
                    separationMax = separation2;
                    separationV1 = false;
                    separationIndex = i;
                }
            } else {
                if (separation1 > separationMax) {
                    separationMax = separation1;
                    separationV1 = true;
                    separationIndex = i;
                }
            }
            if (separation1 > separationMax1) {
                separationMax1 = separation1;
                separationIndex1 = i;
            }
            if (separation2 > separationMax2) {
                separationMax2 = separation2;
                separationIndex2 = i;
            }

            nextSepN = bzDot(vertices[i] - v1Local, nLocal);
            if (nextSepN >= 0.0f && prevSepN < 0.0f) {
                exitStartIndex = (i == 0) ? vertexCount-1 : i-1;
                exitEndIndex = i;
                exitSepN = prevSepN;
            } else if (nextSepN < 0.0f && prevSepN >= 0.0f) {
                enterStartIndex = (i == 0) ? vertexCount-1 : i-1;
                enterEndIndex = i;
                enterSepN = nextSepN;
            }
            if (nextSepN < deepestSepN) {
                deepestSepN = nextSepN;
                //deepestIndex = i;
            }
            prevSepN = nextSepN;
        }

        if (enterStartIndex == -1) {
            // poly is entirely below or entirely above edge, return with no contact:
            return;
        }
        if (separationMax > 0.0f) {
            // poly is laterally disjoint with edge, return with no contact:
            return;
        }

        // if the poly is near a convex corner on the edge
        if ((separationV1 && edge.corner1IsConvex()) || (!separationV1 && edge.corner2IsConvex())) {
            // if shallowest depth was from edge into poly,
            // use the edge's vertex as the contact point:
            if (separationMax > deepestSepN + k_linearSlop) {
                // if -normal angle is closer to adjacent edge than this edge,
                // let the adjacent edge handle it and return with no contact:
                if (separationV1) {
                    if (bzDot(normals[separationIndex1], bzMulT(xf1.R, bzMul(xf2.R, edge.corner1Vector()))) >= 0.0f) {
                        return;
                    }
                } else {
                    if (bzDot(normals[separationIndex2], bzMulT(xf1.R, bzMul(xf2.R, edge.corner2Vector()))) <= 0.0f) {
                        return;
                    }
                }

                manifold.pointCount = 1;
                manifold.normal = bzMul(xf1.R, normals[separationIndex]);
                manifold.points[0].separation = separationMax;
                manifold.points[0].id.features.incidentEdge = separationIndex;
                manifold.points[0].id.features.incidentVertex = k_nullFeature;
                manifold.points[0].id.features.referenceEdge = 0;
                manifold.points[0].id.features.flip = 0;
                if (separationV1) {
                    manifold.points[0].localPoint1 = v1Local;
                    manifold.points[0].localPoint2 = edge.vertex1();
                } else {
                    manifold.points[0].localPoint1 = v2Local;
                    manifold.points[0].localPoint2 = edge.vertex2();
                }
                return;
            }
        }

        // We're going to use the edge's normal now.
        manifold.normal = (-1.0f) * n;

        // Check whether we only need one contact point.
        if (enterEndIndex == exitStartIndex) {
            manifold.pointCount = 1;
            manifold.points[0].id.features.incidentEdge = enterEndIndex;
            manifold.points[0].id.features.incidentVertex = k_nullFeature;
            manifold.points[0].id.features.referenceEdge = 0;
            manifold.points[0].id.features.flip = 0;
            manifold.points[0].localPoint1 = vertices[enterEndIndex];
            manifold.points[0].localPoint2 = bzMulT(xf2, bzMul(xf1, vertices[enterEndIndex]));
            manifold.points[0].separation = enterSepN;
            return;
        }

        manifold.pointCount = 2;

        // dirLocal should be the edge's direction vector, but in the frame of the polygon.
        bzVec2 dirLocal = bzCross(nLocal, -1.0f); // TODO: figure out why this optimization didn't work
        //bzVec2 dirLocal = bzMulT(xf1.R, bzMul(xf2.R, edge.GetDirectionVector()));

        float dirProj1 = bzDot(dirLocal, vertices[enterEndIndex] - v1Local);
        float dirProj2 = 0;

        // The contact resolution is more robust if the two manifold points are
        // adjacent to each other on the polygon. So pick the first two poly
        // vertices that are under the edge:
        exitEndIndex = (enterEndIndex == vertexCount - 1) ? 0 : enterEndIndex + 1;
        if (exitEndIndex != exitStartIndex) {
            exitStartIndex = exitEndIndex;
            exitSepN = bzDot(nLocal, vertices[exitStartIndex] - v1Local);
        }
        dirProj2 = bzDot(dirLocal, vertices[exitStartIndex] - v1Local);

        manifold.points[0].id.features.incidentEdge = enterEndIndex;
        manifold.points[0].id.features.incidentVertex = k_nullFeature;
        manifold.points[0].id.features.referenceEdge = 0;
        manifold.points[0].id.features.flip = 0;

        if (dirProj1 > edge.length()) {
            manifold.points[0].localPoint1 = v2Local;
            manifold.points[0].localPoint2 = edge.vertex2();
            float ratio = (edge.length() - dirProj2) / (dirProj1 - dirProj2);
            if (ratio > 100.0f * float.epsilon && ratio < 1.0f) {
                manifold.points[0].separation = exitSepN * (1.0f - ratio) + enterSepN * ratio;
            } else {
                manifold.points[0].separation = enterSepN;
            }
        } else {
            manifold.points[0].localPoint1 = vertices[enterEndIndex];
            manifold.points[0].localPoint2 = bzMulT(xf2, bzMul(xf1, vertices[enterEndIndex]));
            manifold.points[0].separation = enterSepN;
        }

        manifold.points[1].id.features.incidentEdge = exitStartIndex;
        manifold.points[1].id.features.incidentVertex = k_nullFeature;
        manifold.points[1].id.features.referenceEdge = 0;
        manifold.points[1].id.features.flip = 0;

        if (dirProj2 < 0.0f) {
            manifold.points[1].localPoint1 = v1Local;
            manifold.points[1].localPoint2 = edge.vertex1();
            float ratio = (-dirProj1) / (dirProj2 - dirProj1);
            if (ratio > 100.0f * float.epsilon && ratio < 1.0f) {
                manifold.points[1].separation = enterSepN * (1.0f - ratio) + exitSepN * ratio;
            } else {
                manifold.points[1].separation = exitSepN;
            }
        } else {
            manifold.points[1].localPoint1 = vertices[exitStartIndex];
            manifold.points[1].localPoint2 = bzMulT(xf2, bzMul(xf1, vertices[exitStartIndex]));
            manifold.points[1].separation = exitSepN;
        }
    }
}
