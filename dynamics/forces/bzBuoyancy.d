/*
 * Copyright (c) 2007-2008, Michael Baczynski
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
module blaze.dynamics.forces.bzBuoyancy;

import blaze.bzWorld;
import blaze.collision.shapes.bzShape;
import blaze.collision.shapes.bzShapeType;
import blaze.collision.shapes.bzCircle;
import blaze.dynamics.bzBody;
import blaze.common.bzMath;

import blaze.dynamics.forces.bzForceGenerator;

public class bzBuoyancy : bzForceGenerator {

    float density;
    float linDrag;
    float angDrag;

    bzVec2 velocity;
    bzVec2 planeNormal;
    float planeOffset = 0.0f;

	this (bzBody rBody, float planeOffset, bzVec2 planeNormal,
		  float density, float linDrag = 5, float angDrag = .5, bzVec2 velocity = bzVec2.zeroVect) {

        super(rBody);
        this.density = density;
        this.linDrag = linDrag;
        this.angDrag = angDrag;
		this.velocity = velocity;
        this.planeOffset = planeOffset;
        this.planeNormal = planeNormal;

    }

    override void evaluate() {

        float totalArea = 0;
        float submergedArea = 0;

        float xmin = float.max;
        float xmax = float.min;

        float cx = 0.0f;
        float cy = 0.0f;
        int nOut;
        bzVec2 a, b, c;
        float area = 0.0f;

        //compute submerged area and center of buoyancy

        for (bzShape s = rBody.shapeList(); s; s = s.next) {

			if(s.type == bzShapeType.POLYGON) {
				s.triangulate();
			}

            totalArea += s.area;
			float x = s.worldCenter().x;
			float y = s.worldCenter().y;

            //above plane
            if (s.ymin > planeOffset) continue;

            // below plane - 100% submerged
            if (s.ymax <= planeOffset) {
                area = s.area;
                cx += area * x;
                cy += area * y;
                submergedArea += area;
                if (s.xmin < xmin) xmin = s.xmin;
                if (s.xmax > xmax) xmax = s.xmax;
                continue;
            }

            if (s.type == bzShapeType.CIRCLE) {

				auto circ = cast(bzCircle) s;
				float r = circ.radius;
				float h = s.ymax - planeOffset;

				if(2 * r - h <= float.epsilon) continue;

				// http://mathworld.wolfram.com/CircularSegment.html
				area = s.area - (r * r * acos((r - h) / r) - (r - h) * sqrt(2 * r * h - h * h));
				// http://mathworld.wolfram.com/Semicircle.html
				float gc = 4 * (2 * r - h) / (3 * PI);
				float z = gc - r - h;

				cx += x * area;
				cy += (y - z) * area;
				submergedArea += area;
				if (s.xmin < xmin) xmin = s.xmin;
				if (s.xmax > xmax) xmax = s.xmax;

            } else {

                //clip triangle against plane
                foreach(t; s.triangleList()) {

                    nOut = clipTriangle(t, planeOffset, _clipTri0, _clipTri1);

					//Stdout(nOut).newline;

                    //accumulate submerged area and center of buoyancy

                    if (nOut > 0) {

                        a = _clipTri0.a;
                        b = _clipTri0.b;
                        c = _clipTri0.c;

                        area = ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) / 2;

                        if (area < 0) area = -area;
                        if (area > 1e-5) {
                            cx += (area * (a.x + b.x + c.x) / 3);
                            cy += (area * (a.y + b.y + c.y) / 3);
                            submergedArea += area;
                        }

                    }

                    if (nOut > 1) {
                        a = _clipTri1.a;
                        b = _clipTri1.b;
                        c = _clipTri1.c;

                        area = ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) / 2;

                        if (area < 0) area = -area;
                        if (area > 1e-5) {
                            cx += (area * (a.x + b.x + c.x) / 3);
                            cy += (area * (a.y + b.y + c.y) / 3);
                            submergedArea += area;
                        }

                    }

                    if (s.xmin < xmin) xmin = s.xmin;
                    if (s.xmax > xmax) xmax = s.xmax;
                }
            }
        }

		if (submergedArea <= float.epsilon) return;

        //normalize the centroid by the total volume
        cx /= submergedArea;
        cy /= submergedArea;

        //compute buoyancy force

        float force = density * submergedArea * rBody.world().gravity().y;
        float partialMass = rBody.mass * submergedArea / totalArea;
        float rc_x = cx - rBody.position.x;
        float rc_y = cy - rBody.position.y;

        float fx = (planeNormal.x * force) + ((partialMass * linDrag) * (velocity.x
                   - (rBody.linearVelocity.x - rBody.angularVelocity * rc_y)));
        float fy = (planeNormal.y * force) + ((partialMass * linDrag) * (velocity.y
                   - (rBody.linearVelocity.y + rBody.angularVelocity * rc_x)));

		rBody.force = rBody.force + bzVec2(fx, fy);
        rBody.torque = rBody.torque + ((rc_x * fy - rc_y * fx) + ((-partialMass * angDrag
                       * ((xmax - xmin) * (xmax - xmin))) * rBody.angularVelocity));
    }

private:

    int clipTriangle(bzTri2 tri, float offset, ref ClipTriangle ct0, ref ClipTriangle ct1) {

        //count points below / above plane
        int aboveCount = 0;
        int belowCount = 0;

        bzVec2 aboveVec0;
        bzVec2 belowVec0;
        bzVec2 aboveVec1;
        bzVec2 belowVec1;

        if (tri.a.y < offset) {
            ++belowCount;
            belowVec0 = tri.a;
        } else {
            ++aboveCount;
            aboveVec0 = tri.a;
        }

        if (tri.b.y < offset) {
            ++belowCount;
            if (belowVec0 != bzVec2.zeroVect)
                belowVec1 = tri.b;
            else
                belowVec0 = tri.b;
        } else {
            ++aboveCount;
            if (aboveVec0 != bzVec2.zeroVect)
                aboveVec1 = tri.b;
            else
                aboveVec0 = tri.b;
        }

        if (tri.c.y < offset) {
            ++belowCount;
            if (belowVec0 != bzVec2.zeroVect)
                belowVec1 = tri.c;
            else
                belowVec0 = tri.c;
        } else {
            ++aboveCount;
            if (aboveVec0 != bzVec2.zeroVect)
                aboveVec1 = tri.c;
            else
                aboveVec0 = tri.c;
        }

        //early out
        if (aboveCount == 0) {
            ct0.a = tri.a;
            ct0.b = tri.b;
            ct0.c = tri.c;
            return 1;
        } else if (belowCount == 0)
            return -1;

        //clip triangle against plane
        bzVec2 p1, p2, t;
		float distance0, distance1, interp;

        //two submerged vertices . two clipped triangles
        if (aboveCount == 1) {
            p1 = aboveVec0;
            p2 = belowVec0;

            //offset = -waterLevel;
            distance0 = p1.y - offset;
            distance1 = p2.y - offset;
            interp = distance0 / (distance0 - distance1);
            _cp0.x = p1.x + interp * (p2.x - p1.x);
            _cp0.y = p1.y + interp * (p2.y - p1.y);

            p2 = belowVec1;

            distance1 = p2.y - offset;
            interp = distance0 / (distance0 - distance1);
            _cp1.x = p1.x + interp * (p2.x - p1.x);
            _cp1.y = p1.y + interp * (p2.y - p1.y);

            if (_cp0.x > _cp1.x) {
                t = _cp0;
                _cp0 = _cp1;
                _cp1 = t;
            }

            //(belowVec0 - p1) x (belowVec1, p1)
            float side = (belowVec0.x - p1.x) * (belowVec1.y - p1.y) - (belowVec0.y - p1.y) * (belowVec1.x - p1.x);

            //cp0, cp1 --> b0, b1
            if (belowVec0.x > _cp1.x) {
                ct0.a = belowVec1;
                ct0.b = _cp1;
                ct0.c = belowVec0;

                if (side > 0) {
                    ct1.a = belowVec1;
                    ct1.b = _cp1;
                    ct1.c = _cp0;
                } else {
                    ct1.a = belowVec0;
                    ct1.b = _cp1;
                    ct1.c = _cp0;
                }
                return 2;
            } else
                //b0, b1 <-- cp0, cp1
                if (belowVec1.x < _cp0.x) {
                    ct0.a = belowVec1;
                    ct0.b = _cp0;
                    ct0.c = belowVec0;

                    if (side > 0) {
                        ct1.a = belowVec0;
                        ct1.b = _cp1;
                        ct1.c = _cp0;
                    } else {
                        ct1.a = belowVec1;
                        ct1.b = _cp1;
                        ct1.c = _cp0;
                    }
                    return 2;
                }
            //cp0 --> b0 --> cp1 --> b1
                else {
                    ct0.a = belowVec0;
                    ct0.b = belowVec1;
                    ct0.c = _cp0;

                    ct1.a = _cp0;
                    ct1.b = belowVec1;
                    ct1.c = _cp1;
                    return 2;
                }
        } else
            //one submerged vertice . one clipped triangle
            if (aboveCount == 2) {
                p1 = belowVec0;
                p2 = aboveVec0;

                distance0 = p1.y - offset;
                distance1 = p2.y - offset;
                interp = distance0 / (distance0 - distance1);
                _cp0.x = p1.x + interp * (p2.x - p1.x);
                _cp0.y = p1.y + interp * (p2.y - p1.y);

                p2 = aboveVec1;

                distance1 = p2.y - offset;
                interp = distance0 / (distance0 - distance1);
                _cp1.x = p1.x + interp * (p2.x - p1.x);
                _cp1.y = p1.y + interp * (p2.y - p1.y);

                if (_cp0.x > _cp1.x) {
                    t = _cp0;
                    _cp0 = _cp1;
                    _cp1 = t;
                }

                ct0.a = _cp1;
                ct0.b = _cp0;
                ct0.c = belowVec0;

                return 1;
            }
        return -1;
    }

    ClipTriangle _clipTri0;
    ClipTriangle _clipTri1;
    bzVec2 _cp0;
    bzVec2 _cp1;

}

struct ClipTriangle {
    bzVec2 a;
    bzVec2 b;
    bzVec2 c;
}

struct Plane2 {
    bzVec2 n;
    float d = 0;
}

