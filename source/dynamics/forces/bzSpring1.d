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
module blaze.dynamics.forces.bzSpring1;

import blaze.dynamics.bzBody;
import blaze.dynamics.forces.bzForceGenerator;
import blaze.common.bzMath;

/**
 * an elastic spring based on hooks law connecting a fixed anchor and a rigid rBody.
 */
public class bzSpring1 : bzForceGenerator {
    /**
     * spring anchor in world-space coordinates.
     */
    public bzVec2 anchor;

    /**
     * spring offset in modeling-space coordinates.
     * default is null (no torque)
     */
    public bzVec2 offset;

    public float restLength;
    public float stiffness;
    public float damping;

    this (bzBody rBody, bzVec2 anchor, float stiffness, float restLenght = 0, float damping = 0, bzVec2 offset = bzVec2.zeroVect) {
        super(rBody);
        this.anchor = anchor;
        this.stiffness  = stiffness;
        this.restLength = restLenght < 0 ? -restLenght : restLenght;
        this.damping    = damping    < 0 ? -damping    : damping;
        this.offset = offset;
    }

    /**
     * -k(|x| - d)(x / |x|) - bv
     */
    override void evaluate() {

        float fx, dx, rx;
        float fy, dy, ry;
        float k, bv;

        if (offset != bzVec2.zeroVect) {
            rx = (rBody.xf.R.col1.x * offset.x + rBody.xf.R.col2.x * offset.y);
            ry = (rBody.xf.R.col1.y * offset.x + rBody.xf.R.col2.y * offset.y);
            dx = (rBody.position.x + rx) - anchor.x;
            dy = (rBody.position.y + ry) - anchor.y;
        } else {
            dx = rBody.position.x - anchor.x;
            dy = rBody.position.y - anchor.y;
        }

        if (restLength > 0) {
            //-k(|x| - d)(x / |x|)
            float  l = sqrt(dx * dx + dy * dy) + 1e-6;
            k = -stiffness * (l - restLength);
            fx = k * (dx / l);
            fy = k * (dy / l);
        } else {
            //-kx
            k = -stiffness;
            fx = k * dx;
            fy = k * dy;
        }

        if (offset != bzVec2.zeroVect) {
            float vx;
            float vy;

            if (damping > 0) {
                //-bv
                vx = rBody.linearVelocity.x - rBody.angularVelocity * ry;
                vy = rBody.linearVelocity.y + rBody.angularVelocity * rx;
                bv = -damping * (vx * fx + vy * fy) /  (fx * fx + fy * fy);
                fx += fx * bv;
                fy += fy * bv;
            }

            rBody.torque = rBody.torque + rx * fy - ry * fx;
        } else {
            if (damping > 0) {
                //-bv
                bv = -damping * (rBody.linearVelocity.x * fx + rBody.linearVelocity.y * fy) /  (fx * fx + fy * fy);
                fx += fx * bv;
                fy += fy * bv;
            }
        }

        rBody.force.x = rBody.force.x + fx;
        rBody.force.y = rBody.force.y + fy;
    }
}

