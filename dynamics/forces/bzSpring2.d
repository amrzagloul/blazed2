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
module blaze.dynamics.forces.bzSpring2;

import blaze.dynamics.bzBody;
import blaze.dynamics.forces.bzForceGenerator;
import blaze.common.bzMath;

public class bzSpring2 : bzForceGenerator {

    public bzVec2 offset;
    public bzVec2 offsetOther;
    public bzBody otherBody;

    public float restLength;
    public float stiffness;
    public float damping;

    this (bzBody rBody, bzBody otherBody, float stiffness, float restLenght = 0, float damping = 0,
          bzVec2 offset = bzVec2.zeroVect, bzVec2 offsetOther = bzVec2.zeroVect) {

        super(rBody);
        this.otherBody = otherBody;
        this.stiffness  = stiffness;
        this.restLength = restLenght < 0 ? -restLenght : restLenght;
        this.damping    = damping    < 0 ? -damping    : damping;
        this.offset = offset;
        this.offsetOther = offsetOther;
    }

    /**
     * -k(|x| - d)(x / |x|) - bv
     */
    override void evaluate() {

        float vx, fx, dx, rx0, rx1, ax0, ax1;
        float vy, fy, dy, ry0, ry1, ay0, ay1;
        float k, bv;

        int flag = 0;

        if (offset != bzVec2.zeroVect) {
            rx0 = rBody.xf.R.col1.x * offset.x + rBody.xf.R.col2.x * offset.y;
            ry0 = rBody.xf.R.col1.y * offset.x + rBody.xf.R.col2.y * offset.y;

            ax0 = rBody.position.x + rx0;
            ay0 = rBody.position.y + ry0;

            flag ^= 1;
        } else {
            ax0 = rBody.position.x;
            ay0 = rBody.position.y;

            rx0 = 0;
            ry0 = 0;
        }

        if (offsetOther != bzVec2.zeroVect) {
            rx1 = otherBody.xf.R.col1.x * offsetOther.x + otherBody.xf.R.col2.x * offsetOther.y;
            ry1 = otherBody.xf.R.col1.y * offsetOther.x + otherBody.xf.R.col2.y * offsetOther.y;

            ax1 = otherBody.position.x + rx1;
            ay1 = otherBody.position.y + ry1;

            flag ^= 1;
        } else {
            ax1 = otherBody.position.x;
            ay1 = otherBody.position.y;

            rx1 = 0;
            ry1 = 0;
        }

        dx = ax0 - ax1;
        dy = ay0 - ay1;

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

        if (flag != 0) {
            if (offset != bzVec2.zeroVect) {
                if (damping > 0) {
                    //-bv
                    vx = (rBody.linearVelocity.x - rBody.angularVelocity * ry0)
                        - otherBody.linearVelocity.x;
                    vy = (rBody.linearVelocity.y + rBody.angularVelocity * rx0)
                        - otherBody.linearVelocity.y;
                    bv = -damping * (vx * fx + vy * fy) /  (fx * fx + fy * fy);

                    fx += fx * bv;
                    fy += fy * bv;
                }

                rBody.force.x = rBody.force.x + fx;
                rBody.force.y = rBody.force.y + fy;
                rBody.torque = rBody.torque + rx0 * fy - ry0 * fx;

                otherBody.force.x = otherBody.force.x - fx;
                otherBody.force.y = otherBody.force.y - fy;
            } else {
                if (damping > 0) {
                    //-bv
                    vx = rBody.linearVelocity.x - (otherBody.linearVelocity.x - otherBody.angularVelocity * ry1);
                    vy = rBody.linearVelocity.y - (otherBody.linearVelocity.y + otherBody.angularVelocity * rx1);
                    bv = -damping * (vx * fx + vy * fy) /  (fx * fx + fy * fy);

                    fx += fx * bv;
                    fy += fy * bv;
                }

                rBody.force.x = rBody.force.x + fx;
                rBody.force.y = rBody.force.y + fy;

                otherBody.force.x = otherBody.force.x - fx;
                otherBody.force.y = otherBody.force.y - fy;
                otherBody.torque = otherBody.torque - rx1 * fy - ry1 * fx;
            }
        } else {
            if (offset != bzVec2.zeroVect) {
                if (damping > 0) {
                    //-bv
                    vx = (rBody.linearVelocity.x - rBody.angularVelocity * ry0)
                         - (otherBody.linearVelocity.x - otherBody.angularVelocity * ry1);
                    vy = (rBody.linearVelocity.y + rBody.angularVelocity * rx0)
                         - (otherBody.linearVelocity.y + otherBody.angularVelocity * rx1);
                    bv = -damping * (vx * fx + vy * fy) /  (fx * fx + fy * fy);

                    fx += fx * bv;
                    fy += fy * bv;
                }

                rBody.force.x = rBody.force.x + fx;
                rBody.force.y = rBody.force.y + fy;
                rBody.torque = rBody.torque + rx0 * fy - ry0 * fx;

                otherBody.force.x = otherBody.force.x - fx;
                otherBody.force.y = otherBody.force.y - fy;
                otherBody.torque = otherBody.torque - rx1 * fy - ry1 * fx;
            } else {
                if (damping > 0) {
                    //-bv
                    vx = rBody.linearVelocity.x - otherBody.linearVelocity.x;
                    vy = rBody.linearVelocity.y - otherBody.linearVelocity.y;
                    bv = -damping * (vx * fx + vy * fy) /  (fx * fx + fy * fy);

                    fx += fx * bv;
                    fy += fy * bv;
                }

                rBody.force.x = rBody.force.x + fx;
                rBody.force.y = rBody.force.y + fy;

                otherBody.force.x = otherBody.force.x - fx;
                otherBody.force.y = otherBody.force.y - fy;
            }
        }
    }
}
