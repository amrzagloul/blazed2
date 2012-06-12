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
module blaze.dynamics.forces.bzAttractor;

import blaze.dynamics.forces.bzForceGenerator;
import blaze.dynamics.bzBody;
import blaze.common.bzMath;


public class bzAttractor : bzForceGenerator {

    bzVec2 center;

    float strength;
    float minRadius;
    float maxRadius;

    this (bzBody rBody, bzVec2 center, float strength, float minRadius, float maxRadius) {
        super(rBody);
        this.center = center;
        this.strength = strength;
        this.minRadius = minRadius;
        this.maxRadius = maxRadius;
    }

    override void evaluate() {

        float rx = center.x - rBody.position.x;
        float ry = center.y - rBody.position.y;

        bzVec2 r = center - rBody.position;

        float d = sqrt(rx * rx + ry * ry);
        if (d < 1e-7)
            return;
        else {
            rx /= d;
            ry /= d;
        }

        float ratio = (d - minRadius) / (maxRadius - minRadius);
        if (ratio < 0)
            ratio = 0;
        else
            if (ratio > 1)
                ratio = 1;

        rBody.force = rBody.force + r * ratio * strength;
    }
}

