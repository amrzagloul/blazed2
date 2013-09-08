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
module blaze.dynamics.forces.bzForceRegistry;

import blaze.dynamics.bzBody;
import blaze.dynamics.forces.bzForceGenerator;

public class bzForceRegistry {

    this () {
        _registration = null;
    }

    void add(bzForceGenerator force) {

        bzForceNode n = new bzForceNode(force);
        n.next = _registration;
        if (_registration) _registration.prev = n;
        _registration = n;
    }

    public bool remove(bzForceGenerator force) {
        bzForceNode n = _registration;
        while (n) {
            if (n.force is force) {
                if (n.prev) n.prev.next = n.next;
                if (n.next) n.next.prev = n.prev;
                if (n == _registration) _registration = n.next;
                return true;
            }
            n = n.next;
        }

        return false;
    }

    void clear() {
        bzForceNode n = _registration;
        _registration = null;

        bzForceNode next;
        while (n) {
            next = n.next;
            n.next = n.prev = null;
            n = next;
        }
    }

    public void evaluate() {
        bzForceNode n = _registration;
        bzForceGenerator f;
        while (n) {
            f = n.force;
            if (f.isActive) f.evaluate();
            n = n.next;
        }
    }

    bzForceGenerator[] forces() {
        bzForceNode n = _registration;
        bzForceGenerator[] f;
        while (n) {
            f ~= n.force;
            n = n.next;
        }
        return f;
    }

private:

    bzForceNode _registration;
}

class bzForceNode {

    bzForceGenerator force;

    bzForceNode prev;
    bzForceNode next;

    this (bzForceGenerator force) {
        this.force = force;
        prev = next = null;
    }
}
