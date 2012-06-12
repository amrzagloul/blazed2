/*
 * Copyright (c) 2008-2009, Mason Green (zzzzrrr)
 * Based on Box2D & JBox2D
 *
 * JBox2D homepage: http://jbox2d.sourceforge.net/
 * Box2D homepage: http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
module blaze.collision.nbody.bzPair;

public class bzPair {

    private static immutable int PAIR_BUFFERED = 0x0001;
    private static immutable int PAIR_REMOVED = 0x0002;
    private static immutable int PAIR_FINAL = 0x0004;

    public Object userData;

    public int proxyId1;

    public int proxyId2;

    public int status;

    public int next;

    this() {
    }

    /**
     * Copy constructor
     */
    this(bzPair other) {
        this.userData = other.userData;
        this.proxyId1 = other.proxyId1;
        this.proxyId2 = other.proxyId2;
        this.status = other.status;
    }

    public void setBuffered() {
        status |= PAIR_BUFFERED;
    }

    public void clearBuffered() {
        status &= ~PAIR_BUFFERED;
    }

    public bool isBuffered() {
        return (status & PAIR_BUFFERED) == PAIR_BUFFERED;
    }

    public void clearRemoved() {
        status &= ~PAIR_REMOVED;
    }

    public void setRemoved() {
        status |= PAIR_REMOVED;
    }

    public bool isRemoved() {
        return (status & PAIR_REMOVED) == PAIR_REMOVED;
    }

    public void setFinal() {
        status |= PAIR_FINAL;
    }

    public bool isFinal() {
        return (status & PAIR_FINAL) == PAIR_FINAL;
    }

    public int compareTo(bzPair p) {
        return proxyId1 - p.proxyId1;
    }
}

public class bzBufferedPair {

    int proxyId1;
    int proxyId2;

    ///
    bool opCmp(bzBufferedPair pair2) {

        if (proxyId1 < pair2.proxyId1) {
            return false;
        }

        if (proxyId1 == pair2.proxyId1) {
            if (proxyId2 < pair2.proxyId2)
                return false;
        }

        if (proxyId1 > pair2.proxyId1) {
            return true;
        }

        if (proxyId1 == pair2.proxyId1) {
            if (proxyId2 > pair2.proxyId2)
                return true;
        }

        return false;
    }

    private bool equals(bzBufferedPair other) {
        return proxyId1 == other.proxyId1 && proxyId2 == other.proxyId2;
    }

    private bool minor(bzBufferedPair other) {

        if (proxyId1 < other.proxyId1)
            return true;

        if (proxyId1 == other.proxyId1) {
            return proxyId2 < other.proxyId2;
        }

        return false;
    }

    public int compareTo(bzBufferedPair p) {

        if (minor(p)) {
            return -1;
        } else if (equals(p)) {
            return 0;
        } else {
            return 1;
        }
    }
}
