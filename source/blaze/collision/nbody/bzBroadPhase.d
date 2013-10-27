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
module blaze.collision.nbody.bzBroadPhase;

/*
This broad phase uses the Sweep and Prune algorithm as described in:
Collision Detection in Interactive 3D Environments by Gino van den Bergen
Also, some ideas, such as using integral values for fast compares comes from
Bullet (http:/www.bulletphysics.com).
*/

extern (C) : void* memmove(void* s1, in void* s2, size_t n);

import blaze.common.bzConstants;
import blaze.common.bzMath;
import blaze.collision.bzCollision;
import blaze.collision.nbody.bzPairManager;
import blaze.collision.nbody.bzPair;

/*
This broad phase uses the Sweep and Prune algorithm as described in:
Collision Detection in Interactive 3D Environments by Gino van den Bergen
Also, some ideas, such as using integral values for fast compares comes from
Bullet (http:/www.bulletphysics.com).
*/

// Notes:
// - we use bound arrays instead of linked lists for cache coherence.
// - we use quantized integral values for fast compares.
// - we use short indices rather than pointers to save memory.
// - we use a stabbing count for fast overlap queries (less than order N).
// - we also use a time stamp on each proxy to speed up the registration of
// overlap query results.
// - where possible, we compare bound indices instead of values to reduce
// cache misses (TODO_ERIN).
// - no broadphase is perfect and neither is this one: it is not great for
// huge worlds (use a multi-SAP instead), it is not great for large objects.

struct bzBoundValues {
    public int[2] lowerValues;
    public int[2] upperValues;
}

public class bzBroadPhase {

    public static immutable int INVALID = int.max;

    public static immutable int NULL_EDGE = int.max;

    public bzPairManager m_pairManager;

    public bzProxy m_proxyPool[];

    int m_freeProxy;

    bzBufferedPair pairBuffer[];

    int m_pairBufferCount;

    public bzBound m_bounds[][];

    int m_queryResults[];

    int m_queryResultCount;

    public bzAABB m_worldAABB;

    public bzVec2 m_quantizationFactor;

    public int m_proxyCount;

    int m_timeStamp;

    public static immutable bool s_validate = true;

    this(bzAABB worldAABB, bzPairCallback callback) {

        // array initialization
        m_proxyPool.length = k_maxProxies;
        pairBuffer.length = k_maxPairs;
        m_bounds.length = 2;
        m_bounds[0].length = 2 * k_maxProxies;
        m_bounds[1].length = 2 * k_maxProxies;

        m_queryResults.length = k_maxProxies;

        for (int i = 0; i < 2 * k_maxProxies; i++) {
            m_bounds[0][i] = new bzBound();
            m_bounds[1][i] = new bzBound();
        }

        for (int i = 0; i < k_maxProxies; i++) {
            pairBuffer[i] = new bzBufferedPair();
        }

        m_pairManager = new bzPairManager();
        m_pairManager.initialize(this, callback);

        assert (worldAABB.isValid());

        m_proxyCount = 0;
        bzVec2 d = worldAABB.upperBound - worldAABB.lowerBound;
        m_quantizationFactor = bzVec2(int.max / d.x, int.max / d.y);
        m_worldAABB = worldAABB;

        for (int i = 0; i < k_maxProxies - 1; ++i) {
            m_proxyPool[i] = new bzProxy();
            m_proxyPool[i].setNext(i + 1);
            m_proxyPool[i].timeStamp = 0;
            m_proxyPool[i].overlapCount = INVALID;
            m_proxyPool[i].userData = null;
        }

        m_proxyPool[k_maxProxies - 1] = new bzProxy();
        m_proxyPool[k_maxProxies - 1].setNext(bzPairManager.NULL_PROXY);
        m_proxyPool[k_maxProxies - 1].timeStamp = 0;
        m_proxyPool[k_maxProxies - 1].overlapCount = INVALID;
        m_proxyPool[k_maxProxies - 1].userData = null;
        m_freeProxy = 0;

        m_timeStamp = 1;
        m_queryResultCount = 0;
    }

    // This one is only used for validation.
    bool testOverlap(bzProxy p1, bzProxy p2) {

        for (int axis = 0; axis < 2; ++axis) {
            bzBound[] bounds = m_bounds[axis];

            assert(p1.lowerBounds[axis] < 2 * m_proxyCount);
            assert(p1.upperBounds[axis] < 2 * m_proxyCount);
            assert(p2.lowerBounds[axis] < 2 * m_proxyCount);
            assert(p2.upperBounds[axis] < 2 * m_proxyCount);

            if (bounds[p1.lowerBounds[axis]].value > bounds[p2.upperBounds[axis]].value)
                return false;

            if (bounds[p1.upperBounds[axis]].value < bounds[p2.lowerBounds[axis]].value)
                return false;
        }

        return true;
    }

    private bool testOverlap(bzBoundValues b, bzProxy p) {

        for (int axis = 0; axis < 2; ++axis) {
            bzBound[] bounds = m_bounds[axis];

            assert(p.lowerBounds[axis] < 2 * m_proxyCount);
            assert(p.upperBounds[axis] < 2 * m_proxyCount);

            if (b.lowerValues[axis] > bounds[p.upperBounds[axis]].value)
                return false;

            if (b.upperValues[axis] < bounds[p.lowerBounds[axis]].value)
                return false;
        }

        return true;
    }

    public bzProxy getProxy(int proxyId) {

        if (proxyId == bzPairManager.NULL_PROXY
                || (m_proxyPool[proxyId].isValid() == false)) {
            return null;
        } else {
            return m_proxyPool[proxyId];
        }
    }

    // Create and destroy proxies. These call Flush first.
    int createProxy(bzAABB aabb, Object userData) {

        assert(m_proxyCount < k_maxProxies);
        assert(m_freeProxy != bzPairManager.NULL_PROXY);

        int proxyId = m_freeProxy;
        bzProxy proxy = m_proxyPool[proxyId];
        m_freeProxy = proxy.getNext();

        proxy.overlapCount = 0;
        proxy.userData = userData;

        int boundCount = 2 * m_proxyCount;

        int lowerValues[2];
        int upperValues[2];
        computeBounds(lowerValues, upperValues, aabb);

        for (int axis = 0; axis < 2; ++axis) {
            bzBound[] bounds = m_bounds[axis];
            int[2] indexes;
            query(indexes, lowerValues[axis], upperValues[axis], bounds, boundCount, axis);
            int lowerIndex = indexes[0];
            int upperIndex = indexes[1];

            //memmove(bounds.ptr + upperIndex + 2, bounds.ptr + upperIndex, (boundCount - upperIndex) * bzBound.sizeof);
            //memmove(bounds.ptr + lowerIndex + 1, bounds.ptr + lowerIndex, (upperIndex - lowerIndex) * bzBound.sizeof);

            bzBound[] tmp;
            tmp.length = boundCount - upperIndex;
            for (int i = 0; i < (boundCount - upperIndex); i++) {
                tmp[i] = bounds[upperIndex + i].clone();
            }
            for (int i = 0; i < (boundCount - upperIndex); i++) {
                bounds[upperIndex + 2 + i] = tmp[i];
            }

            tmp.length = upperIndex - lowerIndex;
            for (int i = 0; i < (upperIndex - lowerIndex); i++) {
                tmp[i] = bounds[lowerIndex + i].clone();
            }
            for (int i = 0; i < (upperIndex - lowerIndex); i++) {
                bounds[lowerIndex + 1 + i] = tmp[i];
            }

            // The upper index has increased because of the lower bound
            // insertion.
            ++upperIndex;

            // Copy in the new bounds.

            assert (bounds[lowerIndex] !is null);
            assert (bounds[upperIndex] !is null);

            bounds[lowerIndex].value = lowerValues[axis];
            bounds[lowerIndex].proxyId = proxyId;
            bounds[upperIndex].value = upperValues[axis];
            bounds[upperIndex].proxyId = proxyId;

            bounds[lowerIndex].stabbingCount = lowerIndex == 0 ? 0
                                               : bounds[lowerIndex - 1].stabbingCount;
            bounds[upperIndex].stabbingCount = bounds[upperIndex - 1].stabbingCount;

            // Adjust the stabbing count between the new bounds.
            for (int index = lowerIndex; index < upperIndex; ++index) {
                ++bounds[index].stabbingCount;
            }

            // Adjust the all the affected bound indices.
            for (int index = lowerIndex; index < boundCount + 2; ++index) {
                bzProxy proxyn = m_proxyPool[bounds[index].proxyId];
                if (bounds[index].isLower()) {
                    proxyn.lowerBounds[axis] = index;
                } else {
                    proxyn.upperBounds[axis] = index;
                }
            }
        }

        ++m_proxyCount;

        assert (m_queryResultCount < k_maxProxies);
        // Create pairs if the bzAABB is in range.
        for (int i = 0; i < m_queryResultCount; ++i) {
            assert(m_queryResults[i] < k_maxProxies);
            assert(m_proxyPool[m_queryResults[i]].isValid());

            m_pairManager.addBufferedPair(proxyId, m_queryResults[i]);
        }

        m_pairManager.commit();

        if (s_validate) {
            validate();
        }

        // Prepare for next query.
        m_queryResultCount = 0;
        incrementTimeStamp();

        return proxyId;
    }

    public void destroyProxy(int proxyId) {

        assert(0 < m_proxyCount && m_proxyCount <= k_maxProxies);
        bzProxy proxy = m_proxyPool[proxyId];
        assert(proxy.isValid());

        int boundCount = 2 * m_proxyCount;

        for (int axis = 0; axis < 2; ++axis) {

            bzBound[] bounds = m_bounds[axis];

            int lowerIndex = proxy.lowerBounds[axis];
            int upperIndex = proxy.upperBounds[axis];
            int lowerValue = bounds[lowerIndex].value;
            int upperValue = bounds[upperIndex].value;

            if((upperIndex - lowerIndex - 1) < 0) {
                throw new Exception("this is a known bug");
            }

            //memmove(bounds.ptr + lowerIndex, bounds.ptr + lowerIndex + 1, (upperIndex - lowerIndex - 1) * bzBound.sizeof);
            //memmove(bounds.ptr + upperIndex-1, bounds.ptr + upperIndex + 1, (boundCount - upperIndex - 1) * bzBound.sizeof);

            bzBound[] tmp;
            tmp.length = upperIndex - lowerIndex - 1;
            for (int i = 0; i < (upperIndex - lowerIndex - 1); i++) {
                tmp[i] = bounds[lowerIndex + 1 + i].clone();
            }
            for (int i = 0; i < (upperIndex - lowerIndex - 1); i++) {
                bounds[lowerIndex + i] = tmp[i];
            }

            tmp.length = boundCount - upperIndex - 1;
            for (int i = 0; i < (boundCount - upperIndex - 1); i++) {
                tmp[i] = bounds[upperIndex + 1 + i].clone();
            }
            for (int i = 0; i < (boundCount - upperIndex - 1); i++) {
                bounds[upperIndex - 1 + i] = tmp[i];
            }

            // Fix bound indices.
            for (int index = lowerIndex; index < boundCount - 2; ++index) {
                bzProxy proxyn = m_proxyPool[bounds[index].proxyId];
                if (bounds[index].isLower()) {
                    proxyn.lowerBounds[axis] = index;
                } else {
                    proxyn.upperBounds[axis] = index;
                }
            }

            // Fix stabbing count.
            for (int index = lowerIndex; index < upperIndex - 1; ++index) {
                --bounds[index].stabbingCount;
            }

            // Query for pairs to be removed. lowerIndex and upperIndex are not
            // needed.
            int[2] ignored;
            query(ignored, lowerValue, upperValue, bounds, boundCount - 2, axis);
        }

        assert (m_queryResultCount < k_maxProxies);

        for (int i = 0; i < m_queryResultCount; ++i) {
            assert(m_proxyPool[m_queryResults[i]].isValid());
            m_pairManager.removeBufferedPair(proxyId, m_queryResults[i]);
        }

        m_pairManager.commit();

        // Prepare for next query.
        m_queryResultCount = 0;
        incrementTimeStamp();

        // Return the proxy to the pool.
        proxy.userData = null;
        proxy.overlapCount = bzBroadPhase.INVALID;
        proxy.lowerBounds[0] = bzBroadPhase.INVALID;
        proxy.lowerBounds[1] = bzBroadPhase.INVALID;
        proxy.upperBounds[0] = bzBroadPhase.INVALID;
        proxy.upperBounds[1] = bzBroadPhase.INVALID;

        // Return the proxy to the pool.
        proxy.setNext(m_freeProxy);
        m_freeProxy = proxyId;
        --m_proxyCount;

        if (s_validate) {
            validate();
        }
    }

    // Call MoveProxy as many times as you like, then when you are done
    // call Flush to finalized the proxy pairs (for your time step).
    void moveProxy(int proxyId, bzAABB aabb) {

        if (proxyId == bzPairManager.NULL_PROXY || k_maxProxies <= proxyId) {
            return;
        }

        if (!aabb.isValid()) {
            throw new Exception("Invalid AABB");
        }

        int boundCount = 2 * m_proxyCount;

        bzProxy proxy = m_proxyPool[proxyId];

        //Get new bound values
        bzBoundValues newValues;
        computeBounds(newValues.lowerValues, newValues.upperValues, aabb);

        //Get old bound values
        bzBoundValues oldValues;
        for (int axis = 0; axis < 2; ++axis) {
            oldValues.lowerValues[axis] = m_bounds[axis][proxy.lowerBounds[axis]].value;
            oldValues.upperValues[axis] = m_bounds[axis][proxy.upperBounds[axis]].value;
        }

        for (int axis = 0; axis < 2; ++axis) {
            bzBound[] bounds = m_bounds[axis];

            int lowerIndex = proxy.lowerBounds[axis];
            int upperIndex = proxy.upperBounds[axis];

            int lowerValue = newValues.lowerValues[axis];
            int upperValue = newValues.upperValues[axis];

            int deltaLower = lowerValue - bounds[lowerIndex].value;
            int deltaUpper = upperValue - bounds[upperIndex].value;

            bounds[lowerIndex].value = lowerValue;
            bounds[upperIndex].value = upperValue;

            //
            // Expanding adds overlaps
            //

            // Should we move the lower bound down?
            if (deltaLower < 0) {
                int index = lowerIndex;
                while (index > 0 && lowerValue < bounds[index - 1].value) {
                    bzBound bound = bounds[index];
                    bzBound prevBound = bounds[index - 1];

                    int prevProxyId = prevBound.proxyId;
                    bzProxy prevProxy = m_proxyPool[prevBound.proxyId];

                    ++prevBound.stabbingCount;

                    if (prevBound.isUpper() == true) {
                        if (testOverlap(newValues, prevProxy)) {
                            m_pairManager.addBufferedPair(proxyId, prevProxyId);
                        }

                        ++prevProxy.upperBounds[axis];
                        ++bound.stabbingCount;
                    } else {
                        ++prevProxy.lowerBounds[axis];
                        --bound.stabbingCount;
                    }

                    --proxy.lowerBounds[axis];
                    bzBound tmp = bound.clone();
                    bound.set(prevBound);
                    prevBound.set(tmp);
                    --index;
                }
            }

            // Should we move the upper bound up?
            if (deltaUpper > 0) {
                int index = upperIndex;
                while (index < boundCount - 1
                        && bounds[index + 1].value <= upperValue) {
                    bzBound bound = bounds[index];
                    bzBound nextBound = bounds[index + 1];
                    int nextProxyId = nextBound.proxyId;
                    bzProxy nextProxy = m_proxyPool[nextProxyId];

                    ++nextBound.stabbingCount;

                    if (nextBound.isLower()) {
                        if (testOverlap(newValues, nextProxy)) {
                            m_pairManager.addBufferedPair(proxyId, nextProxyId);
                        }

                        --nextProxy.lowerBounds[axis];
                        ++bound.stabbingCount;
                    } else {
                        --nextProxy.upperBounds[axis];
                        --bound.stabbingCount;
                    }

                    ++proxy.upperBounds[axis];
                    bzBound tmp = bound.clone();
                    bound.set(nextBound);
                    nextBound.set(tmp);
                    ++index;
                }
            }

            //
            // Shrinking removes overlaps
            //

            // Should we move the lower bound up?
            if (deltaLower > 0) {
                int index = lowerIndex;
                while (index < boundCount - 1
                        && bounds[index + 1].value <= lowerValue) {
                    bzBound bound = bounds[index];
                    bzBound nextBound = bounds[index + 1];

                    int nextProxyId = nextBound.proxyId;
                    bzProxy nextProxy = m_proxyPool[nextProxyId];

                    --nextBound.stabbingCount;

                    if (nextBound.isUpper()) {
                        if (testOverlap(oldValues,nextProxy)) {
                            m_pairManager.removeBufferedPair(proxyId, nextProxyId);
                        }

                        --nextProxy.upperBounds[axis];
                        --bound.stabbingCount;
                    } else {
                        --nextProxy.lowerBounds[axis];
                        ++bound.stabbingCount;
                    }

                    ++proxy.lowerBounds[axis];
                    bzBound tmp = bound.clone();
                    bound.set(nextBound);
                    nextBound.set(tmp);
                    ++index;
                }
            }

            // Should we move the upper bound down?
            if (deltaUpper < 0) {
                int index = upperIndex;
                while (index > 0 && upperValue < bounds[index - 1].value) {
                    bzBound bound = bounds[index];
                    bzBound prevBound = bounds[index - 1];

                    int prevProxyId = prevBound.proxyId;
                    bzProxy prevProxy = m_proxyPool[prevProxyId];

                    --prevBound.stabbingCount;

                    if (prevBound.isLower()) {
                        if (testOverlap(oldValues, prevProxy)) {
                            m_pairManager.removeBufferedPair(proxyId, prevProxyId);
                        }

                        ++prevProxy.lowerBounds[axis];
                        --bound.stabbingCount;
                    } else {
                        ++prevProxy.upperBounds[axis];
                        ++bound.stabbingCount;
                    }

                    --proxy.upperBounds[axis];
                    bzBound tmp = bound.clone();
                    bound.set(prevBound);
                    prevBound.set(tmp);
                    --index;
                }
            }
        }

        if (s_validate) {
            validate();
        }
    }

    public void commit() {

        m_pairManager.commit();
    }

    // Query an bzAABB for overlapping proxies, returns the user data and
    // the count, up to the supplied maximum count.
    public Object[] query(bzAABB aabb, int maxCount) {

        int lowerValues[2];
        int upperValues[2];
        computeBounds(lowerValues, upperValues, aabb);

        int indexes[2];

        query(indexes, lowerValues[0], upperValues[0], m_bounds[0], 2 * m_proxyCount, 0);
        query(indexes, lowerValues[1], upperValues[1], m_bounds[1], 2 * m_proxyCount, 1);

        assert (m_queryResultCount < k_maxProxies);

        Object[] results;
        results.length = maxCount;

        int count = 0;
        for (int i = 0; i < m_queryResultCount && count < maxCount; ++i, ++count) {
            assert (m_queryResults[i] < k_maxProxies);
            bzProxy proxy = m_proxyPool[m_queryResults[i]];
            proxy.isValid();
            results[i] = proxy.userData;
        }

        // Prepare for next query.
        m_queryResultCount = 0;
        incrementTimeStamp();

		if(count == 0) return null;

        return results;
    }

    public void validate() {

        for (int axis = 0; axis < 2; ++axis) {
            bzBound[] bounds = m_bounds[axis];

            int boundCount = 2 * m_proxyCount;
            int stabbingCount = 0;

            for (int i = 0; i < boundCount; ++i) {
                bzBound bound = bounds[i];
                assert(i == 0 || bounds[i-1].value <= bound.value);
                assert(bound.proxyId != bzPairManager.NULL_PROXY);
                assert(m_proxyPool[bound.proxyId].isValid());


                if (bound.isLower() == true) {
                    assert (m_proxyPool[bound.proxyId].lowerBounds[axis] == i);
                    ++stabbingCount;
                } else {
                    assert (m_proxyPool[bound.proxyId].upperBounds[axis] == i);
                    --stabbingCount;
                }

                assert (bound.stabbingCount == stabbingCount);
            }
        }

    }


    private void computeBounds(int[] lowerValues, int[] upperValues, bzAABB aabb) {

        assert(aabb.upperBound.x >= aabb.lowerBound.x);
        assert(aabb.upperBound.y >= aabb.lowerBound.y);

        bzVec2 minVertex = bzClamp(aabb.lowerBound, m_worldAABB.lowerBound, m_worldAABB.upperBound);
        bzVec2 maxVertex = bzClamp(aabb.upperBound, m_worldAABB.lowerBound, m_worldAABB.upperBound);

        // Bump lower bounds downs and upper bounds up. This ensures correct
        // sorting of
        // lower/upper bounds that would have equal values.
        // TODO_ERIN implement fast float to int conversion.
        lowerValues[0] = cast(int)abs(cast(int) (m_quantizationFactor.x * (minVertex.x - m_worldAABB.lowerBound.x)) & (int.max - 1));
        upperValues[0] = cast(int)abs(cast(int) (m_quantizationFactor.x * (maxVertex.x - m_worldAABB.lowerBound.x)) | 1);
        lowerValues[1] = cast(int)abs(cast(int) (m_quantizationFactor.y * (minVertex.y - m_worldAABB.lowerBound.y)) & (int.max - 1));
        upperValues[1] = cast(int)abs(cast(int) (m_quantizationFactor.y * (maxVertex.y - m_worldAABB.lowerBound.y)) | 1);
   }


    /**
     * @param results
     *            out variable
     */
    private void query(int[] results, int lowerValue, int upperValue,
                       bzBound[] bounds, int boundCount, int axis) {

        int lowerQuery = binarySearch(bounds, boundCount, lowerValue);
        int upperQuery = binarySearch(bounds, boundCount, upperValue);

        // Easy case: lowerQuery <= lowerIndex(i) < upperQuery
        // Solution: search query range for min bounds.
        for (int i = lowerQuery; i < upperQuery; ++i) {
            if (bounds[i].isLower()) {
                incrementOverlapCount(bounds[i].proxyId);
            }
        }
        // Hard case: lowerIndex(i) < lowerQuery < upperIndex(i)
        // Solution: use the stabbing count to search down the bound array.
        if (lowerQuery > 0) {
            int i = lowerQuery - 1;
            int s = bounds[i].stabbingCount;
            // Find the s overlaps.
            while (s != 0) {
                assert (i >= 0);
                if (bounds[i].isLower()) {
                    bzProxy proxy = m_proxyPool[bounds[i].proxyId];
                    if (lowerQuery <= proxy.upperBounds[axis]) {
                        incrementOverlapCount(bounds[i].proxyId);
                        --s;
                    }
                }
                --i;
            }
        }

        results[0] = lowerQuery;
        results[1] = upperQuery;
    }

    private void incrementOverlapCount(int proxyId) {

        bzProxy proxy = m_proxyPool[proxyId];
        if (proxy.timeStamp < m_timeStamp) {
            proxy.timeStamp = m_timeStamp;
            proxy.overlapCount = 1;
        } else {
            proxy.overlapCount = 2;
            assert (m_queryResultCount < k_maxProxies);
            m_queryResults[m_queryResultCount] = proxyId;
            ++m_queryResultCount;
        }
    }

    private void incrementTimeStamp() {

        if (m_timeStamp == int.max) {
            for (int i = 0; i < k_maxProxies; ++i) {
                m_proxyPool[i].timeStamp = 0;
            }
            m_timeStamp = 1;
        } else {
            ++m_timeStamp;
        }
    }

    static int binarySearch(bzBound[] bounds, int count, int value) {

        int low = 0;
        int high = count - 1;
        while (low <= high) {
            int mid = (low + high) >> 1;
            if (bounds[mid].value > value) {
                high = mid - 1;
            } else if (bounds[mid].value < value) {
                low = mid + 1;
            } else {
                return mid;
            }
        }

        return low;

    }

    public bool inRange(bzAABB aabb) {
       bzVec2 d = bzMax(aabb.lowerBound - m_worldAABB.upperBound, m_worldAABB.lowerBound - aabb.upperBound);
       return max(d.x, d.y) < 0.0f;
    }
}

public class bzBound {

    int value;
    int proxyId;
    int stabbingCount;

    this() {
        value = 0;
        proxyId = 0;
        stabbingCount = 0;
    }

    this(bzBound b) {
        value = b.value;
        proxyId = b.proxyId;
        stabbingCount = b.stabbingCount;
    }

    public void set(bzBound b) {
        value = b.value;
        proxyId = b.proxyId;
        stabbingCount = b.stabbingCount;
    }

    bool isLower() {
        return (value & 1) == 0;
    }

    bool isUpper() {
        return (value & 1) == 1;
    }

    bzBound clone() {
        bzBound clone = new bzBound;
        clone.proxyId = proxyId;
        clone.value = value;
        clone.stabbingCount = stabbingCount;
        return clone;
    }
}

public class bzProxy {

    int lowerBounds[2];
    int upperBounds[2];
    int overlapCount;
    int timeStamp;
    int categoryBits;
    int maskBits;
    int groupIndex;

    Object userData;

    this() {
    }

    int getNext() {
        return lowerBounds[0];
    }

    void setNext(int next) {
        lowerBounds[0] = next;
    }

    public bool isValid() {
        return overlapCount != bzBroadPhase.INVALID;
    }
}
