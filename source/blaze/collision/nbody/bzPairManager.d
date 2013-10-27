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
module blaze.collision.nbody.bzPairManager;

import blaze.common.bzConstants;
import blaze.common.bzMath;
import blaze.collision.nbody.bzBroadPhase;
import blaze.collision.nbody.bzPair;


const bz_maxPairs = int.max;
const bz_nullProxy = int.max;

public abstract class bzPairCallback {
    // This should return the new pair user data. It is okay if the
    // user data is null.
    public abstract Object pairAdded(Object proxyUserData1, Object proxyUserData2);

    // This should free the pair's user data. In extreme circumstances, it is
    // possible
    // this will be called with null pairUserData because the pair never
    // existed.
    public abstract void pairRemoved(Object proxyUserData1, Object proxyUserData2, Object pairUserData);
}

public class bzPairManager {

    public static const int NULL_PAIR = int.max;

    public static const int NULL_PROXY = int.max;

	// must be a power of two
    public static const int TABLE_CAPACITY = k_maxPairs;
    static const int TABLE_MASK = TABLE_CAPACITY - 1;

    public bzPair m_pairs[];

    public int m_pairCount;

    public int m_hashTable[];

    //int m_next[];

    public bzBroadPhase m_broadPhase;

    public bzPairCallback m_callback;

    public int m_freePair;

    public bzBufferedPair[] m_pairBuffer;
    public int m_pairBufferCount;

    this() {

        m_pairs = new bzPair[k_maxPairs];
        m_hashTable.length = TABLE_CAPACITY;
        m_pairBuffer = new bzBufferedPair[k_maxPairs];

        assert (isPowerOfTwo(TABLE_CAPACITY));
        assert (TABLE_CAPACITY >= k_maxPairs);

        for (int i = 0; i < TABLE_CAPACITY; ++i) {
            m_hashTable[i] = NULL_PAIR;
        }
        m_freePair = 0;
        for (int i = 0; i < k_maxPairs; ++i) {
            //m_next[i] = NULL_PAIR;
            m_pairs[i] = new bzPair();
            m_pairs[i].proxyId1 = NULL_PROXY;
            m_pairs[i].proxyId2 = NULL_PROXY;
            m_pairs[i].userData = null;
            m_pairs[i].status = 0;
            m_pairs[i].next = i+1;

            m_pairBuffer[i] = new bzBufferedPair();
        }
        m_pairs[k_maxPairs-1].next = NULL_PAIR;
        m_pairCount = 0;
        m_pairBufferCount = 0;
    }

    public void initialize(bzBroadPhase broadPhase, bzPairCallback callback) {

        m_broadPhase = broadPhase;
        m_callback = callback;
    }

    // Add a pair and return the new pair. If the pair already exists,
    // no new pair is created and the old one is returned.
    public bzPair addPair(int proxyId1, int proxyId2) {

        if (proxyId1 > proxyId2) {
            // integer primitive swap
            proxyId1 += proxyId2;
            proxyId2 = proxyId1 - proxyId2;
            proxyId1 -= proxyId2;
        }

        int hash = hash(proxyId1, proxyId2) & TABLE_MASK;

        bzPair pair = find(proxyId1, proxyId2, hash);
        if (pair !is null) {
            return pair;
        }

		assert(m_pairCount < k_maxPairs && m_freePair != NULL_PAIR);

        int pairIndex = m_freePair;

        pair = m_pairs[pairIndex];
        m_freePair = pair.next;

        pair.proxyId1 = proxyId1;
        pair.proxyId2 = proxyId2;
        pair.status = 0;
        pair.userData = null;
        pair.next = m_hashTable[hash];

        m_hashTable[hash] = pairIndex;

        ++m_pairCount;

        return pair;
    }

    // Remove a pair, return the pair's userData.
    public Object removePair(int proxyId1, int proxyId2) {

        assert(m_pairCount > 0);

        if (proxyId1 > proxyId2) {
            // integer primitive swap (safe for small ints)
            proxyId1 += proxyId2;
            proxyId2 = proxyId1 - proxyId2;
            proxyId1 -= proxyId2;
        }

        int hash = hash(proxyId1, proxyId2) & TABLE_MASK;
        //int* node = &m_hashTable[hash];
        int derefnode = m_hashTable[hash];
        bool isHash = true;
        int pderefnode = 0;
        while (derefnode != NULL_PAIR) {
            if (equals(m_pairs[derefnode], proxyId1, proxyId2)) {
                //int index = *node;
                int index = derefnode;
                //*node = m_pairs[*node].next;
                if (isHash) {
                    m_hashTable[hash] = m_pairs[m_hashTable[hash]].next;
                } else {
                    m_pairs[pderefnode].next = m_pairs[derefnode].next;
                }

                bzPair pair = m_pairs[index];
                Object userData = pair.userData;

                // Scrub
                pair.next = m_freePair;
                pair.proxyId1 = NULL_PROXY;
                pair.proxyId2 = NULL_PROXY;
                pair.userData = null;
                pair.status = 0;

                m_freePair = index;
                --m_pairCount;

                return userData;
            } else {
                //node = &m_pairs[*node].next;
                pderefnode = derefnode;
                derefnode = m_pairs[derefnode].next;
                isHash = false;
            }
        }

        throw new Exception("Attempted to remove a pair that does not exist");
    }

    /*
    As proxies are created and moved, many pairs are created and destroyed. Even worse, the same
    pair may be added and removed multiple times in a single time step of the physics engine. To reduce
    traffic in the pair manager, we try to avoid destroying pairs in the pair manager until the
    end of the physics step. This is done by buffering all the RemovePair requests. AddPair
    requests are processed immediately because we need the hash table entry for quick lookup.

    All user user callbacks are delayed until the buffered pairs are confirmed in Commit.
    This is very important because the user callbacks may be very expensive and client logic
    may be harmed if pairs are added and removed within the same time step.

    Buffer a pair for addition.
    We may add a pair that is not in the pair manager or pair buffer.
    We may add a pair that is already in the pair manager and pair buffer.
    If the added pair is not a new pair, then it must be in the pair buffer (because RemovePair was called).
    */
    public void addBufferedPair(int id1, int id2) {

        assert(id1 != NULL_PROXY && id2 != NULL_PROXY);
        assert(m_pairBufferCount < k_maxPairs);

        bzPair pair = addPair(id1, id2);

        // If this pair is not in the pair buffer ...
        if (pair.isBuffered() == false) {
            // This must be a newly added pair.
            assert(pair.isFinal() == false);

            // Add it to the pair buffer.
            pair.setBuffered();
            m_pairBuffer[m_pairBufferCount].proxyId1 = pair.proxyId1;
            m_pairBuffer[m_pairBufferCount].proxyId2 = pair.proxyId2;
            ++m_pairBufferCount;

            assert(m_pairBufferCount <= m_pairCount);
        }

        // Confirm this pair for the subsequent call to Commit.
        pair.clearRemoved();

        if (bzBroadPhase.s_validate){
            validateBuffer();
        }
    }

    // Buffer a pair for removal.
    public void removeBufferedPair(int id1, int id2) {

        assert(id1 != NULL_PROXY && id2 != NULL_PROXY);
        assert(m_pairBufferCount < k_maxPairs);

        bzPair pair = find(id1, id2);

        if (pair is null) {
            // The pair never existed. This is legal (due to collision filtering).
            return;
        }

        // If this pair is not in the pair buffer ...
        if (pair.isBuffered() == false) {
            // This must be an old pair.
            assert(pair.isFinal() == true);

            pair.setBuffered();
            m_pairBuffer[m_pairBufferCount].proxyId1 = pair.proxyId1;
            m_pairBuffer[m_pairBufferCount].proxyId2 = pair.proxyId2;
            ++m_pairBufferCount;

            assert(m_pairBufferCount <= m_pairCount);
        }

        pair.setRemoved();

        if (bzBroadPhase.s_validate) {
            validateBuffer();
        }
    }

    public void commit() {

        int removeCount = 0;

        bzProxy[] proxies = m_broadPhase.m_proxyPool[];

        for (int i = 0; i < m_pairBufferCount; ++i) {
            bzPair pair = find(m_pairBuffer[i].proxyId1, m_pairBuffer[i].proxyId2);
            assert(pair.isBuffered());
            pair.clearBuffered();

            assert(pair.proxyId1 < k_maxProxies && pair.proxyId2 < k_maxProxies);

            bzProxy proxy1 = proxies[pair.proxyId1];
            bzProxy proxy2 = proxies[pair.proxyId2];

            assert(proxy1.isValid());
            assert(proxy2.isValid());

            if (pair.isRemoved()) {
                // It is possible a pair was added then removed before a commit. Therefore,
                // we should be careful not to tell the user the pair was removed when the
                // the user didn't receive a matching add.
                if (pair.isFinal() == true) {
                    m_callback.pairRemoved(proxy1.userData, proxy2.userData, pair.userData);
                }

                // Store the ids so we can actually remove the pair below.
                m_pairBuffer[removeCount].proxyId1 = pair.proxyId1;
                m_pairBuffer[removeCount].proxyId2 = pair.proxyId2;
                ++removeCount;
            } else {
                assert(m_broadPhase.testOverlap(proxy1, proxy2) == true);

                if (pair.isFinal() == false) {
                    pair.userData = m_callback.pairAdded(proxy1.userData, proxy2.userData);
                    pair.setFinal();
                }
            }
        }

        for (int i = 0; i < removeCount; ++i) {
            removePair(m_pairBuffer[i].proxyId1, m_pairBuffer[i].proxyId2);
        }

        m_pairBufferCount = 0;

        if (bzBroadPhase.s_validate) {
            validateTable();
        }
    }

    /**
     * Unimplemented - for debugging purposes only in C++ version
     */
    public void validateBuffer() {
    }

    /**
     * For debugging
     */
    public void validateTable() {
    debug {
        for (int i = 0; i < TABLE_CAPACITY; ++i) {
            int index = m_hashTable[i];
            while (index != NULL_PAIR) {
                bzPair pair = m_pairs[index];
                assert(pair.isBuffered() == false);
                assert(pair.isFinal() == true);
                assert(pair.isRemoved() == false);

                assert(pair.proxyId1 != pair.proxyId2);
                assert(pair.proxyId1 < k_maxProxies);
                assert(pair.proxyId2 < k_maxProxies);

                bzProxy proxy1 = m_broadPhase.m_proxyPool[pair.proxyId1];
                bzProxy proxy2 = m_broadPhase.m_proxyPool[pair.proxyId2];

                assert(proxy1.isValid() == true);
                assert(proxy2.isValid() == true);

                assert(m_broadPhase.testOverlap(proxy1, proxy2) == true);

                index = pair.next;
            }
        }
    }
    }

    public bzPair find(int proxyId1, int proxyId2, int hash) {

        int index = m_hashTable[hash];

        while (index != NULL_PAIR
                && equals(m_pairs[index], proxyId1, proxyId2) == false) {
            index = m_pairs[index].next;
        }

        if (index == NULL_PAIR) {
            return null;
        }

        assert (index < k_maxPairs);
        return m_pairs[index];
    }

    public bzPair find(int proxyId1, int proxyId2) {

        if (proxyId1 > proxyId2) {
            int tmp = proxyId1;
            proxyId1 = proxyId2;
            proxyId2 = tmp;
        }

        int hash = hash(proxyId1, proxyId2) & TABLE_MASK;

        return find(proxyId1, proxyId2, hash);
    }


    bool equals(bzPair pair, int proxyId1, int proxyId2) {

        return pair.proxyId1 == proxyId1 && pair.proxyId2 == proxyId2;
    }

    bool equals(bzBufferedPair pair1, bzBufferedPair pair2) {

        return pair1.proxyId1 == pair2.proxyId1 && pair1.proxyId2 == pair2.proxyId2;
    }

    // For sorting.
    bool minor (bzBufferedPair pair1, bzBufferedPair pair2){

        if (pair1.proxyId1 < pair2.proxyId1) {
            return true;
        }

        if (pair1.proxyId1 == pair2.proxyId1) {
            return pair1.proxyId2 < pair2.proxyId2;
        }

        return false;
    }
}
