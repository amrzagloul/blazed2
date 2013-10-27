/*
 *  Copyright (c) 2008 Mason Green http://www.dsource.org/projects/blaze
 *
 *   This software is provided 'as-is', without any express or implied
 *   warranty. In no event will the authors be held liable for any damages
 *   arising from the use of this software.
 *
 *   Permission is granted to anyone to use this software for any purpose,
 *   including commercial applications, and to alter it and redistribute it
 *   freely, subject to the following restrictions:
 *
 *   1. The origin of this software must not be misrepresented; you must not
 *   claim that you wrote the original software. If you use this software
 *   in a product, an acknowledgment in the product documentation would be
 *   appreciated but is not required.
 *
 *   2. Altered source versions must be plainly marked as such, and must not be
 *   misrepresented as being the original software.
 *
 *   3. This notice may not be removed or altered from any source
 *   distribution.
 */

// Documentation modified by Brian Schott (Sir Alaran)

module blaze.collision.pairwise.bzAxis;


import blaze.common.bzMath;

/**
 * An axis
 */
struct bzAxis {
	/// The axis normal
	bzVec2 n;

	/// Distance from Origin
	float d;

	/**
    * Sets the axis
	* Params:
	*     n = the new axis normal
	*     d = the distance from the origin
	*/
	static bzAxis opCall(bzVec2 n, float d)
	{
		bzAxis a;
		a.n = n;
		a.d = d;
		return a;
	}


	/**
	 * Returns: a copy of this axis
	 */
	bzAxis clone()
	{
		return bzAxis(n, d);
	}
}
