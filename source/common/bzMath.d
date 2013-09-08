/*
 * Copyright (c) 2008-2009, Mason Green (zzzzrrr)
 * Based on Box2D by Erin Catto, http://www.box2d.org
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

// Documentation comments modified by Brian Schott (SirAlaran)

module blaze.common.bzMath;


version(Tango) {
  public import tango.math.Math;
} else {
  public import std.math;
  alias std.math.fabs abs;
}


/**
 * Params: a = the vector to base the return value off of
 * Returns: a new vector with the x and y components equal to the absolute value
 *     of the corresponding components in a.
 */
bzVec2 bzAbs(bzVec2 a) {
    bzVec2 b;
    b.set(abs(a.x), abs(a.y));
    return b;
}


/**
 * Params: A = the matrix to set the new matrix from
 * Returns: a new matrix with the columns equal to the absolute value of the
 *     corresponding values in A.
 */
bzMat22 bzAbs(bzMat22 A) {
    bzMat22 B;
    B.set(bzAbs(A.col1), bzAbs(A.col2));
    return B;
}

bool isPowerOfTwo(uint x)
{
	bool result = x > 0 && (x & (x - 1)) == 0;
	return result;
}

/// This function is used to ensure that a floating point number is
/// not a NaN or infinity.
bool bzIsValid(float x)
{
    return !isNaN(x) && !isInf(x);
}

bool isInf(float x)
{
    return (x == float.infinity) || (x == -float.infinity);
}

bool isNaN(float x)
{
    return !(x == x);
}

/**
 * Creates a vector such that x = min(a.x, b.x) and y = min(a.y, b.y)
 *
 * Note that this does NOT return the vector with the smallest
 * magnitude. If a's x is smaller and b's y is smaller, it will return a.x and
 * b.y
 * Params:
 *     a = the first of the two vectors
 *     b = the other vector
 * Returns: the new vector
 */
bzVec2 bzMin(bzVec2 a, bzVec2 b)
{
	return bzVec2(min(a.x, b.x), min(a.y, b.y));
}

T max(T)(T a, T b)
{
   return a > b ? a : b;
}

T min(T)(T a, T b) 
{
    return a < b ? a : b;
}

bzVec2 bzMax(bzVec2 a, bzVec2 b)
{
   return bzVec2(max(a.x, b.x), max(a.y, b.y));
}


/**
 * Recursively removes all instances of element in array
 * Params:
 *     array = the array
 *     element = the element to remove
 */
void bzKill(T, U) (ref T[] array, U element) {
    size_t index = 0;
    for (; index < array.length; ++index)
        if (array[index] == element)
            break;

    if (index == array.length)
        return;

    for (; index + 1 < array.length; ++index)
        array[index] = array[index + 1];

    array.length = array.length - 1;
    bzKill(array, element);
}


/**
 * A Two dimensional (2D) vector. Direct modification of the x and y coordinates
 * is believed to be safe.
 */
struct bzVec2 {

    /** X axis coordinate */
    float x = 0.0f;

    /** Y bzAxis coordinate */
    float y = 0.0f;

    /**
     * A zero vector
     */
    static const bzVec2 zeroVect = { 0, 0 };

    /**
     * Constructor
     * Params:
     *     ax = the x-component for the new vector
     *     ay = the y-component for the new vector
     * Returns: A new vector
     */
    static bzVec2 opCall(float ax, float ay) {
        bzVec2 u;
        u.x = ax;
        u.y = ay;
        return u;
    }

    /** Set the vector
     * Params:
     *     px = the new x-component
     *     py = the new y-component
     */
    void set(float px, float py) {
        x = px;
        y = py;
    }

    /**
     * Set the vector components to zero
     */
    void zero() {
        x = 0.0f;
        y = 0.0f;
    }

    /**
     * Params: v = the other vector
     * Returns: the dot product of this and v
     */
    float bzDot(bzVec2 v) {
        return x * v.x + y * v.y;
    }

    /**
     * Returns: the cross product
     */
    float bzCross(bzVec2 v) {
        return x * v.y - y * v.x;
    }

    /** Scalar cross product */
    bzVec2 bzCross(float s) {
        return bzVec2(-s * y, s * x);
    }

    /**
     * Scalar addition
     * This does not add V to the length, but to the x and y components
     * Params: V = the degree to which the vector components are increased
     * Returns: A new vector
     */
    bzVec2 opAdd(float V) {
        return bzVec2(x + V, y + V);
    }

    /**
     * Scalar subtraction
     * See_Also: opAdd
     */
    bzVec2 opSub(float n) {
        return bzVec2(x - n, y - n);
    }

    /**
     * Scalar addition
     * This does not add V to the length, but to the x and y components
     * Params: V = the degree to which the vector components are increased
     * Returns: *this
     */
    bzVec2 opAddAssign(float V) {
        x += V;
        y += V;
        return this;
    }

    /**
     * Scalar subtraction
     * See_Also: opAddAssign
     */
    bzVec2 opSubAssign(float V) {
        x -= V;
        y -= V;
        return this;
    }

    /**
     * Scalar multiplication
     * The x and y components are both multiplied by s
     * Params: s = the number to multiply by
     * Returns: *this
     */
    bzVec2 opMulAssign(float s) {
        x *= s;
        y *= s;
        return this;
    }

    /**
     * Scalar multiplication
     * The x and y components are both multiplied by s
     * Params: s = the number to multiply by
     * Returns: A new vector
     */
    bzVec2 opMul(float s) {
        return bzVec2(x * s, y * s);
    }

    /**
     * 2x2 matrix multiplication
     * Params: a = the matrix to multiply by
     * Returns: A new vector that has been multiplied by the matrix
     */
    bzVec2 opMul(bzMat22 a) {
        return bzVec2(a.col1.x * x + a.col2.x * y, a.col1.y * x + a.col2.y * y);
    }

    /**
     * Scalar division
     * See_Also: opMulAssign
     */
    bzVec2 opDivAssign(float s) {
        x /= s;
        y /= s;
        return this;
    }

    /**
     * Scalar division
     * See_Also: opMul
     */
    bzVec2 opDiv(float s) {
        return bzVec2(x / s, y / s);
    }

    /**
     * Vector addition
     * Params other = the other vector
     * Returns: *this
     */
    bzVec2 opAddAssign(bzVec2 Other) {
        x += Other.x;
        y += Other.y;
        return this;
    }

    /**
     * Vector addition
     * Params other = the other vector
     * Returns: A new vector holding the result of the addition
     */
    bzVec2 opAdd(bzVec2 V) {
        return bzVec2(x + V.x, y + V.y);
    }

    /**
     * Vector subtraction
     * See_Also: opAddAssign
     */
    bzVec2 opSubAssign(bzVec2 Other) {
        x -= Other.x;
        y -= Other.y;
        return this;
    }

    /**
     * Vector subtraction
     * See_Also: opAdd
     */
    bzVec2 opSub(bzVec2 V) {
        return bzVec2(x - V.x, y - V.y);
    }

    /**
     * Negation
     * Returns: a new vector 180 degrees from this one
     */
    bzVec2 opNeg() {
        return bzVec2(-x, -y);
    }

    /**
     * Returns: the length of the vector
     */
    float magnitude() {
        float mag = sqrt(x * x + y * y);
        if (!(mag <>= 0)) mag = float.epsilon;
        return mag;
    }

    /**
     * For performance, use this instead of length() (if possible).
     * Returns: the length of the vector squared.
     */
    float lengthSquared() @property {
        return x * x + y * y;
    }

    /**
     * Returns: the length of the vector.
     * Use lengthSquared when possible
     */
    float length() @property {
        return sqrt(x * x + y * y);
    }

    /**
     * Converts the vector into a unit vector.
     * Returns: the old length.
     */
    float normalize() @property
    {
        float length = length();
        if (length < float.epsilon)
        {
            return 0.0f;
        }
        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;

        return length;
    }

    /**
     * Calculates the distance between two points (stored as vectors)
     * Params: v = the other vector
     * Returns: the distance between the two
     */
    float distance(bzVec2 v) @property {
        bzVec2 delta = bzVec2(x, y) - v;
        return delta.magnitude();
    }

    /**
     * Returns: a bzVec2 pointed 90 degrees to the left of this one (the
     * magnitudes of the two will be equal)
     * Todo: Remove this. It's the same as rotateLeft90
     */
    bzVec2 perp() {
        return bzVec2(-y, x);
    }

    /**
     * Restricts the magnitude of this vector to a specific length
     * Params: max = the maximum length that the vector is allowed to have
     * Returns: a copy of this vector after the length restriction is applied
     */
    bzVec2 clampMax(float max) {
        float l = magnitude();

        if (l > max)
            this *= (max / l);
        return bzVec2(x, y);
    }

    /**
     * I have no idea what this function is for
     */
    bzVec2 interpEquals(float blend, bzVec2 v) {
        x += blend * (v.x - x);
        y += blend * (v.y - y);
        return bzVec2(x, y);
    }

    /**
     * Params: v the other vector
     * Returns: a bzVec2 that represents the projection of this on to v. The
     *     resulting vector will have a direction equal to v.
     */
    bzVec2 projectOnto(bzVec2 v) {
        float dp = bzVec2(x, y).bzDot(v);
        float f = dp / (v.x * v.x + v.y * v.y);

        return bzVec2(f * v.x, f * v.y);
    }

    /**
     * Calculates the angle between v and this
     * Params: v = the other vector
     * Returns: the angle between v and this
     */
    float angle(bzVec2 v) {
        return atan2(bzVec2(x, y).bzCross(v), bzVec2(x, y).bzDot(v));
    }

    /**
     * Creates a unit vector pointed in a specific direction
     * Params: a = the angle that the new vector will point in
     * Returns: a new vector
     */
    static bzVec2 forAngle(float a) {
        return bzVec2(cos(a), sin(a));
    }

    /*
     * Sets this to a unit vector in a specified direction
     * Params: a = the new direction for the vector
     */
    void forAngleEquals(float a) {
        this.x = cos(a);
        this.y = sin(a);
    }

    /**
     * I'm too tired to tell what this does. TODO: fix this docstring
     */
    bzVec2 rotate(bzVec2 v) {
        return bzVec2(x * v.x - y * v.y, x * v.y + y * v.x);
    }

    /**
     * Rotates the vector by a given angle
     * Params: angle = the angle by which to rotate the vector
     * Returns: A new vector containing the result of the rotation
     */
    bzVec2 rotate(float angle) {
        float cos = cos(angle);
        float sin = sin(angle);

        return bzVec2((cos * x) - (sin * y), (cos * y) + (sin * x));
    }

    /**
     * Rotates the vector by a given angle about a point
     * Params:
     *     angle = the angle by which to rotate the vector
     *     point = the point to rotate the vector about
     * Returns: a new vector containing the result of the rotation
     */
    bzVec2 rotateAbout(float angle, bzVec2 point) {
        bzVec2 d = (bzVec2(x, y) - point).rotate(angle);

        x = point.x + d.x;
        y = point.y + d.y;
        return bzVec2(x, y);
    }

    /**
     * Rotates the vector by a given angle and applies the result to *this
     * Params: angle = the angle by which to rotate the vector
     * Returns: A copy of *this
     */
    bzVec2 rotateEquals(float angle) {
        float cos = cos(angle);
        float sin = sin(angle);
        float rx = (cos * x) - (sin * y);
        float ry = (cos * y) + (sin * x);

        x = rx;
        y = ry;
        return bzVec2(x, y);
    }

    /**
     * Creates an array of vectors
     * Params len = the size of the array
     * Returns: an array of vectors of the specified length
     */
    bzVec2[] createVectorArray(int len) {
        bzVec2[] vectorArray;
        vectorArray.length = len;
        return vectorArray;
    }

    /**
     * Returns: true if the vector has zero for its x and y components
     * Bugs: possibly convert this to use !<>
     */
    bool equalsZero() {
        return x == 0 && y == 0;
    }

    /**
     * Returns: A new vector equal to this one but rotated left 90 degrees
     * See_Also: perp
     */
    bzVec2 rotateLeft90() @property {
        return bzVec2(-y, x);
    }

    /**
     * Returns: A new vector equal to this one but rotated right 90 degrees
     */
    bzVec2 rotateRight90() @property {
        return bzVec2(y, -x);
    }

    	/// Does this vector contain finite coordinates?
	bool isValid() {
		return bzIsValid(x) && bzIsValid(y);
	}
}

///
struct bzMat22 {
    ///
    static bzMat22 opCall(bzVec2 c1, bzVec2 c2) {
        bzMat22 m;
        m.col1 = c1;
        m.col2 = c2;
        return m;
    }

    /**
     * For the results of this function to be valid, this must be a 2x2 rotation
     * matrix.
     * Params: angle = the angle to set the matrix to
     */
    static bzMat22 opCall(float angle) {
        bzMat22 m;
        float c = cos(angle), s = sin(angle);
        m.col1.x = c;
        m.col2.x = -s;
        m.col1.y = s;
        m.col2.y = c;
        return m;
    }

    /**
     * Construct this matrix using scalars.
     * Params:
     *     a11 = the value for row 1, column 1
     *     a12 = the value for row 1, column 2
     *     a21 = the value for row 2, column 1
     *     a22 = the value for row 2, column 2
     * Returns: A new matrix
     */
    static bzMat22 opCall(float a11, float a12, float a21, float a22) {
        bzMat22 u;
        u.col1.x = a11;
        u.col1.y = a21;
        u.col2.x = a12;
        u.col2.y = a22;
        return u;
    }

    /**
     * Sets the matrix
     * Params:
     *     c1 = the values for column 1
     *     c2 = the values for column 2
     */
    void set(bzVec2 c1, bzVec2 c2) {
        col1 = c1;
        col2 = c2;
    }

    /**
     * For the results of this function to be valid, this must be a 2x2 rotation
     * matrix.
     * Params: angle = the angle to set the matrix to
     */
    void set(float angle) {
        float c = cos(angle), s = sin(angle);
        col1.x = c;
        col2.x = -s;
        col1.y = s;
        col2.y = c;
    }

    /**
     * Sets this equal to the identity matrix
     */
    void setIdentity() {
        col1.x = 1.0f;
        col2.x = 0.0f;
        col1.y = 0.0f;
        col2.y = 1.0f;
    }

    /**
     * Sets all values in the matrix to zero
     */
    void zero() {
        col1.x = 0.0f;
        col2.x = 0.0f;
        col1.y = 0.0f;
        col2.y = 0.0f;
    }

    /**
     * Inverts the matrix
     */
    bzMat22 invert() {
        float a = col1.x, b = col2.x, c = col1.y, d = col2.y;
        bzMat22 B;
        float det = a * d - b * c;
        assert(det != 0.0f);
        det = 1.0f / det;
        B.col1.x =  det * d;
        B.col2.x = -det * b;
        B.col1.y = -det * c;
        B.col2.y =  det * a;
        return B;
    }

    /**
     * Compute the inverse of this matrix, such that inv(A) * A = identity.
     */
    bzMat22 inverse() {
        float a = col1.x, b = col2.x, c = col1.y, d = col2.y;
        bzMat22 B;
        float det = a * d - b * c;
        assert(det != 0.0f);
        det = 1.0f / det;
        B.col1.x =  det * d;
        B.col2.x = -det * b;
        B.col1.y = -det * c;
        B.col2.y =  det * a;
        return B;
    }

    /**
     * Solve A * x = b, where b is a column vector. This is more efficient
     * than computing the inverse in one-shot cases.
     * Params: b = the column vector
     * Returns: x
     */
    bzVec2 solve(bzVec2 b) {
        float a11 = col1.x, a12 = col2.x, a21 = col1.y, a22 = col2.y;
        float det = a11 * a22 - a12 * a21;
        assert(det != 0.0f);
        det = 1.0f / det;
        bzVec2 x;
        x.x = det * (a22 * b.x - a12 * b.y);
        x.y = det * (a11 * b.y - a21 * b.x);
        return x;
    }

    /**
     * Adds each element of the matrix to each corresponding element of the
     * other matrix
     * Params: B = the other matrix
     * Returns: A new matrix containing the result of the addition
     */
    bzMat22 opAdd(bzMat22 B) {
        bzMat22 C;
        C.set(col1 + B.col1, col2 + B.col2);
        return C;
    }

    /**
     * The columns
     */
    bzVec2 col1, col2;
}

/*
 * A transform contains translation and rotation. It is used to represent
 * the position and orientation of rigid frames.
 */
struct bzXForm {

    /**
     * Initialize using a position vector and a rotation matrix.
     * Params:
     *     position = the initial position
     *     R = the initial rotation
     * Returns: a new transform
     */
    static bzXForm opCall(bzVec2 position, bzMat22 R) {
        bzXForm x;
        x.position = position;
        x.R = R;
        return x;
    }

    /**
     * Set this to the identity transform.(Does not move or rotate anything
     */
    void setIdentity() {
        position.zero();
        R.setIdentity();
    }

    /// The position
    bzVec2 position;

    /// The rotation
    bzMat22 R;
}

/**
 * This describes the motion of a body/shape for TOI computation.
 * Shapes are defined with respect to the body origin, which may
 * not coincide with the center of mass. However, to support dynamics
 * we must interpolate the center of mass position.
 */
struct bzSweep
{
    /**
     * Get the interpolated transform at a specific time.
     * Params:
     *     xf = the transform to interpolate from
     *     t = the normalized time in [0,1].
     */
    void xForm(ref bzXForm xf, float t) {
        // center = p + R * localCenter
        if (1.0f - t0 > float.epsilon) {
            float alpha = (t - t0) / (1.0f - t0);
            xf.position = (1.0f - alpha) * c0 + alpha * c;
            float angle = (1.0f - alpha) * a0 + alpha * a;
            xf.R.set(angle);
        } else {
            xf.position = c;
            xf.R.set(a);
        }

        // Shift to origin
        xf.position -= bzMul(xf.R, localCenter);
    }

    /**
     * Advance the sweep forward, yielding a new initial state.
     * Params: t = the new initial time.
     */
    void advance(float t) {
        if (t0 < t && 1.0f - t0 > float.epsilon) {
            float alpha = (t - t0) / (1.0f - t0);
            c0 = (1.0f - alpha) * c0 + alpha * c;
            a0 = (1.0f - alpha) * a0 + alpha * a;
            t0 = t;
        }
    }

    /// local center of mass position
    bzVec2 localCenter;
    /// center world positions
    bzVec2 c0;
    /// center world positions
    bzVec2 c;
    /// world angles
    float a0 = 0.0f;
    /// world angles
    float a = 0.0f;
    /// time interval = [t0,1], where t0 is in [0,1]
    float t0 = 0.0f;
}

/**
 * Vector dot product
 * Params:
 *     a = the first vector
 *     b = the second vector
 * Returns: the dot product of a and b
 */
float bzDot(bzVec2 a, bzVec2 b) {
    return a.x * b.x + a.y * b.y;
}

/**
 * Vector cross product
 * Params:
 *     a = the first vector
 *     b = the second vector
 * Returns: the cross product of a and b
 */
float bzCross(bzVec2 a, bzVec2 b) {
    return a.x * b.y - a.y * b.x;
}

/// TODO
bzVec2 bzCross(bzVec2 a, float s) {
    bzVec2 v;
    v.set(s * a.y, -s * a.x);
    return v;
}

/// TODO
bzVec2 bzCross(float s, bzVec2 a) {
    bzVec2 v;
    v.set(-s * a.y, s * a.x);
    return v;
}

/// TODO
bzVec2 bzMul(bzMat22 A, bzVec2 v) {
    bzVec2 u;
    u.set(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
    return u;
}

/// TODO
bzVec2 bzMulT(bzMat22 A, bzVec2 v) {
    bzVec2 u;
    u.set(bzDot(v, A.col1), bzDot(v, A.col2));
    return u;
}

/// TODO
bzVec2 bzMulT(bzXForm T, bzVec2 v)
{
    return bzMulT(T.R, v - T.position);
}

/// A * B
bzMat22 bzMul(bzMat22 A, bzMat22 B) {
    bzMat22 C;
    C.set(bzMul(A, B.col1), bzMul(A, B.col2));
    return C;
}

/**
 * Multiply a 3x3 matrix by a 3-component vector.
 * Params:
 *     A = the matrix
 *     v = the vector
 * Returns: A new matrix containing the result
 */
bzVec3 bzMul(bzMat33 A, bzVec3 v) {
    bzVec3 u = (v.x * A.col1) + (v.y * A.col2) + (v.z * A.col3);
    return u;
}

/**
 * Multiply a transformation by a vector
 * Params:
 *     T = the transformation
 *     v = the vector
 * Returns: the resulting vector
 */
bzVec2 bzMul(bzXForm T, bzVec2 v) {
    return (T.position + bzMul(T.R, v));
}

/// A^T * B
bzMat22 bzMulT(bzMat22 A, bzMat22 B) {
    bzVec2 c1;
    c1.set(bzDot(A.col1, B.col1), bzDot(A.col2, B.col1));
    bzVec2 c2;
    c2.set(bzDot(A.col1, B.col2), bzDot(A.col2, B.col2));
    bzMat22 C;
    C.set(c1, c2);
    return C;
}

/// Perform the bzDot product on two vectors.
float bzDot(bzVec3 a, bzVec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Perform the cross product on two vectors.
bzVec3 bzCross(bzVec3 a, bzVec3 b) {
    return bzVec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

/**
 * Constrains a value to a range
 * Params:
 *     a = the vaule
 *     b = the lowest allowed value
 *     c = the highest allowed value
 */
float bzClamp (float a, float low, float high) {
    return max(low, min(a, high));
}

/**
 * Constrains a vector to a given range of x and y coordinates
 * See_also bzMax, bzMin
 * Params:
 *     a = the vector
 *     b = the lowest allowed values
 *     c = the highest allowed values
 */
bzVec2 bzClamp(bzVec2 a, bzVec2 low, bzVec2 high) {
    return bzMax(low, bzMin(a, high));
}

/**
 * Simple swap
 * Params:
 *     a = the first item
 *     b = the second item
 */
void bzSwap(T)(ref T a, ref T b) {
    T tmp = a;
    a = b;
    b = tmp;
}

/**
 * 2d Triangle
 */
struct bzTri2 {
    bzVec2 a;
    bzVec2 b;
    bzVec2 c;
    bzVec2 cm;

    float area = 0.0f;

    /**
     * Constructor
     * Params:
     *     a = the coordinates of the first point
     *     b = the coordinates of the second point
     *     c = the coordinates of the third point
     * Returns: A new triangle
     */
    static bzTri2 opCal(bzVec2 a, bzVec2 b, bzVec2 c) {
        bzTri2 u;
        u.a = a;
        u.b = b;
        u.c = c;
        u.area = ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) / 2;
        u.cm = bzVec2((a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3);
        return u;
    }
}


/**
 * A 2D column vector with 3 elements.
 */
struct bzVec3 {

    float x = 0;
    float y = 0;
    float z = 0;

    /**
     * Construct using coordinates.
     * Params:
     *     x = the x-coordinate
     *     y = the y-coordinate
     *     z = the z-coordinate
     * Returns: a new vector
     */
    static bzVec3 opCall(float x, float y, float z) {
        bzVec3 u;
        u.x = x;
        u.y = y;
        u.z = z;
        return u;
    }

    /**
     * Sets all components equal to zero.
     */
    void zero() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }

    /**
     * Set this vector to the specified coordinates
     * Params:
     *     x = the x-coordinate
     *     y = the y-coordinate
     *     z = the z-coordinate
     */
    void set(float x_, float y_, float z_) {
        x = x_;
        y = y_;
        z = z_;
    }

    /**
     * Negate this vector.
     */
    bzVec3 opNeg() {
        return bzVec3(-x,-y,-z);
    }

    /**
     * Add a vector to this vector.
     * Params: v = the other vector
     */
    void opAddAssign (bzVec3 v) {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    /**
     * Subtract a vector from this vector.
     * Params: v = the other vector
     */
    void opSubAssign (bzVec3 v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    /**
     * Multiply all components of the vector by a scalar.
     * Params: s = the scalar value
     */
    void opMulAssign (float s) {
        x *= s;
        y *= s;
        z *= s;
    }

    /**
     * Multiply all components of the vector by a scalar.
     * Params: s = the scalar value
     * Returns: A new vector containing the result of the multiplication
     */
    bzVec3 opMul(float s) {
        return bzVec3(s * x, s * y, s * z);
    }

    /**
     * Add two vectors component-wise.
     * Params: v = the other vector
     * Returns: A new vector containing the result of the addition
     */
    bzVec3 opAdd(bzVec3 b) {
        return bzVec3(x + b.x, y + b.y, z + b.z);
    }

    /**
     * Subtract two vectors component-wise.
     * Params: v = the other vector
     * Returns: A new vector containing the result of the subtraction
     */
    bzVec3 opSub(bzVec3 v) {
        return bzVec3(x - v.x, y - v.y, z - v.z);
    }

}


/**
 * A 3-by-3 matrix. Stored in column-major order.
 */
struct bzMat33 {

    bzVec3 col1, col2, col3;

    /**
     * Construct matrix using columns.
     * Params:
     *     c1 = the first column
     *     c2 = the second column
     *     c3 = the third column
     * Returns: The new matrix
     */
    static bzMat33 opCall( bzVec3 c1, bzVec3 c2, bzVec3 c3) {
        bzMat33 u;
        u.col1 = c1;
        u.col2 = c2;
        u.col3 = c3;
        return u;
    }

    /**
     * Set this matrix to all zeros.
     */
    void zero() {
        col1.zero();
        col2.zero();
        col3.zero();
    }

    /**
     * Solve A * x = b, where b is a column vector.
     * This is more efficient than computing the inverse in one-shot cases.
     * Params: b = the column in vector form
     * Returns: A new bzVec3 containing the result
     */
    bzVec3 solve33(bzVec3 b) {
        float det = bzDot(col1, bzCross(col2, col3));
        assert(det != 0.0f);
        det = 1.0f / det;
        bzVec3 x;
        x.x = det * bzDot(b, bzCross(col2, col3));
        x.y = det * bzDot(col1, bzCross(b, col3));
        x.z = det * bzDot(col1, bzCross(col2, b));
        return x;
    }

    /**
     * Solve A * x = b, where b is a column vector. This is more efficient
     * than computing the inverse in one-shot cases. This function ignores the
     * third row and third column
     * Params: b = the column in vector form
     * Returns: A new bzVec3 containing the result
     */
    bzVec2 solve22(bzVec2 b) {
        float a11 = col1.x, a12 = col2.x, a21 = col1.y, a22 = col2.y;
        float det = a11 * a22 - a12 * a21;
        assert(det != 0.0f);
        det = 1.0f / det;
        bzVec2 x;
        x.x = det * (a22 * b.x - a12 * b.y);
        x.y = det * (a11 * b.y - a21 * b.x);
        return x;
    }
}


/**
 * I don't know what this is for
 */
struct bzJacobian {

    bzVec2 linear1;
    float angular1 = 0;
    bzVec2 linear2;
    float angular2 = 0;

    void zero() {
        linear1.zero();
        angular1 = 0.0f;
        linear2.zero();
        angular2 = 0.0f;
    }

    void set(bzVec2 x1, float a1, bzVec2 x2, float a2) {
        linear1 = x1;
        angular1 = a1;
        linear2 = x2;
        angular2 = a2;
    }

    float compute(bzVec2 x1, float a1, bzVec2 x2, float a2) {
        return bzDot(linear1, x1) + (angular1 * a1) + bzDot(linear2, x2)
            + (angular2 * a2);
    }

}

	int hash(int Id1, int Id2) {

        int key = (Id2 << 16) | Id1;
        key = ~key + (key << 15);
        key = key ^ (key >>> 12);
        key = key + (key << 2);
        key = key ^ (key >>> 4);
        key = key * 2057;
        key = key ^ (key >>> 16);
        return key;
    }
