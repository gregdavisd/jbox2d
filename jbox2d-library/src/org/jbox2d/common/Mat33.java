/** *****************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ***************************************************************************** */
package org.jbox2d.common;

import java.io.Serializable;
import javax.vecmath.Matrix3f;



/**
 * A 3-by-3 matrix. Stored in column-major order.
 *
 * @author Daniel Murphy
 */
public class Mat33 extends Matrix3f implements Serializable {

	//public final Vec3 ex, ey, ez;
	public Mat33() {

	}

	public Mat33(Vec3 argCol1, Vec3 argCol2, Vec3 argCol3) {
		super(
			argCol1.x, argCol2.x, argCol3.x,
			argCol1.y, argCol2.y, argCol3.y,
			argCol1.z, argCol2.z, argCol3.z);
	}

	public static final void mul22ToOutUnsafe(Mat33 A, Vec2 v, Vec2 out) {
		assert (v != out);
		out.y = A.m10 * v.x + A.m11 * v.y;
		out.x = A.m00 * v.x + A.m01 * v.y;
	}

	public static final void mulToOutUnsafe(Mat33 A, Vec3 v, Vec3 out) {
		assert (out != v);
		out.x = v.x * A.m00 + v.y * A.m01 + v.z * A.m02;
		out.y = v.x * A.m10 + v.y * A.m11 + v.z * A.m12;
		out.z = v.x * A.m20 + v.y * A.m21 + v.z * A.m22;
	}

	/**
	 * Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse in one-shot cases.
	 *
	 * @param b
	 * @return
	 */
	public final void solve22ToOut(Vec2 b, Vec2 out) {
		final float a11 = m00, a12 = m01, a21 = m10, a22 = m11;
		float det = a11 * a22 - a12 * a21;
		if (det != 0.0f) {
			det = 1.0f / det;
		}
		out.x = det * (a22 * b.x - a12 * b.y);
		out.y = det * (a11 * b.y - a21 * b.x);
	}

	/**
	 * Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse in one-shot cases.
	 *
	 * @param b
	 * @param out the result
	 */
	public final void solve33ToOut(Vec3 b, Vec3 out) {
		assert (b != out);
		Vec3 ex= new Vec3();
		getColumn(0, ex);
		Vec3 ey= new Vec3();
		getColumn(1, ey);
		Vec3 ez= new Vec3();
		getColumn(2, ez);

		out.cross(ey, ez);
		float det = ex.dot(out);
		if (det != 0.0f) {
			det = 1.0f / det;
		}
		out.cross(ey, ez);
		final float x = det * b.dot(out);
		out.cross(b, ez);
		final float y = det * ex.dot(out);
		out.cross(ey, b);
		float z = det * ex.dot(out);
		out.x = x;
		out.y = y;
		out.z = z;
	}

	public void getInverse22(Mat33 M) {
		float a = m00, b = m01, c = m10, d = m11;
		float det = a * d - b * c;
		if (det != 0.0f) {
			det = 1.0f / det;
		}

		M.m00 = det * d;
		M.m01 = -det * b;
		M.m20 = 0.0f;
		M.m10 = -det * c;
		M.m11 = det * a;
		M.m21 = 0.0f;
		M.m02 = 0.0f;
		M.m12 = 0.0f;
		M.m22 = 0.0f;
	}

	// / Returns the zero matrix if singular.
	public void getSymInverse33(Mat33 M) {
		float bx = m11 * m22 - m21 * m12;
		float by = m21 * m02 - m01 * m22;
		float bz = m01 * m12 - m11 * m02;
		float det = m00 * bx + m10 * by + m20 * bz;
		if (det != 0.0f) {
			det = 1.0f / det;
		}

		float a11 = m00, a12 = m01, a13 = m02;
		float a22 = m11, a23 = m12;
		float a33 = m22;

		M.m00 = det * (a22 * a33 - a23 * a23);
		M.m10 = det * (a13 * a23 - a12 * a33);
		M.m20 = det * (a12 * a23 - a13 * a22);

		M.m01 = M.m10;
		M.m11 = det * (a11 * a33 - a13 * a13);
		M.m21 = det * (a13 * a12 - a11 * a23);

		M.m02 = M.m20;
		M.m12 = M.m21;
		M.m22 = det * (a11 * a22 - a12 * a12);
	}


}
