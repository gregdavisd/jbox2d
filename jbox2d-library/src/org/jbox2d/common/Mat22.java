/*******************************************************************************
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
 ******************************************************************************/
package org.jbox2d.common;

import java.io.Serializable;
import javax.vecmath.Matrix2f;


/**
 * A 2-by-2 matrix. Stored in column-major order.
 */
public class Mat22 extends Matrix2f implements Serializable {

  /**
   * Construct zero matrix. Note: this is NOT an identity matrix! djm fixed double allocation
   * problem
   */
  public Mat22() {
			super();
  }

  /**
   * Create a matrix with given vectors as columns.
   * 
   * @param c1 Column 1 of matrix
   * @param c2 Column 2 of matrix
   */
  public Mat22(final Vec2 c1, final Vec2 c2) {
			super(c1.x,c2.x,c1.y,c1.y);
  }

  /**
   * Create a matrix from four floats.
   * 
   * @param exx
   * @param col2x
   * @param exy
   * @param col2y
   */
  public Mat22(final float exx, final float col2x, final float exy, final float col2y) {
			super(exx,col2x,exy,col2y);
  }


  public final void invertToOut(final Mat22 out) {
    final float a = m00, b = m01, c = m10, d = m11;
    float det = a * d - b * c;
    // b2Assert(det != 0.0f);
    det = 1.0f / det;
    out.m00 = det * d;
    out.m01 = -det * b;
    out.m10 = -det * c;
    out.m11 = det * a;
  }



  public final void mulToOut(final Vec2 v, final Vec2 out) {
    final float tempy = m10 * v.x + m11 * v.y;
    out.x = m00 * v.x + m01 * v.y;
    out.y = tempy;
  }


  public final Mat22 mulLocal(final Mat22 R) {
    mulToOut(R, this);
    return this;
  }

  public final void mulToOut(final Mat22 R, final Mat22 out) {
    final float tempy1 = this.m10 * R.m00 + this.m11 * R.m10;
    final float tempx1 = this.m00 * R.m00 + this.m01 * R.m10;
    out.m00 = tempx1;
    out.m10 = tempy1;
    final float tempy2 = this.m10 * R.m01 + this.m11 * R.m11;
    final float tempx2 = this.m00 * R.m01 + this.m01 * R.m11;
    out.m01 = tempx2;
    out.m11 = tempy2;
  }
 
 
  public final void solveToOut(final Vec2 b, final Vec2 out) {
    final float a11 = m00, a12 = m01, a21 = m10, a22 = m11;
    float det = a11 * a22 - a12 * a21;
    if (det != 0.0f) {
      det = 1.0f / det;
    }
    final float tempy = det * (a11 * b.y - a21 * b.x);
    out.x = det * (a22 * b.x - a12 * b.y);
    out.y = tempy;
  }
 
  public final static void mulToOutUnsafe(final Mat22 R, final Vec2 v, final Vec2 out) {
    assert (v != out);
    out.x = R.m00 * v.x + R.m01 * v.y;
    out.y = R.m10 * v.x + R.m11 * v.y;
  }



  public final static Mat22 createScaleTransform(float scale) {
    Mat22 mat = new Mat22();
    mat.m00 = scale;
    mat.m11 = scale;
    return mat;
  }

  public final static void createScaleTransform(float scale, Mat22 out) {
    out.m00 = scale;
    out.m11 = scale;
  }


}
