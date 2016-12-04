/*
 * Copyright Gregery Barton
 * All rights reserved
 */
package org.jbox2d.common;

import javax.vecmath.Tuple3f;

/**
 *
 * @author Gregery Barton
 */
public class Vec3 extends Tuple3f {

	public Vec3(float x, float y, float z) {
		super(x, y, z);
	}

	public Vec3(float[] t) {
		super(t);
	}

	public Vec3(Tuple3f t1) {
		super(t1);
	}

	public Vec3() {
	}
	
}
