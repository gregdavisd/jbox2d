/*
 * Copyright Gregery Barton
 * All rights reserved
 */
package org.jbox2d.common;

import javax.vecmath.Tuple2f;

/**
 *
 * @author Gregery Barton
 */
public class Vec2 extends Tuple2f {

	public Vec2(float x, float y) {
		super(x, y);
	}

	public Vec2(float[] t) {
		super(t);
	}

	public Vec2(Tuple2f t1) {
		super(t1);
	}

	public Vec2() {
	}
	
}
