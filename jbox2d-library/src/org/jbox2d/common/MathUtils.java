/*
 * Copyright Gregery Barton
 * All rights reserved
 */
package org.jbox2d.common;

import java.io.Serializable;
import java.util.Random;

/**
 *
 * @author Gregery Barton
 */
public class MathUtils implements Serializable {

	static final long serialVersionUID = 1L;

	public static final float randomFloat(float argLow, float argHigh) {
		return (float) Math.random() * (argHigh - argLow) + argLow;
	}

	public static final float randomFloat(Random r, float argLow, float argHigh) {
		return r.nextFloat() * (argHigh - argLow) + argLow;
	}

	/**
	 * Returns the closest value to 'a' that is in between 'low' and 'high'
	 *
	 * @param a
	 * @param low
	 * @param high
	 * @return
	 */
	public final static float clamp(final float a, final float low, final float high) {
		return Math.max(low, Math.min(a, high));
	}

}
