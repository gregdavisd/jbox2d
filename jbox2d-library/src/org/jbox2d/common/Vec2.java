/**
 * *****************************************************************************
 * Copyright (c) 2016, Gregery Barton
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions.
 *****************************************************************************
 */
package org.jbox2d.common;

import java.io.Serializable;
import javax.vecmath.Tuple2f;

/**
 *
 * @author Gregery Barton
 */
public final class Vec2 extends Tuple2f<Vec2> implements Serializable {

 static final long serialVersionUID = 1L;

 public Vec2(float x, float y) {
  this.x = x;
  this.y = y;
 }

 public Vec2(float[] t) {
  this.x = t[0];
  this.y = t[1];
 }

 public Vec2(Tuple2f t1) {
  this.x = t1.x;
  this.y = t1.y;
 }

 public Vec2() {
 }
}
