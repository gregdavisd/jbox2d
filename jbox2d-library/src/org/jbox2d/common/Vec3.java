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
import javax.vecmath.Tuple3f;

/**
 *
 * @author Gregery Barton
 */
public final class Vec3 extends Tuple3f implements Serializable {

 static final long serialVersionUID = 1L;

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
