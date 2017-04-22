/** *****************************************************************************
 * Copyright (c) 2016, Gregery Barton
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions.
 ***************************************************************************** */
package org.jbox2d.dynamics;

import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class CircularWorld implements Serializable {

	static final long serialVersionUID = 1L;
	private org.jbox2d.dynamics.World m_world;

	public CircularWorld(World world) {
		m_world = world;
	}

	public CircularWorld() {
		m_world = null;
	}

	public final World getWorld() {

		return m_world;
	}

	public final void setWorld(World world) {
		m_world = world;
	}

}
