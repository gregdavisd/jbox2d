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
package org.jbox2d.Dynamics;

import java.lang.ref.WeakReference;
import org.jbox2d.dynamics.World;

/**
 *
 * @author Gregery Barton
 */
public class CircularWorld {

	public WeakReference<org.jbox2d.dynamics.World> m_world;

	public CircularWorld(World world) {
		m_world = new WeakReference<>(world);
	}

	public final World getWorld() {
		World world = m_world.get();
		if (world == null) {
			throw new AssertionError();
		}
		return world;
	}

}
