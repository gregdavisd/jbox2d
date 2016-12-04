/*
 * Copyright Gregery Barton
 * All rights reserved
 */
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
