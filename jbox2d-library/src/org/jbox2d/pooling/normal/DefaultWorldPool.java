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
/**
 * Created at 3:26:14 AM Jan 11, 2011
 */
package org.jbox2d.pooling.normal;

import org.jbox2d.collision.Collision;
import org.jbox2d.collision.Distance;
import org.jbox2d.collision.TimeOfImpact;
import org.jbox2d.common.Settings;

import org.jbox2d.dynamics.contacts.ChainAndCircleContact;
import org.jbox2d.dynamics.contacts.ChainAndPolygonContact;
import org.jbox2d.dynamics.contacts.CircleContact;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.contacts.EdgeAndCircleContact;
import org.jbox2d.dynamics.contacts.EdgeAndPolygonContact;
import org.jbox2d.dynamics.contacts.PolygonAndCircleContact;
import org.jbox2d.dynamics.contacts.PolygonContact;
import org.jbox2d.pooling.IDynamicStack;
import org.jbox2d.pooling.IWorldPool;

/*
 * Removed object pooling except for the singletons. Object pooling was hurting performance and, ironically, causing
 * large amounts of garbage collection. A better solution is to use stack allocation via escape analysis. This means
 * removing static builder methods and instead using 'new' directly while making objects as short lived as possible.
 *
 * The pool is tightly coupled to the collision system and needs redesigning. Now the pool's only active job now is to a
 * class factory of contact types.
 */
/**
 * Provides object pooling for all objects used in the engine. Objects retrieved from here should only be used
 * temporarily, and then pushed back (with the exception of arrays).
 *
 *
 *
 * @author Daniel Murphy
 */
public class DefaultWorldPool implements IWorldPool {

	private final IWorldPool world = this;

	private final MutableStack<Contact> pcstack =
		new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
		@Override
		protected Contact newInstance() {
			return new PolygonContact(world);
		}
 
	};

	private final MutableStack<Contact> ccstack =
		new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
		@Override
		protected Contact newInstance() {
			return new CircleContact(world);
		}
 
	};

	private final MutableStack<Contact> cpstack =
		new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
		@Override
		protected Contact newInstance() {
			return new PolygonAndCircleContact(world);
		}
 
	};

	private final MutableStack<Contact> ecstack =
		new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
		@Override
		protected Contact newInstance() {
			return new EdgeAndCircleContact(world);
		}
 
	};

	private final MutableStack<Contact> epstack =
		new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
		@Override
		protected Contact newInstance() {
			return new EdgeAndPolygonContact(world);
		}
 
	};

	private final MutableStack<Contact> chcstack =
		new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
		@Override
		protected Contact newInstance() {
			return new ChainAndCircleContact(world);
		}
 
	};

	private final MutableStack<Contact> chpstack =
		new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
		@Override
		protected Contact newInstance() {
			return new ChainAndPolygonContact(world);
		}
 
	};

	private final Collision collision;
	private final TimeOfImpact toi;
	private final Distance dist;

	public DefaultWorldPool(int argSize, int argContainerSize) {
		dist = new Distance();
		collision = new Collision(this);
		toi = new TimeOfImpact(this);
	}

	@Override
	public final IDynamicStack<Contact> getPolyContactStack() {
		return pcstack;
	}

	@Override
	public final IDynamicStack<Contact> getCircleContactStack() {
		return ccstack;
	}

	@Override
	public final IDynamicStack<Contact> getPolyCircleContactStack() {
		return cpstack;
	}

	@Override
	public IDynamicStack<Contact> getEdgeCircleContactStack() {
		return ecstack;
	}

	@Override
	public IDynamicStack<Contact> getEdgePolyContactStack() {
		return epstack;
	}

	@Override
	public IDynamicStack<Contact> getChainCircleContactStack() {
		return chcstack;
	}

	@Override
	public IDynamicStack<Contact> getChainPolyContactStack() {
		return chpstack;
	}

	@Override
	public final Collision getCollision() {
		return collision;
	}

	@Override
	public final TimeOfImpact getTimeOfImpact() {
		return toi;
	}

	@Override
	public final Distance getDistance() {
		return dist;
	}

}
