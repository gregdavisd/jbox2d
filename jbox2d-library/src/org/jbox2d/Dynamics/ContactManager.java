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
package org.jbox2d.dynamics;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.jbox2d.callbacks.ContactFilter;
import org.jbox2d.callbacks.ContactListener;
import org.jbox2d.callbacks.PairCallback;
import org.jbox2d.collision.broadphase.BroadPhase;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.contacts.ContactEdge;
import org.jbox2d.Dynamics.CircularWorld;

/**
 * Delegate of World.
 *
 * @author Daniel Murphy
 */
public class ContactManager extends CircularWorld implements PairCallback {

	public BroadPhase m_broadPhase;
	private final List<Contact> m_contactList;
	public ContactFilter m_contactFilter;
	public ContactListener m_contactListener;

	public ContactManager(World argPool, BroadPhase broadPhase) {
		super(argPool);
		m_contactFilter = new ContactFilter();
		m_broadPhase = broadPhase;
		m_contactList = new ArrayList<>();
	}

	/**
	 * Broad-phase callback.
	 *
	 * @param proxyUserDataA
	 * @param proxyUserDataB
	 */
	public void addPair(Object proxyUserDataA, Object proxyUserDataB) {
		FixtureProxy proxyA = (FixtureProxy) proxyUserDataA;
		FixtureProxy proxyB = (FixtureProxy) proxyUserDataB;

		Fixture fixtureA = proxyA.fixture;
		Fixture fixtureB = proxyB.fixture;

		int indexA = proxyA.childIndex;
		int indexB = proxyB.childIndex;

		Body bodyA = fixtureA.getBody();
		Body bodyB = fixtureB.getBody();

		// Are the fixtures on the same body?
		if (bodyA == bodyB) {
			return;
		}

		// TODO_ERIN use a hash table to remove a potential bottleneck when both
		// bodies have a lot of contacts.
		// Does a contact already exist?
		for (int i_contact_edge = 0; i_contact_edge < bodyB.getContactList().size(); ++i_contact_edge) {
			ContactEdge edge = bodyB.getContactList().get(i_contact_edge);
			if (edge.other == bodyA) {
				Fixture fA = edge.contact.getFixtureA();
				Fixture fB = edge.contact.getFixtureB();
				int iA = edge.contact.getChildIndexA();
				int iB = edge.contact.getChildIndexB();

				if (fA == fixtureA && iA == indexA && fB == fixtureB && iB == indexB) {
					// A contact already exists.
					return;
				}

				if (fA == fixtureB && iA == indexB && fB == fixtureA && iB == indexA) {
					// A contact already exists.
					return;
				}
			}

		}

		// Does a joint override collision? is at least one body dynamic?
		if (bodyB.shouldCollide(bodyA) == false) {
			return;
		}

		// Check user filtering.
		if (m_contactFilter != null && m_contactFilter.shouldCollide(fixtureA, fixtureB) == false) {
			return;
		}

		// Call the factory.
		Contact c = getWorld().popContact(fixtureA, indexA, fixtureB, indexB);
		if (c == null) {
			return;
		}

		// Contact creation may swap fixtures.
		fixtureA = c.getFixtureA();
		fixtureB = c.getFixtureB();
		bodyA = fixtureA.getBody();
		bodyB = fixtureB.getBody();

		// Insert into the world.
		m_contactList.add(c);

		// Connect to island graph.
		// Connect to body A
		c.m_nodeA.contact = c;
		c.m_nodeA.other = bodyB;

		bodyA.getContactList().add(c.m_nodeA);

		// Connect to body B
		c.m_nodeB.contact = c;
		c.m_nodeB.other = bodyA;

		bodyB.getContactList().add(c.m_nodeB);

		// wake up the bodies
		if (!fixtureA.isSensor() && !fixtureB.isSensor()) {
			bodyA.setAwake(true);
			bodyB.setAwake(true);
		}

	}

	public final void findNewContacts() {
		m_broadPhase.updatePairs(this);
	}

	public final void destroyContact(Contact c) {
		if (!m_contactList.remove(c)) {
			/*
			 * contact for removal is not in this ContactManager
			 *
			 */
			throw new AssertionError();
		}
		destroy(c);

	}

	private void destroy(Contact c) {
		Fixture fixtureA = c.getFixtureA();
		Fixture fixtureB = c.getFixtureB();
		Body bodyA = fixtureA.getBody();
		Body bodyB = fixtureB.getBody();

		if (m_contactListener != null && c.isTouching()) {
			m_contactListener.endContact(c);
		}

		// Remove from body 1
		if (!bodyA.getContactList().remove(c.m_nodeA)) {
			/*
			 * Body A does not contain this contact edge to remove
			 *
			 */
			throw new AssertionError();
		}

		// Remove from body 2
		if (!bodyB.getContactList().remove(c.m_nodeB)) {
			/*
			 * Body A does not contain this contact edge to remove
			 *
			 */
			throw new AssertionError();
		}

		/*
		 * Awaken any sensors that where in contact
		 *
		 */
		if (c.m_manifold.pointCount > 0 && !fixtureA.isSensor() && !fixtureB.isSensor()) {
			fixtureA.getBody().setAwake(true);
			fixtureB.getBody().setAwake(true);
		}

	}

	/**
	 * This is the top level collision call for the time step. Here all the narrow phase collision is processed for the
	 * world contact list.
	 */
	public void collide() {
		// Update awake contacts.
		/*
		 * need iterator because items may be removed from the list
		 *
		 */
		for (Iterator i_contacts = m_contactList.iterator(); i_contacts.hasNext();) {
			Contact c = (Contact) i_contacts.next();
			Fixture fixtureA = c.getFixtureA();
			Fixture fixtureB = c.getFixtureB();
			int indexA = c.getChildIndexA();
			int indexB = c.getChildIndexB();
			Body bodyA = fixtureA.getBody();
			Body bodyB = fixtureB.getBody();

			// is this contact flagged for filtering?
			if (c.isFlaggedForFiltering()) {
				// Should these bodies collide?
				if (!bodyB.shouldCollide(bodyA)) {
					i_contacts.remove();
					destroy(c);
					continue;
				}

				// Check user filtering.
				if (m_contactFilter != null && !m_contactFilter.shouldCollide(fixtureA, fixtureB)) {
					i_contacts.remove();
					destroy(c);
					continue;
				}

				// Clear the filtering flag.
				c.unflagForFiltering();
			}

			boolean activeA = bodyA.isAwake() && bodyA.m_type != BodyType.STATIC;
			boolean activeB = bodyB.isAwake() && bodyB.m_type != BodyType.STATIC;

			// At least one body must be awake and it must be dynamic or kinematic.
			if (activeA == false && activeB == false) {
				continue;
			}

			int proxyIdA = fixtureA.m_proxies[indexA].proxyId;
			int proxyIdB = fixtureB.m_proxies[indexB].proxyId;
			boolean overlap = m_broadPhase.testOverlap(proxyIdA, proxyIdB);

			// Here we destroy contacts that cease to overlap in the broad-phase.
			if (overlap == false) {
				i_contacts.remove();
				destroy(c);
				continue;
			}

			// The contact persists.
			c.update(m_contactListener);
		}
	}

	public final List<Contact> getContactList() {
		return m_contactList;
	}

	public final int getContactCount() {
		return m_contactList.size();
	}
}
