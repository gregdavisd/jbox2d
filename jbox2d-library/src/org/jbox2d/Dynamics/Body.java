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

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import org.jbox2d.collision.broadphase.BroadPhase;
import org.jbox2d.collision.shapes.MassData;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Rot;
import org.jbox2d.common.Sweep;
import org.jbox2d.common.Transform;

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.contacts.ContactEdge;
import org.jbox2d.dynamics.joints.JointEdge;

/**
 * A rigid body. These are created via World.createBody.
 *
 * @author Daniel Murphy
 */
public class Body extends CircularWorld implements Serializable {

	static final long serialVersionUID = 1L;
//	public static final int E_ISLAND_FLAG = 0x0001;
//	public static final int E_AWAKE_FLAG = 0x0002;
//	public static final int E_AUTO_SLEEP_FLAG = 0x0004;
//	public static final int E_BULLET_FLAG = 0x0008;
//	public static final int E_FIXED_ROTATION_FLAG = 0x0010;
//	public static final int E_ACTIVE_FLAG = 0x0020;
//	public static final int E_TOI_FLAG = 0x0040;
//
	public byte m_type;

	public boolean is_island;
	private boolean is_awake;
	private boolean is_auto_sleep;
	private boolean is_bullet;
	private boolean is_fixed_rotation;
	private boolean broadphase;
	private boolean is_active;

	//public int m_flags;
	public int m_islandIndex;

	/**
	 * The body origin transform.
	 */
	public final Transform m_xf = new Transform();
	/**
	 * The previous transform for particle simulation
	 */
	public final Transform m_xf0 = new Transform();

	/**
	 * The swept motion for CCD
	 */
	public final Sweep m_sweep = new Sweep();

	public final Vec2 m_linearVelocity = new Vec2();
	public float m_angularVelocity = 0;

	public final Vec2 m_force = new Vec2();
	public float m_torque = 0;

	private final List<Fixture> m_fixtureList;
	private final List<JointEdge> m_jointList;
	private final List<ContactEdge> m_contactList;

	public float m_mass, m_invMass;

	// Rotational inertia about the center of mass.
	public float m_I, m_invI;

	public float m_linearDamping;
	public float m_angularDamping;
	public float m_gravityScale;

	public float m_sleepTime;

	private Object m_userData;

	public Body(final BodyDef bd) {
		this(bd, null);
	}

	public Body(final BodyDef bd, World world) {
		super(world);
		//assert (bd.position.isValid());
		//assert (bd.linearVelocity.isValid());
		m_fixtureList = new ArrayList<>();
		assert (bd.gravityScale >= 0.0f);
		assert (bd.angularDamping >= 0.0f);
		assert (bd.linearDamping >= 0.0f);

		if (bd.bullet) {
			is_bullet = true;
		}
		if (bd.fixedRotation) {
			is_fixed_rotation = true;
		}
		if (bd.allowSleep) {
			is_auto_sleep = true;
		}
		if (bd.awake) {
			is_awake = true;
		}
		if (bd.active) {
			is_active = true;
		}
		broadphase = bd.broadphase;

		m_xf.p.set(bd.position);
		m_xf.q.set(bd.angle);

		m_sweep.localCenter.setZero();
		m_sweep.c0.set(m_xf.p);
		m_sweep.c.set(m_xf.p);
		m_sweep.a0 = bd.angle;
		m_sweep.a = bd.angle;
		m_sweep.alpha0 = 0.0f;

		m_jointList = new ArrayList<>();
		m_contactList = new ArrayList<>();

		m_linearVelocity.set(bd.linearVelocity);
		m_angularVelocity = bd.angularVelocity;

		m_linearDamping = bd.linearDamping;
		m_angularDamping = bd.angularDamping;
		m_gravityScale = bd.gravityScale;

		m_force.setZero();
		m_torque = 0.0f;

		m_sleepTime = 0.0f;

		m_type = bd.type;

		if (m_type == BodyType.DYNAMIC) {
			m_mass = 1f;
			m_invMass = 1f;
		} else {
			m_mass = 0f;
			m_invMass = 0f;
		}

		m_I = 0.0f;
		m_invI = 0.0f;

		m_userData = bd.userData;

	}

	/**
	 * Creates a fixture and attach it to this body. Use this function if you need to set some fixture parameters, like friction.
	 * Otherwise you can create the fixture directly from a shape. If the density is non-zero, this function automatically updates
	 * the mass of the body. Contacts are not created until the next time step.
	 *
	 * @param def the fixture definition.
	 * @warning This function is locked during callbacks.
	 */
	public final Fixture createFixture(FixtureDef def) {
		assert (getWorld().isLocked() == false);

		if (getWorld().isLocked() == true) {
			return null;
		}

		Fixture fixture = new Fixture();
		fixture.create(this, def);

		if (is_active && broadphase) {
			BroadPhase broadPhase = getWorld().m_contactManager.m_broadPhase;
			fixture.createProxies(broadPhase, m_xf);
		}

		m_fixtureList.add(fixture);

		fixture.m_body = this;

		// Adjust mass properties if needed.
		if (fixture.m_density > 0.0f) {
			resetMassData();
		}

		// Let the world know we have a new fixture. This will cause new contacts
		// to be created at the beginning of the next time step.
		getWorld().m_flags |= World.NEW_FIXTURE;

		return fixture;
	}

	private final FixtureDef fixDef = new FixtureDef();

	/**
	 * Creates a fixture from a shape and attach it to this body. This is a convenience function. Use FixtureDef if you need to set
	 * parameters like friction, restitution, user data, or filtering. If the density is non-zero, this function automatically
	 * updates the mass of the body.
	 *
	 * @param shape the shape to be cloned.
	 * @param density the shape density (set to zero for static bodies).
	 * @warning This function is locked during callbacks.
	 */
	public final Fixture createFixture(Shape shape, float density) {
		fixDef.shape = shape;
		fixDef.density = density;

		return createFixture(fixDef);
	}

	/**
	 * Destroy a fixture. This removes the fixture from the broad-phase and destroys all contacts associated with this fixture. This
	 * will automatically adjust the mass of the body if the body is dynamic and the fixture has positive density. All fixtures
	 * attached to a body are implicitly destroyed when the body is destroyed.
	 *
	 * @param fixture the fixture to be removed.
	 * @warning This function is locked during callbacks.
	 */
	public final void destroyFixture(Fixture fixture) {
		assert (getWorld().isLocked() == false);
		if (getWorld().isLocked() == true) {
			return;
		}

		assert (fixture.m_body == this);

		// Destroy any contacts associated with the fixture.
		/*
		 * have to loop monkey because ContactManager is deleting items from the contact list being iterated over.
		 */
		int i_contact_edge = 0;
		while (i_contact_edge < m_contactList.size()) {
			ContactEdge edge = m_contactList.get(i_contact_edge);
			Contact c = edge.contact;
			Fixture fixtureA = c.getFixtureA();
			Fixture fixtureB = c.getFixtureB();

			if (fixture == fixtureA || fixture == fixtureB) {
				// This destroys the contact and removes it from
				// this body's contact list.
				getWorld().m_contactManager.destroyContact(c);
			} else {
				++i_contact_edge;
			}
		}

		if (is_active) {
			BroadPhase broadPhase = getWorld().m_contactManager.m_broadPhase;
			fixture.destroyProxies(broadPhase);
		}

		if (!m_fixtureList.remove(fixture)) {
			/*
			 * tried to remove a fixture not connected to this body
			 *
			 */
			throw new AssertionError();
		}

		// Reset the mass data.
		resetMassData();
	}

	/**
	 * Set the position of the body's origin and rotation. This breaks any contacts and wakes the other bodies. Manipulating a body's
	 * transform may cause non-physical behavior. Note: contacts are updated on the next call to World.step().
	 *
	 * @param position the world position of the body's local origin.
	 * @param angle the world rotation in radians.
	 */
	public final void setTransform(Vec2 position, float angle) {
		assert (getWorld().isLocked() == false);
		if (getWorld().isLocked() == true) {
			return;
		}

		m_xf.q.set(angle);
		m_xf.p.set(position);

		// m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
		Transform.mulToOutUnsafe(m_xf, m_sweep.localCenter, m_sweep.c);
		m_sweep.a = angle;

		m_sweep.c0.set(m_sweep.c);
		m_sweep.a0 = m_sweep.a;

		BroadPhase broadPhase = getWorld().m_contactManager.m_broadPhase;
		for (Fixture fixture : m_fixtureList) {
			fixture.synchronize(broadPhase, m_xf, m_xf);
		}
	}

	/**
	 * Get the body transform for the body's origin.
	 *
	 * @return the world transform of the body's origin.
	 */
	public final Transform getTransform() {
		return new Transform(m_xf);
	}

	/**
	 * Get the world body origin position.
	 *
	 * @return the world position of the body's origin.
	 */
	public final Vec2 getPosition() {
		return new Vec2(m_xf.p);
	}

	/**
	 * Get the angle in radians.
	 *
	 * @return the current world rotation angle in radians.
	 */
	public final float getAngle() {
		return m_sweep.a;
	}

	/**
	 * Get a world vector in the direction of this body's x axis;
	 *
	 * @return
	 */
	public final Vec2 getWorldXAxis() {
		return m_xf.q.getXAxis();
	}

	/**
	 * Get a world vector in the direction of this body's y axis;
	 *
	 * @return
	 */
	public final Vec2 getWorldYAxis() {
		return m_xf.q.getYAxis();
	}

	/**
	 * Get the world position of the center of mass.
	 */
	public final Vec2 getWorldCenter() {
		return new Vec2(m_sweep.c);
	}

	/**
	 * Get the local position of the center of mass.
	 */
	public final Vec2 getLocalCenter() {
		return new Vec2(m_sweep.localCenter);
	}

	/**
	 * Set the linear velocity of the center of mass.
	 *
	 * @param v the new linear velocity of the center of mass.
	 */
	public final void setLinearVelocity(Vec2 v) {
		if (m_type == BodyType.STATIC) {
			return;
		}

		if (!isAwake() && (v.lengthSquared() > 0.0f)) {
			setAwake(true);
		}

		m_linearVelocity.set(v);
	}

	/**
	 * Add to the linear velocity of the center of mass.
	 *
	 * @param v delta for the linear velocity of the center of mass.
	 */
	public final void applyLinearVelocity(Vec2 v) {
		if (m_type == BodyType.STATIC) {
			return;
		}

		m_linearVelocity.add(v);

		if (!isAwake() && (m_linearVelocity.lengthSquared() > 0.0f)) {
			setAwake(true);
		}

	}

	/**
	 * Get the linear velocity of the center of mass.
	 *
	 * @return the linear velocity of the center of mass.
	 */
	public final Vec2 getLinearVelocity() {
		return new Vec2(m_linearVelocity);
	}

	/**
	 * Set the angular velocity.
	 *
	 * @param omega the new angular velocity in radians/second.
	 */
	public final void setAngularVelocity(float w) {
		if (m_type == BodyType.STATIC) {
			return;
		}

		if (w * w > 0f) {
			setAwake(true);
		}

		m_angularVelocity = w;
	}

	/**
	 * Get the angular velocity.
	 *
	 * @return the angular velocity in radians/second.
	 */
	public final float getAngularVelocity() {
		return m_angularVelocity;
	}

	/**
	 * Get the gravity scale of the body.
	 *
	 * @return
	 */
	public final float getGravityScale() {
		return m_gravityScale;
	}

	/**
	 * Set the gravity scale of the body.
	 *
	 * @param gravityScale
	 */
	public void setGravityScale(float gravityScale) {
		this.m_gravityScale = gravityScale;
	}

	/**
	 * Apply a force at a world point. If the force is not applied at the center of mass, it will generate a torque and affect the
	 * angular velocity. This wakes up the body.
	 *
	 * @param force the world force vector, usually in Newtons (N).
	 * @param point the world position of the point of application.
	 */
	public final void applyForce(Vec2 force, Vec2 point) {
		if (m_type != BodyType.DYNAMIC) {
			return;
		}

		if (!isAwake()) {
			setAwake(true);
		}

		// m_force.add(force);
		// Vec2 temp = tltemp.get();
		// temp.set(point).sub(m_sweep.c);
		// m_torque += Vec2.cross(temp, force);
		m_force.x += force.x;
		m_force.y += force.y;

		m_torque += (point.x - m_sweep.c.x) * force.y - (point.y - m_sweep.c.y) * force.x;
	}

	/**
	 * Apply a force to the center of mass. This wakes up the body.
	 *
	 * @param force the world force vector, usually in Newtons (N).
	 */
	public final void applyForceToCenter(Vec2 force) {
		if (m_type != BodyType.DYNAMIC) {
			return;
		}

		if (!isAwake()) {
			setAwake(true);
		}

		m_force.x += force.x;
		m_force.y += force.y;
	}

	/**
	 * Apply a torque. This affects the angular velocity without affecting the linear velocity of the center of mass. This wakes up
	 * the body.
	 *
	 * @param torque about the z-axis (out of the screen), usually in N-m.
	 */
	public final void applyTorque(float torque) {
		if (m_type != BodyType.DYNAMIC) {
			return;
		}

		if (!isAwake()) {
			setAwake(true);
		}

		m_torque += torque;
	}

	/**
	 * Apply an impulse at a point. This immediately modifies the velocity. It also modifies the angular velocity if the point of
	 * application is not at the center of mass. This wakes up the body if 'wake' is set to true. If the body is sleeping and 'wake'
	 * is false, then there is no effect.
	 *
	 * @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	 * @param point the world position of the point of application.
	 * @param wake also wake up the body
	 */
	public final void applyLinearImpulse(Vec2 impulse, Vec2 point, boolean wake) {
		if (m_type != BodyType.DYNAMIC) {
			return;
		}

		if (!isAwake()) {
			if (wake) {
				setAwake(true);
			} else {
				return;
			}
		}

		m_linearVelocity.x += impulse.x * m_invMass;
		m_linearVelocity.y += impulse.y * m_invMass;

		m_angularVelocity
			+= m_invI * ((point.x - m_sweep.c.x) * impulse.y - (point.y - m_sweep.c.y) * impulse.x);
	}

	/**
	 * Apply an angular impulse.
	 *
	 * @param impulse the angular impulse in units of kg*m*m/s
	 */
	public void applyAngularImpulse(float impulse) {
		if (m_type != BodyType.DYNAMIC) {
			return;
		}

		if (!isAwake()) {
			setAwake(true);
		}
		m_angularVelocity += m_invI * impulse;
	}

	/**
	 * Get the total mass of the body.
	 *
	 * @return the mass, usually in kilograms (kg).
	 */
	public final float getMass() {
		return m_mass;
	}

	/**
	 * Get the central rotational inertia of the body.
	 *
	 * @return the rotational inertia, usually in kg-m^2.
	 */
	public final float getInertia() {
		return m_I
			+ m_mass
			* (m_sweep.localCenter.x * m_sweep.localCenter.x + m_sweep.localCenter.y
			* m_sweep.localCenter.y);
	}

	/**
	 * Get the mass data of the body. The rotational inertia is relative to the center of mass.
	 *
	 * @return a struct containing the mass, inertia and center of the body.
	 */
	public final void getMassData(MassData data) {
		// data.mass = m_mass;
		// data.I = m_I + m_mass * Vec2.dot(m_sweep.localCenter, m_sweep.localCenter);
		// data.center.set(m_sweep.localCenter);

		data.mass = m_mass;
		data.I
			= m_I
			+ m_mass
			* (m_sweep.localCenter.x * m_sweep.localCenter.x + m_sweep.localCenter.y
			* m_sweep.localCenter.y);
		data.center.x = m_sweep.localCenter.x;
		data.center.y = m_sweep.localCenter.y;
	}

	/**
	 * Set the mass properties to override the mass properties of the fixtures. Note that this changes the center of mass position.
	 * Note that creating or destroying fixtures can also alter the mass. This function has no effect if the body isn't dynamic.
	 *
	 * @param massData the mass properties.
	 */
	public final void setMassData(MassData massData) {
		// TODO_ERIN adjust linear velocity and torque to account for movement of center.
		assert (getWorld().isLocked() == false);
		if (getWorld().isLocked() == true) {
			return;
		}

		if (m_type != BodyType.DYNAMIC) {
			return;
		}

		m_invMass = 0.0f;
		m_I = 0.0f;
		m_invI = 0.0f;

		m_mass = massData.mass;
		if (m_mass <= 0.0f) {
			m_mass = 1f;
		}

		m_invMass = 1.0f / m_mass;

		if (massData.I > 0.0f && !is_fixed_rotation) {
			m_I = massData.I - m_mass * massData.center.dot(massData.center);
			assert (m_I > 0.0f);
			m_invI = 1.0f / m_I;
		}

		final Vec2 oldCenter = new Vec2();
		// Move center of mass.
		oldCenter.set(m_sweep.c);
		m_sweep.localCenter.set(massData.center);
		// m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
		Transform.mulToOutUnsafe(m_xf, m_sweep.localCenter, m_sweep.c0);
		m_sweep.c.set(m_sweep.c0);

		// Update center of mass velocity.
		// m_linearVelocity += Cross(m_angularVelocity, m_sweep.c - oldCenter);
		final Vec2 temp = new Vec2();
		temp.set(m_sweep.c).sub(oldCenter);
		temp.setRightPerpendicular(m_angularVelocity);
		m_linearVelocity.add(temp);

	}

	/**
	 * This resets the mass properties to the sum of the mass properties of the fixtures. This normally does not need to be called
	 * unless you called setMassData to override the mass and you later want to reset the mass.
	 */
	public final void resetMassData() {
		// Compute mass data from shapes. Each shape has its own density.
		m_mass = 0.0f;
		m_invMass = 0.0f;
		m_I = 0.0f;
		m_invI = 0.0f;
		m_sweep.localCenter.setZero();

		// Static and kinematic bodies have zero mass.
		if (m_type == BodyType.STATIC || m_type == BodyType.KINEMATIC) {
			// m_sweep.c0 = m_sweep.c = m_xf.position;
			m_sweep.c0.set(m_xf.p);
			m_sweep.c.set(m_xf.p);
			m_sweep.a0 = m_sweep.a;
			return;
		}

		assert (m_type == BodyType.DYNAMIC);

		// Accumulate mass over all fixtures.
		final Vec2 localCenter = new Vec2();
		localCenter.setZero();
		final Vec2 temp = new Vec2();
		final MassData massData = new MassData();
		for (int i = 0; i < m_fixtureList.size(); ++i) {
			Fixture f = m_fixtureList.get(i);

			if (f.m_density == 0.0f) {
				continue;
			}
			f.getMassData(massData);
			m_mass += massData.mass;
			// center += massData.mass * massData.center;
			temp.set(massData.center).scale(massData.mass);
			localCenter.add(temp);
			m_I += massData.I;
		}

		// Compute center of mass.
		if (m_mass > 0.0f) {
			m_invMass = 1.0f / m_mass;
			localCenter.scale(m_invMass);
		} else {
			// Force all dynamic bodies to have a positive mass.
			m_mass = 1.0f;
			m_invMass = 1.0f;
		}

		if (m_I > 0.0f && !is_fixed_rotation) {
			// Center the inertia about the center of mass.
			m_I -= m_mass * localCenter.dot(localCenter);
			assert (m_I > 0.0f);
			m_invI = 1.0f / m_I;
		} else {
			m_I = 0.0f;
			m_invI = 0.0f;
		}

		Vec2 oldCenter = new Vec2(m_sweep.c);
		// Move center of mass.
		m_sweep.localCenter.set(localCenter);
		// m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
		Transform.mulToOutUnsafe(m_xf, m_sweep.localCenter, m_sweep.c0);
		m_sweep.c.set(m_sweep.c0);

		// Update center of mass velocity.
		// m_linearVelocity += Cross(m_angularVelocity, m_sweep.c - oldCenter);
		temp.set(m_sweep.c).sub(oldCenter);

		final Vec2 temp2 = oldCenter;
		temp2.set(temp).setRightPerpendicular(m_angularVelocity);
		m_linearVelocity.add(temp2);

	}

	/**
	 * Get the world coordinates of a point given the local coordinates.
	 *
	 * @param localPoint a point on the body measured relative the the body's origin.
	 * @return the same point expressed in world coordinates.
	 */
	public final Vec2 getWorldPoint(Vec2 localPoint) {
		Vec2 v = new Vec2();
		getWorldPointToOut(localPoint, v);
		return v;
	}

	public final void getWorldPointToOut(Vec2 localPoint, Vec2 out) {
		Transform.mulToOut(m_xf, localPoint, out);
	}

	/**
	 * Get the world coordinates of a vector given the local coordinates.
	 *
	 * @param localVector a vector fixed in the body.
	 * @return the same vector expressed in world coordinates.
	 */
	public final Vec2 getWorldVector(Vec2 localVector) {
		Vec2 out = new Vec2();
		getWorldVectorToOut(localVector, out);
		return out;
	}

	public final void getWorldVectorToOut(Vec2 localVector, Vec2 out) {
		Rot.mulToOut(m_xf.q, localVector, out);
	}

	public final void getWorldVectorToOutUnsafe(Vec2 localVector, Vec2 out) {
		Rot.mulToOutUnsafe(m_xf.q, localVector, out);
	}

	/**
	 * Gets a local point relative to the body's origin given a world point.
	 *
	 * @param a point in world coordinates.
	 * @return the corresponding local point relative to the body's origin.
	 */
	public final Vec2 getLocalPoint(Vec2 worldPoint) {
		Vec2 out = new Vec2();
		getLocalPointToOut(worldPoint, out);
		return out;
	}

	public final void getLocalPointToOut(Vec2 worldPoint, Vec2 out) {
		Transform.mulTransToOut(m_xf, worldPoint, out);
	}

	/**
	 * Gets a local vector given a world vector.
	 *
	 * @param a vector in world coordinates.
	 * @return the corresponding local vector.
	 */
	public final Vec2 getLocalVector(Vec2 worldVector) {
		Vec2 out = new Vec2();
		getLocalVectorToOut(worldVector, out);
		return out;
	}

	/**
	 * Convert a world angle to an angle relative to this body.
	 *
	 * @param worldAngle the world angle
	 * @return a local angle
	 */
	public final float getLocalAngle(float worldAngle) {
		return worldAngle - getAngle();
	}

	public final void getLocalVectorToOut(Vec2 worldVector, Vec2 out) {
		Rot.mulTrans(m_xf.q, worldVector, out);
	}

	/**
	 * Get the world linear velocity of a world point attached to this body.
	 *
	 * @param a point in world coordinates.
	 * @return the world velocity of a point.
	 */
	public final Vec2 getLinearVelocityFromWorldPoint(Vec2 worldPoint) {
		Vec2 out = new Vec2();
		getLinearVelocityFromWorldPointToOut(worldPoint, out);
		return out;
	}

	public final void getLinearVelocityFromWorldPointToOut(Vec2 worldPoint, Vec2 out) {
		final float tempX = worldPoint.x - m_sweep.c.x;
		final float tempY = worldPoint.y - m_sweep.c.y;
		out.x = -m_angularVelocity * tempY + m_linearVelocity.x;
		out.y = m_angularVelocity * tempX + m_linearVelocity.y;
	}

	/**
	 * Get the world velocity of a local point.
	 *
	 * @param a point in local coordinates.
	 * @return the world velocity of a point.
	 */
	public final Vec2 getLinearVelocityFromLocalPoint(Vec2 localPoint) {
		Vec2 out = new Vec2();
		getLinearVelocityFromLocalPointToOut(localPoint, out);
		return out;
	}

	public final void getLinearVelocityFromLocalPointToOut(Vec2 localPoint, Vec2 out) {
		getWorldPointToOut(localPoint, out);
		getLinearVelocityFromWorldPointToOut(out, out);
	}

	/**
	 * Get the linear damping of the body.
	 */
	public final float getLinearDamping() {
		return m_linearDamping;
	}

	/**
	 * Set the linear damping of the body.
	 */
	public final void setLinearDamping(float linearDamping) {
		m_linearDamping = linearDamping;
	}

	/**
	 * Get the angular damping of the body.
	 */
	public final float getAngularDamping() {
		return m_angularDamping;
	}

	/**
	 * Set the angular damping of the body.
	 */
	public final void setAngularDamping(float angularDamping) {
		m_angularDamping = angularDamping;
	}

	public final int getType() {
		return m_type;
	}

	/**
	 * Set the type of this body. This may alter the mass and velocity.
	 *
	 * @param type
	 */
	public final void setType(byte type) {
		assert (getWorld().isLocked() == false);
		if (getWorld().isLocked() == true) {
			return;
		}

		if (m_type == type) {
			return;
		}

		m_type = type;

		resetMassData();

		if (m_type == BodyType.STATIC) {
			m_linearVelocity.setZero();
			m_angularVelocity = 0.0f;
			m_sweep.a0 = m_sweep.a;
			m_sweep.c0.set(m_sweep.c);
			synchronizeFixtures();
		}

		setAwake(true);

		m_force.setZero();
		m_torque = 0.0f;

		delete_attached_contacts();

		// Touch the proxies so that new contacts will be created (when appropriate)
		BroadPhase broadPhase = getWorld().m_contactManager.m_broadPhase;
		for (int i_fixture = 0; i_fixture < m_fixtureList.size(); ++i_fixture) {
			Fixture f = m_fixtureList.get(i_fixture);
			int proxyCount = f.m_proxyCount;
			for (int i = 0; i < proxyCount; ++i) {
				broadPhase.touchProxy(f.m_proxies[i].proxyId);
			}
		}
	}

	/**
	 * Is this body treated like a bullet for continuous collision detection?
	 */
	public final boolean isBullet() {
		return is_bullet;
	}

	/**
	 * Should this body be treated like a bullet for continuous collision detection?
	 */
	public final void setBullet(boolean flag) {
		is_bullet = flag;
	}

	/**
	 * You can disable sleeping on this body. If you disable sleeping, the body will be woken.
	 *
	 * @param flag
	 */
	public final void setSleepingAllowed(boolean flag) {
		is_auto_sleep = flag;
	}

	/**
	 * Is this body allowed to sleep
	 *
	 * @return
	 */
	public final boolean isSleepingAllowed() {
		return is_auto_sleep;
	}

	/**
	 * Set the sleep state of the body. A sleeping body has very low CPU cost.
	 *
	 * @param flag set to true to put body to sleep, false to wake it.
	 */
	public final void setAwake(boolean flag) {
		if (flag) {
			if (!is_awake) {
				is_awake = true;
				m_sleepTime = 0.0f;
			}
		} else {
			is_awake = false;
			m_sleepTime = 0.0f;
			m_linearVelocity.setZero();
			m_angularVelocity = 0.0f;
			m_force.setZero();
			m_torque = 0.0f;
		}
	}

	/**
	 * Get the sleeping state of this body.
	 *
	 * @return true if the body is awake.
	 */
	public final boolean isAwake() {
		return is_awake;
	}

	/**
	 * Set the active state of the body. An inactive body is not simulated and cannot be collided with or woken up. If you pass a
	 * flag of true, all fixtures will be added to the broad-phase. If you pass a flag of false, all fixtures will be removed from
	 * the broad-phase and all contacts will be destroyed. Fixtures and joints are otherwise unaffected. You may continue to
	 * create/destroy fixtures and joints on inactive bodies. Fixtures on an inactive body are implicitly inactive and will not
	 * participate in collisions, ray-casts, or queries. Joints connected to an inactive body are implicitly inactive. An inactive
	 * body is still owned by a World object and remains in the body list.
	 *
	 * @param flag
	 */
	public final void setActive(boolean flag) {
		assert (getWorld().isLocked() == false);

		if (flag == isActive()) {
			return;
		}

		if (flag) {
			is_active = true;

			if (broadphase) {
				// Create all proxies.
				BroadPhase broadPhase = getWorld().m_contactManager.m_broadPhase;
				for (Fixture fixture : m_fixtureList) {
					fixture.createProxies(broadPhase, m_xf);
				}
			}
			// Contacts are created the next time step.
		} else {
			is_active = false;

			// Destroy all proxies.
			BroadPhase broadPhase = getWorld().m_contactManager.m_broadPhase;
			for (Fixture fixture : m_fixtureList) {
				fixture.destroyProxies(broadPhase);
			}

			// Destroy the attached contacts.
			delete_attached_contacts();
		}
	}

	/**
	 * Get the active state of the body.
	 *
	 * @return
	 */
	public final boolean isActive() {
		return is_active;
	}

	/**
	 * Set this body to have fixed rotation. This causes the mass to be reset.
	 *
	 * @param flag
	 */
	public final void setFixedRotation(boolean flag) {
		is_fixed_rotation = flag;
		resetMassData();
	}

	/**
	 * Does this body have fixed rotation?
	 *
	 * @return
	 */
	public final boolean isFixedRotation() {
		return is_fixed_rotation;
	}

	/**
	 * Get the list of all fixtures attached to this body.
	 *
	 * @return
	 */
	public final List<Fixture> getFixtureList() {
		return m_fixtureList;
	}

	/**
	 * Get the list of all joints attached to this body.
	 *
	 * @return
	 */
	public final List<JointEdge> getJointList() {
		return m_jointList;
	}

	/**
	 * Get the list of all contacts attached to this body.
	 *
	 * @return
	 * @warning this list changes during the time step and you may miss some collisions if you don't use ContactListener.
	 */
	public final List<ContactEdge> getContactList() {
		return m_contactList;
	}

	/**
	 * Get the user data pointer that was provided in the body definition.
	 */
	public final Object getUserData() {
		return m_userData;
	}

	/**
	 * Set the user data. Use this to store your application specific data.
	 */
	public final void setUserData(Object data) {
		m_userData = data;
	}

	// djm pooling
	private final Transform pxf = new Transform();

	protected final void synchronizeFixtures() {
		final Transform xf1 = pxf;
		// xf1.position = m_sweep.c0 - Mul(xf1.R, m_sweep.localCenter);

		// xf1.q.set(m_sweep.a0);
		// Rot.mulToOutUnsafe(xf1.q, m_sweep.localCenter, xf1.p);
		// xf1.p.scale(-1).add(m_sweep.c0);
		// inlined:
		xf1.q.s = (float) Math.sin(m_sweep.a0);
		xf1.q.c = (float) Math.cos(m_sweep.a0);
		xf1.p.x = m_sweep.c0.x - xf1.q.c * m_sweep.localCenter.x + xf1.q.s * m_sweep.localCenter.y;
		xf1.p.y = m_sweep.c0.y - xf1.q.s * m_sweep.localCenter.x - xf1.q.c * m_sweep.localCenter.y;
		// end inline

		for (Fixture fixture : m_fixtureList) {
			fixture.synchronize(getWorld().m_contactManager.m_broadPhase, xf1, m_xf);
		}

	}

	public final void synchronizeTransform() {
		// m_xf.q.set(m_sweep.a);
		//
		// // m_xf.position = m_sweep.c - Mul(m_xf.R, m_sweep.localCenter);
		// Rot.mulToOutUnsafe(m_xf.q, m_sweep.localCenter, m_xf.p);
		// m_xf.p.scale(-1).add(m_sweep.c);
		//
		m_xf.q.s = (float) Math.sin(m_sweep.a);
		m_xf.q.c = (float) Math.cos(m_sweep.a);
		Rot q = m_xf.q;
		Vec2 v = m_sweep.localCenter;
		m_xf.p.x = m_sweep.c.x - q.c * v.x + q.s * v.y;
		m_xf.p.y = m_sweep.c.y - q.s * v.x - q.c * v.y;
	}

	/**
	 * This is used to prevent connected bodies from colliding. It may lie, depending on the collideConnected flag.
	 *
	 * @param other
	 * @return
	 */
	public final boolean shouldCollide(Body other) {
		// At least one body should be dynamic.
		if ((m_type != BodyType.DYNAMIC) && (other.m_type != BodyType.DYNAMIC)) {
			return false;
		}

		// Does a joint prevent collision?
		for (int i_joint_edge = 0; i_joint_edge < m_jointList.size(); ++i_joint_edge) {
			JointEdge jn = m_jointList.get(i_joint_edge);
			if (jn.other == other) {
				if (!jn.joint.getCollideConnected()) {
					return false;
				}
			}
		}

		return true;
	}

	protected final void advance(float t) {
		// Advance to the new safe time. This doesn't sync the broad-phase.
		m_sweep.advance(t);
		m_sweep.c.set(m_sweep.c0);
		m_sweep.a = m_sweep.a0;
		m_xf.q.set(m_sweep.a);
		// m_xf.position = m_sweep.c - Mul(m_xf.R, m_sweep.localCenter);
		Rot.mulToOutUnsafe(m_xf.q, m_sweep.localCenter, m_xf.p);
		m_xf.p.scale(-1).add(m_sweep.c);
	}

	void delete_attached_contacts() {
		// Delete the attached contacts.
		/*
		 * ContactManager.destroy() is removing the items from m_contactList hench the strange while loop
		 *
		 */
		while (!m_contactList.isEmpty()) {
			ContactEdge ce = m_contactList.get(0);
			getWorld().m_contactManager.destroyContact(ce.contact);
		}
	}

//	public Stream<Body> streamBody() {
//		StreamBuilder<Body> builder = new StreamBuilder<>();
//		Set<Body> added = new TreeSet<>();
//		streamBody(added, builder);
//		return builder.build();
//	}
//
//	private void streamBody(Set<Body> added, StreamBuilder<Body> builder) {
//		builder.add(this);
//		added.add(this);
//
//		for (JointEdge jointEdge : m_jointList) {
//			Body other = jointEdge.other;
//			if ((other != null) && !added.contains(other)) {
//				other.streamBody(added, builder);
//			}
//		}
//
//	}
//
//	public Stream<Fixture> streamFixture() {
//		return m_fixtureList.stream();
//	}
}
