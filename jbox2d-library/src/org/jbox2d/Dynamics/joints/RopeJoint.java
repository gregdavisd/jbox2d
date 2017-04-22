package org.jbox2d.dynamics.joints;

import java.io.Serializable;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Rot;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;

import org.jbox2d.dynamics.SolverData;
import org.jbox2d.pooling.IWorldPool;

/**
 * A rope joint enforces a maximum distance between two points on two bodies. It has no other effect. Warning: if you attempt to
 * change the maximum length during the simulation you will get some non-physical behavior. A model that would allow you to
 * dynamically modify the length would have some sponginess, so I chose not to implement it that way. See DistanceJoint if you
 * want to dynamically control length.
 *
 * @author Daniel Murphy
 */
public class RopeJoint extends Joint implements Serializable {

	static final long serialVersionUID = 1L;
	// Solver shared

	private final Vec2 m_localAnchorA = new Vec2();
	private final Vec2 m_localAnchorB = new Vec2();
	private float m_maxLength;
	private float m_length;
	private float m_impulse;

	// Solver temp
	private int m_indexA;
	private int m_indexB;
	private final Vec2 m_u = new Vec2();
	private final Vec2 m_rA = new Vec2();
	private final Vec2 m_rB = new Vec2();
	private final Vec2 m_localCenterA = new Vec2();
	private final Vec2 m_localCenterB = new Vec2();
	private float m_invMassA;
	private float m_invMassB;
	private float m_invIA;
	private float m_invIB;
	private float m_mass;
	private byte m_state;

	protected RopeJoint(IWorldPool worldPool, RopeJointDef def) {
		super(worldPool, def);
		m_localAnchorA.set(def.localAnchorA);
		m_localAnchorB.set(def.localAnchorB);

		m_maxLength = def.maxLength;

		m_mass = 0.0f;
		m_impulse = 0.0f;
		m_state = LimitState.INACTIVE;
		m_length = 0.0f;
	}

	@Override
	public void initVelocityConstraints(final SolverData data) {
		m_indexA = m_bodyA.m_islandIndex;
		m_indexB = m_bodyB.m_islandIndex;
		m_localCenterA.set(m_bodyA.m_sweep.localCenter);
		m_localCenterB.set(m_bodyB.m_sweep.localCenter);
		m_invMassA = m_bodyA.m_invMass;
		m_invMassB = m_bodyB.m_invMass;
		m_invIA = m_bodyA.m_invI;
		m_invIB = m_bodyB.m_invI;

		Vec2 cA = data.positions[m_indexA].c;
		float aA = data.positions[m_indexA].a;
		Vec2 vA = data.velocities[m_indexA].v;
		float wA = data.velocities[m_indexA].w;

		Vec2 cB = data.positions[m_indexB].c;
		float aB = data.positions[m_indexB].a;
		Vec2 vB = data.velocities[m_indexB].v;
		float wB = data.velocities[m_indexB].w;

		final Rot qA = new Rot();
		final Rot qB = new Rot();
		final Vec2 temp = new Vec2();

		qA.set(aA);
		qB.set(aB);

		// Compute the effective masses.
		Rot.mulToOutUnsafe(qA, (Vec2) temp.set(m_localAnchorA).sub(m_localCenterA), m_rA);
		Rot.mulToOutUnsafe(qB, (Vec2) temp.set(m_localAnchorB).sub(m_localCenterB), m_rB);

		m_u.set(cB).add(m_rB).sub(cA).sub(m_rA);

		m_length = m_u.length();

		float C = m_length - m_maxLength;
		if (C > 0.0f) {
			m_state = LimitState.AT_UPPER;
		} else {
			m_state = LimitState.INACTIVE;
		}

		if (m_length > Settings.linearSlop) {
			m_u.scale(1.0f / m_length);
		} else {
			m_u.setZero();
			m_mass = 0.0f;
			m_impulse = 0.0f;
			return;
		}

		// Compute effective mass.
		float crA = m_rA.cross(m_u);
		float crB = m_rB.cross(m_u);
		float invMass = m_invMassA + m_invIA * crA * crA + m_invMassB + m_invIB * crB * crB;

		m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

		if (data.step.warmStarting) {
			// Scale the impulse to support a variable time step.
			m_impulse *= data.step.dtRatio;

			float Px = m_impulse * m_u.x;
			float Py = m_impulse * m_u.y;
			vA.x -= m_invMassA * Px;
			vA.y -= m_invMassA * Py;
			wA -= m_invIA * (m_rA.x * Py - m_rA.y * Px);

			vB.x += m_invMassB * Px;
			vB.y += m_invMassB * Py;
			wB += m_invIB * (m_rB.x * Py - m_rB.y * Px);
		} else {
			m_impulse = 0.0f;
		}

		// data.velocities[m_indexA].v = vA;
		data.velocities[m_indexA].w = wA;
		// data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
	}

	@Override
	public void solveVelocityConstraints(final SolverData data) {
		Vec2 vA = data.velocities[m_indexA].v;
		float wA = data.velocities[m_indexA].w;
		Vec2 vB = data.velocities[m_indexB].v;
		float wB = data.velocities[m_indexB].w;

		// Cdot = dot(u, v + cross(w, r))
		Vec2 vpA = new Vec2();
		Vec2 vpB = new Vec2();
		Vec2 temp = new Vec2();

		vpA.set(m_rA).setRightPerpendicular(wA);
		vpA.add(vA);
		vpB.set(m_rB).setRightPerpendicular(wB);
		vpB.add(vB);

		float C = m_length - m_maxLength;
		float Cdot = m_u.dot(temp.set(vpB).sub(vpA));

		// Predictive constraint.
		if (C < 0.0f) {
			Cdot += data.step.inv_dt * C;
		}

		float impulse = -m_mass * Cdot;
		float oldImpulse = m_impulse;
		m_impulse = Math.min(0.0f, m_impulse + impulse);
		impulse = m_impulse - oldImpulse;

		float Px = impulse * m_u.x;
		float Py = impulse * m_u.y;
		vA.x -= m_invMassA * Px;
		vA.y -= m_invMassA * Py;
		wA -= m_invIA * (m_rA.x * Py - m_rA.y * Px);
		vB.x += m_invMassB * Px;
		vB.y += m_invMassB * Py;
		wB += m_invIB * (m_rB.x * Py - m_rB.y * Px);

		// data.velocities[m_indexA].v = vA;
		data.velocities[m_indexA].w = wA;
		// data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
	}

	@Override
	public boolean solvePositionConstraints(final SolverData data) {
		Vec2 cA = data.positions[m_indexA].c;
		float aA = data.positions[m_indexA].a;
		Vec2 cB = data.positions[m_indexB].c;
		float aB = data.positions[m_indexB].a;

		final Rot qA = new Rot();
		final Rot qB = new Rot();
		final Vec2 u = new Vec2();
		final Vec2 rA = new Vec2();
		final Vec2 rB = new Vec2();
		final Vec2 temp = new Vec2();

		qA.set(aA);
		qB.set(aB);

		// Compute the effective masses.
		Rot.mulToOutUnsafe(qA, (Vec2) temp.set(m_localAnchorA).sub(m_localCenterA), rA);
		Rot.mulToOutUnsafe(qB, (Vec2) temp.set(m_localAnchorB).sub(m_localCenterB), rB);
		u.set(cB).add(rB).sub(cA).sub(rA);

		float length = u.length();
		u.scale(1.0f / length);
		float C = length - m_maxLength;

		C = MathUtils.clamp(C, 0.0f, Settings.maxLinearCorrection);

		float impulse = -m_mass * C;
		float Px = impulse * u.x;
		float Py = impulse * u.y;

		cA.x -= m_invMassA * Px;
		cA.y -= m_invMassA * Py;
		aA -= m_invIA * (rA.x * Py - rA.y * Px);
		cB.x += m_invMassB * Px;
		cB.y += m_invMassB * Py;
		aB += m_invIB * (rB.x * Py - rB.y * Px);

		// data.positions[m_indexA].c = cA;
		data.positions[m_indexA].a = aA;
		// data.positions[m_indexB].c = cB;
		data.positions[m_indexB].a = aB;

		return length - m_maxLength < Settings.linearSlop;
	}

	@Override
	public void getAnchorA(Vec2 argOut) {
		m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
	}

	@Override
	public void getAnchorB(Vec2 argOut) {
		m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
	}

	@Override
	public void getReactionForce(float inv_dt, Vec2 argOut) {
		argOut.set(m_u).scale(inv_dt).scale(m_impulse);
	}

	@Override
	public float getReactionTorque(float inv_dt) {
		return 0f;
	}

	public Vec2 getLocalAnchorA() {
		return m_localAnchorA;
	}

	public Vec2 getLocalAnchorB() {
		return m_localAnchorB;
	}

	public float getMaxLength() {
		return m_maxLength;
	}

	public void setMaxLength(float maxLength) {
		this.m_maxLength = maxLength;
	}

	public int getLimitState() {
		return m_state;
	}

}
