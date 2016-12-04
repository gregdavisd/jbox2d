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
 * Created at 11:34:45 AM Jan 23, 2011
 */
package org.jbox2d.dynamics.joints;

import org.jbox2d.common.Rot;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;

import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.SolverData;
import org.jbox2d.pooling.IWorldPool;

//Gear Joint:
//C0 = (coordinate1 + ratio * coordinate2)_initial
//C = (coordinate1 + ratio * coordinate2) - C0 = 0
//J = [J1 ratio * J2]
//K = J * invM * JT
//= J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
//
//Revolute:
//coordinate = rotation
//Cdot = angularVelocity
//J = [0 0 1]
//K = J * invM * JT = invI
//
//Prismatic:
//coordinate = dot(p - pg, ug)
//Cdot = dot(v + cross(w, r), ug)
//J = [ug cross(r, ug)]
//K = J * invM * JT = invMass + invI * cross(r, ug)^2
/**
 * A gear joint is used to connect two joints together. Either joint can be a revolute or prismatic joint. You specify a
 * gear ratio to bind the motions together: coordinate1 + ratio * coordinate2 = constant The ratio can be negative or
 * positive. If one joint is a revolute joint and the other joint is a prismatic joint, then the ratio will have units
 * of length or units of 1/length.
 *
 * @warning The revolute and prismatic joints must be attached to fixed bodies (which must be body1 on those joints).
 * @warning You have to manually destroy the gear joint if joint1 or joint2 is destroyed.
 * @author Daniel Murphy
 */
public class GearJoint extends Joint {

	private final Joint m_joint1;
	private final Joint m_joint2;

	private final JointType m_typeA;
	private final JointType m_typeB;

	// Body A is connected to body C
	// Body B is connected to body D
	private final Body m_bodyC;
	private final Body m_bodyD;

	// Solver shared
	private final Vec2 m_localAnchorA = new Vec2();
	private final Vec2 m_localAnchorB = new Vec2();
	private final Vec2 m_localAnchorC = new Vec2();
	private final Vec2 m_localAnchorD = new Vec2();

	private final Vec2 m_localAxisC = new Vec2();
	private final Vec2 m_localAxisD = new Vec2();

	private float m_referenceAngleA;
	private float m_referenceAngleB;

	private float m_constant;
	private float m_ratio;

	private float m_impulse;

	// Solver temp
	private int m_indexA, m_indexB, m_indexC, m_indexD;
	private final Vec2 m_lcA = new Vec2(), m_lcB = new Vec2(), m_lcC = new Vec2(),
		m_lcD = new Vec2();
	private float m_mA, m_mB, m_mC, m_mD;
	private float m_iA, m_iB, m_iC, m_iD;
	private final Vec2 m_JvAC = new Vec2(), m_JvBD = new Vec2();
	private float m_JwA, m_JwB, m_JwC, m_JwD;
	private float m_mass;

	protected GearJoint(IWorldPool argWorldPool, GearJointDef def) {
		super(argWorldPool, def);

		m_joint1 = def.joint1;
		m_joint2 = def.joint2;

		m_typeA = m_joint1.getType();
		m_typeB = m_joint2.getType();

		assert (m_typeA == JointType.REVOLUTE || m_typeA == JointType.PRISMATIC);
		assert (m_typeB == JointType.REVOLUTE || m_typeB == JointType.PRISMATIC);

		float coordinateA, coordinateB;

		// TODO_ERIN there might be some problem with the joint edges in Joint.
		m_bodyC = m_joint1.getBodyA();
		m_bodyA = m_joint1.getBodyB();

		// Get geometry of joint1
		Transform xfA = m_bodyA.m_xf;
		float aA = m_bodyA.m_sweep.a;
		Transform xfC = m_bodyC.m_xf;
		float aC = m_bodyC.m_sweep.a;

		if (m_typeA == JointType.REVOLUTE) {
			RevoluteJoint revolute = (RevoluteJoint) def.joint1;
			m_localAnchorC.set(revolute.m_localAnchorA);
			m_localAnchorA.set(revolute.m_localAnchorB);
			m_referenceAngleA = revolute.m_referenceAngle;
			m_localAxisC.setZero();

			coordinateA = aA - aC - m_referenceAngleA;
		} else {
			PrismaticJoint prismatic = (PrismaticJoint) def.joint1;
			m_localAnchorC.set(prismatic.m_localAnchorA);
			m_localAnchorA.set(prismatic.m_localAnchorB);
			m_referenceAngleA = prismatic.m_referenceAngle;
			m_localAxisC.set(prismatic.m_localXAxisA);

			Vec2 pA = new Vec2();
			Vec2 temp = new Vec2();
			Vec2 pC = m_localAnchorC;
			Rot.mulToOutUnsafe(xfA.q, m_localAnchorA, temp);
			temp.add(xfA.p).sub(xfC.p);
			Rot.mulTransUnsafe(xfC.q, temp, pA);
			coordinateA = ((Vec2) pA.sub(pC)).dot(m_localAxisC);
		}

		m_bodyD = m_joint2.getBodyA();
		m_bodyB = m_joint2.getBodyB();

		// Get geometry of joint2
		Transform xfB = m_bodyB.m_xf;
		float aB = m_bodyB.m_sweep.a;
		Transform xfD = m_bodyD.m_xf;
		float aD = m_bodyD.m_sweep.a;

		if (m_typeB == JointType.REVOLUTE) {
			RevoluteJoint revolute = (RevoluteJoint) def.joint2;
			m_localAnchorD.set(revolute.m_localAnchorA);
			m_localAnchorB.set(revolute.m_localAnchorB);
			m_referenceAngleB = revolute.m_referenceAngle;
			m_localAxisD.setZero();

			coordinateB = aB - aD - m_referenceAngleB;
		} else {
			PrismaticJoint prismatic = (PrismaticJoint) def.joint2;
			m_localAnchorD.set(prismatic.m_localAnchorA);
			m_localAnchorB.set(prismatic.m_localAnchorB);
			m_referenceAngleB = prismatic.m_referenceAngle;
			m_localAxisD.set(prismatic.m_localXAxisA);

			Vec2 pB = new Vec2();
			Vec2 temp = new Vec2();
			Vec2 pD = m_localAnchorD;
			Rot.mulToOutUnsafe(xfB.q, m_localAnchorB, temp);
			temp.add(xfB.p).sub(xfD.p);
			Rot.mulTransUnsafe(xfD.q, temp, pB);
			coordinateB = pB.sub(pD).dot(m_localAxisD);
		}

		m_ratio = def.ratio;

		m_constant = coordinateA + m_ratio * coordinateB;

		m_impulse = 0.0f;
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
		argOut.set(m_JvAC).scale(m_impulse);
		argOut.scale(inv_dt);
	}

	@Override
	public float getReactionTorque(float inv_dt) {
		float L = m_impulse * m_JwA;
		return inv_dt * L;
	}

	public void setRatio(float argRatio) {
		m_ratio = argRatio;
	}

	public float getRatio() {
		return m_ratio;
	}

	@Override
	public void initVelocityConstraints(SolverData data) {
		m_indexA = m_bodyA.m_islandIndex;
		m_indexB = m_bodyB.m_islandIndex;
		m_indexC = m_bodyC.m_islandIndex;
		m_indexD = m_bodyD.m_islandIndex;
		m_lcA.set(m_bodyA.m_sweep.localCenter);
		m_lcB.set(m_bodyB.m_sweep.localCenter);
		m_lcC.set(m_bodyC.m_sweep.localCenter);
		m_lcD.set(m_bodyD.m_sweep.localCenter);
		m_mA = m_bodyA.m_invMass;
		m_mB = m_bodyB.m_invMass;
		m_mC = m_bodyC.m_invMass;
		m_mD = m_bodyD.m_invMass;
		m_iA = m_bodyA.m_invI;
		m_iB = m_bodyB.m_invI;
		m_iC = m_bodyC.m_invI;
		m_iD = m_bodyD.m_invI;

		// Vec2 cA = data.positions[m_indexA].c;
		float aA = data.positions[m_indexA].a;
		Vec2 vA = data.velocities[m_indexA].v;
		float wA = data.velocities[m_indexA].w;

		// Vec2 cB = data.positions[m_indexB].c;
		float aB = data.positions[m_indexB].a;
		Vec2 vB = data.velocities[m_indexB].v;
		float wB = data.velocities[m_indexB].w;

		// Vec2 cC = data.positions[m_indexC].c;
		float aC = data.positions[m_indexC].a;
		Vec2 vC = data.velocities[m_indexC].v;
		float wC = data.velocities[m_indexC].w;

		// Vec2 cD = data.positions[m_indexD].c;
		float aD = data.positions[m_indexD].a;
		Vec2 vD = data.velocities[m_indexD].v;
		float wD = data.velocities[m_indexD].w;

		Rot qA = new Rot(), qB = new Rot(), qC = new Rot(), qD = new Rot();
		qA.set(aA);
		qB.set(aB);
		qC.set(aC);
		qD.set(aD);

		m_mass = 0.0f;

		Vec2 temp = new Vec2();

		if (m_typeA == JointType.REVOLUTE) {
			m_JvAC.setZero();
			m_JwA = 1.0f;
			m_JwC = 1.0f;
			m_mass += m_iA + m_iC;
		} else {
			Vec2 rC = new Vec2();
			Vec2 rA = new Vec2();
			Rot.mulToOutUnsafe(qC, m_localAxisC, m_JvAC);
			Rot.mulToOutUnsafe(qC, (Vec2) temp.set(m_localAnchorC).sub(m_lcC), rC);
			Rot.mulToOutUnsafe(qA, (Vec2) temp.set(m_localAnchorA).sub(m_lcA), rA);
			m_JwC = rC.cross(m_JvAC);
			m_JwA = rA.cross(m_JvAC);
			m_mass += m_mC + m_mA + m_iC * m_JwC * m_JwC + m_iA * m_JwA * m_JwA;
		}

		if (m_typeB == JointType.REVOLUTE) {
			m_JvBD.setZero();
			m_JwB = m_ratio;
			m_JwD = m_ratio;
			m_mass += m_ratio * m_ratio * (m_iB + m_iD);
		} else {
			Vec2 u = new Vec2();
			Vec2 rD = new Vec2();
			Vec2 rB = new Vec2();
			Rot.mulToOutUnsafe(qD, m_localAxisD, u);
			Rot.mulToOutUnsafe(qD, (Vec2) temp.set(m_localAnchorD).sub(m_lcD), rD);
			Rot.mulToOutUnsafe(qB, (Vec2) temp.set(m_localAnchorB).sub(m_lcB), rB);
			m_JvBD.set(u).scale(m_ratio);
			m_JwD = m_ratio * rD.cross(u);
			m_JwB = m_ratio * rB.cross(u);
			m_mass += m_ratio * m_ratio * (m_mD + m_mB) + m_iD * m_JwD * m_JwD + m_iB * m_JwB * m_JwB;
		}

		// Compute effective mass.
		m_mass = m_mass > 0.0f ? 1.0f / m_mass : 0.0f;

		if (data.step.warmStarting) {
			vA.x += (m_mA * m_impulse) * m_JvAC.x;
			vA.y += (m_mA * m_impulse) * m_JvAC.y;
			wA += m_iA * m_impulse * m_JwA;

			vB.x += (m_mB * m_impulse) * m_JvBD.x;
			vB.y += (m_mB * m_impulse) * m_JvBD.y;
			wB += m_iB * m_impulse * m_JwB;

			vC.x -= (m_mC * m_impulse) * m_JvAC.x;
			vC.y -= (m_mC * m_impulse) * m_JvAC.y;
			wC -= m_iC * m_impulse * m_JwC;

			vD.x -= (m_mD * m_impulse) * m_JvBD.x;
			vD.y -= (m_mD * m_impulse) * m_JvBD.y;
			wD -= m_iD * m_impulse * m_JwD;
		} else {
			m_impulse = 0.0f;
		}

		// data.velocities[m_indexA].v = vA;
		data.velocities[m_indexA].w = wA;
		// data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
		// data.velocities[m_indexC].v = vC;
		data.velocities[m_indexC].w = wC;
		// data.velocities[m_indexD].v = vD;
		data.velocities[m_indexD].w = wD;
	}

	@Override
	public void solveVelocityConstraints(SolverData data) {
		Vec2 vA = data.velocities[m_indexA].v;
		float wA = data.velocities[m_indexA].w;
		Vec2 vB = data.velocities[m_indexB].v;
		float wB = data.velocities[m_indexB].w;
		Vec2 vC = data.velocities[m_indexC].v;
		float wC = data.velocities[m_indexC].w;
		Vec2 vD = data.velocities[m_indexD].v;
		float wD = data.velocities[m_indexD].w;

		float Cdot =
			m_JvAC.dot((Vec2) new Vec2(vA).sub(vC)) + m_JvBD.dot((Vec2) new Vec2(vB).sub(vD));
		Cdot += (m_JwA * wA - m_JwC * wC) + (m_JwB * wB - m_JwD * wD);

		float impulse = -m_mass * Cdot;
		m_impulse += impulse;

		vA.x += (m_mA * impulse) * m_JvAC.x;
		vA.y += (m_mA * impulse) * m_JvAC.y;
		wA += m_iA * impulse * m_JwA;

		vB.x += (m_mB * impulse) * m_JvBD.x;
		vB.y += (m_mB * impulse) * m_JvBD.y;
		wB += m_iB * impulse * m_JwB;

		vC.x -= (m_mC * impulse) * m_JvAC.x;
		vC.y -= (m_mC * impulse) * m_JvAC.y;
		wC -= m_iC * impulse * m_JwC;

		vD.x -= (m_mD * impulse) * m_JvBD.x;
		vD.y -= (m_mD * impulse) * m_JvBD.y;
		wD -= m_iD * impulse * m_JwD;

		// data.velocities[m_indexA].v = vA;
		data.velocities[m_indexA].w = wA;
		// data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
		// data.velocities[m_indexC].v = vC;
		data.velocities[m_indexC].w = wC;
		// data.velocities[m_indexD].v = vD;
		data.velocities[m_indexD].w = wD;
	}

	public Joint getJoint1() {
		return m_joint1;
	}

	public Joint getJoint2() {
		return m_joint2;
	}

	@Override
	public boolean solvePositionConstraints(SolverData data) {
		Vec2 cA = data.positions[m_indexA].c;
		float aA = data.positions[m_indexA].a;
		Vec2 cB = data.positions[m_indexB].c;
		float aB = data.positions[m_indexB].a;
		Vec2 cC = data.positions[m_indexC].c;
		float aC = data.positions[m_indexC].a;
		Vec2 cD = data.positions[m_indexD].c;
		float aD = data.positions[m_indexD].a;

		Rot qA = new Rot(), qB = new Rot(), qC = new Rot(), qD = new Rot();
		qA.set(aA);
		qB.set(aB);
		qC.set(aC);
		qD.set(aD);

		float linearError = 0.0f;

		float coordinateA, coordinateB;

		Vec2 temp = new Vec2();
		Vec2 JvAC = new Vec2();
		Vec2 JvBD = new Vec2();
		float JwA, JwB, JwC, JwD;
		float mass = 0.0f;

		if (m_typeA == JointType.REVOLUTE) {
			JvAC.setZero();
			JwA = 1.0f;
			JwC = 1.0f;
			mass += m_iA + m_iC;

			coordinateA = aA - aC - m_referenceAngleA;
		} else {
			Vec2 rC = new Vec2();
			Vec2 rA = new Vec2();
			Vec2 pC = new Vec2();
			Vec2 pA = new Vec2();
			Rot.mulToOutUnsafe(qC, m_localAxisC, JvAC);
			Rot.mulToOutUnsafe(qC, (Vec2) temp.set(m_localAnchorC).sub(m_lcC), rC);
			Rot.mulToOutUnsafe(qA, (Vec2) temp.set(m_localAnchorA).sub(m_lcA), rA);
			JwC = rC.cross(JvAC);
			JwA = rA.cross(JvAC);
			mass += m_mC + m_mA + m_iC * JwC * JwC + m_iA * JwA * JwA;

			pC.set(m_localAnchorC).sub(m_lcC);
			Rot.mulTransUnsafe(qC, (Vec2) temp.set(rA).add(cA).sub(cC), pA);
			coordinateA = ((Vec2) pA.sub(pC)).dot(m_localAxisC);
		}

		if (m_typeB == JointType.REVOLUTE) {
			JvBD.setZero();
			JwB = m_ratio;
			JwD = m_ratio;
			mass += m_ratio * m_ratio * (m_iB + m_iD);

			coordinateB = aB - aD - m_referenceAngleB;
		} else {
			Vec2 u = new Vec2();
			Vec2 rD = new Vec2();
			Vec2 rB = new Vec2();
			Vec2 pD = new Vec2();
			Vec2 pB = new Vec2();
			Rot.mulToOutUnsafe(qD, m_localAxisD, u);
			Rot.mulToOutUnsafe(qD, (Vec2) temp.set(m_localAnchorD).sub(m_lcD), rD);
			Rot.mulToOutUnsafe(qB, (Vec2) temp.set(m_localAnchorB).sub(m_lcB), rB);
			JvBD.set(u).scale(m_ratio);
			JwD = rD.cross(u);
			JwB = rB.cross(u);
			mass += m_ratio * m_ratio * (m_mD + m_mB) + m_iD * JwD * JwD + m_iB * JwB * JwB;

			pD.set(m_localAnchorD).sub(m_lcD);
			Rot.mulTransUnsafe(qD, (Vec2) temp.set(rB).add(cB).sub(cD), pB);
			coordinateB = ((Vec2) pB.sub(pD)).dot(m_localAxisD);
		}

		float C = (coordinateA + m_ratio * coordinateB) - m_constant;

		float impulse = 0.0f;
		if (mass > 0.0f) {
			impulse = -C / mass;
		}

		cA.x += (m_mA * impulse) * JvAC.x;
		cA.y += (m_mA * impulse) * JvAC.y;
		aA += m_iA * impulse * JwA;

		cB.x += (m_mB * impulse) * JvBD.x;
		cB.y += (m_mB * impulse) * JvBD.y;
		aB += m_iB * impulse * JwB;

		cC.x -= (m_mC * impulse) * JvAC.x;
		cC.y -= (m_mC * impulse) * JvAC.y;
		aC -= m_iC * impulse * JwC;

		cD.x -= (m_mD * impulse) * JvBD.x;
		cD.y -= (m_mD * impulse) * JvBD.y;
		aD -= m_iD * impulse * JwD;

		// data.positions[m_indexA].c = cA;
		data.positions[m_indexA].a = aA;
		// data.positions[m_indexB].c = cB;
		data.positions[m_indexB].a = aB;
		// data.positions[m_indexC].c = cC;
		data.positions[m_indexC].a = aC;
		// data.positions[m_indexD].c = cD;
		data.positions[m_indexD].a = aD;

		// TODO_ERIN not implemented
		return linearError < Settings.linearSlop;
	}
}
