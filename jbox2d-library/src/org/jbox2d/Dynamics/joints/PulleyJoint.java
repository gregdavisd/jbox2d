/**
 * *****************************************************************************
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
 *****************************************************************************
 */
/**
 * Created at 12:12:02 PM Jan 23, 2011
 */
package org.jbox2d.dynamics.joints;

import java.io.Serializable;
import org.jbox2d.common.Rot;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.SolverData;
import org.jbox2d.pooling.IWorldPool;

/**
 * The pulley joint is connected to two bodies and two fixed ground points. The pulley supports a
 * ratio such that: length1 + ratio * length2 <= constant Yes, the force transmitted is scaled by
 * the ratio. Warning: the pulley joint can get a bit squirrelly by itself. They often work better
 * when combined with prismatic joints. You should also cover the the anchor points with static
 * shapes to prevent one side from going to zero length.
 *
 * @author Daniel Murphy
 */
public class PulleyJoint extends Joint implements Serializable {

 static final long serialVersionUID = 1L;
 public static final float MIN_PULLEY_LENGTH = 2.0f;
 private final Vec2 m_groundAnchorA = new Vec2();
 private final Vec2 m_groundAnchorB = new Vec2();
 private float m_lengthA;
 private float m_lengthB;
 // Solver shared
 private final Vec2 m_localAnchorA = new Vec2();
 private final Vec2 m_localAnchorB = new Vec2();
 private float m_constant;
 private float m_ratio;
 private float m_impulse;
 // Solver temp
 private int m_indexA;
 private int m_indexB;
 private final Vec2 m_uA = new Vec2();
 private final Vec2 m_uB = new Vec2();
 private final Vec2 m_rA = new Vec2();
 private final Vec2 m_rB = new Vec2();
 private final Vec2 m_localCenterA = new Vec2();
 private final Vec2 m_localCenterB = new Vec2();
 private float m_invMassA;
 private float m_invMassB;
 private float m_invIA;
 private float m_invIB;
 private float m_mass;

 protected PulleyJoint(IWorldPool argWorldPool, PulleyJointDef def) {
  super(argWorldPool, def);
  m_groundAnchorA.set(def.groundAnchorA);
  m_groundAnchorB.set(def.groundAnchorB);
  m_localAnchorA.set(def.localAnchorA);
  m_localAnchorB.set(def.localAnchorB);
  assert (def.ratio != 0.0f);
  m_ratio = def.ratio;
  m_lengthA = def.lengthA;
  m_lengthB = def.lengthB;
  m_constant = def.lengthA + m_ratio * def.lengthB;
  m_impulse = 0.0f;
 }

 public float getLengthA() {
  return m_lengthA;
 }

 public float getLengthB() {
  return m_lengthB;
 }

 public float getCurrentLengthA() {
  final Vec2 p = new Vec2();
  m_bodyA.getWorldPointToOut(m_localAnchorA, p);
  p.sub(m_groundAnchorA);
  float length = p.length();
  return length;
 }

 public float getCurrentLengthB() {
  final Vec2 p = new Vec2();
  m_bodyB.getWorldPointToOut(m_localAnchorB, p);
  p.sub(m_groundAnchorB);
  float length = p.length();
  return length;
 }

 public Vec2 getLocalAnchorA() {
  return m_localAnchorA;
 }

 public Vec2 getLocalAnchorB() {
  return m_localAnchorB;
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
  argOut.set(m_uB).scale(m_impulse).scale(inv_dt);
 }

 @Override
 public float getReactionTorque(float inv_dt) {
  return 0f;
 }

 public Vec2 getGroundAnchorA() {
  return m_groundAnchorA;
 }

 public Vec2 getGroundAnchorB() {
  return m_groundAnchorB;
 }

 public float getLength1() {
  final Vec2 p = new Vec2();
  m_bodyA.getWorldPointToOut(m_localAnchorA, p);
  p.sub(m_groundAnchorA);
  float len = p.length();
  return len;
 }

 public float getLength2() {
  final Vec2 p = new Vec2();
  m_bodyB.getWorldPointToOut(m_localAnchorB, p);
  p.sub(m_groundAnchorB);
  float len = p.length();
  return len;
 }

 public float getRatio() {
  return m_ratio;
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
  Rot.mulToOutUnsafe(qA, temp.set(m_localAnchorA).sub(m_localCenterA), m_rA);
  Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).sub(m_localCenterB), m_rB);
  m_uA.set(cA).add(m_rA).sub(m_groundAnchorA);
  m_uB.set(cB).add(m_rB).sub(m_groundAnchorB);
  float lengthA = m_uA.length();
  float lengthB = m_uB.length();
  if (lengthA > 10f * Settings.linearSlop) {
   m_uA.scale(1.0f / lengthA);
  } else {
   m_uA.setZero();
  }
  if (lengthB > 10f * Settings.linearSlop) {
   m_uB.scale(1.0f / lengthB);
  } else {
   m_uB.setZero();
  }
  // Compute effective mass.
  float ruA = m_rA.cross(m_uA);
  float ruB = m_rB.cross(m_uB);
  float mA = m_invMassA + m_invIA * ruA * ruA;
  float mB = m_invMassB + m_invIB * ruB * ruB;
  m_mass = mA + m_ratio * m_ratio * mB;
  if (m_mass > 0.0f) {
   m_mass = 1.0f / m_mass;
  }
  if (data.step.warmStarting) {
   // Scale impulses to support variable time steps.
   m_impulse *= data.step.dtRatio;
   // Warm starting.
   final Vec2 PA = new Vec2();
   final Vec2 PB = new Vec2();
   PA.set(m_uA).scale(-m_impulse);
   PB.set(m_uB).scale(-m_ratio * m_impulse);
   vA.x += m_invMassA * PA.x;
   vA.y += m_invMassA * PA.y;
   wA += m_invIA * m_rA.cross(PA);
   vB.x += m_invMassB * PB.x;
   vB.y += m_invMassB * PB.y;
   wB += m_invIB * m_rB.cross(PB);
  } else {
   m_impulse = 0.0f;
  }
//    data.velocities[m_indexA].v.set(vA);
  data.velocities[m_indexA].w = wA;
//    data.velocities[m_indexB].v.set(vB);
  data.velocities[m_indexB].w = wB;
 }

 @Override
 public void solveVelocityConstraints(final SolverData data) {
  Vec2 vA = data.velocities[m_indexA].v;
  float wA = data.velocities[m_indexA].w;
  Vec2 vB = data.velocities[m_indexB].v;
  float wB = data.velocities[m_indexB].w;
  final Vec2 vpA = new Vec2();
  final Vec2 vpB = new Vec2();
  final Vec2 PA = new Vec2();
  final Vec2 PB = new Vec2();
  vpA.set(m_rA).setRightPerpendicular(wA);
  vpA.add(vA);
  vpB.set(m_rB).setRightPerpendicular(wB);
  vpB.add(vB);
  float Cdot = -m_uA.dot(vpA) - m_ratio * m_uB.dot(vpB);
  float impulse = -m_mass * Cdot;
  m_impulse += impulse;
  PA.set(m_uA).scale(-impulse);
  PB.set(m_uB).scale(-m_ratio * impulse);
  vA.x += m_invMassA * PA.x;
  vA.y += m_invMassA * PA.y;
  wA += m_invIA * m_rA.cross(PA);
  vB.x += m_invMassB * PB.x;
  vB.y += m_invMassB * PB.y;
  wB += m_invIB * m_rB.cross(PB);
//    data.velocities[m_indexA].v.set(vA);
  data.velocities[m_indexA].w = wA;
//    data.velocities[m_indexB].v.set(vB);
  data.velocities[m_indexB].w = wB;
 }

 @Override
 public boolean solvePositionConstraints(final SolverData data) {
  final Rot qA = new Rot();
  final Rot qB = new Rot();
  final Vec2 rA = new Vec2();
  final Vec2 rB = new Vec2();
  final Vec2 uA = new Vec2();
  final Vec2 uB = new Vec2();
  final Vec2 temp = new Vec2();
  final Vec2 PA = new Vec2();
  final Vec2 PB = new Vec2();
  Vec2 cA = data.positions[m_indexA].c;
  float aA = data.positions[m_indexA].a;
  Vec2 cB = data.positions[m_indexB].c;
  float aB = data.positions[m_indexB].a;
  qA.set(aA);
  qB.set(aB);
  Rot.mulToOutUnsafe(qA, temp.set(m_localAnchorA).sub(m_localCenterA), rA);
  Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).sub(m_localCenterB), rB);
  uA.set(cA).add(rA).sub(m_groundAnchorA);
  uB.set(cB).add(rB).sub(m_groundAnchorB);
  float lengthA = uA.length();
  float lengthB = uB.length();
  if (lengthA > 10.0f * Settings.linearSlop) {
   uA.scale(1.0f / lengthA);
  } else {
   uA.setZero();
  }
  if (lengthB > 10.0f * Settings.linearSlop) {
   uB.scale(1.0f / lengthB);
  } else {
   uB.setZero();
  }
  // Compute effective mass.
  float ruA = rA.cross(uA);
  float ruB = rB.cross(uB);
  float mA = m_invMassA + m_invIA * ruA * ruA;
  float mB = m_invMassB + m_invIB * ruB * ruB;
  float mass = mA + m_ratio * m_ratio * mB;
  if (mass > 0.0f) {
   mass = 1.0f / mass;
  }
  float C = m_constant - lengthA - m_ratio * lengthB;
  float linearError = Math.abs(C);
  float impulse = -mass * C;
  PA.set(uA).scale(-impulse);
  PB.set(uB).scale(-m_ratio * impulse);
  cA.x += m_invMassA * PA.x;
  cA.y += m_invMassA * PA.y;
  aA += m_invIA * rA.cross(PA);
  cB.x += m_invMassB * PB.x;
  cB.y += m_invMassB * PB.y;
  aB += m_invIB * rB.cross(PB);
//    data.positions[m_indexA].c.set(cA);
  data.positions[m_indexA].a = aA;
//    data.positions[m_indexB].c.set(cB);
  data.positions[m_indexB].a = aB;
  return linearError < Settings.linearSlop;
 }
}
