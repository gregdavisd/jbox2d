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
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.DistanceJointDef;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

public class DominoTest extends TestbedTest {

 @Override
 public boolean isSaveLoadEnabled() {
  return true;
 }

 public void initTest(boolean argDeserialized) {
  if (argDeserialized) {
   return;
  }
  Body b1;
  {
   EdgeShape shape = new EdgeShape();
   shape.set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
   BodyDef bd = new BodyDef();
   b1 = m_world.createBody(bd);
   b1.createFixture(shape, 0.0f);
  }
  {
   PolygonShape shape = new PolygonShape();
   shape.setAsBox(6.0f, 0.25f);
   BodyDef bd = new BodyDef();
   bd.position.set(-1.5f, 10.0f);
   Body ground = m_world.createBody(bd);
   ground.createFixture(shape, 0.0f);
  }
  {
   PolygonShape shape = new PolygonShape();
   shape.setAsBox(0.1f, 1.0f);
   FixtureDef fd = new FixtureDef();
   fd.shape = shape;
   fd.density = 2000.0f;
   fd.friction = 0.1f;
   for (int i = 0; i < 10; ++i) {
    BodyDef bd = new BodyDef();
    bd.type = BodyType.DYNAMIC;
    bd.position.set(-6.0f + 1.0f * i, 11.25f);
    Body body = m_world.createBody(bd);
    body.createFixture(fd);
   }
  }
  {
   PolygonShape shape = new PolygonShape();
   shape.setAsBox(7.0f, 0.25f, new Vec2(), 0.3f);
   BodyDef bd = new BodyDef();
   bd.position.set(1.0f, 6.0f);
   Body ground = m_world.createBody(bd);
   ground.createFixture(shape, 0.0f);
  }
  Body b2;
  {
   PolygonShape shape = new PolygonShape();
   shape.setAsBox(0.25f, 1.5f);
   BodyDef bd = new BodyDef();
   bd.position.set(-7.0f, 4.0f);
   b2 = m_world.createBody(bd);
   b2.createFixture(shape, 0.0f);
  }
  Body b3;
  {
   PolygonShape shape = new PolygonShape();
   shape.setAsBox(6.0f, 0.125f);
   BodyDef bd = new BodyDef();
   bd.type = BodyType.DYNAMIC;
   bd.position.set(-0.9f, 1.0f);
   bd.angle = -0.15f;
   b3 = m_world.createBody(bd);
   b3.createFixture(shape, 10.0f);
  }
  RevoluteJointDef jd = new RevoluteJointDef();
  Vec2 anchor = new Vec2();
  anchor.set(-2.0f, 1.0f);
  jd.initialize(b1, b3, anchor);
  jd.collideConnected = true;
  m_world.createJoint(jd);
  Body b4;
  {
   PolygonShape shape = new PolygonShape();
   shape.setAsBox(0.25f, 0.25f);
   BodyDef bd = new BodyDef();
   bd.type = BodyType.DYNAMIC;
   bd.position.set(-10.0f, 15.0f);
   b4 = m_world.createBody(bd);
   b4.createFixture(shape, 10.0f);
  }
  anchor.set(-7.0f, 15.0f);
  jd.initialize(b2, b4, anchor);
  m_world.createJoint(jd);
  Body b5;
  {
   BodyDef bd = new BodyDef();
   bd.type = BodyType.DYNAMIC;
   bd.position.set(6.5f, 3.0f);
   b5 = m_world.createBody(bd);
   PolygonShape shape = new PolygonShape();
   FixtureDef fd = new FixtureDef();
   fd.shape = shape;
   fd.density = 10.0f;
   fd.friction = 0.1f;
   shape.setAsBox(1.0f, 0.1f, new Vec2(0.0f, -0.9f), 0.0f);
   b5.createFixture(fd);
   shape.setAsBox(0.1f, 1.0f, new Vec2(-0.9f, 0.0f), 0.0f);
   b5.createFixture(fd);
   shape.setAsBox(0.1f, 1.0f, new Vec2(0.9f, 0.0f), 0.0f);
   b5.createFixture(fd);
  }
  anchor.set(6.0f, 2.0f);
  jd.initialize(b1, b5, anchor);
  m_world.createJoint(jd);
  Body b6;
  {
   PolygonShape shape = new PolygonShape();
   shape.setAsBox(1.0f, 0.1f);
   BodyDef bd = new BodyDef();
   bd.type = BodyType.DYNAMIC;
   bd.position.set(6.5f, 4.1f);
   b6 = m_world.createBody(bd);
   b6.createFixture(shape, 30.0f);
  }
  anchor.set(7.5f, 4.0f);
  jd.initialize(b5, b6, anchor);
  m_world.createJoint(jd);
  Body b7;
  {
   PolygonShape shape = new PolygonShape();
   shape.setAsBox(0.1f, 1.0f);
   BodyDef bd = new BodyDef();
   bd.type = BodyType.DYNAMIC;
   bd.position.set(7.4f, 1.0f);
   b7 = m_world.createBody(bd);
   b7.createFixture(shape, 10.0f);
  }
  DistanceJointDef djd = new DistanceJointDef();
  djd.bodyA = b3;
  djd.bodyB = b7;
  djd.localAnchorA.set(6.0f, 0.0f);
  djd.localAnchorB.set(0.0f, -1.0f);
  Vec2 d = (Vec2) djd.bodyB.getWorldPoint(djd.localAnchorB).sub(djd.bodyA.getWorldPoint(
   djd.localAnchorA));
  djd.length = d.length();
  m_world.createJoint(djd);
  {
   float radius = 0.2f;
   CircleShape shape = new CircleShape();
   shape.m_radius = radius;
   for (int i = 0; i < 4; ++i) {
    BodyDef bd = new BodyDef();
    bd.type = BodyType.DYNAMIC;
    bd.position.set(5.9f + 2.0f * radius * i, 2.4f);
    Body body = m_world.createBody(bd);
    body.createFixture(shape, 10.0f);
   }
  }
 }

 @Override
 public String getTestName() {
  return "Dominos";
 }
}
