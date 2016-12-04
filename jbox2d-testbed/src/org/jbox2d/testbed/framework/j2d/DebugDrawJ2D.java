/** *****************************************************************************
 * Copyright (c) 2013, Daniel Murphy All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met: * Redistributions of source code must retain the
 * above copyright notice, this list of conditions and the following disclaimer. * Redistributions
 * in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************************************************************************** */
package org.jbox2d.testbed.framework.j2d;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.collision.AABB;
import org.jbox2d.common.PrimeColor3f;
import org.jbox2d.common.IViewportTransform;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.particle.ParticleColor;
import org.jbox2d.testbed.pooling.ColorPool;

// pooling local, not thread-safe
/**
 * Implementation of {@link DebugDraw} that uses Java2D! Hooray!</br>
 *
 * @author Daniel Murphy
 */
public class DebugDrawJ2D extends DebugDraw {

	public static int circlePoints = 13;
	public static final float edgeWidth = 0.02f;

	private final TestPanelJ2D panel;
	private final ColorPool<Color> cpool = new ColorPool<Color>() {
		protected Color newColor(float r, float g, float b, float alpha) {
			return new Color(r, g, b, alpha);
		}
	};
	private final boolean yFlip;
	private final BasicStroke stroke;
	private final Shape circle;

	public DebugDrawJ2D(TestPanelJ2D argTestPanel, boolean yFlip) {
		panel = argTestPanel;
		this.yFlip = yFlip;
		stroke = new BasicStroke(0);
		circle = new Ellipse2D.Float(-1, -1, 2, 2);
	}

	@Override
	public void setViewportTransform(IViewportTransform viewportTransform) {
		super.setViewportTransform(viewportTransform);
		viewportTransform.setYFlip(yFlip);
	}

	@Override
	public void drawPoint(Vec2 argPoint, float argRadiusOnScreen, PrimeColor3f argColor) {
		getWorldToScreenToOut(argPoint, sp1);
		Graphics2D g = getGraphics();

		Color c = cpool.getColor(argColor.x, argColor.y, argColor.z);
		g.setColor(c);
		sp1.x -= argRadiusOnScreen;
		sp1.y -= argRadiusOnScreen;
		g.fillOval((int) sp1.x, (int) sp1.y, (int) argRadiusOnScreen * 2, (int) argRadiusOnScreen * 2);
	}

	private final Vec2 sp1 = new Vec2();
	private final Vec2 sp2 = new Vec2();

	@Override
	public void drawSegment(Vec2 p1, Vec2 p2, PrimeColor3f color) {
		getWorldToScreenToOut(p1, sp1);
		getWorldToScreenToOut(p2, sp2);

		Color c = cpool.getColor(color.x, color.y, color.z);
		Graphics2D g = getGraphics();
		g.setColor(c);
		g.setStroke(stroke);
		g.drawLine((int) sp1.x, (int) sp1.y, (int) sp2.x, (int) sp2.y);
	}

	public void drawAABB(AABB argAABB, PrimeColor3f color) {
		Vec2 vecs[] = new Vec2[]{new Vec2(), new Vec2(), new Vec2(), new Vec2()};
		argAABB.getVertices(vecs);
		drawPolygon(vecs, 4, color);
	}

	private final AffineTransform tr = new AffineTransform();
	private AffineTransform oldTrans = new AffineTransform();
	private Stroke oldStroke;

	private void saveState(Graphics2D g) {
		oldTrans = g.getTransform();
		oldStroke = g.getStroke();
	}

	private void restoreState(Graphics2D g) {
		g.setTransform(oldTrans);
		g.setStroke(oldStroke);
	}

	private void transformGraphics(Graphics2D g, Vec2 center) {
		Vec2 e = viewportTransform.getExtents();
		Vec2 vc = viewportTransform.getCenter();
		Mat22 vt = viewportTransform.getMat22Representation();

		int flip = yFlip ? -1 : 1;
		tr.setTransform(vt.m00, flip * vt.m10, vt.m01, flip * vt.m11, e.x, e.y);
		tr.translate(-vc.x, -vc.y);
		tr.translate(center.x, center.y);
		g.transform(tr);
	}

	@Override
	public void drawCircle(Vec2 center, float radius, PrimeColor3f color) {
		Graphics2D g = getGraphics();
		Color s = cpool.getColor(color.x, color.y, color.z, 1f);
		saveState(g);
		transformGraphics(g, center);
		g.setStroke(stroke);
		g.scale(radius, radius);
		g.setColor(s);
		g.drawOval(-1, -1, 2, 2);
		restoreState(g);
	}

	@Override
	public void drawCircle(Vec2 center, float radius, Vec2 axis, PrimeColor3f color) {
		Graphics2D g = getGraphics();
		saveState(g);
		transformGraphics(g, center);
		g.setStroke(stroke);
		Color s = cpool.getColor(color.x, color.y, color.z, 1f);
		g.scale(radius, radius);
		g.setColor(s);
		g.draw(circle);
		if (axis != null) {
			g.rotate((float) Math.atan2(axis.y, axis.x));
			g.drawLine(0, 0, 1, 0);
		}
		restoreState(g);
	}

	@Override
	public void drawSolidCircle(Vec2 center, float radius, Vec2 axis, PrimeColor3f color) {
		Graphics2D g = getGraphics();
		saveState(g);
		transformGraphics(g, center);
		g.setStroke(stroke);
		Color f = cpool.getColor(color.x, color.y, color.z, .4f);
		Color s = cpool.getColor(color.x, color.y, color.z, 1f);
		g.scale(radius, radius);
		g.setColor(f);
		g.fill(circle);
		g.setColor(s);
		g.draw(circle);
		if (axis != null) {
			g.rotate((float) Math.atan2(axis.y, axis.x));
			g.drawLine(0, 0, 1, 0);
		}
		restoreState(g);
	}

	private final Vec2 zero = new Vec2();
	private final Color pcolorA = new Color(1f, 1f, 1f, .4f);

	@Override
	public void drawParticles(Vec2[] centers, float radius, ParticleColor[] colors, int count) {
		Graphics2D g = getGraphics();
		saveState(g);
		transformGraphics(g, zero);
		g.setStroke(stroke);
		for (int i = 0; i < count; i++) {
			Vec2 center = centers[i];
			Color color;
			if (colors == null) {
				color = pcolorA;
			} else {
				ParticleColor c = colors[i];
				color = cpool.getColor(c.r * 1f / 127, c.g * 1f / 127, c.b * 1f / 127, c.a * 1f / 127);
			}
			AffineTransform old = g.getTransform();
			g.translate(center.x, center.y);
			g.scale(radius, radius);
			g.setColor(color);
			g.fill(circle);
			g.setTransform(old);
		}
		restoreState(g);
	}

	private final Color pcolor = new Color(1f, 1f, 1f, 1f);

	@Override
	public void drawParticlesWireframe(Vec2[] centers, float radius, ParticleColor[] colors,
		int count) {
		Graphics2D g = getGraphics();
		saveState(g);
		transformGraphics(g, zero);
		g.setStroke(stroke);
		for (int i = 0; i < count; i++) {
			Vec2 center = centers[i];
			Color color;
			// No alpha channel, it slows everything down way too much.
			if (colors == null) {
				color = pcolor;
			} else {
				ParticleColor c = colors[i];
				color = new Color(c.r * 1f / 127, c.g * 1f / 127, c.b * 1f / 127, 1);
			}
			AffineTransform old = g.getTransform();
			g.translate(center.x, center.y);
			g.scale(radius, radius);
			g.setColor(color);
			g.draw(circle);
			g.setTransform(old);
		}
		restoreState(g);
	}

	private final Vec2 temp = new Vec2();

	@Override
	public void drawSolidPolygon(Vec2[] vertices, int vertexCount, PrimeColor3f color) {
		Color f = cpool.getColor(color.x, color.y, color.z, .4f);
		Color s = cpool.getColor(color.x, color.y, color.z, 1f);
		Graphics2D g = getGraphics();
		saveState(g);
		int[] xInts = new int[vertexCount];
		int[] yInts = new int[vertexCount];
		for (int i = 0; i < vertexCount; i++) {
			getWorldToScreenToOut(vertices[i], temp);
			xInts[i] = (int) temp.x;
			yInts[i] = (int) temp.y;
		}
		g.setStroke(stroke);
		g.setColor(f);
		g.fillPolygon(xInts, yInts, vertexCount);
		g.setColor(s);
		g.drawPolygon(xInts, yInts, vertexCount);
		restoreState(g);
	}

	@Override
	public void drawPolygon(Vec2[] vertices, int vertexCount, PrimeColor3f color) {
		Color s = cpool.getColor(color.x, color.y, color.z, 1f);
		Graphics2D g = getGraphics();
		saveState(g);
		int[] xInts = new int[vertexCount];
		int[] yInts = new int[vertexCount];
		for (int i = 0; i < vertexCount; i++) {
			getWorldToScreenToOut(vertices[i], temp);
			xInts[i] = (int) temp.x;
			yInts[i] = (int) temp.y;
		}
		g.setStroke(stroke);
		g.setColor(s);
		g.drawPolygon(xInts, yInts, vertexCount);
		restoreState(g);
	}

	@Override
	public void drawString(float x, float y, String s, PrimeColor3f color) {
		Graphics2D g = getGraphics();
		if (g == null) {
			return;
		}
		Color c = cpool.getColor(color.x, color.y, color.z);
		g.setColor(c);
		g.drawString(s, x, y);
	}

	private Graphics2D getGraphics() {
		return panel.getDBGraphics();
	}

	private final Vec2 temp2 = new Vec2();

	@Override
	public void drawTransform(Transform xf) {
		Graphics2D g = getGraphics();
		getWorldToScreenToOut(xf.p, temp);
		temp2.setZero();
		float k_axisScale = 0.4f;

		Color c = cpool.getColor(1, 0, 0);
		g.setColor(c);

		temp2.x = xf.p.x + k_axisScale * xf.q.c;
		temp2.y = xf.p.y + k_axisScale * xf.q.s;
		getWorldToScreenToOut(temp2, temp2);
		g.drawLine((int) temp.x, (int) temp.y, (int) temp2.x, (int) temp2.y);

		c = cpool.getColor(0, 1, 0);
		g.setColor(c);
		temp2.x = xf.p.x + -k_axisScale * xf.q.s;
		temp2.y = xf.p.y + k_axisScale * xf.q.c;
		getWorldToScreenToOut(temp2, temp2);
		g.drawLine((int) temp.x, (int) temp.y, (int) temp2.x, (int) temp2.y);
	}
}
