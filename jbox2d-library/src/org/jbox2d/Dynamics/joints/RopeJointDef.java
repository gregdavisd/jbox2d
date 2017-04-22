package org.jbox2d.dynamics.joints;

import java.io.Serializable;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;

/**
 * Rope joint definition. This requires two body anchor points and a maximum lengths. Note: by default the connected objects will
 * not collide. see collideConnected in b2JointDef.
 *
 * @author Daniel Murphy
 */
public class RopeJointDef extends JointDef implements Serializable {

	static final long serialVersionUID = 1L;

	/**
	 * The local anchor point relative to bodyA's origin.
	 */
	public final Vec2 localAnchorA = new Vec2();

	/**
	 * The local anchor point relative to bodyB's origin.
	 */
	public final Vec2 localAnchorB = new Vec2();

	/**
	 * The maximum length of the rope. Warning: this must be larger than b2_linearSlop or the joint will have no effect.
	 */
	public float maxLength;

	public RopeJointDef() {
		super(JointType.ROPE);
		localAnchorA.set(-1.0f, 0.0f);
		localAnchorB.set(1.0f, 0.0f);
	}

	/**
	 * Initialize the bodies, anchors, and length using the world anchors.
	 *
	 * @param b1 First body
	 * @param b2 Second body
	 * @param anchor1 World anchor on first body
	 * @param anchor2 World anchor on second body
	 * @param maxLength Maximum length of rope
	 */
	public void initialize(final Body b1, final Body b2, final Vec2 anchor1, final Vec2 anchor2, float maxLength) {
		bodyA = b1;
		bodyB = b2;
		localAnchorA.set(bodyA.getLocalPoint(anchor1));
		localAnchorB.set(bodyB.getLocalPoint(anchor2));
		this.maxLength = maxLength;
	}
}
