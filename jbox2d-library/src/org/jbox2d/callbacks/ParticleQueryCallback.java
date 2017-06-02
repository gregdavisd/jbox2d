package org.jbox2d.callbacks;

import java.io.Serializable;

/**
 * Callback class for AABB queries. See
 * {@link World#queryAABB(QueryCallback, org.jbox2d.collision.AABB)}.
 *
 * @author dmurph
 *
 */
public interface ParticleQueryCallback extends Serializable {

 static final long serialVersionUID = 1L;

 /**
  * Called for each particle found in the query AABB.
  *
  * @return false to terminate the query.
  */
 boolean reportParticle(int index);
}
