package org.jbox2d.particle;

import java.io.Serializable;

public class ParticleGroupType implements Serializable {

 static final long serialVersionUID = 1L;
 /**
  * resists penetration
  */
 public static final int b2_solidParticleGroup = 1 << 0;
 /**
  * keeps its shape
  */
 public static final int b2_rigidParticleGroup = 1 << 1;
}
