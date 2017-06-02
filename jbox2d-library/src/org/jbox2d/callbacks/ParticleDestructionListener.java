package org.jbox2d.callbacks;

import java.io.Serializable;
import org.jbox2d.particle.ParticleGroup;

public interface ParticleDestructionListener extends Serializable {

 static final long serialVersionUID = 1L;

 /**
  * Called when any particle group is about to be destroyed.
  */
 void sayGoodbye(ParticleGroup group);

 /**
  * Called when a particle is about to be destroyed. The index can be used in conjunction with
  * {@link World#getParticleUserDataBuffer} to determine which particle has been destroyed.
  *
  * @param index
  */
 void sayGoodbye(int index);
}
