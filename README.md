# jbox2d
Optimisations and enhancements for mobile.
This fork of JBox2d is mostly to improve memory handling. This version can run the Piston and Circle stress tests with a 16mb heap (Java -Xmx16m) for an indefinite time, whereas the original version would immediately crash at 16m and at 64m would eventually crash.

- Removed pooling except for singleton objects.  
- Changed Vec2 to be based on vecmath Vector2f
- Changed matrices to vecmath matrices instead of having multiple vector objects for each column, matrices now have local floats for each cell.
- Changed flags from bitfields to booleans as jbox2d was spending up to 15% of collision resolving time decoding bitfields.
- Removed some static builder methods and replaced with constructors, there is still a lot more work needed on this.
- Removed unused classes.
- Removed some static utility methods, still more needed to be removed.
- Removed GWT compatibility.
- Replaced Ad hoc linked lists with ArrayList.

