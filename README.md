# jbox2d
Optimisations an enhancements for mobile

- Removed pooling except for singleton objects.  
- Changed Vec2 to be based on vecmath Vector2f
- Changed matrices to vecmath matrices instead of having multiple vector objects for each column, matrices now have local floats for each cell.
- Changed flags from bitfields to booleans as jbox2d was spending up to 15% of collision resolving time decoding bitfields.
- Removed some static builder methods and replaced with constructors, there is still a lot more work needed on this.
- Removed unused classes.
- Removed some static utility methods, still more needed to be removed.
- Removed GWT compatibility.

