# final_topology
Magic subdivision snapping, retopo, modeling tools from the 21st century.

This Blender 3D addon features many tools that were missing and also organizes some simple, but essential tools for polygonal modeling.

Features:
 - Inverse subdivision snapping: snaps mesh vertices in edit mode so that if the subdivision modifier is used, the resulting surface is as close as possible to the surface.
   - Live, or 'Simulation' mode of this tool, which works realtime during modeling
   - Neighbours levels, Number of iteration settings
   - Single step operator that can be called with a shortcut with arbitrary number of iterations.
- Unsubdivide - takes a mesh that resulted from a subdivision modifier being applied (In blender or in any other application) and recreates the original low-resolution mesh.
    -has settings for number of subdivisions, number of iterations of the algorithm. Higher interation numbers yield precise results.
- Shape Freeze - Enables to change topology of Subdivision surface model while keeping the resulting shape as same as possible. (TODO)
- Loop align to normal plane - takes a loop and aligns it to a plane that is aligned with it's normals directions. 
- Loop slide optimize - balances angles that there are between edges of a loop
- Flatten selection - simple flatten to median plane operator (Similar to looptools)
  
