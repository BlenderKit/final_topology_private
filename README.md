# Final Topology addon for Blender 3D - 2.2.0

Blender 3D addon that introduces subdivision snapping, retopo and modeling tools from the 21st century.

This Blender 3D addon features many tools that were missing and also organizes some simple, but essential tools for polygonal modeling.
The addon is distributed in two editions:

- **Final Topology for Artists** - the subdivision snapping toolset.
- **Final Topology for CAD professionals** - adds the constraint system for precise, CAD-grade subdivision surface modelling, aimed at professional CAD and automotive industry modellers.

## Final Topology for Artists - features

- **Inverse subdivision snapping**: snaps mesh vertices in edit mode so that if the subdivision modifier is used, the resulting surface is as close as possible to the target surface.
  - Live, or 'Simulation' mode of this tool, which works realtime during modeling
  - Neighbours levels, Number of iterations settings
  - Single step operator that can be called with a shortcut with arbitrary number of iterations
  - Supports offset from the target surface
- **Unsubdivide** - takes a mesh that resulted from a subdivision modifier being applied (in Blender or in any other application) and recreates the original low-resolution mesh.
  - Settings for number of subdivisions and number of iterations of the algorithm. Higher iteration numbers yield more precise results.
- **Shape Freeze** - enables changing the topology of a subdivision surface model while keeping the resulting shape as same as possible.

## Final Topology for CAD professionals - features

Everything from the Artists edition, plus the constraint system: edit mode constraints that get evaluated in a loop together with the subdivision surface snapping, so all conditions get optimized toward each other. The main difference compared to various shrinkwrap modifier tricks is that everything converges at once.

### Constraints

- **Inverse subdivision** - the snapping as a constraint: works on a limited selection and mixes with the other constraints. Boundaries of the constrained area are handled well.
- **Plane** - snaps loops to their best-fit plane, or to a fixed plane with editable center and normal. Join Center / Join Normal make multiple loops share one plane. Multiple loops in one constraint supported.
- **Line** - straightens open loops onto the line between their ends, or onto stored fixed lines. Projected mode and even distribution.
- **Circle** - pulls loops onto their best-fit circles, or onto stored fixed circles; open loops land on arcs. Even distribution, projected mode, and Join Center / Join Normal for concentric or coaxial rings.
- **Curve** - snaps loops onto a curve object with true bezier evaluation, in 3D or projected onto the curve plane, optionally evenly distributed.
- **Space** - spaces vertices along loops evenly, with a Same Ratio progression or by local Blur (settles around pinned vertices), sliding along arc, cubic or linear interpolation of the loop shape. Ring Width mode evens out the widths of edge rings instead, e.g. support loops.
- Loop constraints can stop their loops at poles, at turns and, by default, where a creased edge touches the loop from the side.
- **Curvature** - evens out the curvature along loops, measured by arc length or turn angle, aiming for a blurred, constant or linearly changing profile, optionally sampling the loop's surroundings. Direction picks the in/out bending against the surface, the turning on the surface, or both as separate curvatures.
- **Thickness** - keeps the wall thickness under the vertices within minimum/maximum bounds, measured by rays cast against the mesh itself. Colored face overlay shows too thin, fine and too thick areas.
- **Pin** - freezes vertices against all constraints and inverse subdivision; only your own transforms move them. Shift+P pins the current selection, Alt+P unpins it.
- **Alt+C** opens a quick menu that adds the current selection to any existing constraint, or to a new one from the New Mesh Constraint submenu, without activating the constraint first. **Shift+Alt+C** is the reverse: it removes the selection from a chosen constraint, or from all of them. The Select Active Constraint checkbox under the list controls whether clicking a constraint selects its elements.
- **Smooth** - smooths the selected area without shrinking it, respecting mesh borders and sharp feature corners.
- **Slide optimize** - slides edges along a loop so that the edge straightens or the angles on both sides of a vertex get equalized.
- **Inclination limit** - limits face inclination against a chosen axis, for manufacturing purposes.

### Constraint workflow

- Transform gizmos for fixed planes, fixed circles and target curves, following Blender's transform orientation setting.
- Curve control point handles: edit the target curves of all curve constraints without leaving the mesh. Control points lying on the same spot - also across different curves - share one handle and move together. Dragging a handle runs Blender's native transform, so snapping, axis locking and numeric input all work; a single tweak returns to the mesh automatically and longer curve editing sessions come back via the Back to Mesh button.
- Create a curve object directly from a constraint's loop, as a NURBS (default) or bezier curve, from the original vertices or a chosen best-fit point count.
- Add or remove the selection per constraint, or remove it from all constraints at once.
- Every constraint action - adding, deleting, assigning, property changes - undoes with a single step.
- Mirror modifier support: mirror seams stay on their mirror planes and circle fits come out symmetric at the seams.
- Colored overlays visualize constraint targets and the remaining deviation while the solver runs.

### Additional tools

- **Loop align to normal plane** - takes a loop and aligns it to a plane that is aligned with its normals' directions.
- **Loop slide optimize** - balances the angles between the edges of a loop.
- **Flatten selection** - simple flatten to median plane operator (similar to LoopTools).
- **Print-safe tools** - checks aimed at 3D-printable output.

## Download

Final Topology add-on is available in [Full Plan subscription](https://www.blenderkit.com/plans/pricing/) to Blendkit.

## Support

If you face any bug, please create a report in this repository's issue tracker: https://github.com/BlenderKit/final_topology/issues.
If your bug is related to a specific model or project and a .blend file is required to debug the problem, please create the issue here and also send the .blend file to admin@blendkit.com with a subject mentioning the issue number and/or title.

## Basic instructions

After installing the addon, find it in the Edit tab of the Sidebar in 3D view.
Read the tooltips directly in the add-on - every button should have enough info to use the tool.

### Quick start

You need to have a mesh with a Subdivision Surface modifier on it. Turn on the Inverse-Subsurf modal or run single optimization steps - the mesh snaps so its subdivided surface matches the surrounding geometry. In the CAD professionals edition, select loops and add constraints from the Constraints panel; they are solved together with the snapping on every step.
