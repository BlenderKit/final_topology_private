"""Create Curve from Loop operator: NURBS default, point modes, subdiv, undo."""
import bpy, bmesh, math
from mathutils import Vector
MOD = "bl_ext.www_blenderkit_com.final_topology"
bpy.ops.preferences.addon_enable(module=MOD)
P = bpy.context.preferences.addons[MOD].preferences
P.step_weight = 0.35; P.enable_draw_constraints = False; P.use_mirror = False
_KEEP = []
fails = []
def cpts(spline):
    if spline.type == "BEZIER":
        return [b.co.copy() for b in spline.bezier_points]
    return [Vector(p.co[:3]) for p in spline.points]
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

def make_ring(open_loop=False):
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=12, radius=1.0)
    obj = bpy.context.active_object
    obj.modifiers.new("Subdivision", "SUBSURF").levels = 2
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    ring = [v for v in bm.verts if abs(v.co.z) < 1e-4]
    ring.sort(key=lambda v: math.atan2(v.co.y, v.co.x))
    ridx = {v.index for v in ring}
    for v in bm.verts: v.select = False
    for e in bm.edges: e.select = False
    sel = [e for e in bm.edges if e.verts[0].index in ridx and e.verts[1].index in ridx]
    if open_loop: sel = sel[:-4]
    for e in sel:
        e.select = True; e.verts[0].select = True; e.verts[1].select = True
    bmesh.update_edit_mesh(obj.data)
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CURVE", name="Curve")
    return obj, obj.data.ft_custom_constraints[0]

print("=== 0. defaults ===")
obj, c0 = make_ring()
note(c0.projection == "3D", f"projection defaults to 3D ({c0.projection})")
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CURVATURE", name="Cv")
note(obj.data.ft_custom_constraints[1].curvature_context_steps == 0,
     f"curvature surroundings defaults to 0 (got {obj.data.ft_custom_constraints[1].curvature_context_steps})")

print("\n=== 1. original mode: one control point per loop vertex, NURBS default ===")
obj, c = make_ring()
n_objects = len(bpy.data.objects)
r = bpy.ops.object.final_topology_curve_from_selection("EXEC_DEFAULT", point_mode="ORIGINAL")
note(r == {'FINISHED'}, f"operator {r}")
note(len(bpy.data.objects) == n_objects + 1, "curve object created")
note(c.target_curve is not None and c.target_curve.type == "CURVE", "linked as target curve")
spline = c.target_curve.data.splines[0]
note(spline.type == "NURBS", f"nurbs spline by default ({spline.type})")
note(spline.use_endpoint_u, "endpoint clamping on")
pts = cpts(spline)
note(len(pts) == 16, f"one point per loop vert ({len(pts)}, want 16)")
note(spline.use_cyclic_u, "cyclic for the closed loop")
worst = max(abs(co.length - 1.0) for co in pts)
note(worst < 1e-5, f"points sit on the cage loop (max radius error {worst:.2e})")
note((c.target_curve.matrix_world - obj.matrix_world).median_scale is not None and
     max(abs(c.target_curve.matrix_world[i][j] - obj.matrix_world[i][j]) for i in range(4) for j in range(4)) < 1e-9,
     "curve object shares the mesh transform")
note(bpy.context.mode == "EDIT_MESH", "stays in edit mode")

print("\n=== 1b. explicit bezier still available ===")
obj, c = make_ring()
bpy.ops.object.final_topology_curve_from_selection("EXEC_DEFAULT", point_mode="ORIGINAL", curve_type="BEZIER")
spline = c.target_curve.data.splines[0]
note(spline.type == "BEZIER", f"bezier when asked ({spline.type})")
note(len(spline.bezier_points) == 16, f"bezier point count ({len(spline.bezier_points)})")

print("\n=== 2. best fit mode: chosen point count ===")
obj, c = make_ring()
bpy.ops.object.final_topology_curve_from_selection("EXEC_DEFAULT", point_mode="COUNT", point_count=8)
spline = c.target_curve.data.splines[0]
pts = cpts(spline)
note(len(pts) == 8, f"8 requested points ({len(pts)})")
worst = max(abs(co.length - 1.0) for co in pts)
note(worst < 5e-3, f"still hugs the ring (max radius error {worst:.2e})")
gaps = [(pts[(i+1) % 8] - pts[i]).length for i in range(8)]
note(max(gaps) - min(gaps) < 0.05, f"points spread evenly (gap spread {max(gaps)-min(gaps):.4f})")

print("\n=== 3. works on subdivision: curve follows the subdivided shape ===")
obj, c = make_ring()
c.works_on_subdivision = True
bpy.ops.object.final_topology_curve_from_selection("EXEC_DEFAULT", point_mode="ORIGINAL")
spline = c.target_curve.data.splines[0]
radii = [co.length for co in cpts(spline)]
avg = sum(radii) / len(radii)
note(avg < 0.99, f"points on the shrunken subdivided ring (avg radius {avg:.4f} < cage 1.0)")

print("\n=== 4. open loop: non-cyclic, ends exact ===")
obj, c = make_ring(open_loop=True)
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
import importlib
mod = importlib.import_module(MOD)
loops = mod.extras.utils.get_attribute_elements(obj, bm, c, domain="EDGE", as_domain="POINT")
ends = (loops[0][0][0].co.copy(), loops[0][0][-1].co.copy())
bpy.ops.object.final_topology_curve_from_selection("EXEC_DEFAULT", point_mode="ORIGINAL")
spline = c.target_curve.data.splines[0]
note(not spline.use_cyclic_u, "open loop gives a non-cyclic spline")
pts = cpts(spline)
d0 = min((pts[0] - e).length for e in ends)
d1 = min((pts[-1] - e).length for e in ends)
note(d0 < 1e-6 and d1 < 1e-6, f"curve ends at the loop ends ({d0:.2e}, {d1:.2e})")

print("\n=== 5. constraint works right away with the created curve ===")
obj, c = make_ring()
bpy.ops.object.final_topology_curve_from_selection("EXEC_DEFAULT", point_mode="COUNT", point_count=8)
try:
    r = bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=10)
    note(r == {'FINISHED'}, f"step runs with the new curve ({r})")
except Exception as e:
    note(False, f"step raised: {type(e).__name__}: {str(e)[:100]}")

print("\n=== 6. undoable ===")
obj, c = make_ring()
n_objects = len(bpy.data.objects)
bpy.ops.object.final_topology_curve_from_selection("EXEC_DEFAULT", point_mode="ORIGINAL")
bpy.ops.ed.undo_push(message="Create Constraint Curve")  # the UI's automatic push
note(len(bpy.data.objects) == n_objects + 1, "created")
bpy.ops.ed.undo(); bpy.ops.ed.undo()
obj = bpy.context.active_object
if bpy.context.mode != "EDIT_MESH":
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode="EDIT")
c = obj.data.ft_custom_constraints[0]
note(len(bpy.data.objects) == n_objects and c.target_curve is None,
     f"two undos remove the curve and the link ({len(bpy.data.objects)} objects)")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
