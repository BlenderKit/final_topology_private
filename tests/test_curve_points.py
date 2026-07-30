"""Curve control point handles: adapter math, drag semantics, constraint follow."""
import bpy, bmesh, math
from mathutils import Vector, Matrix
MOD = "bl_ext.www_blenderkit_com.final_topology"
bpy.ops.preferences.addon_enable(module=MOD)
P = bpy.context.preferences.addons[MOD].preferences
P.step_weight = 0.5; P.enable_draw_constraints = False; P.use_mirror = False
mod = __import__(MOD, fromlist=["extras", "gizmos"])
_KEEP = []
fails = []
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

note(bpy.types.GizmoGroup.bl_rna_get_subclass_py("OBJECT_GGT_ft_curve_points") is not None,
     "curve points gizmo group registered")

def make():
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    bpy.ops.curve.primitive_bezier_circle_add(radius=1.2)
    curve_ob = bpy.context.active_object
    bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=12, radius=1.0)
    obj = bpy.context.active_object
    obj.modifiers.new("Subdivision", "SUBSURF").levels = 2
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    ring = [v for v in bm.verts if abs(v.co.z) < 1e-4]; ridx = {v.index for v in ring}
    for v in bm.verts: v.select = False
    for e in bm.edges: e.select = False
    for e in bm.edges:
        if e.verts[0].index in ridx and e.verts[1].index in ridx:
            e.select = True; e.verts[0].select = True; e.verts[1].select = True
    bmesh.update_edit_mesh(obj.data)
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CURVE", name="C")
    c = obj.data.ft_custom_constraints[0]
    c.target_curve = curve_ob
    c.projection = "3D"
    return obj, c, curve_ob

print("\n=== 1. adapter basics ===")
obj, c, curve_ob = make()
adapter = mod.extras.TargetCurveControlPoints(curve_ob)
note(adapter.count() == 4, f"bezier circle has 4 control points ({adapter.count()})")
point = curve_ob.data.splines[0].bezier_points[0]
expected = curve_ob.matrix_world @ point.co
note((adapter.location(0) - expected).length < 1e-9, "world location matches the control point")

print("\n=== 2. drag math on a transformed curve object ===")
bpy.ops.object.mode_set(mode="OBJECT")
curve_ob.rotation_euler = (0.4, 0.2, 0.7)
curve_ob.scale = (1.5, 1.5, 1.5)
bpy.context.view_layer.update()
bpy.ops.object.mode_set(mode="EDIT")
adapter = mod.extras.TargetCurveControlPoints(curve_ob)
loc0 = adapter.location(1)
snap = adapter.snapshot(1)
offset = Vector((0.2, -0.1, 0.35))
adapter.translate(1, snap, offset)
err = ((adapter.location(1) - loc0) - offset).length
note(err < 1e-6, f"translate is world-exact under rot+scale (err {err:.2e})")
# with FREE handles the ride-along is exact; AUTO handles recompute, which
# is Blender's own desired behavior
point = curve_ob.data.splines[0].bezier_points[2]
point.handle_left_type = "FREE"; point.handle_right_type = "FREE"
snap2 = adapter.snapshot(2)
adapter.translate(2, snap2, Vector((0.1, 0.2, 0.3)))
h_err = ((point.handle_left - snap2[1]) - (point.co - snap2[0])).length
note(h_err < 1e-6, f"free handles ride along rigidly (err {h_err:.2e})")
# absolute-from-snapshot: repeated set calls don't accumulate
adapter.translate(1, snap, Vector((0.1, 0, 0)))
adapter.translate(1, snap, Vector((0.1, 0, 0)))
first = adapter.location(1).copy()
adapter.translate(1, snap, Vector((0.1, 0, 0)))
note((adapter.location(1) - first).length < 1e-9, "cumulative drag values apply against the snapshot")

print("\n=== 3. constraint follows the edited curve ===")
obj, c, curve_ob = make()
adapter = mod.extras.TargetCurveControlPoints(curve_ob)
snap = adapter.snapshot(0)
adapter.translate(0, snap, Vector((0.5, 0.0, 0.0)))  # pull one control point outward
for _ in range(120):
    bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=3)
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
loops = mod.extras.utils.get_attribute_elements(obj, bm, c, domain="EDGE", as_domain="POINT")
points, cyclic = mod.extras.sample_curve_polyline(curve_ob, "3D")
note(max(p.length for p in points) > 1.4, f"the curve itself bulged out (max {max(p.length for p in points):.3f})")
wm = obj.matrix_world
def dist_to_curve(co):
    return min((co - p).length for p in points)
worst = max(dist_to_curve(wm @ v.co) for v in loops[0][0])
note(worst < 5e-2, f"ring verts follow the edited curve (max distance {worst:.2e})")

print("\n=== 4. poll and non-bezier safety ===")
GG = mod.extras.CurvePointsGizmoGroup
note(GG.poll(bpy.context), "poll True with active curve constraint")
c.target_curve = None
note(not GG.poll(bpy.context), "poll False without a target curve")
# nurbs spline: adapter exposes its control points too
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
bpy.ops.curve.primitive_nurbs_circle_add(radius=1.0)
nurbs_ob = bpy.context.active_object
adapter = mod.extras.TargetCurveControlPoints(nurbs_ob)
note(adapter.count() == 8, f"nurbs circle yields its 8 control points ({adapter.count()})")
pt = nurbs_ob.data.splines[0].points[0]
w0 = pt.co.w
snap = adapter.snapshot(0)
adapter.translate(0, snap, Vector((0.25, 0.0, 0.0)))
moved = (Vector(pt.co[:3]) - Vector(snap[0][:3]) - Vector((0.25, 0.0, 0.0))).length
note(moved < 1e-6 and abs(pt.co.w - w0) < 1e-6,
     f"nurbs point translates and keeps its weight (err {moved:.2e}, w {pt.co.w:.3f})")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
