"""Curve constraint: no tangential slipping on true bezier evaluation."""
import bpy, bmesh, math
from mathutils import Vector
MOD = "bl_ext.www_blenderkit_com.final_topology"
bpy.ops.preferences.addon_enable(module=MOD)
P = bpy.context.preferences.addons[MOD].preferences
P.step_weight = 0.5; P.enable_draw_constraints = False; P.use_mirror = False
mod = __import__(MOD, fromlist=["extras"])
_KEEP = []
fails = []
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

for o in list(bpy.data.objects): bpy.data.objects.remove(o)
bpy.ops.curve.primitive_bezier_circle_add(radius=1.0, location=(0, 0, 0))
curve_ob = bpy.context.active_object
bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=12, radius=1.0, location=(0, 0, 0))
obj = bpy.context.active_object
obj.modifiers.new("Subdivision", "SUBSURF").levels = 2
bpy.ops.object.mode_set(mode="EDIT")
bpy.context.tool_settings.mesh_select_mode = (False, True, False)
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
ring = [v for v in bm.verts if abs(v.co.z) < 1e-4]
ring.sort(key=lambda v: math.atan2(v.co.y, v.co.x))
ridx = [v.index for v in ring]
for v in bm.verts: v.select = False
for e in bm.edges: e.select = False
for e in bm.edges:
    if e.verts[0].index in set(ridx) and e.verts[1].index in set(ridx):
        e.select = True; e.verts[0].select = True; e.verts[1].select = True
bmesh.update_edit_mesh(obj.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CURVE", name="C")
c = obj.data.ft_custom_constraints[0]
c.target_curve = curve_ob
c.projection = "3D"
c.even_distribution = False

print("=== 1. verts land on the curve ===")
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=60)
points, cyclic = mod.extras.sample_curve_polyline(curve_ob, "3D")
note(cyclic, "sampled polyline is cyclic")
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
wm = obj.matrix_world
worst = max(min((wm @ bm.verts[i].co - p).length for p in points) for i in ridx)
note(worst < 2e-2, f"ring verts on the bezier circle (max distance {worst:.2e})")

print("\n=== 2. no tangential drift over many more iterations ===")
def angles():
    bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    return [math.atan2(bm.verts[i].co.y, bm.verts[i].co.x) for i in ridx]
a0 = angles()
for _ in range(10):
    bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=30)
a1 = angles()
drift = 0.0
for x, y in zip(a0, a1):
    d = abs(x - y)
    drift = max(drift, min(d, 2 * math.pi - d))
note(drift < 1e-3, f"no slipping around the circle (max angular drift {drift:.2e} rad)")

print("\n=== 3. multi-spline curve: only the first spline is followed ===")
# add a second, far-away spline to the same curve datablock
spline2 = curve_ob.data.splines.new("POLY")
spline2.points.add(2)
for p, co in zip(spline2.points, [(5, 5, 0), (6, 5, 0), (7, 5, 0)]):
    p.co = (co[0], co[1], co[2], 1.0)
points2, _ = mod.extras.sample_curve_polyline(curve_ob, "3D")
far = max(p.length for p in points2)
note(far < 2.0, f"sampled polyline stays on the first spline (max radius {far:.2f})")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
