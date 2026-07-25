"""Plane and line constraints: flattening, fixed values stay put, line variants."""
import bpy, bmesh, math
from mathutils import Vector
MOD = "bl_ext.www_blenderkit_com.final_topology"
bpy.ops.preferences.addon_enable(module=MOD)
P = bpy.context.preferences.addons[MOD].preferences
P.step_weight = 0.5; P.enable_draw_constraints = False; P.use_mirror = False
_KEEP = []
fails = []
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

def ring_setup(select_open=False):
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=12, radius=1.0)
    obj = bpy.context.active_object
    obj.modifiers.new("Subdivision", "SUBSURF").levels = 2
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    ring = [v for v in bm.verts if abs(v.co.z) < 1e-4]
    ring.sort(key=lambda v: math.atan2(v.co.y, v.co.x))
    ridx = [v.index for v in ring]
    # roughen the ring out of plane
    for k, i in enumerate(ridx):
        bm.verts[i].co.z += 0.08 * math.sin(k * 2.3)
    for v in bm.verts: v.select = False
    for e in bm.edges: e.select = False
    sel = [e for e in bm.edges if e.verts[0].index in set(ridx) and e.verts[1].index in set(ridx)]
    if select_open:
        sel = sel[:-4]
    for e in sel:
        e.select = True; e.verts[0].select = True; e.verts[1].select = True
    bmesh.update_edit_mesh(obj.data)
    return obj, ridx

print("=== 1. plane constraint flattens a roughened ring ===")
obj, ridx = ring_setup()
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
zs = [bm.verts[i].co.z for i in ridx]
spread0 = max(zs) - min(zs)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="PLANE", name="P")
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=100)
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
zs = [bm.verts[i].co.z for i in ridx]
spread = max(zs) - min(zs)
# the correction acts along vertex normals, which tilt with the roughening -
# a small residual is the equilibrium, most of the wiggle must go
note(spread < spread0 * 0.1, f"ring flattens onto a plane (z spread {spread0:.3f} -> {spread:.2e})")

print("\n=== 2. fixed plane: user-set normal survives evaluation ===")
obj, ridx = ring_setup()
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="PLANE_FIXED", name="PF")
c = obj.data.ft_custom_constraints[0]
note(c.fix_center and c.fix_normal, "fixed variant sets both fix flags")
# edit the normal by hand - it must not be overwritten by the evaluation,
# that was the "normal jumps back" bug
tilted = Vector((0.3, 0.0, 1.0)).normalized()
c.normal = tilted
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=20)
kept = Vector(c.normal).normalized()
note((kept - tilted).length < 1e-6, f"edited normal kept ({tuple(round(x, 3) for x in kept)})")
# and the loop actually flattens onto the tilted plane
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
center = Vector(c.center)
ds = [abs((bm.verts[i].co - center).dot(tilted)) for i in ridx]
note(max(ds) < 5e-3, f"ring lies on the tilted plane (max distance {max(ds):.2e})")

print("\n=== 3. line constraint straightens an open loop ===")
obj, ridx = ring_setup(select_open=True)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="LINE", name="L")
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=80)
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
mod = __import__(MOD, fromlist=["extras"])
c = obj.data.ft_custom_constraints[0]
loops = mod.extras.utils.get_attribute_elements(obj, bm, c, domain="EDGE", as_domain="POINT")
verts = loops[0][0]
a, b = verts[0].co, verts[-1].co
direction = (b - a).normalized()
worst = max(((v.co - a) - direction * (v.co - a).dot(direction)).length for v in verts)
note(worst < 1e-3, f"open loop pulled onto its end-to-end line (max deviation {worst:.2e})")

print("\n=== 4. line fix: stored lines hold when the ends move ===")
c.fix_line = True
note(len(c.fixed_lines) >= 1, f"line stored on fixing ({len(c.fixed_lines)})")
stored_start = Vector(c.fixed_lines[0].start)
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=10)
note((Vector(c.fixed_lines[0].start) - stored_start).length < 1e-9, "stored line unchanged by evaluation")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
