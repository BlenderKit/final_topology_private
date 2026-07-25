"""Smooth constraint (Taubin, border handling) and arc spacing stability."""
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

print("=== 1. arc spacing on a nearly straight loop: no explosion ===")
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
me = bpy.data.meshes.new("wire")
coords = []
x = 0.0
for i in range(14):
    # nearly collinear with microscopic lateral noise - an absolute epsilon in
    # the circle fit would let the circumcenter explode here
    coords.append((x, 1e-6 * math.sin(i * 3.7), 0.0))
    x += 0.15 + 0.1 * math.sin(i * 2.3)
me.from_pydata(coords, [(i, i + 1) for i in range(13)], [])
ob = bpy.data.objects.new("wire", me)
bpy.context.collection.objects.link(ob)
bpy.context.view_layer.objects.active = ob
ob.select_set(True)
bpy.ops.object.mode_set(mode="EDIT")
bpy.context.tool_settings.mesh_select_mode = (False, True, False)
bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
for v in bm.verts: v.select = True
for e in bm.edges: e.select = True
bmesh.update_edit_mesh(me)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SPACE", name="S")
c = ob.data.ft_custom_constraints[0]
note(c.space_interpolation == "arc", "arc is the default")
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=100)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
worst = max(v.co.length for v in bm.verts)
note(worst < 3.0, f"positions stay bounded ({worst:.3f})")
ys = [abs(v.co.y) for v in bm.verts]
note(max(ys) < 1e-3, f"loop stays on its line (max |y| {max(ys):.2e})")
row = sorted([v.co.x for v in bm.verts])
gaps = [row[i+1] - row[i] for i in range(len(row)-1)]
note(max(gaps) - min(gaps) < 1e-4, f"and spacing converged (gap spread {max(gaps)-min(gaps):.2e})")

print("\n=== 2. smooth constraint ===")
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=12, radius=1.0)
ob = bpy.context.active_object
ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
bpy.ops.object.mode_set(mode="EDIT")
bpy.context.tool_settings.mesh_select_mode = (True, False, False)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
# roughen a patch and select it
patch = [v for v in bm.verts if v.co.z > 0.4]
pidx = [v.index for v in patch]
for v in bm.verts: v.select = False
for v in patch: v.select = True
bmesh.update_edit_mesh(ob.data)
def roughness():
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    total = 0.0
    for i in pidx:
        v = bm.verts[i]
        if not v.link_edges: continue
        avg = Vector((0.0, 0.0, 0.0))
        for e in v.link_edges: avg += e.other_vert(v).co
        avg /= len(v.link_edges)
        total = max(total, (avg - v.co).length)
    return total
# a smooth sphere still has a curvature residual on this metric - capture it
baseline = roughness()
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
for i, n in enumerate(pidx):
    v = bm.verts[n]
    v.co += v.normal * 0.06 * math.sin(i * 2.9)
bmesh.update_edit_mesh(ob.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SMOOTH", name="Sm")
c = ob.data.ft_custom_constraints[0]
note(c.constraint_type == "SMOOTH" and abs(c.smooth_factor - 0.5) < 1e-9,
     f"smooth constraint added (factor {c.smooth_factor})")
r0 = roughness()
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=30)
r1 = roughness()
note(r1 - baseline < (r0 - baseline) * 0.15,
     f"patch smooths back toward baseline (roughness {r0:.4f} -> {r1:.5f}, baseline {baseline:.5f})")

print("\n=== 3. pin beats smooth ===")
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
for v in bm.verts: v.select = False
hold = pidx[:2]
for i in hold: bm.verts[i].select = True
bmesh.update_edit_mesh(ob.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="PIN", name="Pin")
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
before = {i: bm.verts[i].co.copy() for i in hold}
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=20)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
moved = max((bm.verts[i].co - co).length for i, co in before.items())
note(moved < 1e-9, f"pinned verts resist smoothing ({moved:.2e})")

print("\n=== 4. smooth on subdivision runs ===")
c.works_on_subdivision = True
try:
    r = bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=10)
    note(r == {'FINISHED'}, f"subdiv mode runs ({r})")
except Exception as e:
    note(False, f"raised: {type(e).__name__}: {str(e)[:100]}")

print("\n=== 5. smooth respects borders: no inward collapse ===")
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
bpy.ops.mesh.primitive_grid_add(x_subdivisions=10, y_subdivisions=10, size=2.0)
ob = bpy.context.active_object
ob.location = (0, 0, 0)
bpy.context.view_layer.update()
bpy.ops.object.mode_set(mode="EDIT")
bpy.context.tool_settings.mesh_select_mode = (True, False, False)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
border = [v.index for v in bm.verts if any(len(e.link_faces) == 1 for e in v.link_edges)]
corners = [v.index for v in bm.verts if abs(abs(v.co.x) - 1.0) < 1e-5 and abs(abs(v.co.y) - 1.0) < 1e-5]
for i, v in enumerate(bm.verts):
    v.co.z += 0.05 * math.sin(i * 2.1)
corner_cos = {i: bm.verts[i].co.copy() for i in corners}
for v in bm.verts: v.select = True
bmesh.update_edit_mesh(ob.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SMOOTH", name="Sm")
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=40)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
inward = max(1.0 - max(abs(bm.verts[i].co.x), abs(bm.verts[i].co.y)) for i in border)
note(inward < 1e-4, f"border verts stay on the grid outline (max inward drift {inward:.2e})")
c_moved = max((bm.verts[i].co - co).length for i, co in corner_cos.items())
note(c_moved < 1e-9, f"corner verts stay anchored ({c_moved:.2e})")
interior_z = max(abs(v.co.z) for v in bm.verts if v.index not in set(border))
note(interior_z < 0.02, f"interior still smooths flat (max |z| {interior_z:.4f})")

print("\n=== 6. smooth on a wire polyline: ends anchored, jaggedness gone ===")
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
me = bpy.data.meshes.new("wirez")
coords = [(i * 0.2, 0.03 * math.sin(i * 2.5), 0.0) for i in range(12)]
me.from_pydata(coords, [(i, i + 1) for i in range(11)], [])
ob = bpy.data.objects.new("wirez", me)
bpy.context.collection.objects.link(ob)
bpy.context.view_layer.objects.active = ob
ob.select_set(True)
bpy.ops.object.mode_set(mode="EDIT")
bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
end_cos = (bm.verts[0].co.copy(), bm.verts[11].co.copy())
def wire_jag():
    bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    return max((bm.verts[i - 1].co - 2 * bm.verts[i].co + bm.verts[i + 1].co).length
               for i in range(1, 11))
jag0 = wire_jag()
for v in bm.verts: v.select = True
bmesh.update_edit_mesh(me)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SMOOTH", name="Sm")
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=60)
bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
e_moved = max((bm.verts[0].co - end_cos[0]).length, (bm.verts[11].co - end_cos[1]).length)
note(e_moved < 1e-9, f"wire endpoints anchored ({e_moved:.2e})")
# Taubin keeps the wire's overall shape - the jaggedness is what must go
jag1 = wire_jag()
note(jag1 < jag0 * 0.15, f"wire jaggedness smooths out ({jag0:.4f} -> {jag1:.4f})")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
