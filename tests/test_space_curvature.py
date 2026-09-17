"""Space constraint modes on real loops, curvature on wire loops staying in plane."""
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

print("=== 1. arc spacing on a circle ring stays on the circle ===")
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
# uneven-ify along the circle
for k, i in enumerate(ridx):
    v = bm.verts[i]
    a = math.atan2(v.co.y, v.co.x) + 0.3 * math.sin(k * 1.9) * (2 * math.pi / len(ridx))
    r = v.co.xy.length
    v.co.x, v.co.y = math.cos(a) * r, math.sin(a) * r
for v in bm.verts: v.select = False
for e in bm.edges: e.select = False
for e in bm.edges:
    if e.verts[0].index in set(ridx) and e.verts[1].index in set(ridx):
        e.select = True; e.verts[0].select = True; e.verts[1].select = True
bmesh.update_edit_mesh(obj.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SPACE", name="S")
c = obj.data.ft_custom_constraints[0]
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
radius0 = sum(bm.verts[i].co.xy.length for i in ridx) / len(ridx)
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=100)
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
radii = [bm.verts[i].co.xy.length for i in ridx]
note(max(radii) - min(radii) < 5e-3 and abs(sum(radii) / len(radii) - radius0) < 0.02,
     f"arc interpolation keeps the ring round (radius spread {max(radii)-min(radii):.2e})")
gaps = []
for k in range(len(ridx)):
    a = bm.verts[ridx[k]].co
    b = bm.verts[ridx[(k + 1) % len(ridx)]].co
    gaps.append((a - b).length)
note(max(gaps) - min(gaps) < 5e-3, f"even spacing reached (gap spread {max(gaps)-min(gaps):.2e})")

print("\n=== 2. same-ratio spacing on an open loop: monotone gap progression ===")
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
me = bpy.data.meshes.new("row")
coords = []
x = 0.0
gaps_in = []
for i in range(10):
    coords.append((x, 0.0, 0.0))
    g = 0.1 * (1.35 ** i)  # clearly growing gaps
    gaps_in.append(g)
    x += g
me.from_pydata(coords, [(i, i + 1) for i in range(9)], [])
ob = bpy.data.objects.new("row", me)
bpy.context.collection.objects.link(ob)
bpy.context.view_layer.objects.active = ob
ob.select_set(True)
bpy.ops.object.mode_set(mode="EDIT")
bpy.context.tool_settings.mesh_select_mode = (False, True, False)
bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
# perturb the interior so RATIO has to re-establish the progression
for i in range(1, 9):
    bm.verts[i].co.x += 0.05 * math.sin(i * 2.7)
for v in bm.verts: v.select = True
for e in bm.edges: e.select = True
bmesh.update_edit_mesh(me)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SPACE", name="S")
c = ob.data.ft_custom_constraints[0]
c.space_method = "RATIO"
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=200)
bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
xs = sorted(v.co.x for v in bm.verts)
gaps = [xs[i + 1] - xs[i] for i in range(9)]
ratios = [gaps[i + 1] / gaps[i] for i in range(8)]
note(max(ratios) - min(ratios) < 0.02,
     f"neighbouring gaps keep one ratio (ratio spread {max(ratios)-min(ratios):.3f})")
note(all(r > 1.05 for r in ratios), f"growing tendency preserved (ratios ~{sum(ratios)/8:.2f})")

print("\n=== 3. curvature on a planar wire loop stays in its plane ===")
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
me = bpy.data.meshes.new("arc")
coords = []
n = 12
for i in range(n):
    a = math.pi * 0.15 + math.pi * 0.7 * i / (n - 1)
    r = 1.0 + 0.1 * math.sin(i * 2.4)  # bumpy radius, curvature must even out
    coords.append((math.cos(a) * r, 0.0, math.sin(a) * r))  # XZ plane, Y = 0
me.from_pydata(coords, [(i, i + 1) for i in range(n - 1)], [])
ob = bpy.data.objects.new("arc", me)
bpy.context.collection.objects.link(ob)
bpy.context.view_layer.objects.active = ob
ob.select_set(True)
bpy.ops.object.mode_set(mode="EDIT")
bpy.context.tool_settings.mesh_select_mode = (False, True, False)
bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
for v in bm.verts: v.select = True
for e in bm.edges: e.select = True
bmesh.update_edit_mesh(me)
def circum_k(a, b, c):
    # curvature of the circle through three points
    area2 = (b - a).cross(c - a).length
    d = (b - a).length * (c - b).length * (c - a).length
    return 2.0 * area2 / d if d > 1e-12 else 0.0
def curvature_spread():
    bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    ks = [circum_k(bm.verts[i - 1].co, bm.verts[i].co, bm.verts[i + 1].co)
          for i in range(1, n - 1)]
    return max(ks) - min(ks)
k0 = curvature_spread()
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CURVATURE", name="Cv")
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=150)
bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
# wire verts have position-based normals in Blender - the constraint must use
# the loop plane instead, or the loop would drift out of Y=0
worst_y = max(abs(v.co.y) for v in bm.verts)
note(worst_y < 1e-5, f"wire loop stays in its plane (max |y| {worst_y:.2e})")
k1 = curvature_spread()
note(k1 < k0 * 0.25, f"curvature evens out along the loop (spread {k0:.3f} -> {k1:.3f})")

print("\n=== 4. same-turn mode runs on a real loop ===")
c = ob.data.ft_custom_constraints[0]
c.curvature_split_turns = True
note(c.curvature_split_turns and c.curvature_mode == "BLUR", "same turn combines with the (blur by default) profile")
r = bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=30)
bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
note(r == {'FINISHED'} and max(abs(v.co.y) for v in bm.verts) < 1e-5,
     f"step runs and the loop stays planar ({r})")

print("\n=== 5. one batch of steps equals the same steps taken one at a time ===")
# vertex normals must track the moving loop inside a batch, otherwise a
# batch and a stepwise run (fresh normals each call) diverge - which showed
# up as constraints re-solving whenever a selection change refreshed normals
def curved_ring_setup():
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=12, radius=1.0)
    ob = bpy.context.active_object
    ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    ring = [v for v in bm.verts if abs(v.co.z) < 1e-4]
    ring.sort(key=lambda v: math.atan2(v.co.y, v.co.x))
    ridx = [v.index for v in ring]
    for k, i in enumerate(ridx):
        bm.verts[i].co.z += 0.12 * math.sin(k * 1.3)  # a wavy ring, turns both ways
    for v in bm.verts: v.select = False
    for e in bm.edges: e.select = False
    for e in bm.edges:
        if e.verts[0].index in set(ridx) and e.verts[1].index in set(ridx):
            e.select = True; e.verts[0].select = True; e.verts[1].select = True
    bmesh.update_edit_mesh(ob.data)
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CURVATURE", name="Cv")
    ob.data.ft_custom_constraints[0].curvature_split_turns = True
    return ob, ridx
ob, ridx = curved_ring_setup()
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=40)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
batch = [bm.verts[i].co.copy() for i in ridx]
ob, ridx = curved_ring_setup()
for _ in range(40):
    bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=1)
    bm = bmesh.from_edit_mesh(ob.data); bm.normal_update(); _KEEP.append(bm)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
stepwise = [bm.verts[i].co.copy() for i in ridx]
diff = max((a - b).length for a, b in zip(batch, stepwise))
note(diff < 1e-5, f"batch and stepwise runs agree (max difference {diff:.2e})")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
