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
note(c.smooth_mode == "CURVATURE", f"round is the default mode ({c.smooth_mode})")
c.smooth_mode = "BLEND"
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
ob.data.ft_custom_constraints[0].smooth_mode = "BLEND"
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=40)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
inward = max(1.0 - max(abs(bm.verts[i].co.x), abs(bm.verts[i].co.y)) for i in border)
note(inward < 1e-4, f"border verts stay on the grid outline (max inward drift {inward:.2e})")
c_moved = max((bm.verts[i].co - co).length for i, co in corner_cos.items())
note(c_moved < 1e-9, f"corner verts stay anchored ({c_moved:.2e})")
# blend interpolates the anchored corners, which keep their wiggle - the
# interior settles well under the initial 0.05 amplitude but not at zero
interior_z = max(abs(v.co.z) for v in bm.verts if v.index not in set(border))
note(interior_z < 0.03, f"interior still smooths flat (max |z| {interior_z:.4f})")

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
me.ft_custom_constraints[0].smooth_mode = "BLEND"
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=60)
bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
e_moved = max((bm.verts[0].co - end_cos[0]).length, (bm.verts[11].co - end_cos[1]).length)
note(e_moved < 1e-9, f"wire endpoints anchored ({e_moved:.2e})")
# Taubin keeps the wire's overall shape - the jaggedness is what must go
jag1 = wire_jag()
note(jag1 < jag0 * 0.15, f"wire jaggedness smooths out ({jag0:.4f} -> {jag1:.4f})")

print("\n=== 7. smoothing a curved region keeps its curvature ===")
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
bpy.ops.mesh.primitive_uv_sphere_add(segments=24, ring_count=16, radius=1.0)
ob = bpy.context.active_object
ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
bpy.ops.object.mode_set(mode="EDIT")
bpy.context.tool_settings.mesh_select_mode = (True, False, False)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
cap = [v.index for v in bm.verts if v.co.z > 0.3]
for v in bm.verts: v.select = v.index in set(cap)
bmesh.update_edit_mesh(ob.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SMOOTH", name="Sm")
ob.data.ft_custom_constraints[0].smooth_mode = "SHAPE"
def cap_radius():
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    return sum(bm.verts[i].co.length for i in cap) / len(cap)
r0 = cap_radius()
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=300)
r1 = cap_radius()
# the modal reapplies smoothing endlessly - without drift compensation the
# cap radius walked off by about a percent per hundred iterations
note(abs(r1 - r0) < 0.002, f"sphere cap keeps its radius over 300 passes ({r0:.4f} -> {r1:.4f})")

print("\n=== 8. blend mode melts a bump into its surroundings ===")
def bump_grid():
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    bpy.ops.mesh.primitive_grid_add(x_subdivisions=14, y_subdivisions=14, size=2.0, location=(0, 0, 0))
    ob = bpy.context.active_object
    ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (True, False, False)
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    bump = []
    for v in bm.verts:
        r = v.co.xy.length
        if r < 0.5:
            v.co.z = 0.4 * (0.5 - r)  # a cone-ish bump
            bump.append(v.index)
    for v in bm.verts: v.select = v.index in set(bump)
    bmesh.update_edit_mesh(ob.data)
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SMOOTH", name="Sm")
    return ob, bump
def bump_height(ob, bump):
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    return max(bm.verts[i].co.z for i in bump)
ob, bump = bump_grid()
ob.data.ft_custom_constraints[0].smooth_mode = "BLEND"
h0 = bump_height(ob, bump)
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=200)
h_blend = bump_height(ob, bump)
note(h_blend < h0 * 0.25, f"blend melts the bump toward the flat surroundings ({h0:.3f} -> {h_blend:.3f})")
ob, bump = bump_grid()
ob.data.ft_custom_constraints[0].smooth_mode = "SHAPE"
h0 = bump_height(ob, bump)
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=200)
h_shape = bump_height(ob, bump)
note(h_shape > h0 * 0.6, f"keep shape preserves the bump ({h0:.3f} -> {h_shape:.3f})")

print("\n=== 9. round mode fillets a plane-cylinder junction ===")
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
# lathe mesh: flat plane running into a vertical cylinder wall, sharp crease
profile0 = [(1.2, 0.0), (1.0, 0.0), (0.8, 0.0), (0.65, 0.0), (0.5, 0.0),
            (0.3, 0.0), (0.3, 0.15), (0.3, 0.3), (0.3, 0.45), (0.3, 0.6)]
SEG = 12
coords, faces = [], []
for r, z in profile0:
    for k in range(SEG):
        a = 2 * math.pi * k / SEG
        coords.append((math.cos(a) * r, math.sin(a) * r, z))
for ring in range(len(profile0) - 1):
    for k in range(SEG):
        a = ring * SEG + k
        b = ring * SEG + (k + 1) % SEG
        faces.append((a, b, b + SEG, a + SEG))
me = bpy.data.meshes.new("lathe")
me.from_pydata(coords, [], faces)
ob = bpy.data.objects.new("lathe", me)
bpy.context.collection.objects.link(ob)
bpy.context.view_layer.objects.active = ob
ob.select_set(True)
ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
bpy.ops.object.mode_set(mode="EDIT")
bpy.context.tool_settings.mesh_select_mode = (True, False, False)
bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
# select the crease ring and the wall ring above it, like the user would
movable = [v.index for v in bm.verts if 5 * SEG <= v.index < 7 * SEG]
for v in bm.verts: v.select = v.index in set(movable)
bmesh.update_edit_mesh(me)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SMOOTH", name="Sm")
c = ob.data.ft_custom_constraints[0]
c.smooth_mode = "CURVATURE"
c.smooth_factor = 1.0
def ring_profile():
    bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    prof = []
    for ring in range(len(profile0)):
        vs = [bm.verts[ring * SEG + k].co for k in range(SEG)]
        prof.append((sum(math.hypot(v.x, v.y) for v in vs) / SEG,
                     sum(v.z for v in vs) / SEG))
    return prof
def crease_angle(prof):
    # turn between the plane->crease and crease->wall segments (rings 4,5,6)
    import mathutils
    a = mathutils.Vector((prof[5][0] - prof[4][0], prof[5][1] - prof[4][1]))
    b = mathutils.Vector((prof[6][0] - prof[5][0], prof[6][1] - prof[5][1]))
    return math.degrees(a.angle(b))
angle0 = crease_angle(ring_profile())
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=400)
prof1 = ring_profile()
angle1 = crease_angle(prof1)
note(angle0 > 80, f"sharp 90-degree crease to start ({angle0:.0f} deg)")
note(angle1 < angle0 * 0.7, f"crease rounds out ({angle0:.0f} -> {angle1:.0f} deg)")
movable_prof = [prof1[5], prof1[6]]
note(min(r for r, z in movable_prof) > 0.25,
     f"no necking of the wall (min radius {min(r for r, z in movable_prof):.3f})")
note(min(z for r, z in prof1) > -0.02,
     f"no undershoot below the plane (min z {min(z for r, z in prof1):.3f})")

print("\n=== 10. round mode blends a bump toward flat surroundings ===")
ob, bump = bump_grid()
ob.data.ft_custom_constraints[0].smooth_mode = "CURVATURE"
h0 = bump_height(ob, bump)
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=300)
h1 = bump_height(ob, bump)
# fairing removes the cone's creases, not the whole shape - blend mode is
# the one that flattens
note(h1 < h0 * 0.75, f"round fairs the cone's creases down ({h0:.3f} -> {h1:.3f})")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
