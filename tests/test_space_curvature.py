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

print("\n=== 6. bridge poles: a dented pole recovers with the loops ending on it ===")
def dented_sphere(bridge):
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    bpy.ops.mesh.primitive_uv_sphere_add(segments=12, ring_count=8, radius=1.0, location=(0, 0, 0))
    ob = bpy.context.active_object
    ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    pole = max(bm.verts, key=lambda v: v.co.z)
    pole.co.z -= 0.15   # push the north pole in
    pidx = pole.index   # the add-constraint mode hop invalidates the bmesh
    for v in bm.verts: v.select = True
    for e in bm.edges: e.select = True
    bmesh.update_edit_mesh(ob.data)
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CURVATURE", name="Cv")
    c = ob.data.ft_custom_constraints[0]
    c.curvature_bridge_poles = bridge
    c.stop_at_poles = True
    bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=150)
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    return bm.verts[pidx].co.z
z_anchored = dented_sphere(False)
z_bridged = dented_sphere(True)
note(abs(z_anchored - 0.85) < 1e-6, f"without bridging the pole is a dead anchor (z stays {z_anchored:.3f})")
note(z_bridged > 0.93, f"with bridging the loops pull the pole back out (z {z_anchored:.3f} -> {z_bridged:.3f})")

print("\n=== 7. space Blur settles around a pinned vertex ===")
def uneven_row(method, pin_index=None):
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    me = bpy.data.meshes.new("row")
    xs = [0.0, 0.5, 1.5, 1.8, 3.2, 3.5, 4.9, 5.2, 6.0]
    me.from_pydata([(x, 0.0, 0.0) for x in xs], [(i, i + 1) for i in range(len(xs) - 1)], [])
    ob = bpy.data.objects.new("row", me); bpy.context.collection.objects.link(ob)
    bpy.context.view_layer.objects.active = ob; ob.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(ob.data); _KEEP.append(bm)
    for v in bm.verts: v.select = True
    for e in bm.edges: e.select = True
    bmesh.update_edit_mesh(ob.data)
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SPACE", name="S")
    ob.data.ft_custom_constraints[0].space_method = method
    if pin_index is not None:
        bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
        for v in bm.verts: v.select = v.index == pin_index
        bmesh.update_edit_mesh(ob.data)
        bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="PIN", name="P")
    bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=300)
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    return [bm.verts[i].co.x for i in range(len(xs))]
xs = uneven_row("BLUR")
gaps = [b - a for a, b in zip(xs, xs[1:])]
note(max(gaps) - min(gaps) < 2e-2 and abs(xs[0]) < 1e-6 and abs(xs[-1] - 6.0) < 1e-6,
     f"Blur alone converges to even spacing (gap spread {max(gaps)-min(gaps):.2e})")
xs = uneven_row("BLUR", pin_index=4)
gl = [b - a for a, b in zip(xs[:5], xs[1:5])]
gr = [b - a for a, b in zip(xs[4:], xs[5:])]
note(abs(xs[4] - 3.2) < 1e-6, f"pinned vertex stays put (x {xs[4]:.3f})")
note(max(gl) - min(gl) < 2e-2 and max(gr) - min(gr) < 2e-2,
     f"Blur evens both sides of the pin separately (spreads {max(gl)-min(gl):.2e}, {max(gr)-min(gr):.2e})")
note(abs(gl[0] - gr[0]) > 0.05, f"the two sides keep their own spacing ({gl[0]:.3f} vs {gr[0]:.3f})")

print("\n=== 8. ring width: rungs of a ladder even out, blur keeps a held rung ===")
def ladder(method, pin_verts=()):
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    me = bpy.data.meshes.new("ladder")
    widths = [1.0, 0.4, 1.6, 0.7, 1.3, 0.9, 1.1]
    verts, faces = [], []
    for i, w in enumerate(widths):
        verts.append((float(i), -w / 2, 0.0)); verts.append((float(i), w / 2, 0.0))
    for i in range(len(widths) - 1):
        faces.append((2 * i, 2 * i + 2, 2 * i + 3, 2 * i + 1))
    me.from_pydata(verts, [], faces)
    ob = bpy.data.objects.new("ladder", me); bpy.context.collection.objects.link(ob)
    bpy.context.view_layer.objects.active = ob; ob.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(ob.data); _KEEP.append(bm)
    for v in bm.verts: v.select = False
    for e in bm.edges:
        rung = abs(e.verts[0].co.x - e.verts[1].co.x) < 1e-6
        e.select = rung
        if rung: e.verts[0].select = e.verts[1].select = True
    bmesh.update_edit_mesh(ob.data)
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SPACE", name="R")
    c = ob.data.ft_custom_constraints[0]
    c.space_target = "RING"; c.space_method = method
    if pin_verts:
        bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
        for v in bm.verts: v.select = v.index in pin_verts
        bmesh.update_edit_mesh(ob.data)
        bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="PIN", name="P")
    bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=200)
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    return [(bm.verts[2 * i + 1].co - bm.verts[2 * i].co).length for i in range(len(widths))], \
           [bm.verts[2 * i].co.x for i in range(len(widths))]
widths, xs = ladder("EVEN")
note(max(widths) - min(widths) < 1e-3 and abs(sum(widths) / len(widths) - 1.0) < 1e-3,
     f"Even: all rungs reach the mean width (spread {max(widths)-min(widths):.2e}, mean {sum(widths)/len(widths):.3f})")
note(max(abs(x - i) for i, x in enumerate(xs)) < 1e-6, "rungs only move along themselves")
widths, _ = ladder("BLUR")
note(max(widths) - min(widths) < 2e-2, f"Blur: rungs converge to one width too (spread {max(widths)-min(widths):.2e})")
widths, _ = ladder("BLUR", pin_verts=(4, 5))
note(abs(widths[2] - 1.6) < 1e-6, f"pinned rung keeps its width ({widths[2]:.3f})")
note(all(abs(b - a) < 0.35 for a, b in zip(widths, widths[1:])),
     f"neighbours blend towards the held rung instead of jumping ({', '.join(f'{w:.2f}' for w in widths)})")

print("\n=== 9. On Surface next to a narrow ring: slides along the rungs, never flips ===")
def narrow_ladder(with_ring):
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    me = bpy.data.meshes.new("narrow")
    n = 9
    verts, faces = [], []
    for i in range(n):
        # an S-shaped band, 1mm wide: evening its turning wants to move
        # vertices sideways by about the band's width
        wave = 0.003 * math.sin(2 * math.pi * i / (n - 1))
        kink = 0.003 if i == 4 else 0.0   # plus a sideways kink in the top loop
        verts.append((i * 0.005, wave, 0.0)); verts.append((i * 0.005, wave + 0.0005 + kink, 0.0))
    for i in range(n - 1):
        faces.append((2 * i, 2 * i + 2, 2 * i + 3, 2 * i + 1))
    me.from_pydata(verts, [], faces)
    ob = bpy.data.objects.new("narrow", me); bpy.context.collection.objects.link(ob)
    bpy.context.view_layer.objects.active = ob; ob.select_set(True)
    ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(ob.data); _KEEP.append(bm)
    def select_edges(pred):
        bm = bmesh.from_edit_mesh(ob.data); _KEEP.append(bm)
        for v in bm.verts: v.select = False
        for e in bm.edges:
            e.select = pred(e)
            if e.select: e.verts[0].select = e.verts[1].select = True
        bmesh.update_edit_mesh(ob.data)
    select_edges(lambda e: abs(e.verts[0].co.x - e.verts[1].co.x) > 1e-6)   # the two loops
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CURVATURE", name="T")
    c = ob.data.ft_custom_constraints[0]
    c.curvature_direction = "TURN"; c.curvature_mode = "LINEAR"; c.curvature_measure = "LENGTH"
    if with_ring:
        select_edges(lambda e: abs(e.verts[0].co.x - e.verts[1].co.x) < 1e-6)   # the rungs
        bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SPACE", name="R")
        r = ob.data.ft_custom_constraints[1]
        r.space_target = "RING"; r.space_method = "BLUR"
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    kink0 = bm.verts[9].co.y - bm.verts[8].co.y - 0.0005
    flipped = 0
    for _ in range(60):
        bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=5)
        bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
        for i in range(n):
            if bm.verts[2 * i + 1].co.y - bm.verts[2 * i].co.y <= 0: flipped += 1
    widths = [bm.verts[2 * i + 1].co.y - bm.verts[2 * i].co.y for i in range(n)]
    return flipped, kink0, bm.verts[9].co.y - bm.verts[8].co.y - sum(widths[:3]) / 3, min(widths)
flipped, kink0, kink1, narrowest = narrow_ladder(with_ring=False)
note(flipped == 0 and narrowest > 0.0001, f"On Surface alone on a 0.5mm wide band never crosses the other loop (narrowest {narrowest*1000:.2f}mm)")
note(abs(kink1) < abs(kink0) * 0.5, f"and still evens the kink ({kink0*1000:.2f} -> {kink1*1000:.2f}mm)")
flipped, kink0, kink1, narrowest = narrow_ladder(with_ring=True)
note(flipped == 0 and narrowest > 0.0001, f"with Ring Width fighting it: no flip either (narrowest {narrowest*1000:.2f}mm, {flipped} flipped checks)")

print("\n=== 11. a cube with a loop cut rounds off symmetrically ===")
def cut_cube():
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    bpy.ops.mesh.primitive_cube_add(size=2, location=(0, 0, 0))
    ob = bpy.context.active_object
    ob.modifiers.new("Subdivision", "SUBSURF").levels = 2
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(ob.data); _KEEP.append(bm)
    vertical = [e for e in bm.edges if abs(e.verts[0].co.z - e.verts[1].co.z) > 1.5]
    bmesh.ops.subdivide_edges(bm, edges=vertical, cuts=1, use_grid_fill=True)
    bmesh.update_edit_mesh(ob.data)
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    for v in bm.verts: v.select = True
    for e in bm.edges: e.select = True
    bmesh.update_edit_mesh(ob.data)
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CURVATURE", name="Cv")
    c = ob.data.ft_custom_constraints[0]
    c.stop_at_poles = False; c.stop_at_turns = False; c.stop_at_crease = False
    c.curvature_mode = "BLUR"; c.curvature_bridge_poles = True
    return ob, c
ob, c = cut_cube()
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
corners = [v.index for v in bm.verts if len(v.link_edges) == 3]
cuts = [v.index for v in bm.verts if len(v.link_edges) == 4]
note(len(corners) == 8 and len(cuts) == 4 and len(bm.verts) == 12, f"cube with one loop cut: {len(corners)} corners, {len(cuts)} cut verts")
mod = __import__(MOD, fromlist=["utils"])
loops = mod.utils.get_attribute_elements(ob, bm, c, domain="EDGE", as_domain="POINT")
shapes = sorted((len(v), circ) for v, circ in loops)
note(shapes == [(2, False)] * 8 + [(3, False)] * 4 + [(4, True)], f"decomposition is the symmetric one: 8 single edges, 4 corner-cut-corner loops, the ring ({shapes})")
r0 = {i: bm.verts[i].co.length for i in range(12)}
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=150)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
rc = [bm.verts[i].co.length for i in corners]
ru = [bm.verts[i].co.length for i in cuts]
note(max(rc) - min(rc) < 1e-4 and max(ru) - min(ru) < 1e-4, f"all corners alike, all cut verts alike (spreads {max(rc)-min(rc):.1e}, {max(ru)-min(ru):.1e})")
moved = max(abs(bm.verts[i].co.length - r0[i]) for i in corners)
note(moved > 0.05, f"the corners move too, bridged through the right angles ({moved:.3f})")
ratio0 = math.sqrt(3) / math.sqrt(2)
ratio = (sum(rc) / 8) / (sum(ru) / 4)
note(ratio < ratio0 - 0.1, f"corner to cut-vert radius ratio heads toward a sphere ({ratio0:.3f} -> {ratio:.3f})")

print("\n=== 6b. bridged poles move even with Surroundings on ===")
z_ctx_anchored = None
def dented_sphere_ctx(bridge, context):
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    bpy.ops.mesh.primitive_uv_sphere_add(segments=12, ring_count=8, radius=1.0, location=(0, 0, 0))
    ob = bpy.context.active_object
    ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    pole = max(bm.verts, key=lambda v: v.co.z)
    pole.co.z -= 0.15
    pidx = pole.index
    # assign only the upper half: the loops end on the north pole and, at
    # the equator, in unassigned surroundings
    for v in bm.verts: v.select = v.co.z > -0.01
    for e in bm.edges: e.select = e.verts[0].select and e.verts[1].select
    bmesh.update_edit_mesh(ob.data)
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CURVATURE", name="Cv")
    c = ob.data.ft_custom_constraints[0]
    c.curvature_bridge_poles = bridge; c.stop_at_poles = True; c.curvature_context_steps = context
    bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=150)
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    return bm.verts[pidx].co.z
z_a = dented_sphere_ctx(False, 2)
z_b = dented_sphere_ctx(True, 2)
note(abs(z_a - 0.85) < 1e-6, f"Surroundings 2, no bridging: the pole is a dead anchor (z {z_a:.3f})")
note(z_b > 0.93, f"Surroundings 2 with bridging: the pole moves out again (z {z_a:.3f} -> {z_b:.3f})")

print("\n=== 10. coincident vertices never divide by zero ===")
mod = __import__(MOD, fromlist=["extras"])
ex = mod.extras
a, b = Vector((0, 0, 0)), Vector((1, 0, 0))
note(ex.circle_through_points(a, a, b) is None and ex.circle_through_points(a, b, b) is None and ex.circle_through_points(a, a, a) is None,
     "a circle through two coincident points is reported as none")
class _V:
    def __init__(self, co): self.co = Vector(co)
verts = [_V((0, 0, 0)), _V((1, 0, 0)), _V((1, 0, 0)), _V((2, 0.5, 0)), _V((3, 0, 0))]
tknots = [0.0, 1.0, 1.0, 2.118, 3.236]
try:
    pos = ex.evaluate_arc(verts, tknots, 1.5)
    ok = all(math.isfinite(x) for x in pos)
except ZeroDivisionError:
    ok = False
note(ok, "evaluate_arc across a doubled vertex (mid edge slide) stays finite")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
