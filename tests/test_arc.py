"""Arc constraint: open loops settle on circular arcs, with one angle and
one radius for all when asked."""
import bpy, bmesh, math
from mathutils import Vector
MOD = "bl_ext.www_blenderkit_com.final_topology"
bpy.ops.preferences.addon_enable(module=MOD)
mod = __import__(MOD, fromlist=["extras"])
P = bpy.context.preferences.addons[MOD].preferences
P.step_weight = 0.5; P.enable_draw_constraints = False; P.use_mirror = False
_KEEP = []
fails = []
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

N = 9
def arcs_mesh(sweeps=(90.0,), radii=(1.0,), noise=0.03):
    """One open wire loop per arc, side by side along X, each N vertices"""
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    me = bpy.data.meshes.new("arcs")
    coords, edges = [], []
    for k, (sweep, radius) in enumerate(zip(sweeps, radii)):
        base = len(coords)
        for i in range(N):
            a = math.radians(sweep) * i / (N - 1)
            wobble = noise * math.sin(i * 2.3) if 0 < i < N - 1 else 0.0
            coords.append((k * 4.0 + math.cos(a) * (radius + wobble), math.sin(a) * (radius + wobble), 0.0))
        edges += [(base + i, base + i + 1) for i in range(N - 1)]
    me.from_pydata(coords, edges, [])
    ob = bpy.data.objects.new("arcs", me); bpy.context.collection.objects.link(ob)
    bpy.context.view_layer.objects.active = ob; ob.select_set(True)
    ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(me); _KEEP.append(bm)
    for v in bm.verts: v.select = True
    for e in bm.edges: e.select = True
    bmesh.update_edit_mesh(me)
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="ARC", name="A")
    return ob, me.ft_custom_constraints[0]

def measure(ob, k=0):
    """(center, radius, sweep degrees, chord length, endpoints, residual) of arc k"""
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    verts = [bm.verts[k * N + i] for i in range(N)]
    fit = mod.extras.arc_fit(verts)
    center, normal, radius, sweep, t, apex = fit
    residual = max(abs((v.co - center).length - radius) for v in verts)
    a, b = verts[0].co.copy(), verts[-1].co.copy()
    return center, radius, math.degrees(abs(sweep)), (b - a).length, (a, b), residual

def run(n=120):
    bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=n)

print("=== 1. nothing set: the loop settles on its own arc, ends fixed ===")
ob, c = arcs_mesh()
_, r0, s0, chord0, (a0, b0), res0 = measure(ob)
run()
center, r, sweep, chord, (a, b), res = measure(ob)
note(res < 1e-4 and res0 > 1e-2, f"wobble ironed out onto a circle (residual {res0:.3f} -> {res:.1e})")
note((a - a0).length < 1e-6 and (b - b0).length < 1e-6, "ends stay where they were")
note(abs(sweep - s0) < 3.0, f"sweep close to the measured one ({s0:.1f} -> {sweep:.1f} deg)")

print("\n=== 2. same angle: ends stay, the arc bulges to the angle ===")
ob, c = arcs_mesh()
c.arc_same_angle = True
note(abs(math.degrees(c.arc_angle) - 90.0) < 3.0, f"switching on starts from the measured sweep ({math.degrees(c.arc_angle):.1f} deg)")
c.arc_angle = math.radians(150.0)
_, _, _, _, (a0, b0), _ = measure(ob)
run()
center, r, sweep, chord, (a, b), res = measure(ob)
note(abs(sweep - 150.0) < 0.5 and res < 1e-4, f"sweep reaches 150 deg on a clean circle ({sweep:.2f}, residual {res:.1e})")
note((a - a0).length < 1e-6 and (b - b0).length < 1e-6, "ends still fixed")
expected_r = chord / (2 * math.sin(math.radians(75.0)))
note(abs(r - expected_r) < 1e-3, f"radius follows from the chord ({r:.4f} vs {expected_r:.4f})")
c.arc_angle = math.radians(40.0)
run()
_, r, sweep, chord, (a, b), res = measure(ob)
note(abs(sweep - 40.0) < 0.5 and (a - a0).length < 1e-4 and (b - b0).length < 1e-4, f"flatter arc too ({sweep:.2f} deg), ends within float drift")

print("\n=== 3. same radius: ends stay, the sweep follows from the chord ===")
ob, c = arcs_mesh()
c.circle_same_radius = True
c.circle_radius = 2.0
_, _, _, chord0, (a0, b0), _ = measure(ob)
run()
_, r, sweep, chord, (a, b), res = measure(ob)
expected_sweep = math.degrees(2 * math.asin(chord0 / 4.0))
note(abs(r - 2.0) < 2e-3 and abs(sweep - expected_sweep) < 0.5 and (a - a0).length < 1e-6,
     f"radius 2 with the chord kept: sweep {sweep:.2f} deg (expect {expected_sweep:.2f})")

print("\n=== 4. angle and radius: the ends slide along the chord ===")
ob, c = arcs_mesh()
c.arc_same_angle = True; c.arc_angle = math.radians(120.0)
c.circle_same_radius = True; c.circle_radius = 0.8
_, _, _, _, (a0, b0), _ = measure(ob)
mid0 = (a0 + b0) * 0.5
run(200)
_, r, sweep, chord, (a, b), res = measure(ob)
expected_chord = 2 * 0.8 * math.sin(math.radians(60.0))
note(abs(r - 0.8) < 2e-3 and abs(sweep - 120.0) < 0.5, f"radius 0.8 and 120 deg reached ({r:.4f}, {sweep:.2f})")
note(abs(chord - expected_chord) < 2e-3 and ((a + b) * 0.5 - mid0).length < 1e-4 and abs((b - a).normalized().dot((b0 - a0).normalized())) > 0.9999,
     f"chord resized on its own line ({chord:.4f} vs {expected_chord:.4f})")

print("\n=== 5. two arcs share angle and radius, each around its own center ===")
ob, c = arcs_mesh(sweeps=(90.0, 60.0), radii=(1.0, 1.5))
c.arc_same_angle = True
note(abs(math.degrees(c.arc_angle) - 75.0) < 3.0, f"same angle starts from the mean sweep ({math.degrees(c.arc_angle):.1f})")
c.arc_angle = math.radians(100.0)
run()
sweeps = [measure(ob, k)[2] for k in range(2)]
radii = [measure(ob, k)[1] for k in range(2)]
chords = [measure(ob, k)[3] for k in range(2)]
expected = [ch / (2 * math.sin(math.radians(50.0))) for ch in chords]
note(all(abs(sw - 100.0) < 0.5 for sw in sweeps) and all(abs(r - e) < 2e-3 for r, e in zip(radii, expected)),
     f"both sweep 100 deg with the radius their own chord gives ({sweeps[0]:.1f}, {sweeps[1]:.1f}; r {radii[0]:.3f}, {radii[1]:.3f})")
c.circle_same_radius = True
note(abs(c.circle_radius - sum(radii) / 2) < 2e-2, f"same radius starts from the mean radius ({c.circle_radius:.3f})")
c.circle_radius = 1.2
run(200)
sweeps = [measure(ob, k)[2] for k in range(2)]
radii = [measure(ob, k)[1] for k in range(2)]
centers = [measure(ob, k)[0] for k in range(2)]
note(all(abs(sw - 100.0) < 0.5 for sw in sweeps) and all(abs(r - 1.2) < 2e-3 for r in radii) and (centers[0] - centers[1]).length > 3.0,
     f"both 100 deg at radius 1.2, separate centers (r {radii[0]:.3f}, {radii[1]:.3f})")

print("\n=== 6. even distribution spreads the vertices at equal angles ===")
ob, c = arcs_mesh()
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
for i in range(1, N - 1):   # bunch the vertices toward the start
    a = math.radians(90.0) * (i / (N - 1)) ** 2
    bm.verts[i].co = (math.cos(a), math.sin(a), 0.0)
bmesh.update_edit_mesh(ob.data)
c.even_distribution = True
run()
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
center = measure(ob)[0]
angles = [math.atan2(bm.verts[i].co.y - center.y, bm.verts[i].co.x - center.x) for i in range(N)]
gaps = [angles[i + 1] - angles[i] for i in range(N - 1)]
note(max(gaps) - min(gaps) < 1e-3, f"equal angular gaps (spread {math.degrees(max(gaps) - min(gaps)):.3f} deg)")

print("\n=== 6b. a pinned vertex places the arc ===")
ob, c = arcs_mesh()
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
mid = N // 2
bm.verts[mid].co *= 1.2       # pushed outward radially
bpy.context.tool_settings.mesh_select_mode = (True, False, False)
for v in bm.verts: v.select = v.index == mid
bmesh.update_edit_mesh(ob.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="PIN", name="P")
run(200)
center, r, sweep, chord, (a, b), res = measure(ob)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
pin_off = abs((bm.verts[mid].co - center).length - r)
note(res < 5e-3 and pin_off < 5e-3, f"arc passes through the pinned vertex (pin off by {pin_off:.1e}, worst {res:.1e})")

print("\n=== 7. a closed loop is left alone ===")
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
bpy.ops.mesh.primitive_circle_add(vertices=12, radius=1.0, location=(0, 0, 0))
ob = bpy.context.active_object
ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
bpy.ops.object.mode_set(mode="EDIT")
bpy.context.tool_settings.mesh_select_mode = (False, True, False)
bm = bmesh.from_edit_mesh(ob.data); _KEEP.append(bm)
bm.verts.ensure_lookup_table()
bm.verts[0].co.x += 0.2
x_before = bm.verts[0].co.x
for v in bm.verts: v.select = True
for e in bm.edges: e.select = True
bmesh.update_edit_mesh(ob.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="ARC", name="A")
run(20)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
note(abs(bm.verts[0].co.x - x_before) < 1e-6, "closed loops are not arcs, nothing moves")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
