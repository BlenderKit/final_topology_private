"""Circle constraint join center/normal: concentric and coaxial loops."""
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

def two_ring_mesh(center_b, tilt_b=0.0, radius_b=0.5):
    """Wire mesh with ring A (r=1 at origin, XY plane) and ring B."""
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    coords, edges = [], []
    n = 16
    for i in range(n):
        a = 2 * math.pi * i / n
        coords.append((math.cos(a), math.sin(a), 0.0))
        edges.append((i, (i + 1) % n))
    base = len(coords)
    for i in range(n):
        a = 2 * math.pi * i / n
        x, y, z = math.cos(a) * radius_b, math.sin(a) * radius_b, 0.0
        # tilt around X axis, then move to center_b
        y, z = y * math.cos(tilt_b) - z * math.sin(tilt_b), y * math.sin(tilt_b) + z * math.cos(tilt_b)
        coords.append((x + center_b[0], y + center_b[1], z + center_b[2]))
        edges.append((base + i, base + (i + 1) % n))
    me = bpy.data.meshes.new("rings")
    me.from_pydata(coords, edges, [])
    ob = bpy.data.objects.new("rings", me)
    bpy.context.collection.objects.link(ob)
    bpy.context.view_layer.objects.active = ob
    ob.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    for v in bm.verts: v.select = True
    for e in bm.edges: e.select = True
    bmesh.update_edit_mesh(me)
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CIRCLE", name="C")
    return ob, ob.data.ft_custom_constraints[0], n

def fitted_circles(ob, n):
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    fits = []
    for start in (0, n):
        verts = [bm.verts[i] for i in range(start, start + n)]
        fits.append(mod.extras.fit_circle_to_loop(verts))
    return fits

def run(iters=120):
    bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=iters)

print("=== 1. without join: offset rings keep their own centers ===")
ob, c, n = two_ring_mesh(center_b=(0.3, 0.1, 0.0))
run()
(ca, na, ra), (cb, nb, rb) = fitted_circles(ob, n)
apart = (ca - cb).length
note(apart > 0.25, f"centers stay apart without join ({apart:.3f})")

print("\n=== 2. join center: coplanar rings become concentric ===")
ob, c, n = two_ring_mesh(center_b=(0.3, 0.1, 0.0))
c.join_center = True
run()
(ca, na, ra), (cb, nb, rb) = fitted_circles(ob, n)
apart = (ca - cb).length
note(apart < 1e-3, f"centers meet ({apart:.2e})")
note(abs(ra - 1.0) < 0.1 and abs(rb - 0.5) < 0.1,
     f"each ring keeps its own radius (a={ra:.3f}, b={rb:.3f})")

print("\n=== 3. join center on stacked rings: coaxial, not collapsed ===")
ob, c, n = two_ring_mesh(center_b=(0.3, 0.1, 0.6))
c.join_center = True
run()
(ca, na, ra), (cb, nb, rb) = fitted_circles(ob, n)
axial = abs((ca - cb).dot(Vector((0, 0, 1))))
lateral = ((ca - cb) - Vector((0, 0, 1)) * (ca - cb).dot(Vector((0, 0, 1)))).length
note(lateral < 1e-3, f"rings share the axis (lateral offset {lateral:.2e})")
note(axial > 0.5, f"but keep their heights ({axial:.3f})")

print("\n=== 4. join normal: tilted ring aligns its plane ===")
ob, c, n = two_ring_mesh(center_b=(0.3, 0.1, 0.0), tilt_b=0.5)
c.join_normal = True
run()
(ca, na, ra), (cb, nb, rb) = fitted_circles(ob, n)
angle = math.degrees(min(na.angle(nb), math.pi - na.angle(nb)))
note(angle < 1.0, f"normals align ({angle:.2f} deg apart)")

print("\n=== 5. join both: tilted offset ring turns concentric and coplanar-oriented ===")
ob, c, n = two_ring_mesh(center_b=(0.3, 0.1, 0.0), tilt_b=0.4)
c.join_center = True
c.join_normal = True
run(200)
(ca, na, ra), (cb, nb, rb) = fitted_circles(ob, n)
angle = math.degrees(min(na.angle(nb), math.pi - na.angle(nb)))
lateral = ((ca - cb) - na * (ca - cb).dot(na)).length
note(angle < 1.0 and lateral < 5e-3,
     f"shared axis and orientation ({angle:.2f} deg, lateral {lateral:.2e})")

print("\n=== 6. fix circle still wins over join flags ===")
ob, c, n = two_ring_mesh(center_b=(0.3, 0.1, 0.0))
c.fix_circle = True
c.join_center = True
note(len(c.fixed_circles) == 2, f"two circles stored ({len(c.fixed_circles)})")
stored = [(Vector(item.center), item.radius) for item in c.fixed_circles]
run(60)
(ca, na, ra), (cb, nb, rb) = fitted_circles(ob, n)
match = min((ca - s).length for s, _ in stored) < 1e-3 and min((cb - s).length for s, _ in stored) < 1e-3
note(match, "loops stay on their stored circles, join ignored")

print("\n=== 7. Same Radius: one radius for all fixed circles ===")
ob, c, n = two_ring_mesh(center_b=(0.3, 0.1, 0.0), radius_b=0.5)
c.fix_circle = True
radii = [item.radius for item in c.fixed_circles]
note(len(radii) == 2 and not c.circle_same_radius and abs(radii[0] - radii[1]) > 0.3,
     f"off by default, each circle keeps its own radius ({radii[0]:.3f}, {radii[1]:.3f})")
c.circle_same_radius = True
note(abs(c.circle_radius - sum(radii) / 2) < 1e-6 and all(abs(item.radius - c.circle_radius) < 1e-6 for item in c.fixed_circles),
     f"switching on starts from the mean and writes it onto both ({c.circle_radius:.3f})")
c.circle_radius = 0.7
note(all(abs(item.radius - 0.7) < 1e-6 for item in c.fixed_circles), "editing the radius updates every circle")
run(80)
(ca, na, ra), (cb, nb, rb) = fitted_circles(ob, n)
note(abs(ra - 0.7) < 5e-3 and abs(rb - 0.7) < 5e-3, f"both loops reach radius 0.7 ({ra:.3f}, {rb:.3f})")
# a hand edit of one circle does not break the rule while it is on
c.fixed_circles[0].radius = 0.4
run(40)
(ca, na, ra), (cb, nb, rb) = fitted_circles(ob, n)
note(abs(ra - 0.7) < 5e-3, f"a per-circle edit is overridden while Same Radius is on ({ra:.3f})")
c.circle_same_radius = False
c.fixed_circles[0].radius = 0.4
run(80)
(ca, na, ra), (cb, nb, rb) = fitted_circles(ob, n)
note(abs(ra - 0.4) < 5e-3 and abs(rb - 0.7) < 5e-3, f"off again: circles are independent ({ra:.3f}, {rb:.3f})")
c.circle_same_radius = True
before = c.circle_radius   # the mean of 0.4 and 0.7
bpy.ops.ed.undo_push(message="ui")
c.circle_radius = 0.9
bpy.ops.ed.undo_push(message="ui")
bpy.ops.ed.undo()
note(c.circle_same_radius and (abs(c.circle_radius - before) < 1e-6 or abs(c.circle_radius - 0.9) < 1e-6)
     and all(abs(item.radius - c.circle_radius) < 1e-6 for item in c.fixed_circles),
     f"undo of a radius edit restores a consistent state (radius {c.circle_radius:.2f}, circles {[round(i.radius, 2) for i in c.fixed_circles]})")

print("\n=== 8. Same Radius without Fix Circle: live fits, dictated size ===")
ob, c, n = two_ring_mesh(center_b=(0.3, 0.1, 0.0), radius_b=0.5)
(ca0, _, _), (cb0, _, _) = fitted_circles(ob, n)
c.circle_same_radius = True
note(not c.fix_circle and abs(c.circle_radius - 0.75) < 2e-2, f"switching on starts from the mean of the live fits ({c.circle_radius:.3f})")
c.circle_radius = 0.6
run(80)
(ca, na, ra), (cb, nb, rb) = fitted_circles(ob, n)
note(abs(ra - 0.6) < 5e-3 and abs(rb - 0.6) < 5e-3, f"both loops reach radius 0.6 ({ra:.3f}, {rb:.3f})")
note((ca - ca0).length < 2e-2 and (cb - cb0).length < 2e-2, f"each around its own center ({(ca-ca0).length:.3f}, {(cb-cb0).length:.3f})")
c.join_center = True
run(80)
(ca, na, ra), (cb, nb, rb) = fitted_circles(ob, n)
note(abs(ra - 0.6) < 5e-3 and abs(rb - 0.6) < 5e-3 and (ca - cb).length < 5e-3,
     f"with Join Center too: concentric and equal ({ra:.3f}, {rb:.3f}, centers {(ca-cb).length:.3f} apart)")
c.circle_same_radius = False
run(1)
note(len(c.fixed_circles) == 0, "no circles get stored by Same Radius alone")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
