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

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
