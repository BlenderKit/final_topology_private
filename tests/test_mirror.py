"""Mirror support: seam verts stick to the mirror plane, symmetric circle fits."""
import bpy, bmesh, math
from mathutils import Vector
MOD = "bl_ext.www_blenderkit_com.final_topology"
bpy.ops.preferences.addon_enable(module=MOD)
P = bpy.context.preferences.addons[MOD].preferences
P.step_weight = 0.5; P.enable_draw_constraints = False; P.use_mirror = True
mod = __import__(MOD, fromlist=["extras"])
_KEEP = []
fails = []
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

def half_ring_mesh():
    """Open half-ring in the x>=0 half space, ends on the X=0 mirror plane."""
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    coords, edges = [], []
    n = 9
    for i in range(n):
        a = -math.pi / 2 + math.pi * i / (n - 1)
        # uneven spacing so the space constraint has work to do
        a += 0.2 * math.sin(i * 1.7) * (math.pi / n)
        coords.append((math.cos(a), math.sin(a), 0.0))
    # clamp the ends exactly onto the plane
    coords[0] = (0.0, -1.0, 0.0)
    coords[-1] = (0.0, 1.0, 0.0)
    for i in range(n - 1):
        edges.append((i, i + 1))
    me = bpy.data.meshes.new("half")
    me.from_pydata(coords, edges, [])
    ob = bpy.data.objects.new("half", me)
    bpy.context.collection.objects.link(ob)
    bpy.context.view_layer.objects.active = ob
    ob.select_set(True)
    # the step operator needs a subsurf - with only a mirror modifier it bails
    ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
    mirror = ob.modifiers.new("Mirror", "MIRROR")
    mirror.use_axis[0] = True
    mirror.use_clip = True
    mirror.use_mirror_merge = True
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    for v in bm.verts: v.select = True
    for e in bm.edges: e.select = True
    bmesh.update_edit_mesh(me)
    return ob, n

print("=== 1. space constraint keeps the seam ends on the mirror plane ===")
ob, n = half_ring_mesh()
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SPACE", name="S")
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=80)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
seam_x = max(abs(bm.verts[0].co.x), abs(bm.verts[n - 1].co.x))
note(seam_x < 1e-5, f"seam ends stay on the plane (max |x| {seam_x:.2e})")
gaps = [(bm.verts[i + 1].co - bm.verts[i].co).length for i in range(n - 1)]
note(max(gaps) - min(gaps) < 5e-3, f"spacing still converges (gap spread {max(gaps)-min(gaps):.2e})")

print("\n=== 2. circle constraint fits symmetrically at the seam ===")
ob, n = half_ring_mesh()
# squash the half ring, the circle fit must recenter on the plane
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
for v in bm.verts:
    if 0 < v.index < n - 1:
        v.co.x *= 0.8
bmesh.update_edit_mesh(ob.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CIRCLE", name="C")
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=120)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
fit = mod.extras.fit_circle_to_loop(list(bm.verts))
note(fit is not None, "circle still fits")
if fit:
    center, normal, radius = fit
    note(abs(center.x) < 1e-3, f"fitted center sits on the mirror plane (|x| {abs(center.x):.2e})")
seam_x = max(abs(bm.verts[0].co.x), abs(bm.verts[n - 1].co.x))
note(seam_x < 1e-5, f"seam ends kept on the plane (max |x| {seam_x:.2e})")

print("\n=== 3. mirror off: no seam interference ===")
P.use_mirror = False
ob, n = half_ring_mesh()
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SPACE", name="S")
try:
    r = bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=20)
    note(r == {'FINISHED'}, f"step runs with mirror handling off ({r})")
except Exception as e:
    note(False, f"raised: {type(e).__name__}: {str(e)[:100]}")
P.use_mirror = True

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
