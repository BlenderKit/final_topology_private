"""Inverse subdivide constraint: adding doesn't crash, cage snaps to target."""
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

print("=== 1. adding the constraint on a plain selection works ===")
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
# the snap target FIRST: a slightly larger sphere the cage should inflate toward
bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=24, radius=1.05)
bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=12, radius=1.0)
obj = bpy.context.active_object
obj.modifiers.new("Subdivision", "SUBSURF").levels = 2
bpy.ops.object.mode_set(mode="EDIT")
bpy.context.tool_settings.mesh_select_mode = (True, False, False)
bpy.ops.mesh.select_all(action="SELECT")
try:
    r = bpy.ops.object.final_topology_add_constraint(
        "EXEC_DEFAULT", constraint_type="INVERSE_SUBDIVIDE", name="IS"
    )
    note(r == {'FINISHED'}, f"operator {r}")
except Exception as e:
    note(False, f"raised: {type(e).__name__}: {str(e)[:120]}")
c = obj.data.ft_custom_constraints[0]
note(c.constraint_type == "INVERSE_SUBDIVIDE", "constraint stored")

print("\n=== 2. the cage inflates toward the larger target sphere ===")
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
r0 = sum(v.co.length for v in bm.verts) / len(bm.verts)
try:
    bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=40)
    ok = True
except Exception as e:
    ok = False
    note(False, f"step raised: {type(e).__name__}: {str(e)[:120]}")
if ok:
    bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    r1 = sum(v.co.length for v in bm.verts) / len(bm.verts)
    note(r1 > r0 + 0.01, f"cage inflated toward the target (mean radius {r0:.4f} -> {r1:.4f})")

print("\n=== 3. deleting it removes the frozen state object too ===")
n_objects = len(bpy.data.objects)
frozen = [o for o in bpy.data.objects if o.name.startswith("FROZEN_MESH_STATE")]
bpy.ops.object.final_topology_delete_constraint()
note(len(obj.data.ft_custom_constraints) == 0, "constraint removed")
still = [o for o in bpy.data.objects if o.name.startswith("FROZEN_MESH_STATE")]
note(len(still) == 0, f"frozen state cleaned up ({len(frozen)} -> {len(still)})")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
