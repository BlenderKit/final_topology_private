"""Gizmo library: registration, orientation modes, fixed plane adapter math."""
import bpy, bmesh, math
from mathutils import Vector, Matrix
MOD = "bl_ext.www_blenderkit_com.final_topology"
bpy.ops.preferences.addon_enable(module=MOD)
P = bpy.context.preferences.addons[MOD].preferences
P.step_weight = 0.5; P.enable_draw_constraints = False; P.use_mirror = False
mod = __import__(MOD, fromlist=["extras", "gizmos"])
_KEEP = []
fails = []
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

print("=== 1. gizmo classes registered ===")
note(bpy.types.Gizmo.bl_rna_get_subclass_py("VIEW3D_GT_ft_point_handle") is not None,
     "custom point handle gizmo registered")
note(bpy.types.GizmoGroup.bl_rna_get_subclass_py("OBJECT_GGT_ft_curve_points") is not None,
     "curve points group registered")
note(bpy.types.GizmoGroup.bl_rna_get_subclass_py("OBJECT_GGT_ft_curve_constraint") is not None,
     "curve transform group registered")

print("\n=== 2. ring shape geometry for the handles ===")
ring = mod.gizmos._ring_tris(outer=1.0, inner=0.72, segments=32)
note(len(ring) == 32 * 6, f"annulus triangle list complete ({len(ring)} verts)")
radii = [math.hypot(x, y) for x, y, _ in ring]
note(min(radii) > 0.71 and max(radii) < 1.01,
     f"ring radii within [inner, outer] ({min(radii):.3f}..{max(radii):.3f})")
disc = mod.gizmos._disc_tris()
note(len(disc) == 32 * 3, f"select disc complete ({len(disc)} verts)")

print("\n=== 3. orientation mode helper follows the scene setting ===")
scene = bpy.context.scene
slot = scene.transform_orientation_slots[0]
slot.type = "GLOBAL"
note(mod.gizmos.orientation_mode(bpy.context) == "GLOBAL", "global")
slot.type = "LOCAL"
note(mod.gizmos.orientation_mode(bpy.context) == "LOCAL", "local")
slot.type = "NORMAL"
note(mod.gizmos.orientation_mode(bpy.context) == "NORMAL", "normal")
slot.type = "GLOBAL"

print("\n=== 4. fixed plane adapter: world math under object transform ===")
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=12, radius=1.0)
obj = bpy.context.active_object
obj.location = (1.0, 2.0, 3.0)
obj.rotation_euler = (0.3, 0.1, 0.6)
bpy.context.view_layer.update()
bpy.ops.object.mode_set(mode="EDIT")
bpy.context.tool_settings.mesh_select_mode = (False, True, False)
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
ridx = {v.index for v in bm.verts if abs(v.co.z) < 1e-4}
for v in bm.verts: v.select = False
for e in bm.edges: e.select = False
for e in bm.edges:
    if e.verts[0].index in ridx and e.verts[1].index in ridx:
        e.select = True; e.verts[0].select = True; e.verts[1].select = True
bmesh.update_edit_mesh(obj.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="PLANE_FIXED", name="PF")
c = obj.data.ft_custom_constraints[0]
target = mod.extras.FixedPlaneGizmoTarget(obj, c)
expected = obj.matrix_world @ Vector(c.center)
note((target.location() - expected).length < 1e-6, "location is the world-space stored center")
note(target.translation_axes() == (True, True, True), "arrows shown while center fixed")
c.fix_center = False
axes = target.translation_axes()
note(not any(axes), "arrows hidden when the center is free")
c.fix_center = True
basis = target.orientation()
note(abs(basis.determinant() - 1.0) < 1e-5, f"orientation basis is orthonormal (det {basis.determinant():.4f})")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
