"""Thickness constraint: thin walls thicken to the minimum, print-safe style."""
import bpy, bmesh, math
from mathutils import Vector
MOD = "bl_ext.www_blenderkit_com.final_topology"
bpy.ops.preferences.addon_enable(module=MOD)
P = bpy.context.preferences.addons[MOD].preferences
P.step_weight = 0.5; P.enable_draw_constraints = False; P.use_mirror = False
mod = __import__(MOD, fromlist=["extras", "draw"])
_KEEP = []
fails = []
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

def thin_slab(thickness):
    """A closed box 0.2 x 0.2 wide and `thickness` tall, subdivided."""
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(0, 0, 0))
    ob = bpy.context.active_object
    ob.scale = (0.2, 0.2, thickness)
    bpy.ops.object.transform_apply(scale=True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.subdivide(number_cuts=3)
    bpy.context.tool_settings.mesh_select_mode = (True, False, False)
    bpy.ops.mesh.select_all(action="SELECT")
    return ob

def wall_thickness(ob):
    """Distance between the centers of the top and bottom faces."""
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    top = [v.co.z for v in bm.verts if v.co.z > 0 and abs(v.co.x) < 0.03 and abs(v.co.y) < 0.03]
    bottom = [v.co.z for v in bm.verts if v.co.z < 0 and abs(v.co.x) < 0.03 and abs(v.co.y) < 0.03]
    return sum(top) / len(top) - sum(bottom) / len(bottom)

print("=== 1. a 3mm wall grows to the 6mm minimum ===")
ob = thin_slab(0.003)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="THICKNESS", name="T")
c = ob.data.ft_custom_constraints[0]
note(c.thickness_use_min and not c.thickness_use_max, "min on, max off by default")
note(abs(c.thickness_min - 0.005) < 1e-9 and abs(c.thickness_max - 0.02) < 1e-9,
     f"defaults 5mm/2cm ({c.thickness_min*1000:.0f}mm/{c.thickness_max*1000:.0f}mm)")
c.thickness_min = 0.006
t0 = wall_thickness(ob)
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=60)
t1 = wall_thickness(ob)
note(abs(t0 - 0.003) < 5e-4, f"starts at 3mm ({t0*1000:.2f}mm)")
note(abs(t1 - 0.006) < 1e-3, f"wall grew to the minimum ({t0*1000:.2f}mm -> {t1*1000:.2f}mm, want 6mm)")

print("\n=== 2. a wall already thick enough is left alone ===")
ob = thin_slab(0.010)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="THICKNESS", name="T")
c = ob.data.ft_custom_constraints[0]
c.thickness_min = 0.006
c.thickness_ray_length = 0.05
t0 = wall_thickness(ob)
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=30)
t1 = wall_thickness(ob)
note(abs(t1 - t0) < 5e-4, f"10mm wall unchanged ({t0*1000:.2f}mm -> {t1*1000:.2f}mm)")

print("\n=== 3. max thickness pulls an over-thick wall in ===")
ob = thin_slab(0.015)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="THICKNESS", name="T")
c = ob.data.ft_custom_constraints[0]
c.thickness_use_min = False
c.thickness_use_max = True
c.thickness_max = 0.008
c.thickness_ray_length = 0.05
t0 = wall_thickness(ob)
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=60)
t1 = wall_thickness(ob)
note(t1 < t0 - 0.003, f"wall pulled in toward the maximum ({t0*1000:.2f}mm -> {t1*1000:.2f}mm)")

print("\n=== 4. colored face overlay drawn while selected ===")
ob = thin_slab(0.003)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="THICKNESS", name="T")
P.enable_draw_constraints = True
mod.draw.clear_draw_list()
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=1)
note(len(mod.draw.draw_colored_tris_pos) > 0,
     f"deviation-colored triangles present ({len(mod.draw.draw_colored_tris_pos)} verts)")
P.enable_draw_constraints = False

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
