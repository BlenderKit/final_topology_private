"""Multi-curve control point handles: coincident points share one handle."""
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

def make_poly_curve(name, coords):
    data = bpy.data.curves.new(name, type="CURVE")
    data.dimensions = "3D"
    spline = data.splines.new("POLY")
    spline.points.add(len(coords) - 1)
    for p, co in zip(spline.points, coords):
        p.co = (co[0], co[1], co[2], 1.0)
    ob = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(ob)
    return ob

for o in list(bpy.data.objects): bpy.data.objects.remove(o)
# two curves whose ends meet at (1, 0, 0)
curve_a = make_poly_curve("A", [(-1, 0, 0), (0, 0.5, 0), (1, 0, 0)])
curve_b = make_poly_curve("B", [(1, 0, 0), (2, 0.5, 0), (3, 0, 0)])
# a third point only NEARLY coincident - inside the relative tolerance
curve_c = make_poly_curve("C", [(1, 0, 0.001), (1, -1, 0), (1, -2, 0)])

bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=12, radius=1.0)
obj = bpy.context.active_object
bpy.ops.object.mode_set(mode="EDIT")
bpy.context.tool_settings.mesh_select_mode = (False, True, False)
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
ring = [v for v in bm.verts if abs(v.co.z) < 1e-4]; ridx = {v.index for v in ring}
for v in bm.verts: v.select = False
for e in bm.edges: e.select = False
for e in bm.edges:
    if e.verts[0].index in ridx and e.verts[1].index in ridx:
        e.select = True; e.verts[0].select = True; e.verts[1].select = True
bmesh.update_edit_mesh(obj.data)
for name, curve in (("CuA", curve_a), ("CuB", curve_b), ("CuC", curve_c)):
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CURVE", name=name)
    c = obj.data.ft_custom_constraints[len(obj.data.ft_custom_constraints) - 1]
    c.target_curve = curve

print("=== 1. all curves' points show while a curve constraint is active ===")
obj.data.ft_custom_constraints_index = 0
ob, curves = mod.extras.get_curve_constraint_curves(bpy.context)
note(len(curves) == 3, f"all three target curves gathered ({len(curves)})")
multi = mod.extras.TargetCurveControlPointsMulti(curves)
# 9 points total, 3 of them coincident at (1,0,0) -> 7 handles
note(multi.count() == 7, f"coincident ends share one handle (9 points -> {multi.count()} handles)")

print("\n=== 2. the shared handle moves all its member points together ===")
shared = None
for i in range(multi.count()):
    if (multi.location(i) - Vector((1, 0, 0))).length < 0.01:
        shared = i
        break
note(shared is not None, "found the shared handle at the meeting point")
snap = multi.snapshot(shared)
note(len(snap) == 3, f"it holds all three coincident points ({len(snap)})")
offset = Vector((0.0, 0.0, 0.5))
multi.translate(shared, snap, offset)
end_a = Vector(curve_a.data.splines[0].points[2].co[:3])
end_b = Vector(curve_b.data.splines[0].points[0].co[:3])
end_c = Vector(curve_c.data.splines[0].points[0].co[:3])
ok = ((end_a - Vector((1, 0, 0.5))).length < 1e-6
      and (end_b - Vector((1, 0, 0.5))).length < 1e-6
      and (end_c - Vector((1, 0, 0.501))).length < 1e-6)
note(ok, f"both exact ends and the near one moved together (a={tuple(round(x,3) for x in end_a)})")
mid_b = Vector(curve_b.data.splines[0].points[1].co[:3])
note((mid_b - Vector((2, 0.5, 0))).length < 1e-9, "other points untouched")

print("\n=== 3. drag survives reclustering: repeated absolute offsets ===")
multi2 = mod.extras.TargetCurveControlPointsMulti(curves)
shared2 = None
for i in range(multi2.count()):
    if (multi2.location(i) - Vector((1, 0, 0.5))).length < 0.01:
        shared2 = i
        break
snap2 = multi2.snapshot(shared2)
multi2.translate(shared2, snap2, Vector((0.2, 0, 0)))
multi2.translate(shared2, snap2, Vector((0.2, 0, 0)))  # same absolute offset again
end_a = Vector(curve_a.data.splines[0].points[2].co[:3])
note((end_a - Vector((1.2, 0, 0.5))).length < 1e-6,
     f"offsets apply against the snapshot, not cumulatively ({tuple(round(x,3) for x in end_a)})")

print("\n=== 4. gizmo group poll and target ===")
GG = mod.extras.CurvePointsGizmoGroup
note(GG.poll(bpy.context), "poll True with an active curve constraint")
target = GG.get_point_target(GG, bpy.context)
note(target is not None and target.count() == 7, f"group target covers all curves ({target.count() if target else None})")
obj.data.ft_custom_constraints[0].enabled = False
obj.data.ft_custom_constraints_index = 0
note(not GG.poll(bpy.context), "poll False when the active constraint is disabled")
obj.data.ft_custom_constraints[0].enabled = True
ob2, curves2 = mod.extras.get_curve_constraint_curves(bpy.context)
note(len(curves2) == 3, "re-enabled")

print("\n=== 5. tweak handoff: curve edit mode with the cluster selected ===")
members = []
for adapter_index, point_index in multi.clusters[shared][1]:
    adapter = multi.adapters[adapter_index]
    spline_index, spline_point, is_bezier = adapter._index_map[point_index]
    members.append((adapter.curve_object.name, spline_index, spline_point, is_bezier))
mesh_object = mod.extras.enter_curve_tweak(bpy.context, members)
note(mesh_object is obj, "remembers the mesh to come back to")
note(mod.extras._curve_tweak_state["mesh_name"] == obj.name, "return state stored")
note(bpy.context.mode == "EDIT_CURVE", f"hopped into curve edit mode ({bpy.context.mode})")
in_edit = {o.name for o in bpy.context.objects_in_mode}
note(in_edit == {"A", "B", "C"}, f"all member curves entered together ({sorted(in_edit)})")
selected, others = 0, 0
for name in ("A", "B", "C"):
    for spline in bpy.data.objects[name].data.splines:
        for p in spline.points:
            if p.select: selected += 1
            else: others += 1
note(selected == 3 and others == 6, f"exactly the cluster is selected ({selected} of {selected + others})")
# the back panel is available while the return state exists
note(mod.extras.VIEW3D_PT_final_topology_curve_tweak.poll(bpy.context), "back panel shows in curve edit mode")
note(mod.extras.FinishCurveTweakOperator.poll(bpy.context), "back operator available")
r = bpy.ops.object.final_topology_finish_curve_tweak()
note(r == {'FINISHED'} and bpy.context.mode == "EDIT_MESH" and bpy.context.active_object is obj,
     f"back operator returns to the mesh edit mode ({bpy.context.mode})")
note(mod.extras._curve_tweak_state["mesh_name"] is None, "return state cleared after coming back")
note(not mod.extras.VIEW3D_PT_final_topology_curve_tweak.poll(bpy.context), "panel gone again")

print("\n=== 6. constraint evaluation still runs with all three ===")
try:
    r = bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=5)
    note(r == {'FINISHED'}, f"step runs ({r})")
except Exception as e:
    note(False, f"raised: {type(e).__name__}: {str(e)[:100]}")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
