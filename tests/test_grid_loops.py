"""Whole-grid selections decompose into parallel loops, not zigzags."""
import bpy, bmesh, math
from mathutils import Vector
MOD = "bl_ext.www_blenderkit_com.final_topology"
bpy.ops.preferences.addon_enable(module=MOD)
P = bpy.context.preferences.addons[MOD].preferences
P.step_weight = 0.5; P.enable_draw_constraints = False; P.use_mirror = False
mod = __import__(MOD, fromlist=["extras", "utils"])
_KEEP = []
fails = []
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

def grid_setup(subdivisions=8):
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    bpy.ops.mesh.primitive_grid_add(
        x_subdivisions=subdivisions, y_subdivisions=subdivisions, size=2.0,
        location=(0, 0, 0),
    )
    ob = bpy.context.active_object
    ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    for v in bm.verts: v.select = True
    for e in bm.edges: e.select = True
    bmesh.update_edit_mesh(ob.data)
    return ob

print("=== 1. full grid decomposes into straight parallel loops ===")
ob = grid_setup()
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
n_side = round(math.sqrt(len(bm.verts)))
# stopping at poles (default): the grid corners are poles, so the border
# splits into four straight open loops - every row and column is one loop
loops = mod.utils.sort_edges_into_loops(list(bm.edges), stop_at_poles=True)
note(len(loops) == 2 * n_side and not any(c for _, c in loops),
     f"one open loop per row and column ({len(loops)} for {n_side}x{n_side})")
covered = sum(len(i) - 1 for i, _ in loops)
note(covered == len(bm.edges), f"every edge in exactly one loop ({covered}/{len(bm.edges)})")
straight = 0
for indices, circular in loops:
    xs = {round(bm.verts[i].co.x, 5) for i in indices}
    ys = {round(bm.verts[i].co.y, 5) for i in indices}
    if len(xs) == 1 or len(ys) == 1:
        straight += 1
note(straight == len(loops), f"all loops run straight along one grid direction ({straight}/{len(loops)})")
note(all(len(indices) == n_side for indices, _ in loops), "each loop spans the full grid width")
# without the pole stop the border fuses into one ring around the corners
loops = mod.utils.sort_edges_into_loops(list(bm.edges), stop_at_poles=False)
rings = [l for l in loops if l[1]]
note(len(rings) == 1 and len(rings[0][0]) == 4 * (n_side - 1),
     f"without pole stops the border is one ring ({len(rings)} ring)")

print("\n=== 1b. poles end loops: a uv sphere's meridians stop at the caps ===")
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=12, radius=1.0)
sph = bpy.context.active_object
bpy.ops.object.mode_set(mode="EDIT")
bm = bmesh.from_edit_mesh(sph.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
loops = mod.utils.sort_edges_into_loops(list(bm.edges), stop_at_poles=True)
rings = [l for l in loops if l[1]]
meridians = [l for l in loops if not l[1]]
note(len(rings) == 11 and len(meridians) == 16,
     f"11 rings + 16 pole-to-pole meridians ({len(rings)} rings, {len(meridians)} open)")
note(all(len(m[0]) == 13 for m in meridians), "each meridian runs from pole to pole")
loops = mod.utils.sort_edges_into_loops(list(bm.edges), stop_at_poles=False)
note(sum(1 for l in loops if not l[1]) < 16,
     f"without pole stops meridians run on through the poles ({len(loops)} loops)")

print("\n=== 2. simple ring selections keep working as before ===")
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=12, radius=1.0)
ob2 = bpy.context.active_object
bpy.ops.object.mode_set(mode="EDIT")
bm = bmesh.from_edit_mesh(ob2.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
ridx = {v.index for v in bm.verts if abs(v.co.z) < 1e-4}
ring_edges = [e for e in bm.edges if e.verts[0].index in ridx and e.verts[1].index in ridx]
loops = mod.utils.sort_edges_into_loops(ring_edges)
note(len(loops) == 1 and loops[0][1] and len(loops[0][0]) == len(ridx),
     f"equator ring is one circular loop ({len(loops)} loops, circular={loops[0][1]})")

print("\n=== 3. the user scenario: whole-grid curvature, pinned border + guide vert ===")
ob = grid_setup()
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CURVATURE", name="Cv")
# raise one interior vertex as the guide, pin it together with the border
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
guide = min(bm.verts, key=lambda v: v.co.xy.length).index
bm.verts[guide].co.z = 0.5
border = {v.index for v in bm.verts if any(len(e.link_faces) == 1 for e in v.link_edges)}
for v in bm.verts: v.select = v.index in border or v.index == guide
bmesh.update_edit_mesh(ob.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="PIN", name="Pin")
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
pinned_before = {i: bm.verts[i].co.copy() for i in border | {guide}}
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=150)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
pin_moved = max((bm.verts[i].co - co).length for i, co in pinned_before.items())
note(pin_moved < 1e-9, f"pinned border and guide stay put ({pin_moved:.2e})")
zs = [v.co.z for v in bm.verts]
note(max(zs) <= 0.5 + 1e-6 and min(zs) > -0.5, f"surface stays bounded (z {min(zs):.3f}..{max(zs):.3f})")
# the guide must have lifted its surroundings: neighbours clearly above the floor
neighbor_z = [e.other_vert(bm.verts[guide]).co.z for e in bm.verts[guide].link_edges]
note(min(neighbor_z) > 0.1,
     f"curvature blends the surface up to the pinned guide (neighbour z {min(neighbor_z):.3f})")

print("\n=== 4. whole-grid spacing stays a grid ===")
ob = grid_setup()
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
n_side = round(math.sqrt(len(bm.verts)))
# make the interior spacing uneven, in the plane
for v in bm.verts:
    if abs(abs(v.co.x) - 1.0) > 1e-5 and abs(abs(v.co.y) - 1.0) > 1e-5:
        v.co.x += 0.05 * math.sin(v.co.y * 7 + 1)
        v.co.y += 0.05 * math.sin(v.co.x * 6 + 2)
bmesh.update_edit_mesh(ob.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SPACE", name="S")
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=150)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
worst = max(v.co.length for v in bm.verts)
note(worst < 2.0, f"grid stays bounded ({worst:.3f})")
# rows must still be rows: y nearly constant along each row after spacing
rows = {}
for v in bm.verts:
    rows.setdefault(round(v.co.y, 1), []).append(v)
row_wobble = max(
    max(u.co.y for u in vs) - min(u.co.y for u in vs) for vs in rows.values() if len(vs) > 2
)
note(row_wobble < 0.08, f"rows stay coherent (max y wobble within a row {row_wobble:.3f})")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
