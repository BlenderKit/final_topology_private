"""Per-constraint Influence scales how far a constraint moves its vertices."""
import bpy, bmesh
MOD = "bl_ext.www_blenderkit_com.final_topology"
bpy.ops.preferences.addon_enable(module=MOD)
P = bpy.context.preferences.addons[MOD].preferences
P.step_weight = 0.5; P.enable_draw_constraints = False; P.use_mirror = False
_KEEP = []
fails = []
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

for o in list(bpy.data.objects): bpy.data.objects.remove(o)
me = bpy.data.meshes.new("row")
xs = [0.0, 0.5, 1.5, 1.8, 3.2, 3.5, 4.9, 5.2, 6.0]
me.from_pydata([(x, 0.0, 0.0) for x in xs], [(i, i + 1) for i in range(len(xs) - 1)], [])
ob = bpy.data.objects.new("row", me); bpy.context.collection.objects.link(ob)
bpy.context.view_layer.objects.active = ob; ob.select_set(True)
ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
bpy.ops.object.mode_set(mode="EDIT")
bpy.context.tool_settings.mesh_select_mode = (False, True, False)
bm = bmesh.from_edit_mesh(me); _KEEP.append(bm)
for v in bm.verts: v.select = True
for e in bm.edges: e.select = True
bmesh.update_edit_mesh(me)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SPACE", name="S")
c = me.ft_custom_constraints[0]
c.space_method = "EVEN"
note(abs(c.influence - 1.0) < 1e-9, "influence defaults to 1")

def one_step(influence):
    bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    for i, x in enumerate(xs): bm.verts[i].co = (x, 0.0, 0.0)
    bmesh.update_edit_mesh(me)
    c.influence = influence
    bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=1)
    bm = bmesh.from_edit_mesh(me); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    return [bm.verts[i].co.x - x for i, x in enumerate(xs)]
full = one_step(1.0)
half = one_step(0.5)
none = one_step(0.0)
double = one_step(2.0)
moved = max(abs(d) for d in full)
note(moved > 1e-3, f"the constraint moves vertices at influence 1 (max {moved:.4f})")
note(max(abs(h - f * 0.5) for h, f in zip(half, full)) < 1e-6, "influence 0.5 moves exactly half as far")
note(max(abs(d) for d in none) < 1e-6, "influence 0 moves nothing")   # float32 coordinates
note(max(abs(d - f * 2.0) for d, f in zip(double, full)) < 1e-6, "influence 2 moves twice as far")

# the value rides the constraint undo state like every other property
c.influence = 1.0
bpy.ops.ed.undo_push(message="ui")
c.influence = 0.25
bpy.ops.ed.undo_push(message="ui")
bpy.ops.ed.undo()
note(abs(me.ft_custom_constraints[0].influence - 1.0) < 1e-9 or abs(me.ft_custom_constraints[0].influence - 0.25) < 1e-9,
     f"influence survives an undo step without breaking the list ({me.ft_custom_constraints[0].influence})")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
