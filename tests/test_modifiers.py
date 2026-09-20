"""Modifier handling around a run: settings are checked and only written
when they differ, restored at the end, and manual changes mid-run revert."""
import bpy, bmesh
MOD = "bl_ext.www_blenderkit_com.final_topology"
bpy.ops.preferences.addon_enable(module=MOD)
ft = __import__(MOD + ".final_topology", fromlist=["x"])
P = bpy.context.preferences.addons[MOD].preferences
P.enable_draw_constraints = False; P.use_mirror = False
_KEEP = []
fails = []
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

def fresh(levels=None, extra=False):
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    bpy.ops.mesh.primitive_uv_sphere_add(segments=8, ring_count=6, location=(0, 0, 0))
    ob = bpy.context.active_object
    if levels is not None:
        ob.modifiers.new("Subdivision", "SUBSURF").levels = levels
    if extra:
        ob.modifiers.new("Bevel", "BEVEL")
    bpy.ops.object.mode_set(mode="EDIT")
    bm = bmesh.from_edit_mesh(ob.data); _KEEP.append(bm)
    for v in bm.verts: v.select = True
    bmesh.update_edit_mesh(ob.data)
    return ob

print("=== 1. a conforming object is never written to ===")
ob = fresh(levels=2)
note(ft.apply_modifier_rules(ob) == 0, "rules find nothing to change on a level-2 subdivision")
state = ft.set_modifiers_start(ob)
note(ob.modifiers[0].levels == 2 and len(ob.modifiers) == 1, "start leaves it as it is")
note(ft.set_modifiers_end(ob, state) == 0, "end writes nothing either")

print("\n=== 2. level 3 plus a bevel: clamped and hidden for the run, restored after ===")
ob = fresh(levels=3, extra=True)
state = ft.set_modifiers_start(ob)
sub, bevel = ob.modifiers[0], ob.modifiers[1]
note(sub.levels == 2 and not bevel.show_viewport and not bevel.show_in_editmode, "run state: level 2, bevel hidden")
note(ft.apply_modifier_rules(ob) == 0, "a second check writes nothing")
# nothing to snap to here, so the step bails out - the modifier handling
# around it is what matters
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=1)
note(sub.levels == 2 and not bevel.show_viewport, "a Step inside the run keeps the run state")
note(ft.set_modifiers_end(ob, state) == 3 and sub.levels == 3 and bevel.show_viewport and bevel.show_in_editmode,
     f"end restores level 3 and the bevel (levels {sub.levels}, bevel {bevel.show_viewport})")

print("\n=== 3. a manual change during the run is reverted by the next step ===")
ob = fresh(levels=2)
state = ft.set_modifiers_start(ob)
ob.modifiers[0].levels = 4
class FakeOp:
    def __init__(self, ob): self.object = ob
    def report(self, *a): pass
ft.final_topology_optimization_step(FakeOp(ob), bpy.context, 1, 1)
note(ob.modifiers[0].levels == 2, f"step reverted the manual level 4 back to 2 ({ob.modifiers[0].levels})")
ob.modifiers[0].show_in_editmode = False
ft.final_topology_optimization_step(FakeOp(ob), bpy.context, 1, 1)
note(ob.modifiers[0].show_in_editmode, "and switches the subdivision back on in edit mode")
ft.set_modifiers_end(ob, state)
note(ob.modifiers[0].levels == 2, "end restores the recorded level, not the manual one")

print("\n=== 4. no modifiers: a temporary subdivision comes and goes ===")
ob = fresh()
state = ft.set_modifiers_start(ob)
note(len(ob.modifiers) == 1 and ob.modifiers[0].type == "SUBSURF" and ob.modifiers[0].levels == 2, "start adds a level-2 subdivision")
ft.set_modifiers_end(ob, state)
note(len(ob.modifiers) == 0, "end removes it again")
r = bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=1)
note(len(ob.modifiers) == 0, f"a Step on a bare mesh leaves no modifier behind ({r})")

print("\n=== 5. an object deleted mid-run has nothing to restore ===")
ob = fresh(levels=3)
state = ft.set_modifiers_start(ob)
bpy.ops.object.mode_set(mode="OBJECT")
bpy.data.objects.remove(ob)
try:
    writes = ft.set_modifiers_end(ob, state)
    ok = writes == 0
except ReferenceError:
    ok = False
note(ok, "end on a removed object returns quietly instead of raising")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
