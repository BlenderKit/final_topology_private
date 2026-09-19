"""Alt+C quick menu: add the selection to any constraint by index without
losing the selection, and the Select Active Constraint preference."""
import bpy, bmesh
MOD = "bl_ext.www_blenderkit_com.final_topology"
bpy.ops.preferences.addon_enable(module=MOD)
P = bpy.context.preferences.addons[MOD].preferences
P.enable_draw_constraints = False; P.use_mirror = False
_KEEP = []
fails = []
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

for o in list(bpy.data.objects): bpy.data.objects.remove(o)
bpy.ops.mesh.primitive_grid_add(x_subdivisions=6, y_subdivisions=6, size=2, location=(0, 0, 0))
ob = bpy.context.active_object
ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
bpy.ops.object.mode_set(mode="EDIT")
mesh = ob.data

def select_only(ids):
    bpy.context.tool_settings.mesh_select_mode = (True, False, False)
    bm = bmesh.from_edit_mesh(mesh); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    for f in bm.faces: f.select = False
    for e in bm.edges: e.select = False
    for v in bm.verts: v.select = v.index in ids
    bm.select_flush(True)
    bmesh.update_edit_mesh(mesh)
def selected():
    bm = bmesh.from_edit_mesh(mesh); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    return {v.index for v in bm.verts if v.select}
def members(index):
    bm = bmesh.from_edit_mesh(mesh); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    c = mesh.ft_custom_constraints[index]
    layer = bm.verts.layers.float.get(c.attribute_name)
    return {v.index for v in bm.verts if v[layer] == 1.0} if layer else set()

print("=== 1. two constraints, the second one active ===")
first, second, third = {0, 1, 2}, {10, 11, 12}, {20, 21, 22, 23}
select_only(first)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="PIN", name="A")
select_only(second)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SMOOTH", name="B")
note(mesh.ft_custom_constraints_index == 1 and members(0) == first and members(1) == second, "setup: A pins 3 verts, B smooths 3, B active")

print("\n=== 2. quick add by index keeps the selection and shows the target ===")
select_only(third)
r = bpy.ops.object.final_topology_add_selection_to_constraint("EXEC_DEFAULT", index=0)
note(r == {"FINISHED"} and members(0) == first | third, f"selection added to A while B was active ({sorted(members(0))})")
note(members(1) == second, "B untouched")
note(selected() == third, f"selection kept ({sorted(selected())})")
note(mesh.ft_custom_constraints_index == 0, "A became the active constraint")
# without index the active one is used, as before
select_only({30})
bpy.ops.object.final_topology_add_selection_to_constraint("EXEC_DEFAULT")
note(members(0) == first | third | {30}, "no index: goes to the active constraint")
# a constraint that lost its attribute gets a fresh one instead of a KeyError
mesh.ft_custom_constraints[1].attribute_name = ""
select_only({40, 41})
r = bpy.ops.object.final_topology_add_selection_to_constraint("EXEC_DEFAULT", index=1)
note(r == {"FINISHED"} and members(1) == {40, 41}, f"constraint without attribute gets a new one ({sorted(members(1))})")
try:
    r = bpy.ops.object.final_topology_add_selection_to_constraint("EXEC_DEFAULT", index=7)
except RuntimeError as e:   # reported errors raise when called from a script
    r = {"CANCELLED"} if "No such constraint" in str(e) else {"ERROR"}
note(r == {"CANCELLED"} and members(0) == first | third | {30} and members(1) == {40, 41}, "out-of-range index is refused")

print("\n=== 3. Select Active Constraint preference ===")
P.select_active_constraint = True
select_only({40})
mesh.ft_custom_constraints_index = 0
note(selected() == members(0), f"on: activating A selects its verts ({len(selected())})")
P.select_active_constraint = False
select_only({45, 46})
mesh.ft_custom_constraints_index = 1
note(selected() == {45, 46}, "off: activating B keeps the current selection")
mesh.ft_custom_constraints_index = 0
note(selected() == {45, 46}, "off: and so does activating A")
P.select_active_constraint = True
mesh.ft_custom_constraints_index = 1
note(selected() == members(1), "on again: activating B selects its verts")

print("\n=== 4. menus registered, Alt+C bound ===")
note(hasattr(bpy.types, "FT_MT_constraint_quick") and hasattr(bpy.types, "FT_MT_new_constraint"), "quick menu and New Mesh Constraint submenu exist")
kmi = []
for km in bpy.context.window_manager.keyconfigs.addon.keymaps:
    kmi += [k for k in km.keymap_items if k.idname == "wm.call_menu" and k.type == "C" and k.alt and not k.ctrl and not k.shift]
# enabling the addon in a test registers on top of the startup registration;
# registration purges stale copies, so exactly one binding remains
note(len(kmi) == 1 and kmi[0].properties.name == "FT_MT_constraint_quick", f"Alt+C calls the quick menu ({len(kmi)} binding(s))")
pins = []
for km in bpy.context.window_manager.keyconfigs.addon.keymaps:
    pins += [k for k in km.keymap_items if k.idname == "object.final_topology_pin_selection"]
note(len(pins) == 2 and sorted(bool(k.properties.unpin) for k in pins) == [False, True], f"one Shift+P and one Alt+P binding, no leftovers ({len(pins)})")
bpy.ops.preferences.addon_disable(module=MOD); bpy.ops.preferences.addon_enable(module=MOD)
# the cycle re-creates the preferences and reloads the module: the old
# references dangle (writing through the old P crashes Blender)
P = bpy.context.preferences.addons[MOD].preferences
P.enable_draw_constraints = False; P.use_mirror = False
ex = __import__(MOD + ".extras", fromlist=["x"])
pins = []
for km in bpy.context.window_manager.keyconfigs.addon.keymaps:
    pins += [k for k in km.keymap_items if k.idname == "object.final_topology_pin_selection"]
note(len(pins) == 2, f"a disable/enable cycle keeps it at two ({len(pins)})")
ex = __import__(MOD + ".extras", fromlist=["x"])
items = bpy.ops.object.final_topology_add_constraint.get_rna_type().properties["constraint_type"].enum_items
note(all(ex.constraint_type_icon(i.identifier) != "CONSTRAINT" for i in items), f"every constraint type has an icon for the menus ({len(items)} types)")

# draw both menus into a recording layout - the same calls the UI would make
class FakeOp:
    def __init__(self): self.__dict__["props"] = {}
    def __setattr__(self, k, v): self.props[k] = v
class FakeLayout:
    def __init__(self): self.calls = []; self.operator_context = None
    def operator(self, idname, text="", icon="NONE"):
        op = FakeOp(); self.calls.append(("operator", idname, text, icon, op)); return op
    def menu(self, idname, text="", icon="NONE"): self.calls.append(("menu", idname, text, icon, None))
    def separator(self): self.calls.append(("separator", None, None, None, None))
class FakeSelf:
    def __init__(self): self.layout = FakeLayout()
quick = FakeSelf(); ex.FT_MT_constraint_quick.draw(quick, bpy.context)
ops = [c for c in quick.layout.calls if c[0] == "operator"]
note(len(ops) == 2 and [c[2] for c in ops] == ["A", "B"] and [c[4].props["index"] for c in ops] == [0, 1]
     and quick.layout.calls[-1][:2] == ("menu", "FT_MT_new_constraint"),
     f"quick menu lists A and B by index and ends with the New Mesh Constraint submenu ({len(quick.layout.calls)} entries)")
new = FakeSelf(); ex.FT_MT_new_constraint.draw(new, bpy.context)
ops = [c for c in new.layout.calls if c[0] == "operator"]
note(len(ops) == len(items) and all(c[4].props["constraint_type"] == i.identifier and c[4].props["name"] == i.name for c, i in zip(ops, items)),
     f"submenu offers every constraint type with its label as the name ({len(ops)})")

print("\n=== 5. Shift+Alt+C removes ===")
kmi = []
for km in bpy.context.window_manager.keyconfigs.addon.keymaps:
    kmi += [k for k in km.keymap_items if k.idname == "wm.call_menu" and k.type == "C" and k.alt and k.shift and not k.ctrl]
note(len(kmi) == 1 and kmi[0].properties.name == "FT_MT_constraint_quick_remove", f"Shift+Alt+C calls the remove menu ({len(kmi)})")
rem = FakeSelf(); ex.FT_MT_constraint_quick_remove.draw(rem, bpy.context)
ops = [c for c in rem.layout.calls if c[0] == "operator"]
note([c[2] for c in ops[:2]] == ["A", "B"] and all(c[4].props["remove"] is True and c[4].props["index"] == i for i, c in enumerate(ops[:2]))
     and ops[-1][1] == "object.final_topology_remove_selection_from_all",
     "remove menu lists A and B with remove set, and From All last")
# removing by index while the other constraint is active
P.select_active_constraint = True
mesh.ft_custom_constraints_index = 1
select_only({0, 1})
r = bpy.ops.object.final_topology_add_selection_to_constraint("EXEC_DEFAULT", index=0, remove=True)
note(r == {"FINISHED"} and members(0) == (first | third | {30}) - {0, 1} and selected() == {0, 1},
     f"removed from A by index, selection kept ({sorted(members(0))})")
select_only({2})
r = bpy.ops.object.final_topology_add_selection_to_constraint("EXEC_DEFAULT", index=1, remove=True)
note(members(1) == {40, 41}, "removing verts that are not in B changes nothing")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
