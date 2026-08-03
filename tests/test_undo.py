"""Constraint undo: version-stamp mechanism, single Ctrl+Z, redo, memfile path."""
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

def ui_push(msg):
    # what the UI does automatically after an operator or property edit
    bpy.ops.ed.undo_push(message=msg)

def setup():
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=12, radius=1.0)
    obj = bpy.context.active_object
    obj.modifiers.new("Subdivision", "SUBSURF").levels = 2
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
    ui_push("baseline")
    return obj

def counts(obj):
    names = [a for a in obj.data.attributes.keys() if a.startswith("ft_constraint")]
    return len(obj.data.ft_custom_constraints), len(names)

print("=== 1. add constraint -> ONE undo removes it, still in edit mode ===")
obj = setup()
c0, a0 = counts(obj)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CIRCLE", name="C")
ui_push("Add Constraint")
c1, a1 = counts(obj)
note(c1 == c0 + 1, f"constraint added ({c0}->{c1})")
bpy.ops.ed.undo()
c2, a2 = counts(obj)
note(bpy.context.mode == "EDIT_MESH", f"stays in edit mode ({bpy.context.mode})")
note(c2 == c0 and a2 == a0,
     f"single undo removes constraint and attribute ({c1}->{c2} constraints, {a1}->{a2} attributes)")

print("\n=== 2. redo brings it back ===")
bpy.ops.ed.redo()
c3, a3 = counts(obj)
note(c3 == c1 and a3 == a1, f"redo restores constraint and attribute ({c2}->{c3})")

print("\n=== 3. delete constraint -> one undo brings it back with settings ===")
obj = setup()
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CIRCLE", name="C")
ui_push("Add Constraint")
obj.data.ft_custom_constraints[0].even_distribution = True
ui_push("Change Constraint")
c1, a1 = counts(obj)
bpy.ops.object.final_topology_delete_constraint()
ui_push("Delete Constraint")
c2, a2 = counts(obj)
note(c2 == c1 - 1, f"deleted ({c1}->{c2})")
bpy.ops.ed.undo()
c3, a3 = counts(obj)
ok = c3 == c1 and a3 == a1
if ok:
    c = obj.data.ft_custom_constraints[0]
    ok = c.constraint_type == "CIRCLE" and c.even_distribution
note(ok, f"one undo restores the constraint incl. settings ({c2}->{c3})")

print("\n=== 4. property change -> one undo reverts just the property ===")
obj = setup()
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CIRCLE", name="C")
ui_push("Add Constraint")
c = obj.data.ft_custom_constraints[0]
note(not c.even_distribution, "even off before")
c.even_distribution = True   # fires the update callback, which stamps
ui_push("Change Constraint")
bpy.ops.ed.undo()
c = obj.data.ft_custom_constraints[0]
note(len(obj.data.ft_custom_constraints) == 1 and not c.even_distribution,
     f"undo reverts the property, constraint stays ({len(obj.data.ft_custom_constraints)} constraints, even={c.even_distribution})")

print("\n=== 5. add selection -> one undo restores the attribute values ===")
obj = setup()
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CIRCLE", name="C")
ui_push("Add Constraint")
c = obj.data.ft_custom_constraints[0]
def marked_count(obj, name):
    bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    layer = bm.edges.layers.float.get(name)
    return sum(1 for e in bm.edges if e[layer] == 1.0) if layer else -1
n1 = marked_count(obj, c.attribute_name)
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
zs = sorted({round(v.co.z, 5) for v in bm.verts if v.co.z > 0.1})
ring2 = {v.index for v in bm.verts if abs(v.co.z - zs[0]) < 1e-6}
for v in bm.verts: v.select = False
for e in bm.edges: e.select = False
for e in bm.edges:
    if e.verts[0].index in ring2 and e.verts[1].index in ring2:
        e.select = True; e.verts[0].select = True; e.verts[1].select = True
bmesh.update_edit_mesh(obj.data)
bpy.ops.object.final_topology_add_selection_to_constraint("EXEC_DEFAULT")
ui_push("Add Selection to Constraint")
c = obj.data.ft_custom_constraints[0]
n2 = marked_count(obj, c.attribute_name)
note(n2 > n1, f"selection added to the attribute ({n1}->{n2})")
bpy.ops.ed.undo()
c = obj.data.ft_custom_constraints[0]
n3 = marked_count(obj, c.attribute_name)
note(n3 == n1, f"one undo restores the attribute values ({n2}->{n3})")

print("\n=== 5b. remove selection from ALL constraints, undoable ===")
obj = setup()
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CIRCLE", name="C1")
ui_push("Add Constraint")
c1 = obj.data.ft_custom_constraints[0]
# a second, point-domain constraint on part of the same ring
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
ring_ids = {v.index for v in bm.verts if abs(v.co.z) < 1e-4}
ring_half = {v.index for v in bm.verts if v.index in ring_ids and v.co.x > 0}
for v in bm.verts: v.select = v.index in ring_ids
bmesh.update_edit_mesh(obj.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="PIN", name="P1")
ui_push("Add Constraint")
c2 = obj.data.ft_custom_constraints[1]
def edge_marks(name):
    bm = bmesh.from_edit_mesh(obj.data); _KEEP.append(bm)
    layer = bm.edges.layers.float.get(name)
    return sum(1 for e in bm.edges if e[layer] == 1.0) if layer else -1
def vert_marks(name):
    bm = bmesh.from_edit_mesh(obj.data); _KEEP.append(bm)
    layer = bm.verts.layers.float.get(name)
    return sum(1 for v in bm.verts if v[layer] == 1.0) if layer else -1
e1, v1 = edge_marks(c1.attribute_name), vert_marks(c2.attribute_name)
note(e1 > 0 and v1 > 0, f"both constraints marked ({e1} edges, {v1} verts)")
# select half of the ring and pull it out of every constraint at once
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
for v in bm.verts: v.select = v.index in ring_half
for e in bm.edges: e.select = e.verts[0].index in ring_half and e.verts[1].index in ring_half
bmesh.update_edit_mesh(obj.data)
r = bpy.ops.object.final_topology_remove_selection_from_all("EXEC_DEFAULT")
ui_push("Remove Selection from All Constraints")
c1 = obj.data.ft_custom_constraints[0]; c2 = obj.data.ft_custom_constraints[1]
e2, v2 = edge_marks(c1.attribute_name), vert_marks(c2.attribute_name)
note(r == {'FINISHED'} and e2 < e1 and v2 < v1,
     f"selection removed from both domains ({e1}->{e2} edges, {v1}->{v2} verts)")
bpy.ops.ed.undo()
c1 = obj.data.ft_custom_constraints[0]; c2 = obj.data.ft_custom_constraints[1]
e3, v3 = edge_marks(c1.attribute_name), vert_marks(c2.attribute_name)
note(e3 == e1 and v3 == v1, f"one undo restores all assignments ({e2}->{e3} edges, {v2}->{v3} verts)")

print("\n=== 5c. reorder constraints, undoable ===")
obj = setup()
for name, ctype in (("A", "CIRCLE"), ("B", "SPACE"), ("C", "CURVATURE")):
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type=ctype, name=name)
    ui_push("Add Constraint")
def order():
    return [c.name for c in obj.data.ft_custom_constraints]
note(order() == ["A", "B", "C"], f"initial order ({order()})")
note(obj.data.ft_custom_constraints_index == 2, "last added is active")
r = bpy.ops.object.final_topology_move_constraint("EXEC_DEFAULT", direction="UP")
ui_push("Move Constraint")
note(r == {'FINISHED'} and order() == ["A", "C", "B"], f"active moved up ({order()})")
note(obj.data.ft_custom_constraints_index == 1, "active index follows the constraint")
bpy.ops.object.final_topology_move_constraint("EXEC_DEFAULT", direction="UP")
ui_push("Move Constraint")
note(order() == ["C", "A", "B"] and obj.data.ft_custom_constraints_index == 0,
     f"moved to the top ({order()})")
r = bpy.ops.object.final_topology_move_constraint("EXEC_DEFAULT", direction="UP")
note(r == {'CANCELLED'} and order() == ["C", "A", "B"], "moving past the top does nothing")
bpy.ops.ed.undo()
note(order() == ["A", "C", "B"], f"one undo restores the previous order ({order()})")
bpy.ops.object.final_topology_move_constraint("EXEC_DEFAULT", direction="DOWN")
ui_push("Move Constraint")
note(order() == ["A", "B", "C"] and obj.data.ft_custom_constraints_index == 2,
     f"down goes back ({order()})")

print("\n=== 6. mesh edits in between survive constraint undo ===")
obj = setup()
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CIRCLE", name="C")
ui_push("Add Constraint")
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
poke_index = bm.verts[0].index
old_co = bm.verts[0].co.copy()
bm.verts[0].co += Vector((0.3, 0.0, 0.0))
bmesh.update_edit_mesh(obj.data)
ui_push("Move Vert")
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SPACE", name="S")
ui_push("Add Constraint")
note(len(obj.data.ft_custom_constraints) == 2, "two constraints stacked")
bpy.ops.ed.undo()  # undo the second add only
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
moved = (bm.verts[poke_index].co - old_co).length
note(len(obj.data.ft_custom_constraints) == 1 and moved > 0.29,
     f"undo removes the last constraint, keeps the mesh edit ({len(obj.data.ft_custom_constraints)} constraints, vert offset {moved:.3f})")
bpy.ops.ed.undo()  # now the mesh edit
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
moved = (bm.verts[poke_index].co - old_co).length
note(len(obj.data.ft_custom_constraints) == 1 and moved < 1e-6,
     f"next undo reverts the mesh edit, constraint stays ({len(obj.data.ft_custom_constraints)} constraints, offset {moved:.2e})")

print("\n=== 7. curve creation stays undoable (memfile path, two undos) ===")
obj = setup()
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="CURVE", name="Cu")
ui_push("Add Constraint")
c = obj.data.ft_custom_constraints[0]
n_objects = len(bpy.data.objects)
bpy.ops.object.final_topology_curve_from_selection("EXEC_DEFAULT", point_mode="ORIGINAL")
ui_push("Create Constraint Curve")
note(len(bpy.data.objects) == n_objects + 1 and c.target_curve is not None, "curve created and linked")
bpy.ops.ed.undo(); bpy.ops.ed.undo()
obj = bpy.context.active_object
if bpy.context.mode != "EDIT_MESH":
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode="EDIT")
c = obj.data.ft_custom_constraints[0]
note(len(bpy.data.objects) == n_objects and c.target_curve is None,
     f"two undos remove the curve object and the link ({len(bpy.data.objects)} objects)")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
