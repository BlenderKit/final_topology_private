"""Pin constraint: frozen against constraints and inverse subdivision."""
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

def ring_setup():
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=12, radius=1.0)
    obj = bpy.context.active_object
    obj.modifiers.new("Subdivision", "SUBSURF").levels = 2
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    ring = [v for v in bm.verts if abs(v.co.z) < 1e-4]
    ring.sort(key=lambda v: math.atan2(v.co.y, v.co.x))
    ridx = [v.index for v in ring]
    for v in bm.verts: v.select = False
    for e in bm.edges: e.select = False
    for e in bm.edges:
        if e.verts[0].index in set(ridx) and e.verts[1].index in set(ridx):
            e.select = True; e.verts[0].select = True; e.verts[1].select = True
    bmesh.update_edit_mesh(obj.data)
    return obj, ridx

print("=== 1. pinned verts resist a space constraint ===")
obj, ridx = ring_setup()
# uneven-ify the ring so SPACE has work to do
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
for k, i in enumerate(ridx):
    v = bm.verts[i]
    a = math.atan2(v.co.y, v.co.x) + 0.25 * math.sin(k * 2.0) * (2 * math.pi / len(ridx))
    v.co.x, v.co.y = math.cos(a), math.sin(a)
bmesh.update_edit_mesh(obj.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="SPACE", name="S")
# pin two of the ring verts
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
pinned = ridx[:2]
for v in bm.verts: v.select = False
for i in pinned: bm.verts[i].select = True
bmesh.update_edit_mesh(obj.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="PIN", name="Pin")
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
before_pinned = {i: bm.verts[i].co.copy() for i in pinned}
before_free = {i: bm.verts[i].co.copy() for i in ridx[4:8]}
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=40)
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
pin_moved = max((bm.verts[i].co - co).length for i, co in before_pinned.items())
free_moved = max((bm.verts[i].co - co).length for i, co in before_free.items())
note(pin_moved < 1e-9, f"pinned verts frozen ({pin_moved:.2e})")
note(free_moved > 1e-3, f"free verts still solved ({free_moved:.4f})")

print("\n=== 2. pin overlay markers queued even with constraint overlays off ===")
P.enable_draw_constraints = False
mod.draw.clear_draw_list()
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=1)
note(len(mod.draw.draw_pins) == len(pinned),
     f"one red square per pinned vert ({len(mod.draw.draw_pins)} for {len(pinned)})")
note(all(n is not None for _, n in mod.draw.draw_pins), "every pin carries its world normal")
# facing test: a pin on the near side shows, one on the far side is culled in
# solid shading; without a normal it always shows
from mathutils import Vector as _V
eye = _V((0, 0, 5))
note(mod.draw.pin_faces_viewer((0, 0, 1), (0, 0, 1), eye, None) and not mod.draw.pin_faces_viewer((0, 0, -1), (0, 0, -1), eye, None),
     "perspective: pin facing the eye shows, pin facing away is culled")
note(mod.draw.pin_faces_viewer((0, 0, 1), (0, 0, 1), None, _V((0, 0, 1))) and not mod.draw.pin_faces_viewer((0, 0, -1), (0, 0, -1), None, _V((0, 0, 1))),
     "orthographic: judged against the view direction")
note(mod.draw.pin_faces_viewer((0, 0, -1), None, eye, None), "a pin without a normal always shows")

# the squares follow the theme's vertex size, so they stay larger than
# enlarged vertices, e.g. when recording
draw = __import__(MOD + ".draw", fromlist=["x"])
theme = bpy.context.preferences.themes[0].view_3d
scale = bpy.context.preferences.system.ui_scale
theme.vertex_size = 3
small = draw.pin_half_size()
theme.vertex_size = 12
large = draw.pin_half_size()
theme.vertex_size = 3
P.overlays_alpha = 0.5
half = draw.pin_alpha()
P.overlays_alpha = 0.2
low = draw.pin_alpha()
P.overlays_alpha = 1.0
full = draw.pin_alpha()
note(abs(half - 0.5) < 1e-6 and abs(low - 0.2) < 1e-6 and abs(full - 1.0) < 1e-6,
     f"pins follow Overlays Alpha at double strength ({low:.2f} at 0.2, {half:.2f} at 0.5, {full:.2f} at 1.0)")
note(small >= 4.0 and large * 2 > 12 * scale and large > small, f"pin square outgrows the vertex dot ({small*2:.1f}px at size 3, {large*2:.1f}px at size 12, vertex {12*scale:.1f}px)")

print("\n=== 3. pinned verts resist inverse subdivision snapping ===")
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
# target first: a bigger sphere the working cage would inflate toward
bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=24, radius=1.05)
bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=12, radius=1.0)
obj = bpy.context.active_object
obj.modifiers.new("Subdivision", "SUBSURF").levels = 2
bpy.ops.object.mode_set(mode="EDIT")
bpy.context.tool_settings.mesh_select_mode = (True, False, False)
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
bpy.ops.mesh.select_all(action="SELECT")
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="INVERSE_SUBDIVIDE", name="IS")
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
hold = [v.index for v in bm.verts if v.co.z > 0.9]
for v in bm.verts: v.select = False
for i in hold: bm.verts[i].select = True
bmesh.update_edit_mesh(obj.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="PIN", name="Pin")
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
before_hold = {i: bm.verts[i].co.copy() for i in hold}
r0 = sum(v.co.length for v in bm.verts) / len(bm.verts)
bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=40)
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
hold_moved = max((bm.verts[i].co - co).length for i, co in before_hold.items())
free = [v for v in bm.verts if v.index not in set(hold)]
r1 = sum(v.co.length for v in free) / len(free)
note(hold_moved < 1e-9, f"pinned verts resist snapping ({hold_moved:.2e})")
note(r1 > r0 + 0.01, f"free verts snap toward the target sphere ({r0:.4f} -> {r1:.4f})")

print("\n=== 4. pin / unpin selection (Shift+P, Alt+P operators) ===")
obj, ridx = ring_setup()
mesh = obj.data
def pin_marks():
    bm = bmesh.from_edit_mesh(mesh); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    marked = set()
    for c in mesh.ft_custom_constraints:
        if c.constraint_type != "PIN" or not c.attribute_name: continue
        layer = bm.verts.layers.float.get(c.attribute_name)
        if layer:
            marked |= {v.index for v in bm.verts if v[layer] == 1.0}
    return marked
def select_only(ids):
    # a consistent vertex selection, as the UI would leave it: edges and
    # faces follow the verts, otherwise the operator's object-mode round
    # trip flushes stale face flags back down and selects everything
    bpy.context.tool_settings.mesh_select_mode = (True, False, False)
    bm = bmesh.from_edit_mesh(mesh); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    for f in bm.faces: f.select = False
    for e in bm.edges: e.select = False
    for v in bm.verts: v.select = v.index in ids
    bm.select_flush(True)
    bmesh.update_edit_mesh(mesh)
# no pin constraint yet: the toggle creates one from the selection
first = set(ridx[:3])
select_only(first)
r = bpy.ops.object.final_topology_pin_selection("EXEC_DEFAULT")
pins = [c for c in mesh.ft_custom_constraints if c.constraint_type == "PIN"]
note(r == {'FINISHED'} and len(pins) == 1, f"creates a pin constraint when none exists ({len(pins)})")
note(pin_marks() == first, f"selection pinned ({len(pin_marks())} verts)")
# partially new selection: adds to the existing pin, no second constraint
second = set(ridx[2:6])
select_only(second)
bpy.ops.object.final_topology_pin_selection("EXEC_DEFAULT")
pins = [c for c in mesh.ft_custom_constraints if c.constraint_type == "PIN"]
note(len(pins) == 1, "no duplicate pin constraint created")
note(pin_marks() == first | second, f"selection added to the existing pin ({len(pin_marks())} verts)")
# fully pinned selection: Shift+P never unpins, it just keeps the pins
select_only(second)
r = bpy.ops.object.final_topology_pin_selection("EXEC_DEFAULT")
note(pin_marks() == first | second, f"already-pinned selection stays pinned under Shift+P ({sorted(pin_marks())})")
# explicit unpin (Alt+P) frees it
bpy.ops.object.final_topology_pin_selection("EXEC_DEFAULT", unpin=True)
note(pin_marks() == first - second, f"Alt+P unpins the selection ({sorted(pin_marks())})")
# explicit unpin (Alt+P): a mixed selection only loses its pinned members,
# nothing gets pinned, and unpinning verts that aren't pinned is a no-op
remaining = pin_marks()
mixed = set(list(remaining)[:1]) | set(ridx[8:10])
select_only(mixed)
r = bpy.ops.object.final_topology_pin_selection("EXEC_DEFAULT", unpin=True)
note(r == {'FINISHED'} and pin_marks() == remaining - mixed and not (pin_marks() & mixed),
     f"Alt+P unpins only the pinned part of a mixed selection ({sorted(pin_marks())})")
select_only(set(ridx[8:10]))
r = bpy.ops.object.final_topology_pin_selection("EXEC_DEFAULT", unpin=True)
note(r == {'CANCELLED'} and pin_marks() == remaining - mixed, "Alt+P on unpinned verts changes nothing")
# the property must not be remembered: a plain call right after Alt+P pins
select_only(set(ridx[8:10]))
r = bpy.ops.object.final_topology_pin_selection("EXEC_DEFAULT")
note(r == {'FINISHED'} and set(ridx[8:10]) <= pin_marks(), f"Shift+P right after Alt+P still pins ({sorted(pin_marks())})")
km_items = [k for km in bpy.context.window_manager.keyconfigs.addon.keymaps for k in km.keymap_items if k.idname == "object.final_topology_pin_selection"]
note(all(k.properties.unpin == k.alt for k in km_items), "the bindings state unpin explicitly: Alt+P on, Shift+P off")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
