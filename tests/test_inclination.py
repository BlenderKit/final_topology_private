"""Inclination limit: free axis vector, presets, the fan overlay and the
view-aligned dial that turns the axis."""
import bpy, bmesh, math
from mathutils import Vector, Matrix
MOD = "bl_ext.www_blenderkit_com.final_topology"
bpy.ops.preferences.addon_enable(module=MOD)
mod = __import__(MOD, fromlist=["extras", "draw"])
ex, draw = mod.extras, mod.draw
P = bpy.context.preferences.addons[MOD].preferences
P.step_weight = 0.5; P.enable_draw_constraints = True; P.use_mirror = False
_KEEP = []
fails = []
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

def overhang_plane():
    """A grid tilted 30 degrees from facing straight down: an overhang for
    the default +Z (up) axis with 50 degrees max - its normal is 30 degrees
    from straight down, closer than the 40 the limit allows"""
    for o in list(bpy.data.objects): bpy.data.objects.remove(o)
    bpy.ops.mesh.primitive_grid_add(x_subdivisions=4, y_subdivisions=4, size=2, location=(0, 0, 0))
    ob = bpy.context.active_object
    ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
    bpy.ops.object.mode_set(mode="EDIT")
    bm = bmesh.from_edit_mesh(ob.data); _KEEP.append(bm)
    rot = Matrix.Rotation(math.radians(180 - 30), 3, "Y")   # normal from +Z to 30 deg off -Z
    for v in bm.verts: v.co = rot @ v.co
    bpy.context.tool_settings.mesh_select_mode = (False, False, True)
    for f in bm.faces: f.select = True
    for e in bm.edges: e.select = True
    for v in bm.verts: v.select = True
    bmesh.update_edit_mesh(ob.data)
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="INCLINATION_LIMIT", name="I")
    return ob, ob.data.ft_custom_constraints[0]

def coords(ob):
    bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
    return [v.co.copy() for v in bm.verts]

def step(n=1):
    bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=n)

print("=== 1. axis presets and the free vector ===")
ob, c = overhang_plane()
note(c.inclination_axis == "+Z" and (ex.inclination_axis_of(c) - Vector((0, 0, 1))).length < 1e-6, "default axis +Z, up")
c.inclination_axis = "+Y"
note((Vector(c.inclination_axis_vector) - Vector((0, 1, 0))).length < 1e-6, "choosing a preset also sets the free vector")
c.inclination_axis = "CUSTOM"
c.inclination_axis_vector = (1.0, 0.0, -1.0)
note((ex.inclination_axis_of(c) - Vector((1, 0, -1)).normalized()).length < 1e-6, "custom uses the normalized vector")

print("\n=== 2. the solver takes the vector: custom +Z equals the preset ===")
ob, c = overhang_plane()
before = coords(ob)
step()
preset = coords(ob)
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
for v, co in zip(bm.verts, before): v.co = co
bmesh.update_edit_mesh(ob.data)
c.inclination_axis = "CUSTOM"; c.inclination_axis_vector = (0.0, 0.0, 1.0)
step()
custom = coords(ob)
moved = max((a - b).length for a, b in zip(preset, before))
note(moved > 1e-4, f"the tilted plane is an overhang, it moves ({moved:.4f})")
note(max((a - b).length for a, b in zip(preset, custom)) < 1e-9, "custom vector +Z gives the same step as the preset")
# the same plane facing up is no overhang at all
bm = bmesh.from_edit_mesh(ob.data); bm.verts.ensure_lookup_table(); _KEEP.append(bm)
for v, co in zip(bm.verts, before): v.co = co
bmesh.update_edit_mesh(ob.data)
bpy.ops.mesh.select_all(action="SELECT"); bpy.ops.mesh.flip_normals()
c.inclination_axis = "+Z"
step()
note(max((a - b).length for a, b in zip(coords(ob), before)) < 1e-9, "flipped to face up, the plane is within the limit and nothing moves")

print("\n=== 2b. flat caps are skipped ===")
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
bpy.ops.mesh.primitive_cube_add(size=2, location=(0, 0, 0))
ob = bpy.context.active_object
ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
bpy.ops.object.mode_set(mode="EDIT")
bpy.context.tool_settings.mesh_select_mode = (False, False, True)
bm = bmesh.from_edit_mesh(ob.data); _KEEP.append(bm)
for f in bm.faces:
    f.select = abs(f.normal.z) > 0.5     # top and bottom caps only
    for v in f.verts: v.select = v.select or f.select
bmesh.update_edit_mesh(ob.data)
bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type="INCLINATION_LIMIT", name="I")
before = coords(ob)
step(20)
note(max((a - b).length for a, b in zip(coords(ob), before)) < 1e-9, "top and bottom caps of a cube: the bottom is a full overhang but flat, nothing moves")

print("\n=== 3. the cone overlay sits at the faces' centroid ===")
ob, c = overhang_plane()
draw.clear_draw_list()
step()
tris = draw.draw_colored_tris_pos
cols = draw.draw_colored_tris_col
# drawn from the mesh as it was when the step started: every triangle
# starts at that centroid, the apex
apex = Vector(tris[0])
apexes = [Vector(tris[i]) for i in range(0, len(tris), 3)]
note(len(tris) == (32 + 36) * 3 and all((a - apex).length < 1e-6 for a in apexes),
     f"32 cone triangles and 36 overhang-fan triangles, all from one apex ({len(tris)//3})")
note(apex.length < 0.06, f"the apex is the centroid of the tilted grid ({tuple(round(x, 3) for x in apex)})")
reds = sum(1 for i in range(0, len(cols), 3) if cols[i][0] > 0.9)
greens = sum(1 for i in range(0, len(cols), 3) if cols[i][1] > 0.8)
note(greens == 32 and reds == 36, f"green cone of allowed surfaces, red overhang fans ({greens} green, {reds} red)")
# the cone opens along the +Z axis, upward, with the half angle 50: its
# walls are the steepest allowed faces
rim = [Vector(tris[i + 1]) - apex for i in range(0, 32 * 3, 3)]
angles = [math.degrees(r.angle(Vector((0, 0, 1)))) for r in rim]
note(max(angles) - min(angles) < 1e-3 and abs(angles[0] - 50.0) < 1e-3, f"cone half angle {angles[0]:.2f} degrees off +Z, a true circle")
# the red fans reach from the wall down to the horizontal
red_dirs = [Vector(tris[i + 2]) - apex for i in range(32 * 3, len(tris), 3)]
red_angles = [math.degrees(r.angle(Vector((0, 0, 1)))) for r in red_dirs]
note(min(red_angles) > 50.0 - 1e-3 and abs(max(red_angles) - 90.0) < 1e-3, f"overhang fans span {min(red_angles):.1f} to {max(red_angles):.1f} degrees off up")
slant = rim[0].length
note(abs(max(r.length for r in rim) - slant) < 1e-6, "all rim points at the slant length")
key = (ob.data.name, c.name)
note(key in ex._inclination_centers and (ob.matrix_world @ ex._inclination_centers[key] - apex).length < 1e-6
     and abs(ex._inclination_slants[key] - slant) < 1e-6, "centroid and slant cached for the gizmo")
c.inclination_max_angle = 80.0
draw.clear_draw_list(); step()
apex2 = Vector(draw.draw_colored_tris_pos[0])
rim = [Vector(draw.draw_colored_tris_pos[i + 1]) - apex2 for i in range(0, 32 * 3, 3)]
note(abs(math.degrees(rim[0].angle(Vector((0, 0, 1)))) - 80.0) < 1e-3, "a larger max inclination opens the cone to 80 degrees")

print("\n=== 4. the rim handles set Max Inclination, like a spot light's ===")
ob, c = overhang_plane()
step()
target = ex.InclinationConeTarget(ob, c)
note(target.count() == 2, "two handles, one on each side of the cone")
apex = ob.matrix_world @ ex._inclination_centers[(ob.data.name, c.name)]
h0, h1 = target.location(0), target.location(1)
a0 = math.degrees((h0 - apex).angle(Vector((0, 0, 1)))); a1 = math.degrees((h1 - apex).angle(Vector((0, 0, 1))))
note(abs(a0 - 50.0) < 1e-4 and abs(a1 - 50.0) < 1e-4
     and (h0 - apex).cross(h1 - apex).length > 1e-9 and abs((h0 - apex).length - target.slant) < 1e-6,
     f"handles sit on the rim, 50 degrees off up on opposite sides ({a0:.4f}, {a1:.4f})")
# drag the first handle outward: a point 70 degrees off up opens the cone to 70
snap = target.snapshot(0)
wanted = apex + (Vector((0, 0, 1)) * math.cos(math.radians(70)) + target.side * math.sin(math.radians(70))) * target.slant
target.translate(0, snap, wanted - snap)
note(abs(c.inclination_max_angle - 70.0) < 1e-4, f"handle at 70 degrees off up means max inclination {c.inclination_max_angle:.1f}")
wanted = apex + (Vector((0, 0, 1)) * math.cos(math.radians(5)) + target.side * math.sin(math.radians(5))) * target.slant * 0.5
target.translate(0, snap, wanted - snap)
note(abs(c.inclination_max_angle - 5.0) < 1e-4, f"pulled in towards the up direction at any distance: {c.inclination_max_angle:.1f}")
note(c.inclination_axis == "+Z", "the axis itself is untouched by the handles")
target.translate(0, snap, Vector((0, 0, 0)))
note(abs(c.inclination_max_angle - 50.0) < 1e-4, "cancelling a drag puts the limit back")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
