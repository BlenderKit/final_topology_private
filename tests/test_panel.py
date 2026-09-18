"""The constraint panel draws for every constraint type without errors, and
shows the fields each type needs - drawn into a recording layout."""
import bpy, bmesh
MOD = "bl_ext.www_blenderkit_com.final_topology"
bpy.ops.preferences.addon_enable(module=MOD)
ex = __import__(MOD + ".extras", fromlist=["x"])
P = bpy.context.preferences.addons[MOD].preferences
P.enable_draw_constraints = False; P.use_mirror = False
_KEEP = []
fails = []
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

class FakeOp:
    def __setattr__(self, k, v): pass
class FakeLayout:
    """Records prop() calls; every other layout call returns a layout."""
    def __init__(self, record): self.__dict__["record"] = record
    def __setattr__(self, k, v): pass          # operator_context, scale_y, ...
    def prop(self, data, name, **kw): self.record.append(name); return None
    def operator(self, idname, **kw): self.record.append("op:" + idname); return FakeOp()
    def __getattr__(self, name):
        def call(*a, **kw): return FakeLayout(self.record)
        return call
class FakePanel:
    def __init__(self): self.record = []; self.layout = FakeLayout(self.record)

for o in list(bpy.data.objects): bpy.data.objects.remove(o)
bpy.ops.mesh.primitive_grid_add(x_subdivisions=4, y_subdivisions=4, size=2, location=(0, 0, 0))
ob = bpy.context.active_object
ob.modifiers.new("Subdivision", "SUBSURF").levels = 1
bpy.ops.object.mode_set(mode="EDIT")
bm = bmesh.from_edit_mesh(ob.data); _KEEP.append(bm)
for v in bm.verts: v.select = True
for e in bm.edges: e.select = True
for f in bm.faces: f.select = True
bmesh.update_edit_mesh(ob.data)
mesh = ob.data
panel_cls = ex.VIEW3D_PT_final_topology_constraints

def draw_for(constraint_type, setup=None):
    while len(mesh.ft_custom_constraints):
        mesh.ft_custom_constraints_index = 0
        bpy.ops.object.final_topology_delete_constraint("EXEC_DEFAULT")
    bpy.ops.object.final_topology_add_constraint("EXEC_DEFAULT", constraint_type=constraint_type, name=constraint_type)
    ac = mesh.ft_custom_constraints[mesh.ft_custom_constraints_index]
    if setup: setup(ac)
    panel = FakePanel()
    try:
        panel_cls.draw(panel, bpy.context)
    except Exception as e:
        return None, repr(e)
    return panel.record, None

types = [i.identifier for i in bpy.ops.object.final_topology_add_constraint.get_rna_type().properties["constraint_type"].enum_items]
print("=== 1. every constraint type draws ===")
for t in types:
    if t == "CURVE":
        continue   # needs a curve object; covered by the curve tests
    record, err = draw_for(t)
    note(err is None and record and "name" in record and "influence" in (record if t != "PIN" else ["influence"]),
         f"{t}: panel draws ({err or f'{len(record)} entries'})")

print("\n=== 2. inverse subdivide shows its target fields ===")
record, _ = draw_for("INVERSE_SUBDIVIDE", lambda ac: setattr(ac, "use_object_or_collection", "OBJECT"))
note("invsubdiv_target_object" in record, "object mode: target object field visible")
record, _ = draw_for("INVERSE_SUBDIVIDE", lambda ac: setattr(ac, "use_object_or_collection", "COLLECTION"))
note("invsubdiv_target_collection" in record, "collection mode: target collection field visible")
record, _ = draw_for("INVERSE_SUBDIVIDE", lambda ac: setattr(ac, "use_object_or_collection", "SCENE"))
note("invsubdiv_target_object" not in record and "invsubdiv_target_collection" not in record and "invsubdiv_normal_offset" in record,
     "scene mode: no target field, normal offset still there")

print("\n=== 3. type-specific fields ===")
record, _ = draw_for("SPACE", lambda ac: setattr(ac, "space_target", "RING"))
note("space_method" in record and "space_interpolation" not in record, "ring width hides the interpolation row")
record, _ = draw_for("CURVATURE")
note("curvature_direction" in record and "curvature_bridge_poles" in record, "curvature shows direction and bridge poles")
record, _ = draw_for("CIRCLE")
note("circle_same_radius" in record and "circle_radius" in record and "fix_circle" in record, "circle shows Same Radius without Fix Circle")
record, _ = draw_for("PIN")
note("influence" not in record, "pin has no influence slider")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
