"""The artists build (no extras.py): still registers and snaps.

Builds a copy of the addon without extras.py - like build.py does for the
artists variant - and runs it in a second, isolated Blender process, so the
installed PRO extension can't leak into the result.
"""
import os
import shutil
import subprocess
import sys
import tempfile

import bpy

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IGNORE = shutil.ignore_patterns(
    "extras.py", "build.py", "out", "tests", ".git*", "*.md", ".DS_Store"
)

fails = []
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

work = tempfile.mkdtemp(prefix="ft_artists_")
addon_dir = os.path.join(work, "scripts", "addons", "final_topology_artists")
shutil.copytree(REPO, addon_dir, ignore=IGNORE)
note(not os.path.exists(os.path.join(addon_dir, "extras.py")), "artists copy has no extras.py")

INNER = r'''
import bpy, bmesh
fails = []
def note(ok, msg):
    print(f"{'PASS' if ok else 'FAIL'}  {msg}")
    if not ok: fails.append(msg)

bpy.ops.preferences.addon_enable(module="final_topology_artists")
import final_topology_artists as ft
note(ft.has_extras is False, f"loads with has_extras False ({ft.has_extras})")
note(hasattr(bpy.ops.mesh, "final_topology_optimization_step"), "step operator registered")
note(hasattr(bpy.ops.mesh, "final_topology_modal"), "modal operator registered")
note(not hasattr(bpy.types, "OBJECT_OT_final_topology_add_constraint")
     or bpy.types.Mesh.bl_rna.properties.get("ft_custom_constraints") is None,
     "constraint system absent")
P = bpy.context.preferences.addons["final_topology_artists"].preferences
P.step_weight = 0.5

# the artists core flow: inverse subdivide snapping to the scene
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=24, radius=1.05)
bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=12, radius=1.0)
obj = bpy.context.active_object
obj.modifiers.new("Subdivision", "SUBSURF").levels = 2
bpy.ops.object.mode_set(mode="EDIT")
bpy.ops.mesh.select_all(action="SELECT")
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table()
r0 = sum(v.co.length for v in bm.verts) / len(bm.verts)
try:
    r = bpy.ops.mesh.final_topology_optimization_step("EXEC_DEFAULT", iterations=40)
    note(r == {"FINISHED"}, f"step runs ({r})")
except Exception as e:
    note(False, f"step raised: {type(e).__name__}: {str(e)[:120]}")
bm = bmesh.from_edit_mesh(obj.data); bm.verts.ensure_lookup_table()
r1 = sum(v.co.length for v in bm.verts) / len(bm.verts)
note(r1 > r0 + 0.01, f"cage snaps toward the target sphere ({r0:.4f} -> {r1:.4f})")

bpy.ops.preferences.addon_disable(module="final_topology_artists")
note(True, "addon disables cleanly")
print("INNER " + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
'''

inner_path = os.path.join(work, "inner_test.py")
with open(inner_path, "w") as f:
    f.write(INNER)

env = dict(os.environ)
env["BLENDER_USER_SCRIPTS"] = os.path.join(work, "scripts")
result = subprocess.run(
    [bpy.app.binary_path, "--background", "--factory-startup", "--python", inner_path],
    capture_output=True, text=True, env=env, timeout=300,
)
inner_ok = False
for line in result.stdout.splitlines():
    if line.startswith(("PASS", "FAIL")):
        print("  " + line)
        if line.startswith("FAIL"):
            fails.append(line)
    if line.startswith("INNER "):
        inner_ok = line == "INNER ALL PASSED"
note(inner_ok, "isolated artists Blender run passed")
if not inner_ok and result.stderr:
    print(result.stderr[-1500:])

shutil.rmtree(work, ignore_errors=True)
print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
sys.exit(1 if fails else 0)
