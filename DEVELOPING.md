# Final Topology - Development Guide

## Build (for artists)

To build the `Final Topology for Artists` add-on in format compatible with extensions format and legacy format, run:

```
python build.py
```

Add-on .zip will be present in `./out/final_topology.zip`.
To automatically install the add-on into Blender scripts, you can use `--install-at <path-to-blender-scripts-folder>`.

## Build (for CAD Professionals)

To build the PRO variant of the add-on, add `--pro` flag to the build command.
This will automatically change the ID in the toml file and rename the final .zip file.

Please run:
```
python build.py --pro
```

Add-on .zip will be present in `./out/final_topology_pro.zip`.

To also automatically install the add-on, you can use:
```
python build.py --install-at <path-to-blender-scripts-folder> --pro
```


## Releasing

1. Make sure that version in `bl_info` is the same as the version in `blender_manifest.toml`.
2. Check the versions are the same again.
3. Run the build of basic for artists version, copy the zip from the ./out directory.
4. Run the build of PRO version with `--pro` flag, copy the zip from the ./out directory.

## Tests

Headless regression suites live in `tests/`. Each `test_*.py` runs the real
operators in background Blender against the installed PRO addon (the
constraints live in `extras.py`, so the artists build can't run them) and
prints PASS/FAIL lines ending with `ALL PASSED` or `FAILURES: [...]`.

Run everything:

```
./tests/run_all.sh
```

One suite, or a custom Blender binary:

```
./tests/run_all.sh test_undo.py
BLENDER=/path/to/blender ./tests/run_all.sh
```

Notes for writing tests:
- Primitives spawn at the 3D cursor, which is not guaranteed to be at the
  origin in a user's startup file - pass `location=(0, 0, 0)`, set
  `ob.location` explicitly, or compare in world space.
- Keep a module-level `_KEEP` list and append every `bmesh.from_edit_mesh`
  result to it, otherwise the bmesh wrappers can be freed mid-test.
- Simulate the UI's automatic undo step with
  `bpy.ops.ed.undo_push(message=...)` after operators when testing undo -
  background mode doesn't push it for you.
- The `blendkit_validator` errors in the console are normal background-mode
  noise from an unrelated addon.

The `tests/` folder is excluded from the built addon zips.
