# Final Topology - Development Guide

## Build

To build the add-on in format compatible with extensions format and legacy format, run:

```
python build.py
```

Add-on .zip will be present in `./out/final_topology.zip`.
To automatically install the add-on into Blender scripts, you can use `--install-at <path-to-blender-scripts-folder>`.


## Releasing

1. Make sure that version in `bl_info` is the same as the version in `blender_manifest.toml`.
2. Check the versions are the same again.
3. Run the build.
