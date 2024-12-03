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
