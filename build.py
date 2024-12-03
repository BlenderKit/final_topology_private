import argparse
import os
import shutil
import toml

OUT_DIR = "out"
IGNORE_PATTERNS = [
    os.path.basename(__file__), # do not include this file
    OUT_DIR, # do not include output directory
    ".gitignore",
    "*.md",
    ".DS_Store",
]

def pro_changes(workdir: str):
    manifest = os.path.join(workdir, "blender_manifest.toml")
    with open(manifest, "r") as file:
        data = toml.load(file)

    data["id"] = "final_topology_pro"
    with open(manifest, "w") as file:
        toml.dump(data, file)


def do_build(install_at: str, pro_variant: str):
    if pro_variant:
        addon_name = "final_topology_pro"
    else:
        addon_name = "final_topology"
        IGNORE_PATTERNS.append("extras.py")

    print(f"Building {addon_name} addon...")
    src_dir = os.path.abspath(".")
    out_dir = os.path.abspath(OUT_DIR)
    addon_build_dir = os.path.join(out_dir, addon_name)
    shutil.rmtree(out_dir, ignore_errors=True)
    
    print("- copying files...")
    shutil.copytree(src_dir, addon_build_dir, ignore=shutil.ignore_patterns(*IGNORE_PATTERNS))

    if pro_variant:
        pro_changes(addon_build_dir)

    print("- creating archive...")
    shutil.make_archive(addon_build_dir, "zip", out_dir, addon_name)
    
    if install_at is not None:
        install_at = os.path.join(install_at, addon_name)
        print(f"- copying to {install_at}...")
        shutil.rmtree(install_at, ignore_errors=True)
        shutil.copytree(addon_build_dir, install_at)

    print("Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--install-at",
        type=str,
        default=None,
        help="If path is specified, then builded addon will be also copied to that location.",
    )
    parser.add_argument(
        "--pro",
        action='store_true',
        help="Set to True to build 'for CAD professionals' variant of the add-on.",
    )
    args = parser.parse_args()
    do_build(args.install_at, args.pro)
