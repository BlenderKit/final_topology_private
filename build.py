import argparse
import os
import shutil
import toml

OUT_DIR = "out"
IGNORE_PATTERNS = [
    os.path.basename(__file__), # do not include this file
    OUT_DIR, # do not include output directory
    ".gitignore",
    ".git",       # the repository itself must never ship
    ".claude",
    "__pycache__",
    "*.pyc",
    "*.md",
    ".DS_Store",
    "tests", # test suites are for development only
]

def variant_changes(workdir: str, pro_variant: bool):
    """Stamp the distribution identity of the variant into its manifest."""
    manifest = os.path.join(workdir, "blender_manifest.toml")
    with open(manifest, "r") as file:
        data = toml.load(file)

    if pro_variant:
        data["id"] = "final_topology_pro"
        data["name"] = "Final Topology for CAD professionals"
    else:
        data["name"] = "Final Topology for Artists"
    with open(manifest, "w") as file:
        toml.dump(data, file)


def do_build(install_at: str, pro_variant: bool):
    if pro_variant:
        addon_name = "final_topology_pro"
        ignore_patterns = IGNORE_PATTERNS.copy()
    else:
        addon_name = "final_topology"
        ignore_patterns = IGNORE_PATTERNS + ["extras.py"]

    print(f"Building {addon_name} addon...")
    src_dir = os.path.abspath(".")
    out_dir = os.path.abspath(OUT_DIR)
    addon_build_dir = os.path.join(out_dir, addon_name)
    
    # Read version from manifest
    manifest_path = os.path.join(src_dir, "blender_manifest.toml")
    with open(manifest_path, "r") as file:
        manifest_data = toml.load(file)
    version = manifest_data.get("version", "unknown")
    
    print("- copying files...")
    shutil.copytree(src_dir, addon_build_dir, ignore=shutil.ignore_patterns(*ignore_patterns))

    variant_changes(addon_build_dir, pro_variant)

    print("- creating archive...")
    # Create zip filename with version and appropriate suffix
    if pro_variant:
        zip_name = f"final_topology_{version}"
    else:
        zip_name = f"final_topology_{version}_artists"
    
    zip_path = os.path.join(out_dir, zip_name)
    shutil.make_archive(zip_path, "zip", out_dir, addon_name)
    
    if install_at is not None:
        install_at = os.path.join(install_at, addon_name)
        print(f"- copying to {install_at}...")
        shutil.rmtree(install_at, ignore_errors=True)
        shutil.copytree(addon_build_dir, install_at)

    print(f"Done building {addon_name}.")


def clean_output_dir():
    out_dir = os.path.abspath(OUT_DIR)
    print("Cleaning output directory...")
    shutil.rmtree(out_dir, ignore_errors=True)


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
    parser.add_argument(
        "--all",
        action='store_true',
        help="Build both regular and pro variants.",
    )
    args = parser.parse_args()
    
    if args.all and args.pro:
        print("Error: Cannot use --all and --pro together. Use either --all or --pro.")
        exit(1)
    
    clean_output_dir()
    
    if args.all:
        print("Building both variants...")
        do_build(args.install_at, False)  # regular variant
        do_build(args.install_at, True)   # pro variant
        print("All builds completed.")
    else:
        do_build(args.install_at, args.pro)
