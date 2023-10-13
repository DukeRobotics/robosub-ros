#!/usr/bin/env python3
# A convenience script to automatically install all Foxglove extensions.
# Foxglove extensions are defined as any folder matching '*-panel' in the current directory.
# Usage: python install.py (Must be run from the root of the foxglove directory)

# Dependencies: yarn and npm must both be installed and exposed on PATH.

import subprocess
import functools
import json
import pathlib
import datetime
import platform
import argparse
from typing import Sequence

VERBOSE = False

if (SYSTEM := platform.system()) not in ("Linux", "Darwin", "Windows"):
    raise Exception(f"Unsupported platform: {SYSTEM}")
LAYOUT_INSTALL_PATH = {
    "Linux": pathlib.Path.home() / ".config/Foxglove Studio/studio-datastores/layouts-local/",
    "Darwin": pathlib.Path.home() / "Library/Application Support/Foxglove Studio/studio-datastores/layouts-local/",
    "Windows": pathlib.Path.home() / "AppData/Roaming/Foxglove Studio/studio-datastores/layouts-local/"
}[SYSTEM]
EXTENSION_INSTALL_PATH = pathlib.Path.home() / ".foxglove-studio/extensions/"

FOXGLOVE_PATH = pathlib.Path(__file__).parent.resolve()
EXTENSIONS_PATH = FOXGLOVE_PATH / "extensions"
EXTENSION_PATHS = tuple(d for d in EXTENSIONS_PATH.iterdir() if d.is_dir())
LAYOUTS_PATH = FOXGLOVE_PATH / "layouts"


def run_at_path(command: str, directory: pathlib.Path):
    """Helper function to run a command in a given directory."""
    if command == "":
        raise ValueError("Command must not be empty")

    print(f"{directory.name}: {command}")

    args = command.split(' ')

    if SYSTEM == 'Windows':
        args[0] += ".cmd"

    process = subprocess.Popen(args, cwd=directory, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output, error = process.communicate()
    if process.returncode != 0:
        raise Exception(f"Error executing command: {command}\n{error.decode()}")

    if VERBOSE:
        print(output.decode())


def install_extensions(extensions: Sequence[pathlib.Path]):
    successes = 0
    for extension in extensions:
        run = functools.partial(run_at_path, directory=extension)

        if not (extension / "package.json").is_file():
            print(f"{extension.name}: skipped (no package.json)")
            continue

        run("yarn install")
        (extension / "yarn.lock").unlink()
        run("npm ci --legacy-peer-deps")
        run("npm run local-install")

        successes += 1

    print(f"Successfully installed {successes} extension(s)")


def install_layouts(layout_path: pathlib.Path, install_path: pathlib.Path):
    """
    Install all layout JSON files in `layout_path` to Foxglove.

    Args:
        layout_path: Path to layouts.
    """
    return
    layouts = layout_path.glob("*.json")
    for layout in layouts:
        with open(layout) as f:
            data = json.load(f)

        baseline = {
            "data": data,
            "savedAt": datetime.datetime.now().isoformat()
        }

        layout = {
            "id": f"dukerobotics.{layout.stem}",
            "name": layout.stem,
            "permission": "CREATOR_WRITE",
            "baseline": baseline,
        }

        with open('data.json', 'w') as f:
            json.dump(layout, f)


def uninstall_extensions():
    extensions = [d for d in EXTENSION_INSTALL_PATH.iterdir() if d.name.startswith("dukerobotics")]

    for extension in extensions:
        (extension).unlink()

    print(f"Successfully uninstalled {len(extensions)} extension(s)")


def uninstall_layouts():
    return
    layouts = [d for d in LAYOUT_INSTALL_PATH.iterdir() if d.name.startswith("dukerobotics")]
    for extension in layouts:
        (extension).unlink()

    print(f"Successfully uninstalled {len(layouts)} layouts(s)")


def extension_package(name: str):
    """
    Type for argparse that checks if a given extension name is valid.
    """
    for extension in EXTENSION_PATHS:
        if name == extension.name:
            return extension

    raise argparse.ArgumentTypeError(f"{name} is not a valid extension name")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Install/Uninstall Foxglove extensions and layouts.")
    subparsers = parser.add_subparsers(dest="action", required=True)

    install_parser = subparsers.add_parser(
        'install',
        help='Install Foxglove extensions and layouts. By default, all extensions and layouts are installed.'
    )
    install_parser.add_argument(
        '-e', '--extensions',
        action='extend',
        nargs='*',
        type=extension_package,
        help="Install extension(s) by name. By default, all extensions are installed."
    )
    install_parser.add_argument(
        '-l', '--layouts',
        action='store_true',
        help="Install all layouts."
    )

    uninstall_parser = subparsers.add_parser(
        'uninstall',
        help='Uninstall Foxglove extensions and layouts. By default, all extensions and layouts are uninstalled.'
    )
    uninstall_parser.add_argument('-e', '--extensions', action='store_true', help="Uninstall all extensions.")
    uninstall_parser.add_argument('-l', '--layouts', action='store_true', help="Uninstall all layouts.")

    args = parser.parse_args()

    print(args)

    if args.action == "install":
        # Defaults
        if args.extensions is None and args.layouts is False:
            args.extensions = EXTENSION_PATHS
            args.layouts = True
        if args.extensions == []:
            args.extensions = EXTENSION_PATHS

        if args.extensions is not None:
            install_extensions(args.extensions)
        if args.layouts:
            install_layouts(LAYOUTS_PATH, LAYOUT_INSTALL_PATH)

    elif args.action == "uninstall":
        # Defaults
        if args.extensions is False and args.layouts is False:
            args.extensions = True
            args.layouts = True

        if args.extensions:
            uninstall_extensions()
        if args.layouts:
            uninstall_layouts()
