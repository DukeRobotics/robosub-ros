#!/usr/bin/env python3
"""
A CLI to automatically install/uninstall Foxglove extensions and layouts.
usage: foxglove.py [--verbose] {install,uninstall} [--extensions] [--layouts]

To install a specific extension, use the -e flag:
python3 foxglove.py install -e <extension_name_1> <extension_name_2> ...

For more information, use the -h flag:
python3 foxglove.py -h
python3 foxglove.py install -h
python3 foxglove.py uninstall -h
"""

import subprocess
import functools
import json
import shutil
import pathlib
import datetime
import platform
import argparse
from typing import Sequence

if (SYSTEM := platform.system()) not in ("Linux", "Darwin", "Windows"):
    raise SystemExit(f"Unsupported platform: {SYSTEM}")
LAYOUT_INSTALL_PATH = {
    "Linux": pathlib.Path.home() / ".config/Foxglove Studio/studio-datastores/layouts-local/",
    "Darwin": pathlib.Path.home() / "Library/Application Support/Foxglove Studio/studio-datastores/layouts-local/",
    "Windows": pathlib.Path.home() / "AppData/Roaming/Foxglove Studio/studio-datastores/layouts-local/"
}[SYSTEM]
EXTENSION_INSTALL_PATH = pathlib.Path.home() / ".foxglove-studio/extensions/"

FOXGLOVE_PATH = pathlib.Path(__file__).parent.resolve()
EXTENSION_PATHS = [d for d in (FOXGLOVE_PATH / "extensions").iterdir() if d.is_dir()]
LAYOUTS_PATH = FOXGLOVE_PATH / "layouts"


def run_at_path(command: str, directory: pathlib.Path, system: str = SYSTEM, verbose: bool = False):
    """
    Run a command at a given path.

    Args:
        command: Command to run.
        directory: Path to run command at.
        verbose: Flag to print command output. Defaults to False.
        windows: Windows compatability. Defaults to False.

    Raises:
        ValueError: If command empty.
        Exception: If command returns non-zero exit code.
    """
    if command == "":
        raise ValueError("Command must not be empty")

    args = command.split(' ')
    if system == "Windows":
        args[0] += ".cmd"

    completed_process = subprocess.run(args, cwd=directory, capture_output=verbose, text=verbose)

    if verbose:
        print(f"{directory.name}: {command}")
        print(completed_process.stdout)


def install_extensions(extension_paths: Sequence[pathlib.Path], verbose: bool = False):
    """
    Install all extensions to Foxglove.

    Args:
        extension_paths: List of extension paths to install.
        verbose: Defaults to False.
    """

    try:
        run_at_path("npm -v", FOXGLOVE_PATH, verbose=verbose)
    except FileNotFoundError:
        raise SystemExit("npm not found. Install npm and try again.")
    try:
        run_at_path("yarn -v", FOXGLOVE_PATH, verbose=verbose)
    except FileNotFoundError:
        raise SystemExit("yarn not found. Install with `npm install -g yarn` and try again.")

    successes = 0
    for extension in extension_paths:
        run = functools.partial(run_at_path, directory=extension, verbose=verbose)

        if not (extension / "package.json").is_file():
            print(f"{extension.name}: skipped (no package.json)")
            continue

        run("yarn install")
        (extension / "yarn.lock").unlink()
        run("npm ci --legacy-peer-deps")
        run("npm run local-install")

        print(f"{extension.name}: installed")

        successes += 1

    print(f"Successfully installed {successes} extension(s)\n")


def install_layouts(layouts_path: pathlib.Path = LAYOUTS_PATH, install_path: pathlib.Path = LAYOUT_INSTALL_PATH):
    """
    Install all layout JSON files in `layout_path` to Foxglove.

    Args:
        layout_path: Path to layouts.
    """
    successes = 0
    for layout in layouts_path.glob("*.json"):
        with open(layout) as f:
            data = json.load(f)

        baseline = {
            "data": data,
            "savedAt": datetime.datetime.now(datetime.timezone.utc).isoformat()
        }

        id = f"dukerobotics.{layout.stem}"
        name = layout.stem
        packaged_layout = {
            "id": id,
            "name": name,
            "permission": "CREATOR_WRITE",
            "baseline": baseline,
        }

        with open(install_path / id, 'w') as f:
            json.dump(packaged_layout, f)

        print(f"{name}: installed")

        successes += 1

    print(f"Successfully installed {successes} layout(s)\n")


def uninstall_extensions(install_path: pathlib.Path = EXTENSION_INSTALL_PATH):
    """
    Uninstall all Duke Robotics extensions from Foxglove.

    Duke Robotics extensions are identified with the prefix 'dukerobotics'.
    """
    extensions = [d for d in install_path.iterdir() if d.name.startswith("dukerobotics")]
    for extension in extensions:
        shutil.rmtree(extension)
        print(f"{extension.name}: uninstalled")

    print(f"Successfully uninstalled {len(extensions)} extension(s)\n")


def uninstall_layouts(install_path: pathlib.Path = LAYOUT_INSTALL_PATH):
    """
    Uninstall all Duke Robotics layouts from Foxglove.

    Duke Robotics layouts are identified with the prefix 'dukerobotics'.
    """
    layouts = [d for d in install_path.iterdir() if d.name.startswith("dukerobotics")]
    for layout in layouts:
        layout.unlink()
        print(f"{layout.name}: uninstalled")

    print(f"Successfully uninstalled {len(layouts)} layouts(s)\n")


def extension_package(name: str, extension_paths: Sequence[pathlib.Path] = EXTENSION_PATHS):
    """
    Type for argparse that checks if a given extension name is valid.

    Args:
        name: Name of extension.
        extension_paths: Defaults to EXTENSION_PATHS.

    Raises:
        argparse.ArgumentTypeError: If name is not a valid extension name.

    Returns:
        pathlib.Path: Path to extension.
    """
    for extension in extension_paths:
        if name == extension.name:
            return extension

    raise argparse.ArgumentTypeError(f"{name} is not a valid extension name")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Install/Uninstall Foxglove extensions and layouts.")

    parser.add_argument('-v', '--verbose', action='store_true', help="Print verbose output.")

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

    if args.action == "install":
        # Defaults
        if args.extensions is None and args.layouts is False:
            args.extensions = EXTENSION_PATHS
            args.layouts = True
        if args.extensions == []:
            args.extensions = EXTENSION_PATHS

        if args.extensions is not None:
            install_extensions(args.extensions, verbose=args.verbose)
        if args.layouts:
            install_layouts()

    elif args.action == "uninstall":
        # Defaults
        if args.extensions is False and args.layouts is False:
            args.extensions = True
            args.layouts = True

        if args.extensions:
            uninstall_extensions()
        if args.layouts:
            uninstall_layouts()
