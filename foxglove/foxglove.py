#!/usr/bin/env python3
"""
A CLI to automatically install/uninstall Foxglove extensions and layouts.
usage: foxglove.py {install,uninstall} [--extensions] [--layouts]

To install a specific extension, use the -e flag:
python foxglove.py install -e <extension-1> <extension-2> ...
By default, the `-e` flag without arguments will install all extensions.

To install all layouts, use:
python foxglove.py install -l

For more information, use the -h flag:
python foxglove.py -h
python foxglove.py install -h
python foxglove.py uninstall -h
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

ORGANIZATION = "dukerobotics"

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


def run_at_path(command: str, directory: pathlib.Path, system: str = SYSTEM):
    """
    Run a command at a given path.

    Args:
        command: Command to run.
        directory: Path to run command at.
        system: If "Windows", run command with .cmd extension. Defaults to platform.system().

    Raises:
        ValueError: If command empty.
        subprocess.CalledProcessError: If command returns non-zero exit code.
    """
    if command == "":
        raise ValueError("Command must not be empty")

    args = command.split(' ')
    if system == "Windows":
        args[0] += ".cmd"

    print(f"{directory.name}: {command}")
    subprocess.run(args, cwd=directory, check=True)


def install_extensions(extension_paths: Sequence[pathlib.Path]):
    """
    Install custom Foxglove extensions.

    Args:
        extension_paths: Sequence of extension paths to install.
    """

    try:
        run_at_path("npm -v", FOXGLOVE_PATH)
    except (FileNotFoundError, subprocess.CalledProcessError):
        raise SystemExit("npm not found. Install npm and try again.")

    run_at_path("npm ci", FOXGLOVE_PATH)
    run_at_path("npx patch-package --patch-dir patches", FOXGLOVE_PATH)

    successes = 0
    for extension in extension_paths:
        run = functools.partial(run_at_path, directory=extension)

        if not (extension / "package.json").is_file():
            print(f"{extension.name}: skipped (no package.json)")
            continue

        run("npm run local-install")

        print(f"{extension.name}: installed")

        successes += 1

    print(f"Successfully installed {successes} extension(s)\n")


def install_layouts(layouts_path: pathlib.Path = LAYOUTS_PATH, install_path: pathlib.Path = LAYOUT_INSTALL_PATH):
    """
    Install custom Foxglove layouts.

    Args:
        layouts_path: Path to layouts directory.
        install_path: Path to install layouts to.
    """
    successes = 0
    for layout in layouts_path.glob("*.json"):
        with open(layout) as f:
            data = json.load(f)

        baseline = {
            "data": data,
            "savedAt": datetime.datetime.now(datetime.timezone.utc).isoformat()
        }

        id = f"{ORGANIZATION}.{layout.stem}"
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

        print(f"{name}: installed")

        successes += 1

    print(f"Successfully installed {successes} layout(s)\n")


def uninstall_extensions(install_path: pathlib.Path = EXTENSION_INSTALL_PATH):
    """
    Uninstall all Duke Robotics extensions from Foxglove.

    Duke Robotics extensions are identified with the prefix 'dukerobotics'.

    Args:
        install_path: Path where extensions are installed.
    """
    extensions = [d for d in install_path.iterdir() if d.name.startswith(ORGANIZATION)]
    for extension in extensions:
        shutil.rmtree(extension)
        print(f"{extension.name}: uninstalled")

    print(f"Successfully uninstalled {len(extensions)} extension(s)\n")


def uninstall_layouts(install_path: pathlib.Path = LAYOUT_INSTALL_PATH):
    """
    Uninstall all Duke Robotics layouts from Foxglove.

    Duke Robotics layouts are identified with the prefix 'dukerobotics'.

    Args:
        install_path: Path where layouts are installed.
    """
    layouts = [d for d in install_path.iterdir() if d.name.startswith(ORGANIZATION)]
    for layout in layouts:
        layout.unlink()
        print(f"{layout.name}: uninstalled")

    print(f"Successfully uninstalled {len(layouts)} layouts(s)\n")


def extension_package(name: str, extension_paths: Sequence[pathlib.Path] = EXTENSION_PATHS):
    """
    Type for argparse that checks if a given extension name is valid.

    Args:
        name: Name of extension to check.
        extension_paths: Sequence of extension paths to check against.

    Raises:
        argparse.ArgumentTypeError: If name is not a valid extension name.

    Returns:
        pathlib.Path: Full path to extension.
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
    VERBOSE = args.verbose

    if args.action == "install":
        # Without flags, install everything
        if args.extensions is None and args.layouts is False:
            args.extensions = EXTENSION_PATHS
            args.layouts = True
        # If only the -e flag is set without additional arguments, install all extensions
        if args.extensions == []:
            args.extensions = EXTENSION_PATHS

        if args.extensions is not None:
            install_extensions(args.extensions)
        if args.layouts:
            install_layouts()

    elif args.action == "uninstall":
        # Without flags, uninstall everything
        if args.extensions is False and args.layouts is False:
            args.extensions = True
            args.layouts = True

        if args.extensions:
            uninstall_extensions()
        if args.layouts:
            uninstall_layouts()
