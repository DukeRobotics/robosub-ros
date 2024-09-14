#!/usr/bin/env python3
"""
A CLI to automatically install/uninstall Foxglove extensions and layouts.

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
from typing import Sequence, Union
import time
import git

ORGANIZATION = "dukerobotics"

if (SYSTEM := platform.system()) not in ("Linux", "Darwin", "Windows"):
    raise SystemExit(f"Unsupported platform: {SYSTEM}")

STUDIO_DATASTORES_PATH = {
    "Linux": pathlib.Path.home() / ".config/Foxglove Studio/studio-datastores/",
    "Darwin": pathlib.Path.home() / "Library/Application Support/Foxglove Studio/studio-datastores/",
    "Windows": pathlib.Path.home() / "AppData/Roaming/Foxglove Studio/studio-datastores/"
}[SYSTEM]
LAYOUT_INSTALL_PATH = STUDIO_DATASTORES_PATH / "layouts-local/"
EXTENSION_INSTALL_PATH = pathlib.Path.home() / ".foxglove-studio/extensions/"

FOXGLOVE_PATH = pathlib.Path(__file__).parent.resolve()
EXTENSION_PATHS = [d for d in (FOXGLOVE_PATH / "extensions").iterdir() if d.is_dir()]
LAYOUTS_PATH = FOXGLOVE_PATH / "layouts"


def run_at_path(command: Union[str, Sequence[str]], directory: pathlib.Path, system: str = SYSTEM,
                windows_append_cmd: bool = True):
    """
    Run a command at a given path.

    Args:
        command: Command to run.
        directory: Path to run command at.
        system: Can be "Linux", "Darwin", or "Windows". Defaults to platform.system().
        windows_append_cmd: If True, append ".cmd" to command on Windows systems.

    Raises:
        ValueError: If command empty.
        subprocess.CalledProcessError: If command returns non-zero exit code.
    """
    if not command:
        raise ValueError("Command must not be empty")

    if isinstance(command, str):
        args = command.split(' ')
    else:
        args = list(command)

    if system == "Windows" and windows_append_cmd:
        args[0] += ".cmd"

    print(f"{directory.name}: {command}")
    subprocess.run(args, cwd=directory, check=True)


def check_npm():
    """
    Check if npm is installed.

    Raises:
        SystemExit: If npm not found.
    """
    try:
        run_at_path("npm -v", FOXGLOVE_PATH)
    except (FileNotFoundError, subprocess.CalledProcessError):
        raise SystemExit("npm not found. Install npm and try again.")


def check_foxglove_cli():
    """
    Check if the Foxglove CLI is installed.

    Raises:
        SystemExit: If the Foxglove CLI is not found.
    """
    try:
        run_at_path("foxglove version", FOXGLOVE_PATH)
    except (FileNotFoundError, subprocess.CalledProcessError):
        raise SystemExit("The Foxglove CLI was not found. Install the Foxglove CLI and try again.")


def build_deps(skip_ci: bool = False, extension_paths: Sequence[pathlib.Path] = EXTENSION_PATHS):
    """
    Build all necessary dependencies for Foxglove.

    Args:
        extension_paths: Sequence of extension paths to build.
    """
    run = functools.partial(run_at_path, directory=FOXGLOVE_PATH)

    # Create necessary paths (either directories or empty files) so that npm ci can symlink them to node_modules
    # Files must have a suffix, otherwise they are treated as directories
    paths = [
        FOXGLOVE_PATH / "shared/ros-typescript-generator/build/main/cli/cli.js"
    ]
    for path in paths:
        if path.suffix:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
        else:
            path.mkdir(parents=True, exist_ok=True)

    # Install external dependencies and symlink local dependencies
    if not skip_ci:
        run("npm ci")

        # Patch external dependencies
        run("npx patch-package --patch-dir patches")

    # Compile local shared dependencies
    dependencies = ["ros-typescript-generator", "defs", "theme"]  # Specify build order
    for dep in dependencies:
        run_at_path("npm run build --if-present", pathlib.Path("shared") / dep)

    # Compile local nonshared dependencies
    for extension in extension_paths:
        run_at_path("npm run build-deps --if-present", extension)


def install_extensions(extension_paths: Sequence[pathlib.Path]):
    """
    Install custom Foxglove extensions.

    Args:
        extension_paths: Sequence of extension paths to install.
    """
    successes = 0
    for extension in extension_paths:
        if not (extension / "package.json").is_file():
            print(f"{extension.name}: skipped (no package.json)")
            continue

        run_at_path("npm run local-install", extension)

        print(f"{extension.name}: installed")

        successes += 1

    print(f"Successfully installed {successes} extension(s)\n")


def publish_extensions(extension_paths: Sequence[pathlib.Path], version: str = None):
    """
    Publish custom Foxglove extensions.

    Args:
        extension_paths: Sequence of extension paths to publish.
        version: Version to publish extensions under. If None, the short HEAD commit hash is used.

    Raises:
        SystemExit: If the Foxglove directory is dirty and no version is given.
    """
    repo = git.Repo(path=FOXGLOVE_PATH.parent)
    is_dirty = repo.is_dirty(untracked_files=True, path=FOXGLOVE_PATH)
    if is_dirty and version is None:
        raise SystemExit("The foxglove directory is dirty! Commit changes before publishing.")
    if version is None:
        version = repo.head.object.hexsha[:7]

    successes = 0
    for extension in extension_paths:
        if not (extension / "package.json").is_file():
            print(f"{extension.name}: skipped (no package.json)")
            continue

        # Update package.json version
        with open(extension / "package.json", 'r') as file:
            package = json.load(file)
        package['version'] = version
        with open(extension / "package.json", 'w') as file:
            json.dump(package, file, indent=2)

        # Build extension package
        run_at_path("npm run package", extension)

        # Publish extension package
        package_name = f'{ORGANIZATION}.{extension.name}-{version}.foxe'
        try:
            run_at_path(f"foxglove extensions publish {package_name}", extension)
        finally:
            run_at_path(f"rm {package_name}", extension)

        print(f"{extension.name}: published")

        successes += 1

    print(f"Successfully published {successes} extension(s)\n")


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

        print(f"{id}: installed")

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


def uninstall_layouts():
    """
    Uninstall all Duke Robotics layouts from Foxglove.

    Duke Robotics layouts are identified with the prefix 'dukerobotics'.

    Args:
        install_path: Path where layouts are installed.
    """
    remote_layouts = [d for d in STUDIO_DATASTORES_PATH.iterdir() if d.name.startswith("layouts-remote")]

    successes = 0
    for path in (remote_layouts + [LAYOUT_INSTALL_PATH]):
        layouts = [d for d in path.iterdir() if d.name.startswith(ORGANIZATION)]
        for layout in layouts:
            layout.unlink()
            print(f"{layout.name}: uninstalled")

            successes += 1

    print(f"Successfully uninstalled {successes} layouts(s)\n")


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


def create_new_layout(name: str, install_path: pathlib.Path = LAYOUT_INSTALL_PATH):
    """
    Create a new layout in Foxglove Desktop.

    Foxglove does not allow creating more than one layout when not signed in.
    This script circumvents this issue.

    Args:
        name: Name of the new layout.
        install_path: Path to install layouts to.
    """
    with open("empty-layout.json") as f:
        data = json.load(f)

    baseline = {
        "data": data,
        "savedAt": datetime.datetime.now(datetime.timezone.utc).isoformat()
    }

    id = f"{ORGANIZATION}.{name}"
    packaged_layout = {
        "id": id,
        "name": name,
        "permission": "CREATOR_WRITE",
        "baseline": baseline,
    }

    with open(install_path / id, 'w') as f:
        json.dump(packaged_layout, f)


def clean_foxglove():
    """
    Clean up the foxglove monorepo.
    """
    run_at_path("git clean -fdx", FOXGLOVE_PATH, windows_append_cmd=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Install/Uninstall Foxglove extensions and layouts.")
    parser.add_argument(
        '--new-layout',
        nargs='?',
        const=str(time.time_ns()),  # Use current time if no name is given to avoid collisions
        action='store',
        help="Create a new layout by name. If no name is given, use current time in nanoseconds as name."
    )

    subparsers = parser.add_subparsers(dest="action")

    install_parser = subparsers.add_parser(
        'install',
        aliases=['i'],
        help='Install Foxglove extensions and layouts. By default, all extensions and layouts are installed.'
    )
    install_parser.add_argument(
        '-e', '--extensions',
        action='extend',
        nargs='*',
        type=extension_package,
        help="Install extension(s) by name. If no name(s) are given, all extensions are installed."
    )
    install_parser.add_argument(
        '-l', '--layouts',
        action='store_true',
        help="Install all layouts."
    )
    install_parser.add_argument(
        '--skip-ci',
        action='store_true',
        help="Use existing node_modules instead of clean installing external dependencies."
    )

    uninstall_parser = subparsers.add_parser(
        'uninstall',
        aliases=['u'],
        help='Uninstall Foxglove extensions and layouts. By default, all extensions and layouts are uninstalled.'
    )
    uninstall_parser.add_argument('-e', '--extensions', action='store_true', help="Uninstall all extensions.")
    uninstall_parser.add_argument('-l', '--layouts', action='store_true', help="Uninstall all layouts.")

    build_parser = subparsers.add_parser(
        'build',
        aliases=['b'],
        help='Build all necessary dependencies for Foxglove. This is automatically run when installing extensions.'
    )
    build_parser.add_argument(
        '--skip-ci',
        action='store_true',
        help="Use existing node_modules instead of clean installing external dependencies."
    )

    clean_parser = subparsers.add_parser(
        'clean',
        aliases=['c'],
        help='Clean up the foxglove monorepo.'
    )

    publish_parser = subparsers.add_parser(
        'publish',
        aliases=['p'],
        help='Publish Foxglove extensions. By default, all extensions are published.'
    )
    publish_parser.add_argument(
        '-e', '--extensions',
        action='extend',
        nargs='*',
        type=extension_package,
        help="Specify extension(s) to publish. If no name(s) are given, all extensions are published."
    )
    publish_parser.add_argument(
        '-v', '--version',
        action='store',
        nargs='?',
        help="Version to publish extensions under. If no version is given, the short HEAD commit hash is used."
    )

    args = parser.parse_args()

    if args.action in ("install", "i"):
        # Without flags, install everything
        if args.extensions is None and args.layouts is False:
            args.extensions = EXTENSION_PATHS
            args.layouts = True
        # If only the -e flag is set without additional arguments, install all extensions
        if args.extensions == []:
            args.extensions = EXTENSION_PATHS

        if args.extensions is not None:
            check_npm()
            build_deps(skip_ci=args.skip_ci, extension_paths=args.extensions)
            install_extensions(args.extensions)
        if args.layouts:
            install_layouts()

    elif args.action in ("publish", "p"):
        # Without flags, publish all extensions
        if args.extensions is None or args.extensions == []:
            args.extensions = EXTENSION_PATHS

        check_npm()
        check_foxglove_cli()
        publish_extensions(args.extensions, args.version)

    elif args.action in ("uninstall", "u"):
        # Without flags, uninstall everything
        if args.extensions is False and args.layouts is False:
            args.extensions = True
            args.layouts = True

        if args.extensions:
            uninstall_extensions()
        if args.layouts:
            uninstall_layouts()

    elif args.action in ("build", "b"):
        check_npm()
        build_deps(skip_ci=args.skip_ci)

    elif args.action in ("clean", "c"):
        clean_foxglove()

    if args.new_layout:
        create_new_layout(args.new_layout)
