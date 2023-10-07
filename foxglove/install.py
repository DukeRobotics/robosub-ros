#!/usr/bin/env python3
# A convenience script to automatically install all Foxglove extensions.
# Foxglove extensions are defined as any folder matching '*-panel' in the current directory.
# Usage: python install.py (Must be run from the root of the foxglove directory)

# Dependencies: yarn and npm must both be installed and exposed on PATH.

# TODO: Check if package.json exists before running yarn/npm
import os
import subprocess
import functools


def run_at_path(command: str, directory: str, verbose: bool = True):
    """Helper function to run a command in a given directory."""
    if verbose:
        print(f"{directory}: {command}")

    args = command.split(' ')
    
    if os.name == 'nt':  # Windows
        args[0] += ".cmd"

    process = subprocess.Popen(args, cwd=directory, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output, error = process.communicate()
    if process.returncode != 0:
        raise Exception(f"Error executing command: {command}\n{error.decode()}")

    return output.decode()


def main():
    cwd = os.getcwd()

    leaf = os.path.basename(os.path.normpath(cwd))
    if leaf != "foxglove":
        raise Exception("./install.py must be run from the root of the foxglove directory")

    # Find all directories that match '*-panel'
    extensions = [d for d in os.listdir(cwd) if os.path.isdir(d) and d.endswith('-panel')]

    installs = 0
    for extension in extensions:
        run = functools.partial(run_at_path, directory=extension)

        # Yarn
        run("yarn install")
        if os.path.exists("yarn.lock"):
            os.remove("yarn.lock")  # We use NPM not Yarn

        # NPM
        run("npm ci --legacy-peer-deps")

        # Build the extension
        run("npm run local-install")

        print()

        installs += 1

    print(f"Successfully installed {installs} extensions")


if __name__ == "__main__":
    main()
