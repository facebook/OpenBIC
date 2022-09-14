#!/usr/bin/env python3
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import argparse
import sys
import os
import subprocess


def verify_paths(source_path, destination_path):
    """
    Verify that both paths exist and examine the destination directory to see
    if it contains a .git directory signifying it's a git repo.

    :param source_path: File path to the patches root directory.
    :type source_path: str
    :param destination_path: File path to destination repo where patches will
                             be applied.
    :type destination_path: str
    """
    if os.path.isdir(source_path) is False:
        print(source_path + " does not appear to be a directory")
        sys.exit(1)
    else:
        print(source_path + "verified to exist.")

    if os.path.isdir(destination_path) is False:
        print(destination_path + " does not appear to be a directory")
        sys.exit(1)

    if ".git" not in os.listdir(destination_path):
        print(destination_path + "does not appear to contain a .git directory")
        print("Are you sure this is a git repository root directory?")
        sys.exit(1)
    else:
        print(destination_path + " appears to be a git repo.")


def gather_patches(source_path) -> list:
    """
    Recursively explore directories and gather all patch files (.patch).
    Patches will be ordered alphabetically by file name and grouped
    alphabetically by directory.

    :param source_path: File path to the patches root directory.
    :type source_path: list (str)
    :return: The paths of all patches to apply.
    :rtype: list
    """
    patches = []
    for (dirpath, _, filenames) in os.walk(source_path):
        for file in filenames:
            full_path = os.path.join(dirpath, file)
            patches.append(full_path)

    patches = list(filter(lambda x: x.endswith(".patch"), patches))

    print("Discovered the following patches:")
    for patch in patches:
        print("\t" + patch)

    return patches


def patches_unwind(applied_patch_list, destination_path):
    """
    Attempt to remove the patches in the order they are found in patch_list.
    If this function fails then the repo may be left in an invalid state.

    :param patch_list: List of paths to the patches to undo
    :type patch_list: list
    :param destination_path: Path to the repo to revert the patches
    :type destination_path: str
    """

    print("Attempting to remove applied patches in order...")

    for patch in applied_patch_list:
        command = [
            "git",
            "apply",
            "--unsafe-paths",
            "-R",
            patch,
            "--directory=",
            destination_path
        ]

        print("Attempting to revert patch: " + patch)

        return_code = subprocess.run(command)
        print(return_code)

        if return_code != 0:
            print("FAILED TO REVERT PATCH!")
            print("The destination repo is probably now in a bad state.")
            print("It may require some manual cleanup.")
            patches_unwind(applied_patch_list, destination_path)
            sys.exit(255)
        else:
            applied_patch_list.append(patch)


def apply_patches(patch_list, destination_path):
    """
    Applies patches to the Git repo in the order patch_list contains them.
    If applying a patch fails this function will attempt to clean up the repo
    state by rolling back the patches using `git apply -R`

    :param patch_list: The paths of all patches to apply.
    :type patch_list: list (str)
    :param destination_path: File path to destination repo where patches will
                             be applied.
    :type destination_path: str
    """

    applied_patch_list = []

    for patch in patch_list:
        command = [
            "git",
            "apply",
            "--unsafe-paths",
            patch,
            "--directory=" + destination_path
        ]

        print("Attempting to apply patch: " + patch)

        try:
            return_code = subprocess.run(command)
        except subprocess.CalledProcessError as grepexc:
            print("error code", grepexc.returncode, grepexc.output)

            if return_code != 0:
                print("FAILED TO APPLY PATCH!")
                patches_unwind(applied_patch_list, destination_path)
                sys.exit(255)

        applied_patch_list.insert(0, patch)


def attempt_patch(source_path, destination_path):
    """
    Main driver function that verifies paths, gathers patches, then applies
    them to the destination repo.

    :param source_path: File path to the patches root directory.
    :type source_path: str
    :param destination_path: File path to destination repo where patches will
                             be applied.
    :type destination_path: str
    """

    print("Attempting to patch files...")

    verify_paths(source_path, destination_path)
    patch_list = gather_patches(source_path)
    apply_patches(patch_list, destination_path)

    print("Successfully applied all patches!")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Apply the patches specified in the source directory "
        "to the destination git repository. Patches will be applied in "
        "alphabetic order (0001, 002, etc) with patches in the same directory "
        "grouped together. Groups of patches will be applied alphabetically "
        "based on their parent directory")

    parser.add_argument(
        '-s',
        '--source', required=True,
        help="The source directory containing all the patches to apply. This "
        "script will recursively search for and apply patches in this parent "
        "directory")

    parser.add_argument(
        '-d', '--destination', required=True,
        help="The destination git repository to apply the patches to")

    args = parser.parse_args()
    attempt_patch(args.source, args.destination)
