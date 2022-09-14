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
import re


def get_manifest(manifest_path) -> str:
    """
    This function reads in the manifest file give the path and checks to
    see if it has the manifest keyword at the beginning of the line
    somewhere in the file.

    It then reads in the file as a single string.

    :param manifest_path: The path to a west manifest yml file.
    :type manifest_path: str
    :return: String containing the raw file contents.
    :rtype: str
    """
    try:
        manifest_file = open(manifest_path, "r")
    except FileNotFoundError:
        print("Manifest file not found at: " + manifest_path)
        sys.exit(255)
    except Exception as e:
        print("Error opening manifest file.")
        print(e)
        sys.exit(255)

    try:
        file_contents = manifest_file.read()
    except Exception as e:
        print("Error reading manifest File Contents.")
        print(e)
        sys.exit(255)

    file_match = re.search("manifest:", file_contents)
    if file_match is None:
        print("File does not appear to be a manifest file.")
        print("Missing \"manifest\" key in file.")
        sys.exit(255)

    return file_contents


def parse_revisions(manifest_contents) -> dict:
    """
    Parses the contents of the manifest file using a regex and then stores the
    output of the matches in a dictionary using the project name as the key and
    the returned revision as the value.

    :param manifest_contents: Raw contents of the manifest file.
    :type manifest_contents: str
    :return: Dictionary containing project name and revision as key, values.
    :rtype: dict
    """
    project_body_pattern = r"projects:([\s\S]*)self:"
    body_match = re.findall(project_body_pattern, manifest_contents, re.M)
    if body_match is []:
        print("Could not identify project field of manifest.")
        sys.exit(255)

    body = body_match[0]
    regex_pattern = r"- name:[ ]*([^ \n]*)[\s\S]*?revision:[ ]*([a-z,0-9,\.]*)"
    matches = re.findall(regex_pattern, body, re.M)

    revisions = {}
    for revision_pair in matches:
        if revision_pair is not None and len(revision_pair) == 2:
            revisions[revision_pair[0]] = revision_pair[1]

    return revisions


def print_all_revisions(revisions):
    """
    If no project name is supplied on the command line this function will be
    used to print out all project revisions. It will print a single project and
    it's revision per line.

    :param revisions: A dictionary containing project name and revisions.
    :type revisions: dict
    """
    if len(revisions) == 0:
        print("No projects and revisions could be parsed from manifest.")
        sys.exit(255)

    for project, revision in revisions.items():
        print(project + " " + revision)


def print_revision(project, revisions):
    """
    Attempts to print the revision for the given project if it exists.

    :param project: The project name to print.
    :type project: str
    :param revisions: A dictionary contain project names and revisions.
    :type revisions: dict
    """
    if len(revisions) == 0:
        print("No projects and revisions could be parsed from manifest.")
        return

    if project not in revisions:
        print("Could not find project " + project + " in manifest.")
        print("Possible projects in manifest with revisions:")
        for project, revision in revisions.items():
            print("- " + project)
        sys.exit(255)

    else:
        print(revisions[project])


def get_revision(project, manifest_path):
    """
    Main work function that takes the CLI input and loads the file, parses
    the revisions, and prints the requested revision(s) if possible.

    :param project: The project name to print, or None if none supplied.
    :type project: str
    :param manifest_path: The path to the manifest file to parse.
    :type manifest_path: str
    """
    manifest_contents = get_manifest(manifest_path)
    project_revisions = parse_revisions(manifest_contents)

    if project is None:
        print_all_revisions(project_revisions)
    else:
        print_revision(project, project_revisions)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Get revision from the specified project')

    parser.add_argument(
        '-p',
        '--project',
        help="The name of the project to retrieve the revision for. "
        "If this option is not used then all revisions "
        "will be printed")
    parser.add_argument(
        '-m', '--manifest', required=True,
        help="The path to the west manifest file to parse for the requested "
        "revision")

    args = parser.parse_args()
    get_revision(args.project, args.manifest)
