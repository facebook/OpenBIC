#!/usr/bin/env python3

import argparse
import sys
import hashlib
import binascii
from datetime import date
from os.path import exists

# Numeric encoding of development stage names.
stage_map = {
    "evt": "001",
    "dvt": "010",
    "pvt": "011",
    "mp": "100"}

# Supported boards for each platform.
valid_projects = {
    "yv35": {"cl", "bb", },
    "gt": {"cc", }
}

# Numeric encoding of boards.
board_map = {
    "cl": "00001",
    "bb": "00010",
}

# Mapping between short and full project names.
project_name_mapping = {
    "yv35": "Yosemite V3.5",
    "gt": "Grand Teton"
}


def get_padded_project_name(project_name) -> str:
    """
    Returns a project name padded to a specific length.

    :param project_name: Short name of the project (such as yv35).
    :type project_name: str
    :return: left justified string padded with spaces.
    :rtype: str
    """
    MAX_LENGTH = 16

    if project_name not in project_name_mapping:
        print("Could not identify project name in map!")
        print("Choices are: ")
        for short, long in project_name_mapping:
            print("- " + short + " : " + long)

    project_name = project_name_mapping[project_name]

    if len(project_name) > MAX_LENGTH:
        return project_name[:MAX_LENGTH]
    else:
        return project_name.ljust(MAX_LENGTH)


def get_md5_from_file(file_path) -> str:
    """
    Generate an md5 from the file found at the supplied path.

    :param file_path: File path of the file used to generate the checksum.
    :type file_path: str
    :return: Hex string of the md5 hash for the file.
    :rtype: str
    """
    hash_md5 = hashlib.md5()

    with open(file_path, "rb") as image:
        for chunk in iter(lambda: image.read(4096), b""):
            hash_md5.update(chunk)

    return hash_md5.hexdigest()


def hex_dump_to_image(data, output_path):
    """
    Write the hex dump to a file using the path supplied on the CLI.

    :param data: Hex data containing the imgae with the generated checksum
    info appended onto the end.

    :type data: str
    :param args: File path to write the new signed image to.
    :type args: str
    """

    with open(output_path, 'bw+') as image:
        image.write(binascii.unhexlify(data))


def sign_image(args):
    """
    Main driving function for generating checksum info and appending it to the
    image file before writing it to an output file.

    :param args: Validated CLI arguments.
    :type args: obj
    """

    # Read in the file as a hex string of the binary.
    with open(args.input_file, "rb") as image:
        image_dump = binascii.hexlify(image.read())

    # Generate the MD5 and append it onto the image.
    md5_digest = get_md5_from_file(args.input_file)
    image_dump += bytes(md5_digest, 'utf-8')

    # Generate project code hex and append to binary.
    padded_project_name = get_padded_project_name(args.project)
    project_hex = binascii.hexlify(padded_project_name.encode())[0:32]
    image_dump += project_hex

    # Generate project version hex and append to binary.
    padded_version = args.version.ljust(13)
    version_hex = binascii.hexlify(padded_version.encode())[0:26]
    image_dump += version_hex

    # Component: cpld=001, bic=010, bios=011.
    COMPONENT_CODE = "010"

    # Reserved for future use. if used 000000000001.
    INSTANCE_ID = "000000000000"

    # Concatenate the project info into a single string.
    identity_string = INSTANCE_ID
    identity_string += COMPONENT_CODE
    identity_string += stage_map[args.stage]
    identity_string += board_map[args.board]

    # Generate short string for project identity string.
    byte3_errorPoof = f'{int(identity_string, 2):x}'
    byte3_errorPoof = byte3_errorPoof.rjust(6, '0')

    # Swap so LSB comes first and append to image hex.
    lsb_error = byte3_errorPoof[4:6]
    lsb_error += byte3_errorPoof[2:4]
    lsb_error += byte3_errorPoof[0:2]
    image_dump += bytes(lsb_error, 'utf-8')

    # Generate md5 of project identification strings.
    md5_2 = md5_digest
    md5_2 += project_hex.decode()
    md5_2 += version_hex.decode()
    md5_2 += lsb_error
    md5_2 = hashlib.md5(binascii.unhexlify(md5_2)).hexdigest()

    # Append new checksum to image.
    image_dump += bytes(md5_2, 'utf-8')

    # Write new image with checksum to file.
    hex_dump_to_image(image_dump, args.output_file)


def validate_version_number(version) -> str:
    """
    Validate that the version number given on the command line matches the
    proper format (YYY.WW.VV)

    YYYY - 4 digit year number
    WW - 2 digit week number
    VV - 2 digit year number starting at 01

    :param version: Version passed in on the CLI.
    :type version: str
    :return: Valid version number (possibly with year and week added.)
    :rtype: str
    """
    parsed_version = version.split(".")
    version_comps = {}

    # Accept either all info or just version and use current year and week.
    if len(parsed_version) == 3:
        version_comps["year"] = parsed_version[0]
        version_comps["week"] = parsed_version[1]
        version_comps["version"] = parsed_version[2]
    elif len(parsed_version) == 1:
        version_comps["version"] = parsed_version[0]
    else:
        print("Version string is invalid")
        print("Version should be in the format of \"YYYY.WW.VV\".")
        sys.exit(255)

    # Validate year.
    if "year" in version_comps and len(version_comps["year"]) != 4:
        print("The year component of the version should be 4 characters, "
              "but \"" + version_comps["year"] + "\" is " +
              str(len(version_comps["year"])))
        sys.exit(255)

    # Validate week.
    if "week" in version_comps and len(version_comps["week"]) != 2:
        print("The week component of the version should be 2 characters, "
              "but \"" + version_comps["week"] + "\" is " +
              str(len(version_comps["week"])))
        sys.exit(255)

    # Validate version.
    if "version" in version_comps and len(version_comps["version"]) != 2:
        print("The version component of the version should be 2 characters, "
              "but \"" + version_comps["version"] + "\" is " +
              str(len(version_comps["version"])))
        sys.exit(255)

    # If only version number is supplied add current week and year.
    if len(parsed_version) == 1:
        today = date.today()
        year, week, _day = today.isocalendar()
        version_comps["year"] = year
        version_comps["week"] = week

        print("Inferring " + version + "as " + version_comps["year"] + "." +
              version_comps["week"] + "." + version_comps["version"])

    # Recombine parts into new version string.
    version_string = version_comps["year"] + "." + \
        version_comps["week"] + "." + version_comps["version"]

    return version_string


def validate_inputs(args):
    """
    Validates the input to verify valid project specification, version
    numbering and that the input file exists.

    :param args: args parsed by argparse
    :type args: argparse object
    """

    # Check to make sure it's a valid platform
    if args.project not in valid_projects:
        print(args.project + " not a valid OpenBIC project.")
        print("Current valid projects: ")
        for project in valid_projects.keys():
            print(" - " + project)
        sys.exit(255)

    # Check to make sure it's a valid board for one of our platforms
    if args.board not in valid_projects[args.project]:
        print(args.board + " not a valid " + args.project + " board.")
        print("Current valid boards: ")
        for board in valid_projects[args.project]:
            print(" - " + board)
        sys.exit(255)

    # Verify that the version string matches what we expect
    args.version = validate_version_number(args.version)

    # Verify that the development stage is a valid stage
    if args.stage not in stage_map:
        print(args.stage + " is not a valid development stage.")
        print("Current valid stages: ")
        for stage in stage_map.keys():
            print(" - " + stage)
        sys.exit(255)

    # Verify that the unsigned image path is valid and exists
    path = args.input_file
    if exists(path) is False:
        print("Input image file path does not exist!")
        print("Path: " + path)
        sys.exit(255)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='')

    parser.add_argument(
        '-p',
        '--project',
        required=True,
        help="The project containing the BIC. For example yv35 for Yosemite "
        "v3.5.")

    parser.add_argument(
        '-b',
        '--board',
        required=True,
        help="The board the BIC is located on for a project, for example cl "
        "(Crater Lake) and bb (Baseboard) are both boards belonging to the "
        "Yosemite v3.5 project.")

    parser.add_argument(
        '-v',
        '--version',
        required=True,
        help="The version of the firmware image. This should be in the format "
        "\"YYYY.WW.VV\" where YYYY is the complete year number. WW is the "
        "current work week, and VV is an incremental version number starting "
        "with 01. For example the first version tagged on June 3, 2022 would "
        "be \"2022.22.01\".")

    parser.add_argument(
        '-s',
        '--stage',
        required=True,
        help="The current development stage for the project. Current valid "
        "stages are <evt | dvt | pvt | mp>.")

    parser.add_argument(
        '-i',
        '--input-file',
        required=True,
        help="Path to the original unsigned OpenBIC image.")

    parser.add_argument(
        '-o',
        '--output-file',
        required=True,
        help="Path to write the final output image.")

    args = parser.parse_args()

    validate_inputs(args)
    sign_image(args)
