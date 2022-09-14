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

# This is used to map the tag name to the extended platform name.
# When onboarding a new OpenBIC platform it should be added here in
# alphabetical order.
PLATFORM_NAME_MAP = {
    "obgt-cc": "Grand Teton : Clear Creek",
    "obwc-mb": "Waimea Canyon : MainBoard",
    "oby3-dl": "Yosemite v3 : Delta Lake",
    "oby3-vf": "Yosemite v3 : Vernal Falls",
    "oby35-bb": "Yosemite v3.5 : BaseBoard",
    "oby35-cl": "Yosemite v3.5 : Crater Lake",
    "oby35-hd": "Yosemite v3.5 : Half Dome",
    "oby35-rf": "Yosemite v3.5 : Rainbow Falls",
}

# These are the currently supported platforms.
# When onboarding a new OpenBIC platform it should be added here in
# alphabetical order.
SUPPORTED_PLATFORMS = {
    "obgt-cc",
    "obwc-mb",
    "oby3-dl",
    "oby3-vf",
    "oby35-bb",
    "oby35-cl",
    "oby35-hd",
    "oby35-rf",
}

PLATFORM_RELEASE_MAPPINGS = {
    "gt-cc": "obgt-cc",
    "wc-mb": "obwc-mb",
    "yv3-dl": "oby3-dl",
    "yv3-vf": "oby3-vf",
    "yv35-bb": "oby35-bb",
    "yv35-cl": "oby35-cl",
    "yv35-hd": "oby35-hd",
    "yv35-rf": "oby35-rf",
}

PLATFORM_DIRECTORIES = {
    "gt-cc",
    "wc-mb",
    "yv3-dl",
    "yv3-vf",
    "yv35-bb",
    "yv35-cl",
    "yv35-hd",
    "yv35-rf",
}

# Sometimes titles include tags for the platform,
# These tags mostly clutter the titles so we can strip them from
# the release notes.
#
# When onboarding a new OpenBIC platform it should be added here in
# alphabetical order.
TITLE_TAGS = {
    "bb:",
    "cc:",
    "cl:",
    "dl:",
    "fby3:",
    "fby3.5:",
    "fby35:",
    "gt:",
    "hd:",
    "mb:",
    "rf:",
    "vf:",
    "wc:",
    "Common",
    "common",
}


class color:
    ''' These can be used to create colored terminal output.'''
    PURPLE = '\033[1;35;48m'
    CYAN = '\033[1;36;48m'
    BOLD = '\033[1;37;48m'
    BLUE = '\033[1;34;48m'
    GREEN = '\033[1;32;48m'
    YELLOW = '\033[1;33;48m'
    RED = '\033[1;31;48m'
    BLACK = '\033[1;30;48m'
    UNDERLINE = '\033[4;37;48m'
    END = '\033[1;37;0m'


LOGO = r"""
  /$$$$$$                                /$$$$$$$  /$$$$$$  /$$$$$$
 /$$__  $$                              | $$__  $$|_  $$_/ /$$__  $$
| $$  \ $$  /$$$$$$   /$$$$$$  /$$$$$$$ | $$  \ $$  | $$  | $$  \__/
| $$  | $$ /$$__  $$ /$$__  $$| $$__  $$| $$$$$$$   | $$  | $$
| $$  | $$| $$  \ $$| $$$$$$$$| $$  \ $$| $$__  $$  | $$  | $$
| $$  | $$| $$  | $$| $$_____/| $$  | $$| $$  \ $$  | $$  | $$    $$
|  $$$$$$/| $$$$$$$/|  $$$$$$$| $$  | $$| $$$$$$$/ /$$$$$$|  $$$$$$/
 \______/ | $$____/  \_______/|__/  |__/|_______/ |______/ \______/
          | $$
          | $$
          |__/
"""
