#!/usr/bin/env python3

import sys
import xml.etree.ElementTree as ET
import csv
import io
from xml.dom import minidom
import subprocess
import urdf_utils
from urdf_utils import run_subprocess, save_element_tree_as_urdf


def remove_collision(urdf_tree):
    root = urdf_tree

    for link in root.findall(".//link"):
        name = link.get("name")

        existing_collision = link.find("collision")
        if existing_collision is not None:
            link.remove(existing_collision)
    return root


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} INPUT_URDF")
        sys.exit(1)

    urdf_path = sys.argv[1]

    urdf_str = None
    with open(urdf_path, "r", encoding="utf-8") as f:
        urdf_str = f.read()
    root = ET.fromstring(urdf_str)

    remove_collision(root)

    save_element_tree_as_urdf(root, urdf_path)


if __name__ == "__main__":
    main()
