#!/usr/bin/env python3

import sys
import re
import subprocess
import xml.etree.ElementTree as ET
from xml.dom import minidom
from urdf_utils import run_subprocess, save_element_tree_as_urdf


def parse_mass_properties(txt):
    link_data = {}
    current_link = None

    for line in txt.splitlines():
        m = re.match(r"^([^\s]+)の質量特性", line)
        if m:
            current_link = m.group(1)
            link_data[current_link] = {}
            continue

        if current_link is None:
            continue

        # m = re.match(r"^\s*質量\s*=\s*([\d\.]+)", line)
        m = re.match(r"^\s*質量(?:\s*\(.*?\))?\s*=\s*([\d\.]+)", line)
        if m:
            link_data[current_link]["mass"] = float(m.group(1))
            continue

        m = re.match(r"^\s*X\s*=\s*([-\d\.]+)", line)
        if m:
            link_data[current_link]["com_x"] = float(m.group(1))
            continue

        m = re.match(r"^\s*Y\s*=\s*([-\d\.]+)", line)
        if m:
            link_data[current_link]["com_y"] = float(m.group(1))
            continue

        m = re.match(r"^\s*Z\s*=\s*([-\d\.]+)", line)
        if m:
            link_data[current_link]["com_z"] = float(m.group(1))
            continue

        m = re.match(
            r"^\s*Lxx\s*=\s*([-\d\.]+)\s*Lxy\s*=\s*([-\d\.]+)\s*Lxz\s*=\s*([-\d\.]+)",
            line,
        )
        if m:
            link_data[current_link]["Lxx"] = float(m.group(1))
            link_data[current_link]["Lxy"] = float(m.group(2))
            link_data[current_link]["Lxz"] = float(m.group(3))
            continue

        m = re.match(
            r"^\s*Lyx\s*=\s*([-\d\.]+)\s*Lyy\s*=\s*([-\d\.]+)\s*Lyz\s*=\s*([-\d\.]+)",
            line,
        )
        if m:
            link_data[current_link]["Lyx"] = float(m.group(1))
            link_data[current_link]["Lyy"] = float(m.group(2))
            link_data[current_link]["Lyz"] = float(m.group(3))
            continue

        m = re.match(
            r"^\s*Lzx\s*=\s*([-\d\.]+)\s*Lzy\s*=\s*([-\d\.]+)\s*Lzz\s*=\s*([-\d\.]+)",
            line,
        )
        if m:
            link_data[current_link]["Lzx"] = float(m.group(1))
            link_data[current_link]["Lzy"] = float(m.group(2))
            link_data[current_link]["Lzz"] = float(m.group(3))
            continue

    return link_data


def update_urdf_inertial_xml(urdf_file, output_file, link_data):
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    for link in root.findall(".//link"):
        link_name = link.get("name")
        if link_name not in link_data:
            continue

        print(f"Updating link: {link_name}")
        inertial = link.find("inertial")
        if inertial is not None:
            link.remove(inertial)

        # inertial
        inertial = ET.SubElement(link, "inertial")

        # mass
        mass_elem = ET.SubElement(inertial, "mass")
        mass_elem.set("value", str(link_data[link_name]["mass"]))

        # origin
        origin_elem = ET.SubElement(inertial, "origin")
        origin_elem.set(
            "xyz",
            f"{link_data[link_name]['com_x']} {link_data[link_name]['com_y']} {link_data[link_name]['com_z']}",
        )
        origin_elem.set("rpy", "0 0 0")

        # inertia
        inertia_elem = ET.SubElement(inertial, "inertia")
        inertia_elem.set("ixx", str(link_data[link_name]["Lxx"]))
        inertia_elem.set("ixy", str(link_data[link_name]["Lxy"]))
        inertia_elem.set("ixz", str(link_data[link_name]["Lxz"]))
        inertia_elem.set("iyy", str(link_data[link_name]["Lyy"]))
        inertia_elem.set("iyz", str(link_data[link_name]["Lyz"]))
        inertia_elem.set("izz", str(link_data[link_name]["Lzz"]))

    save_element_tree_as_urdf(root, output_file)


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} URDF_FILE TXT_FILE")
        sys.exit(1)

    urdf_file = sys.argv[1]
    txt_file = sys.argv[2]

    with open(txt_file, "r", encoding="utf-8") as f:
        txt_content = f.read()

    link_data = parse_mass_properties(txt_content)

    print(f"Parsed links: {list(link_data.keys())}")

    output_file = None
    if urdf_file.endswith(".urdf"):
        output_file = urdf_file.replace(".urdf", "_inertial_updated.urdf")
    elif urdf_file.endswith(".xacro"):
        output_file = urdf_file.replace(".xacro", "_inertial_updated.xacro")
    else:
        output_file = urdf_file + "_inertial_updated"

    update_urdf_inertial_xml(urdf_file, output_file, link_data)

    print(f"Updated URDF/Xacro written to {output_file}")


if __name__ == "__main__":
    main()
