#!/usr/bin/env python3

import sys
import pandas as pd
import xml.etree.ElementTree as ET
from xml.dom import minidom


def generate_transmissions_from_csv(csv_path, output_path):
    df = pd.read_csv(csv_path)
    robot = ET.Element("robot", name="transmissions")

    for _, row in df.iterrows():
        transmission = ET.SubElement(robot, "transmission", name=row["transmission_name"])

        type_tag = ET.SubElement(transmission, "type")
        type_tag.text = row["transmission_type"]

        joint = ET.SubElement(transmission, "joint", name=row["joint"])
        joint_hw = ET.SubElement(joint, "hardwareInterface")
        joint_hw.text = row["hardwareInterface"]

        actuator = ET.SubElement(transmission, "actuator", name=row["actuator_name"])
        actuator_hw = ET.SubElement(actuator, "hardwareInterface")
        actuator_hw.text = row["hardwareInterface"]

        mech_red = ET.SubElement(actuator, "mechanicalReduction")
        mech_red.text = str(row["mechanical_reduction"])

    xml_str = ET.tostring(robot, encoding="utf-8")
    pretty_xml = minidom.parseString(xml_str).toprettyxml(indent="  ")

    with open(output_path, "w") as f:
        f.write(pretty_xml)

    print(f"parse done: {output_path}")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: python generate_transmissions.py input.csv output.urdf")
        sys.exit(1)

    csv_path = sys.argv[1]
    output_path = sys.argv[2]

    generate_transmissions_from_csv(csv_path, output_path)
