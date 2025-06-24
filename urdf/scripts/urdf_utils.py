import sys
import subprocess
import xml.etree.ElementTree as ET
from xml.dom import minidom


def run_subprocess(cmd):
    if sys.version.split(".")[0] == "2":
        subprocess.call(cmd, shell=True)
    if sys.version.split(".")[0] == "3":
        subprocess.run(cmd, shell=True)


def save_element_tree_as_urdf(root, output_file):
    # save
    xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
    with open(output_file, "w") as f:
        f.write(xmlstr)

    # remove brank line in xml
    cmd = "sed -i '/^[[:space:]]*$/d' {}".format(output_file)
    run_subprocess(cmd)
