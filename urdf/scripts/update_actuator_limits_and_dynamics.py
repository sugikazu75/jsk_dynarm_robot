#!/usr/bin/env python3

import sys
import xml.etree.ElementTree as ET
import csv
import io
from xml.dom import minidom
import pandas as pd
import subprocess
from urdf_utils import run_subprocess, save_element_tree_as_urdf


def read_joint_limits_from_csv(csv_path):
    df = pd.read_csv(csv_path, encoding="utf-8-sig")

    required_cols = {"joint", "lower", "upper", "effort", "velocity"}
    if not required_cols.issubset(df.columns):
        raise ValueError(f"Missing required columns. Found: {df.columns.tolist()}")

    joint_limits = {}
    for _, row in df.iterrows():
        name = row["joint"]

        if pd.isna(row["lower"]) or pd.isna(row["upper"]) or pd.isna(row["effort"]) or pd.isna(row["velocity"]):
            print(f"[SKIP] joint '{name}' has missing limit values")
            continue
        joint_limits[name] = {
            "lower": str(row["lower"]),
            "upper": str(row["upper"]),
            "effort": str(row["effort"]),
            "velocity": str(row["velocity"]),
        }
        print(f"get limits for '{name}': {joint_limits[name]}")

    return joint_limits


def read_joint_dynamics_from_csv(csv_path):
    df = pd.read_csv(csv_path, encoding="utf-8-sig")

    required_cols = {"joint", "damping", "friction"}
    if not required_cols.issubset(df.columns):
        raise ValueError(f"Missing required columns. Found: {df.columns.tolist()}")

    joint_dynamics = {}

    for _, row in df.iterrows():
        name = row["joint"]

        if pd.isna(row["damping"]) or pd.isna(row["friction"]):
            print(f"[SKIP] joint '{name}' has missing dynamics values")
            continue
        joint_dynamics[name] = {"damping": str(row["damping"]), "friction": str(row["friction"])}
        print(f"get dynamics for '{name}': {joint_dynamics[name]}")

    return joint_dynamics


def update_urdf_joint_limits(urdf_tree, joint_limits):
    root = urdf_tree

    for joint in root.findall(".//joint"):
        name = joint.get("name")
        jtype = joint.get("type")
        if jtype == "fixed" or name not in joint_limits:
            print(f"[SKIP] joint '{name}' is skipped because fixed or not included in joint_dynamics")
            continue

        # remove existing limit tag
        existing_limit = joint.find("limit")
        if existing_limit is not None:
            joint.remove(existing_limit)

        # add limit tag
        limits = joint_limits[name]
        limit_elem = ET.SubElement(joint, "limit")
        limit_elem.set("lower", limits["lower"])
        limit_elem.set("upper", limits["upper"])
        limit_elem.set("effort", limits["effort"])
        limit_elem.set("velocity", limits["velocity"])
        print(f"updated limits for '{name}' as '{joint_limits[name]}'")
    return root


def update_urdf_joint_dynamics(urdf_tree, joint_dynamics):
    root = urdf_tree

    for joint in root.findall(".//joint"):
        name = joint.get("name")
        jtype = joint.get("type")
        if jtype == "fixed" or name not in joint_dynamics:
            print(f"[SKIP] joint '{name}' is skipped because fixed or not included in joint_dynamics")
            continue

        # remove existing dynamics tag
        existing_dynamics = joint.find("dynamics")
        if existing_dynamics is not None:
            joint.remove(existing_dynamics)

        # add dynamics tag
        dynamics = joint_dynamics[name]
        dynamics_elem = ET.SubElement(joint, "dynamics")
        dynamics_elem.set("damping", dynamics["damping"])
        dynamics_elem.set("friction", dynamics["friction"])
        print(f"updated dynamics for '{name}' as '{joint_dynamics[name]}'")
    return root


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} INPUT_URDF ACTUATOR_CSV")
        sys.exit(1)

    urdf_path = sys.argv[1]
    csv_path = sys.argv[2]

    urdf_str = None
    with open(urdf_path, "r", encoding="utf-8") as f:
        urdf_str = f.read()
    root = ET.fromstring(urdf_str)

    joint_limits = read_joint_limits_from_csv(csv_path)
    root = update_urdf_joint_limits(root, joint_limits)

    joint_dynamics = read_joint_dynamics_from_csv(csv_path)
    root = update_urdf_joint_dynamics(root, joint_dynamics)

    output_path = urdf_path.replace(".urdf", "_joint_updated.urdf")
    save_element_tree_as_urdf(root, output_path)

    print(f"Updated URDF written to: {output_path}")


if __name__ == "__main__":
    main()
