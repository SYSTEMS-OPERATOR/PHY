from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import List

from skeleton.base import BoneSpec
from joints.joint_spec import JointSpec
from geometry.geometry_agent import GeometryAgent


def bones_to_urdf(root: BoneSpec, joints: List[JointSpec], bones: List[BoneSpec]) -> str:
    agent = GeometryAgent()
    for b in bones:
        agent.build_geometry(b)

    robot = ET.Element("robot", name="skeleton")

    for b in bones:
        link = ET.SubElement(robot, "link", name=b.unique_id)
        inertial = ET.SubElement(link, "inertial")
        com = b.geometry.get("COM", (0, 0, 0))
        ET.SubElement(inertial, "origin", xyz=f"{com[0]} {com[1]} {com[2]}", rpy="0 0 0")
        mass_kg = b.material.get("density", 1) * b.geometry.get("V_cm3", 1) / 1e6
        ET.SubElement(inertial, "mass", value=str(mass_kg))

    for j in joints:
        joint = ET.SubElement(robot, "joint", name=j.name, type=j.joint_type)
        ET.SubElement(joint, "parent", link=j.parent_uid)
        ET.SubElement(joint, "child", link=j.child_uid)
        ET.SubElement(joint, "origin", xyz=" ".join(map(str, j.origin_xyz)), rpy=" ".join(map(str, j.origin_rpy)))
        if j.joint_type != "fixed" and j.limit_deg:
            limit = ET.SubElement(joint, "limit")
            limit.set("lower", str(j.limit_deg[0]))
            if isinstance(j.limit_deg, tuple) and len(j.limit_deg) > 1:
                limit.set("upper", str(j.limit_deg[1]))
    return ET.tostring(robot, encoding="unicode")
