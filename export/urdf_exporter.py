from __future__ import annotations

from pathlib import Path
from typing import Dict, List
from xml.etree.ElementTree import Element, SubElement, tostring

from skeleton.base import BoneSpec
from joints.joint_spec import JointSpec


def export_urdf(bones: Dict[str, BoneSpec], joints: List[JointSpec], out: str, mesh="primitive") -> None:
    robot = Element("robot", name="skeleton")
    for b in bones.values():
        link = SubElement(robot, "link", name=b.unique_id)
        inertial = SubElement(link, "inertial")
        com = b.geometry.get("COM", (0,0,0))
        SubElement(inertial, "origin", xyz="%f %f %f"%com)
        mass = b.material.get("density",1800)*b.geometry.get("V_cm3",0)/1e6
        SubElement(inertial, "mass", value=str(mass))
    for j in joints:
        el = SubElement(robot, "joint", name=j.name, type=j.joint_type)
        SubElement(el, "parent", link=j.parent_uid)
        SubElement(el, "child", link=j.child_uid)
        SubElement(el, "origin", xyz="%f %f %f"%j.origin_xyz, rpy="%f %f %f"%j.origin_rpy)
        if j.joint_type != 'fixed':
            SubElement(el, "axis", xyz="%f %f %f"%j.axis)
    Path(out).write_text(tostring(robot, encoding="unicode"))

