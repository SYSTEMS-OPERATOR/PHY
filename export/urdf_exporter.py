from __future__ import annotations

from typing import Tuple
from xml.etree.ElementTree import Element, SubElement, tostring
import math

from kinematics.kinematic_chain import KinematicChain


def _inertia_tag(parent: Element, inertia) -> None:
    inertia_el = SubElement(parent, 'inertia')
    inertia_el.set('ixx', str(inertia[0][0]))
    inertia_el.set('iyy', str(inertia[1][1]))
    inertia_el.set('izz', str(inertia[2][2]))


def export_chain_urdf(chain: KinematicChain, path: str, mesh: str = 'primitive') -> None:
    robot = Element('robot')
    robot.set('name', 'skeleton')
    for bone in chain.bones.values():
        link = SubElement(robot, 'link')
        link.set('name', bone.unique_id)
        inertial = SubElement(link, 'inertial')
        mass = SubElement(inertial, 'mass')
        mass.set('value', str(bone.mass_kg() or 1.0))
        _inertia_tag(inertial, bone.geometry.get('inertia_kgm2', [[0,0,0],[0,0,0],[0,0,0]]))
        visual = SubElement(link, 'visual')
        geometry = SubElement(visual, 'geometry')
        if bone.geometry.get('type') == 'cylinder':
            cyl = SubElement(geometry, 'cylinder')
            cyl.set('radius', str(bone.geometry.get('radius_m', 0.01)))
            cyl.set('length', str(bone.geometry.get('length_m', 0.01)))
        else:
            box = SubElement(geometry, 'box')
            box.set('size', f"{bone.geometry.get('width_m',0.01)} {bone.geometry.get('thickness_m',0.01)} {bone.geometry.get('length_m',0.01)}")
        collision = SubElement(link, 'collision')
        cg = SubElement(collision, 'geometry')
        if bone.geometry.get('type') == 'cylinder':
            cc = SubElement(cg, 'cylinder')
            cc.set('radius', str(bone.geometry.get('radius_m', 0.01)))
            cc.set('length', str(bone.geometry.get('length_m', 0.01)))
        else:
            cb = SubElement(cg, 'box')
            cb.set('size', f"{bone.geometry.get('width_m',0.01)} {bone.geometry.get('thickness_m',0.01)} {bone.geometry.get('length_m',0.01)}")
    for joint in chain.joints:
        j = SubElement(robot, 'joint')
        j.set('name', joint.name)
        type_map = {'hinge': 'revolute', 'pivot': 'revolute', 'ball': 'continuous', 'fixed': 'fixed'}
        j.set('type', type_map.get(joint.joint_type, joint.joint_type))
        parent = SubElement(j, 'parent')
        parent.set('link', joint.parent_uid)
        child = SubElement(j, 'child')
        child.set('link', joint.child_uid)
        origin = SubElement(j, 'origin')
        origin.set('xyz', f"{joint.origin_xyz[0]} {joint.origin_xyz[1]} {joint.origin_xyz[2]}")
        origin.set('rpy', f"{joint.origin_rpy[0]} {joint.origin_rpy[1]} {joint.origin_rpy[2]}")
        if joint.joint_type != 'fixed':
            axis = SubElement(j, 'axis')
            axis.set('xyz', f"{joint.axis[0]} {joint.axis[1]} {joint.axis[2]}")
            limit = SubElement(j, 'limit')
            if len(joint.limit_deg) == 2:
                limit.set('lower', str(math.radians(joint.limit_deg[0])))
                limit.set('upper', str(math.radians(joint.limit_deg[1])))
            else:
                limit.set('effort', '0')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(tostring(robot, encoding='unicode'))

