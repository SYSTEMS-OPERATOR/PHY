from __future__ import annotations

from pathlib import Path
from typing import List

from skeleton.base import BoneSpec
from joints.joint_spec import JointSpec


def export_urdf(bones: List[BoneSpec], joints: List[JointSpec], out_file: str, mesh: str = "primitive") -> None:
    """Serialize skeleton configuration to a URDF file."""
    lines = ["<robot name=\"skeleton\">"]
    for bone in bones:
        lines.append(f"  <link name=\"{bone.unique_id}\">")
        inert = bone.geometry.get("inertia_kgm2")
        com = bone.geometry.get("COM", (0.0, 0.0, 0.0))
        mass = bone.mass_kg() or 0.001
        if inert:
            ixx, _, _ = inert[0]
            _, iyy, _ = inert[1]
            _, _, izz = inert[2]
        else:
            ixx = iyy = izz = 1e-6
        lines.append("    <inertial>")
        lines.append(f"      <origin xyz=\"{com[0]} {com[1]} {com[2]}\"/>")
        lines.append(f"      <mass value=\"{mass}\"/>")
        lines.append(
            f"      <inertia ixx=\"{ixx}\" ixy=\"0\" ixz=\"0\" iyy=\"{iyy}\" iyz=\"0\" izz=\"{izz}\"/>"
        )
        lines.append("    </inertial>")
        if mesh == "primitive":
            r = bone.dimensions.get("width_cm", 1) / 200.0
            h = bone.dimensions.get("length_cm", 1) / 100.0
            lines.append("    <visual>")
            lines.append("      <geometry>")
            lines.append(f"        <cylinder radius=\"{r}\" length=\"{h}\"/>")
            lines.append("      </geometry>")
            lines.append("    </visual>")
        lines.append("  </link>")
    for joint in joints:
        lines.append(
            f"  <joint name=\"{joint.name}\" type=\"{joint.joint_type}\">"
        )
        lines.append(
            f"    <parent link=\"{joint.parent_uid}\"/>"
        )
        lines.append(f"    <child link=\"{joint.child_uid}\"/>")
        lines.append(
            f"    <origin xyz=\"{' '.join(map(str, joint.origin_xyz))}\" rpy=\"{' '.join(map(str, joint.origin_rpy))}\"/>"
        )
        lines.append(
            f"    <axis xyz=\"{' '.join(map(str, joint.axis))}\"/>")
        lines.append("  </joint>")
    lines.append("</robot>")
    Path(out_file).write_text("\n".join(lines))
