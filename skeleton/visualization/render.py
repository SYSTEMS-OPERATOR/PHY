from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Tuple

from ..base import BoneSpec


def get_color(property_value: Any, property_name: str, colormap: str = 'viridis') -> str:
    """Map property value to a color string."""
    try:
        import matplotlib.cm as cm
        import matplotlib.colors as mcolors
    except Exception:
        cm = None
        mcolors = None

    material_colors = {
        'organic': '#a0522d',
        'Ti6Al4V': '#b0b0b0',
        'virtual': '#888888',
        'physical': '#1f77b4',
    }
    if property_name == 'material':
        return material_colors.get(str(property_value), '#cccccc')

    if cm and mcolors and isinstance(property_value, (int, float)):
        norm = mcolors.Normalize(vmin=0.0, vmax=100.0)
        cmap = cm.get_cmap(colormap)
        return mcolors.to_hex(cmap(norm(float(property_value))))

    return '#cccccc'


@dataclass
class SkeletonRenderNode:
    """Visualization-friendly container for a BoneSpec."""

    bone_spec: BoneSpec
    render_id: str
    position_3d: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation_quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    color: str = '#cccccc'
    geometry_type: str = 'sphere'
    geometry_params: Dict[str, Any] = field(default_factory=dict)
    is_visible: bool = True
    tooltip_data: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_bone(cls, bone: BoneSpec, color_by: str = 'material') -> 'SkeletonRenderNode':
        color_value = getattr(bone, color_by, None)
        if color_by == 'material':
            color_value = bone.material.get('name')
        color = get_color(color_value, color_by)
        tooltip = {
            'name': bone.name,
            'uid': bone.unique_id,
            'embodiment': bone.embodiment,
            'material': bone.material.get('name'),
            'faults': list(bone.state_faults),
        }
        geo_params = {
            'radius': (bone.dimensions.get('width_cm') or 1.0) * 0.5,
        }
        return cls(
            bone_spec=bone,
            render_id=bone.unique_id,
            position_3d=bone.position,
            orientation_quat=bone.orientation,
            color=color,
            geometry_type='sphere',
            geometry_params=geo_params,
            tooltip_data=tooltip,
        )
