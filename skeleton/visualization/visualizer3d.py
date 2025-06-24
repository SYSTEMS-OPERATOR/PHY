from __future__ import annotations

from typing import Dict, List, Tuple

try:
    import plotly.graph_objects as go
except Exception:  # pragma: no cover - optional dependencies
    go = None

from .render import SkeletonRenderNode


class SkeletonVisualizer3D:
    """3D visualization utilities using plotly."""

    def render_points(self, nodes: List[SkeletonRenderNode], **kwargs) -> None:
        if not go:
            raise RuntimeError("plotly is required for 3D visualization")
        x, y, z, c = [], [], [], []
        for n in nodes:
            x.append(n.position_3d[0])
            y.append(n.position_3d[1])
            z.append(n.position_3d[2])
            c.append(n.color)
        fig = go.Figure(data=[go.Scatter3d(x=x, y=y, z=z, mode='markers', marker=dict(color=c, size=4))])
        fig.show()

    def render_cylinders(
        self, nodes: List[SkeletonRenderNode], connection_data: List[Tuple[str, str]], **kwargs
    ) -> None:
        if not go:
            raise RuntimeError("plotly is required for 3D visualization")
        id_map: Dict[str, SkeletonRenderNode] = {n.render_id: n for n in nodes}
        fig = go.Figure()
        for rid, node in id_map.items():
            fig.add_trace(go.Scatter3d(x=[node.position_3d[0]], y=[node.position_3d[1]], z=[node.position_3d[2]],
                                      mode='markers', marker=dict(color=node.color, size=4)))
        for a, b in connection_data:
            n1 = id_map.get(a)
            n2 = id_map.get(b)
            if not n1 or not n2:
                continue
            fig.add_trace(
                go.Scatter3d(
                    x=[n1.position_3d[0], n2.position_3d[0]],
                    y=[n1.position_3d[1], n2.position_3d[1]],
                    z=[n1.position_3d[2], n2.position_3d[2]],
                    mode='lines',
                    line=dict(color='gray', width=5),
                )
            )
        fig.show()
