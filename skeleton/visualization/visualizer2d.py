from __future__ import annotations

from typing import List, Tuple

try:
    import matplotlib.pyplot as plt
    import networkx as nx
except Exception:  # pragma: no cover - optional dependencies
    plt = None
    nx = None

from .render import SkeletonRenderNode


class SkeletonVisualizer2D:
    """Simple 2D visualizations using networkx and matplotlib."""

    def render_graph(
        self,
        skeleton_nodes: List[SkeletonRenderNode],
        connection_data: List[Tuple[str, str]],
        **kwargs,
    ) -> None:
        if not plt or not nx:
            raise RuntimeError("matplotlib and networkx are required for 2D visualization")
        G = nx.Graph()
        for node in skeleton_nodes:
            G.add_node(node.render_id, color=node.color)
        G.add_edges_from(connection_data)
        layout = kwargs.get("layout", nx.spring_layout)
        pos = layout(G)
        node_colors = [G.nodes[n]["color"] for n in G.nodes]
        nx.draw(G, pos, with_labels=True, node_color=node_colors, edge_color="gray")
        plt.show()

    def render_hierarchy(self, field, **kwargs) -> None:
        nodes = field.to_render_nodes()
        edges = []
        for bone in field.bones.values():
            for art in bone.articulations:
                partner = field.get(art.get("bone"))
                if partner:
                    edges.append((bone.unique_id, partner.unique_id))
        self.render_graph(nodes, edges, **kwargs)
