from __future__ import annotations

from typing import Iterable, List

from .render import SkeletonRenderNode


class AnimationController:
    """Simple animation controller for SkeletonField snapshots."""

    def __init__(self, frames: Iterable[List[SkeletonRenderNode]]):
        self.frames = list(frames)
        self.index = 0

    def next_frame(self) -> List[SkeletonRenderNode]:
        self.index = (self.index + 1) % len(self.frames)
        return self.frames[self.index]

    def previous_frame(self) -> List[SkeletonRenderNode]:
        self.index = (self.index - 1) % len(self.frames)
        return self.frames[self.index]

    def current_frame(self) -> List[SkeletonRenderNode]:
        return self.frames[self.index]
