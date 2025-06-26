from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class LanguageAgent:
    """Minimal STT/TTS wrapper with simple intent parsing."""

    intents: Dict[str, Dict[str, str]] = field(default_factory=dict)

    def say(self, text: str) -> None:
        print(text)

    def listen(self, text: Optional[str] = None) -> Dict[str, str]:
        if text is None:
            return {}
        words = text.lower().split()
        intent = {}
        if "pick" in words and "cube" in words:
            intent = {"action": "pick", "obj": "cube"}
        elif "drop" in words and "cube" in words:
            intent = {"action": "drop", "obj": "cube"}
        self.intents[text] = intent
        return intent
