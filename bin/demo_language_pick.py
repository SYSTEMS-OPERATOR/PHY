#!/usr/bin/env python
from language.language_agent import LanguageAgent


def main() -> None:
    lang = LanguageAgent()
    print(lang.listen("pick up cube"))
    lang.say("picked")


if __name__ == "__main__":
    main()
