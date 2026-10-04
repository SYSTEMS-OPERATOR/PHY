#!/usr/bin/env python3
"""Run PHY Studio locally. No account, inference service or internet needed."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import webbrowser

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--directory", type=Path, default=ROOT / "studio/dist")
    args = parser.parse_args()
    page = args.directory / "PHY-Studio.html"
    # The checked-in portable app works before the optional CAD tools are installed.
    if not page.exists() and args.directory == ROOT / "studio/dist":
        portable = ROOT / "studio/PHY-Studio.html"
        if portable.exists():
            args.directory = portable.parent
        else:
            parser.error("Build the app first: python bin/export_phy_studio.py --without-a0")
    server = ThreadingHTTPServer((args.host, args.port), partial(SimpleHTTPRequestHandler, directory=str(args.directory.resolve())))
    url = f"http://{args.host}:{server.server_port}/PHY-Studio.html"
    print(f"PHY Studio: {url}\nCtrl+C to stop. All models are local.", flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
