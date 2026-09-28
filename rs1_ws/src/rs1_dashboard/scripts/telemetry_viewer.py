#!/usr/bin/env python3
"""Serve the upstream Lichtblick web bundle independently of robot processes."""

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class ViewerRequestHandler(SimpleHTTPRequestHandler):
    def list_directory(self, path):
        self.send_error(404, "File not found")
        return None

    def end_headers(self):
        self.send_header("X-Content-Type-Options", "nosniff")
        # The dashboard embeds this page from its own port. Do not set
        # X-Frame-Options: SAMEORIGIN or DENY here.
        super().end_headers()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    directory = args.directory.expanduser().resolve()
    if not (directory / "index.html").is_file():
        parser.error(
            f"Lichtblick assets are missing from {directory}. "
            "Run bash .devcontainer/scripts/setup_telemetry_viewer.sh "
            "from the repository root."
        )
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    handler = partial(ViewerRequestHandler, directory=str(directory))
    try:
        server = ThreadingHTTPServer((args.host, args.port), handler)
    except OSError as error:
        parser.error(f"Cannot serve Lichtblick on {args.host}:{args.port}: {error}")
    print(f"Lichtblick viewer: http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
