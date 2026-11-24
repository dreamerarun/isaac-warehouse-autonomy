#!/usr/bin/env python3
"""Static server for the dashboard (web/) that tells browsers to revalidate every file, so a
reloaded page always runs the current app.js/style.css after an update.

    python3 tools/web_server.py [port] [directory]
"""
import functools, http.server, os, sys


class NoCache(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def log_message(self, fmt, *args):            # keep logs/web.log short: errors only
        if args and str(args[1])[:1] in ("4", "5"):
            super().log_message(fmt, *args)


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8090
    root = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "web")
    handler = functools.partial(NoCache, directory=os.path.abspath(root))
    with http.server.ThreadingHTTPServer(("0.0.0.0", port), handler) as srv:
        print(f"dashboard on http://0.0.0.0:{port}/ from {os.path.abspath(root)}", flush=True)
        srv.serve_forever()


if __name__ == "__main__":
    main()
