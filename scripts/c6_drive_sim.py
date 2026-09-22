#!/usr/bin/env python3
"""Serve the local C6 simulator. Never opens robot ports or drives hardware."""
import argparse
import os
import re
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class LocalFiles(SimpleHTTPRequestHandler):
    extensions_map = {**SimpleHTTPRequestHandler.extensions_map, '.mjs': 'text/javascript'}

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Accept-Ranges', 'bytes')
        super().end_headers()

    def send_head(self):
        # Video seeking needs byte ranges; the stdlib handler ignores Range.
        self.remaining = None
        path = Path(self.translate_path(self.path))
        value = self.headers.get('Range')
        if not value or not path.is_file() or self.headers.get('If-Range'):
            return super().send_head()
        match = re.fullmatch(r'bytes=(\d*)-(\d*)', value)
        # Unsupported multipart/malformed requests fall back to the full file.
        if not match or not any(match.groups()):
            return super().send_head()
        try:
            source = path.open('rb')
        except OSError:
            self.send_error(404, 'File not found')
            return None
        size = os.fstat(source.fileno()).st_size
        first, last = match.groups()
        start = int(first) if first else max(0, size-int(last))
        end = min(size-1, int(last)) if first and last else size-1
        if start >= size or end < start or (not first and int(last) == 0):
            source.close()
            self.send_response(416)
            self.send_header('Content-Range', f'bytes */{size}')
            self.send_header('Content-Length', '0')
            self.end_headers()
            return None
        source.seek(start)
        self.remaining = end-start+1
        self.send_response(206)
        self.send_header('Content-type', self.guess_type(str(path)))
        self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
        self.send_header('Content-Length', str(self.remaining))
        self.end_headers()
        return source

    def copyfile(self, source, outputfile):
        try:
            if self.remaining is None:
                return super().copyfile(source, outputfile)
            while self.remaining:
                block = source.read(min(65536, self.remaining))
                if not block:
                    break
                outputfile.write(block)
                self.remaining -= len(block)
        except (BrokenPipeError, ConnectionResetError):
            pass  # A browser changing scenes may cancel the previous download.

    def list_directory(self, path):
        self.send_error(403, 'Directory listing disabled')
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', type=int, default=8088)
    args = ap.parse_args()
    root = Path(__file__).resolve().parents[1] / 'c6_drive'
    server = ThreadingHTTPServer(('127.0.0.1', args.port), partial(LocalFiles, directory=str(root)))
    print(f'WhiteBoardBot simulator: http://127.0.0.1:{args.port}/sim/', flush=True)
    print('Simulation only; no serial, Wi-Fi robot commands or firmware changes.', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
