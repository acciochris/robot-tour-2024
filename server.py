from microdot import Microdot
from tarfile import TarFile
from pathlib import Path
from network import WLAN
import socket

import tarfile
import network
import time
import os
import io

server = Microdot()

DIR_HTML = """\
<html>
    <head>
        <title>{dir}</title>
    </head>
    <body>
        <h1>{dir}</h1>
        <ul>
            {files}
        </ul>
    </body>
<html>
"""

DIR_ELEMENT = """\
<li>
    <a href="{url}">{text}</a>
</li>
"""

HOST_CONFIG = {"essid": "lhs-robot-tour", "password": "12345678"}


@server.post("/update")
def update_program(request):
    tar = TarFile(fileobj=io.BytesIO(request.body))
    try:
        for file in tar:
            if file.type == tarfile.DIRTYPE:
                Path(file.name).mkdir(parents=True, exist_ok=True)
            else:
                contents = tar.extractfile(file)
                Path(file.name).write_bytes(contents.read())
    except OSError as e:
        return {
            "status": "error",
            "details": str(e),
        }

    return {"status": "success"}


def _format_html(path):
    return DIR_HTML.format(
        dir=path,
        files="\n".join(
            DIR_ELEMENT.format(
                url=f"/files{(path / file).resolve()}",
                text=file,
            )
            for file in os.listdir(path.resolve())
        ),
    ), {"Content-Type": "text/html"}


@server.get("/files")
def display_file_root(request):
    return _format_html(Path("/"))


@server.get("/files/<path:path>")
def display_file(request, path):
    path = Path(path)
    if not path.exists():
        return str(path), 404
    if path.is_file():
        return path.read_text(), {"Content-Type": "text/plain"}

    # a directory
    return _format_html(path)


@server.get("/shutdown")
def shutdown(request):
    request.app.shutdown()
    return {"status": "success"}


def setup():
    ap = WLAN(network.AP_IF)
    ap.active(True)
    ap.config(**HOST_CONFIG)

    while not ap.active():
        time.sleep_ms(100)


def run():
    server.run()
