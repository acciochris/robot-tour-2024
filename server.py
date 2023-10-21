from microdot import Microdot
from tarfile import TarFile
from pathlib import Path
from network import WLAN
from machine import I2C, Pin
from ssd1306 import SSD1306_I2C
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
STATION_CONFIG = [
    {"ssid": "acciochris", "password": "NablaDotB=0"},
]


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


def _wait_until(callback):
    for _ in range(100):
        if callback():
            break
        time.sleep_ms(100)
    else:
        raise RuntimeError("Connection timed out")


def setup():
    # try station first
    sta = WLAN(network.STA_IF)
    sta.active(True)
    ssids = [info[0].decode() for info in sta.scan()]
    for known_network in STATION_CONFIG:
        if known_network["ssid"] in ssids:
            sta.connect(known_network["ssid"], known_network["password"])
            _wait_until(sta.isconnected)
            print(f"Connected to {known_network['ssid']}")

            # show info about network
            display = SSD1306_I2C(128, 64, I2C(sda=Pin(4), scl=Pin(5)))
            display.text(known_network["ssid"] + ":", 0, 0, 1)
            display.text(sta.ifconfig()[0], 0, 8, 1)
            display.show()
            return

    # use ap as fallback
    sta.active(False)
    del sta

    ap = WLAN(network.AP_IF)
    ap.active(True)
    ap.config(**HOST_CONFIG)

    _wait_until(ap.active)
    print(f"Running as ap {HOST_CONFIG['essid']}")


def run():
    server.run()
