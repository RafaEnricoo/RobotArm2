"""
=====================================================================
 SERVER LAPTOP — ROBOT ARM + CONVEYOR (WiFi / WebSocket)
=====================================================================
 Browser  <--ws /ws-->  server.py  <--ws /esp (WiFi)-->  ESP32

 Tugas server:
   - menyajikan RobotArmControl.html di http://localhost:8765
   - meneruskan perintah dari browser ke ESP32, dan status ESP32 ke semua browser
   - mengirim "ping" ke ESP tiap 1 detik selama ada browser terbuka
     (kalau berhenti, ESP menghentikan conveyor sendiri setelah 3 detik)
   - broadcast UDP "ROBOTARM <port>" supaya ESP bisa menemukan IP laptop otomatis

 Install : pip install aiohttp
 Jalankan: python server.py            (opsi: --port 8765)
=====================================================================
"""

import argparse
import asyncio
import json
import socket
from pathlib import Path

from aiohttp import WSMsgType, web

HERE = Path(__file__).resolve().parent
HTML_FILE = HERE / "RobotArmControl.html"
DISCOVERY_PORT = 8766

esp = None          # koneksi WebSocket ESP32 (hanya satu)
esp_ip = ""
browsers = set()    # semua tab browser yang terbuka


def link_msg():
    return json.dumps({"t": "link", "esp": esp is not None, "ip": esp_ip})


def local_msg(level, txt):
    return json.dumps({"t": "msg", "lv": level, "txt": txt})


async def to_browsers(txt):
    if browsers:
        await asyncio.gather(*(b.send_str(txt) for b in list(browsers)), return_exceptions=True)


async def to_esp(obj):
    if esp is not None:
        try:
            await esp.send_str(json.dumps(obj))
        except ConnectionError:
            pass


# ---------------------------------------------------------------- browser
async def browser_handler(request):
    ws = web.WebSocketResponse(heartbeat=10)
    await ws.prepare(request)
    browsers.add(ws)
    print(f"[web] browser terhubung ({len(browsers)} tab)")
    await ws.send_str(link_msg())
    await to_esp({"cmd": "hello"})   # minta ESP kirim cfg + program + status

    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                cmd = json.loads(msg.data)
            except ValueError:
                continue
            if cmd.get("cmd") == "ping":
                continue             # ping ke ESP diurus heartbeat server
            if esp is None:
                await ws.send_str(local_msg("err", "ESP32 belum terhubung ke server."))
                continue
            await to_esp(cmd)
    finally:
        browsers.discard(ws)
        print(f"[web] browser terputus ({len(browsers)} tab)")
    return ws


# ---------------------------------------------------------------- ESP32
async def esp_handler(request):
    global esp, esp_ip
    ws = web.WebSocketResponse(heartbeat=5)   # ESP mati/hilang WiFi -> terdeteksi ~5 detik
    await ws.prepare(request)

    if esp is not None:                        # ESP restart: buang koneksi lama
        await esp.close()
    esp, esp_ip = ws, request.remote or ""
    print(f"[esp] ESP32 terhubung dari {esp_ip}")
    await to_browsers(link_msg())
    if browsers:
        await to_esp({"cmd": "hello"})

    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                await to_browsers(msg.data)    # cfg / prog / st / msg langsung diteruskan
    finally:
        if esp is ws:
            esp, esp_ip = None, ""
            print("[esp] ESP32 terputus")
            await to_browsers(link_msg())
    return ws


# ---------------------------------------------------------------- latar belakang
async def heartbeat():
    while True:
        await asyncio.sleep(1)
        if browsers:
            await to_esp({"cmd": "ping"})


class Beacon(asyncio.DatagramProtocol):
    pass


async def discovery(port):
    loop = asyncio.get_running_loop()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    transport, _ = await loop.create_datagram_endpoint(Beacon, sock=sock)
    data = f"ROBOTARM {port}".encode()
    try:
        while True:
            try:
                transport.sendto(data, ("255.255.255.255", DISCOVERY_PORT))
            except OSError:
                pass                 # belum ada jaringan, coba lagi nanti
            await asyncio.sleep(1)
    finally:
        transport.close()


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))   # tidak benar-benar mengirim data
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


async def index(request):
    return web.FileResponse(HTML_FILE, headers={"Cache-Control": "no-cache"})


async def on_startup(app):
    port = app["port"]
    app["tasks"] = [asyncio.create_task(heartbeat()), asyncio.create_task(discovery(port))]


async def on_cleanup(app):
    for t in app["tasks"]:
        t.cancel()
    await asyncio.gather(*app["tasks"], return_exceptions=True)


def main():
    ap = argparse.ArgumentParser(description="Server WebSocket Robot Arm")
    ap.add_argument("--port", type=int, default=8765)
    args = ap.parse_args()

    app = web.Application()
    app["port"] = args.port
    app.router.add_get("/", index)
    app.router.add_get("/ws", browser_handler)
    app.router.add_get("/esp", esp_handler)
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)

    ip = lan_ip()
    print("=" * 60)
    print(f" Buka di browser : http://localhost:{args.port}")
    print(f" IP laptop       : {ip}  (isi SERVER_HOST di sketch kalau auto-cari gagal)")
    print("=" * 60)
    web.run_app(app, host="0.0.0.0", port=args.port, print=None)


if __name__ == "__main__":
    main()
