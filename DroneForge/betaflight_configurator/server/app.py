"""Hand-written ASGI app: static files from ../web/ and the /ws JSON protocol."""
import asyncio
import json
import mimetypes
import os

from .controller import KNOWN

WEB_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "web")


def make_app(ctrl, web_dir=WEB_DIR, emulator=None):
    async def http(scope, receive, send):
        path = scope["path"]
        if emulator is not None and path.startswith("/demo/armed/"):
            emulator.armed = path.endswith("/1")
            return await respond(send, 200, b"armed=%d\n" % emulator.armed, "text/plain")
        rel = "index.html" if path in ("", "/") else path.lstrip("/")
        full = os.path.realpath(os.path.join(web_dir, rel))
        root = os.path.realpath(web_dir)
        if scope["method"] not in ("GET", "HEAD") or not (full == root or full.startswith(root + os.sep)) or not os.path.isfile(full):
            return await respond(send, 404, b"not found\n", "text/plain")
        with open(full, "rb") as f:
            body = f.read()
        ctype = mimetypes.guess_type(full)[0] or "application/octet-stream"
        if ctype.startswith("text/") or ctype in ("application/javascript", "application/json"):
            ctype += "; charset=utf-8"
        await respond(send, 200, body, ctype)

    async def respond(send, status, body, ctype):
        await send({"type": "http.response.start", "status": status,
                    "headers": [(b"content-type", ctype.encode()), (b"content-length", str(len(body)).encode()),
                                (b"cache-control", b"no-store")]})
        await send({"type": "http.response.body", "body": body})

    async def ws(scope, receive, send):
        ev = await receive()
        if ev["type"] != "websocket.connect":
            return
        await send({"type": "websocket.accept"})
        loop = asyncio.get_running_loop()
        q = asyncio.Queue()

        def push(event, data):
            try:
                loop.call_soon_threadsafe(q.put_nowait, json.dumps({"event": event, "data": data}, default=str))
            except RuntimeError:
                pass

        ctrl.subscribe(push)

        async def writer():
            while True:
                await send({"type": "websocket.send", "text": await q.get()})

        async def handle(text):
            rid = None
            try:
                req = json.loads(text)
                rid = req.get("id")
                result = await asyncio.to_thread(ctrl.call, req["method"], req.get("params") or {})
                msg = {"id": rid, "result": result}
            except KNOWN as e:
                msg = {"id": rid, "error": str(e.args[0]) if e.args else type(e).__name__}
            except Exception as e:
                msg = {"id": rid, "error": "internal error: %r" % (e,)}
            q.put_nowait(json.dumps(msg, default=str))

        wt = asyncio.create_task(writer())
        tasks = set()
        try:
            while True:
                ev = await receive()
                if ev["type"] == "websocket.disconnect":
                    break
                if ev["type"] == "websocket.receive":
                    text = ev.get("text") or (ev.get("bytes") or b"").decode("utf-8", "replace")
                    t = asyncio.create_task(handle(text))
                    tasks.add(t)
                    t.add_done_callback(tasks.discard)
        finally:
            ctrl.unsubscribe(push)
            wt.cancel()
            for t in tasks:
                t.cancel()

    async def app(scope, receive, send):
        if scope["type"] == "lifespan":
            while True:
                ev = await receive()
                if ev["type"] == "lifespan.startup":
                    await send({"type": "lifespan.startup.complete"})
                elif ev["type"] == "lifespan.shutdown":
                    ctrl.shutdown()
                    await send({"type": "lifespan.shutdown.complete"})
                    return
        elif scope["type"] == "http":
            await http(scope, receive, send)
        elif scope["type"] == "websocket":
            if scope["path"] == "/ws":
                await ws(scope, receive, send)
            else:
                await send({"type": "websocket.close", "code": 1008})

    return app
