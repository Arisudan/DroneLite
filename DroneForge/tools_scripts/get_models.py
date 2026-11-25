#!/usr/bin/env python3
"""Download / check the models DroneForge needs. Standard library only, so it runs before anything else is installed.

  python3 tools_scripts/get_models.py                 download what is missing (the 4 required files, ~1.4 GB), verify size + sha256
  python3 tools_scripts/get_models.py --check         only check what is there (exit 1 if something is missing or wrong)
  python3 tools_scripts/get_models.py --list          what the models are for and where they come from
  python3 tools_scripts/get_models.py --optional      also the face-recognition model (~290 MB, GCS FACE REC)
  python3 tools_scripts/get_models.py --from DIR|URL  take the files from a folder (USB stick, another laptop) or a URL base instead of the internet
  python3 tools_scripts/get_models.py --dir DIR       models folder (default <DroneForge>/gazebo/vision/models)
  options: --only NAME[,NAME]  --force (download again)

An interrupted download resumes where it stopped (name.part). Nothing is replaced unless the new file passes the size and sha256 check."""
import argparse, hashlib, json, os, shutil, sys, time, urllib.error, urllib.request, zipfile

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
UA = {"User-Agent": "DroneForge-get-models/1.0"}


def sha256_of(path, chunk=1 << 22):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(chunk), b""): h.update(b)
    return h.hexdigest()


def human(n): return f"{n / 1e9:.2f} GB" if n >= 1e9 else f"{n / 1e6:.1f} MB"


def verify(path, size=None, sha=None):
    """'' when the file is good, else the reason."""
    if not os.path.isfile(path): return "missing"
    if size and os.path.getsize(path) != size: return f"size {os.path.getsize(path)} (expected {size})"
    if sha and sha256_of(path) != sha: return "sha256 does not match"
    return ""


def fetch(url, dest, size=None, retries=4, quiet=False):
    """Resumable download into dest + '.part', then rename. Raises on failure."""
    part = dest + ".part"; os.makedirs(os.path.dirname(dest), exist_ok=True)
    for attempt in range(1, retries + 1):
        have = os.path.getsize(part) if os.path.exists(part) else 0
        req = urllib.request.Request(url, headers={**UA, **({"Range": f"bytes={have}-"} if have else {})})
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                if have and r.status != 206: have = 0                                 # the server ignored the range: start over
                total = (have + int(r.headers.get("Content-Length", 0))) if r.status == 206 else int(r.headers.get("Content-Length", 0)) or (size or 0)
                t0 = time.time(); got = have; last = 0.0
                with open(part, "ab" if have else "wb") as f:
                    while True:
                        b = r.read(1 << 20)
                        if not b: break
                        f.write(b); got += len(b)
                        if not quiet and time.time() - last > 0.5 and total:
                            last = time.time(); rate = (got - have) / max(time.time() - t0, 1e-6)
                            sys.stdout.write(f"\r    {got / total * 100:5.1f} %  {human(got)} / {human(total)}  {rate / 1e6:.1f} MB/s   "); sys.stdout.flush()
                if not quiet: sys.stdout.write("\r" + " " * 70 + "\r")
            if size and os.path.getsize(part) != size: raise IOError(f"got {os.path.getsize(part)} bytes, expected {size}")
            os.replace(part, dest); return
        except (urllib.error.URLError, IOError, TimeoutError, ConnectionError) as e:
            print(f"    attempt {attempt}/{retries} failed: {e}")
            if isinstance(e, urllib.error.HTTPError) and e.code in (403, 404): raise
            if attempt == retries: raise
            time.sleep(2 * attempt)


def from_source(src, m, dest, quiet):
    """Take the file from a folder or a URL base (looks for its path, then its name)."""
    rel = [m["path"], m["name"]] if "path" in m else [m["name"]]
    if src.startswith(("http://", "https://")):
        last = None
        for r in rel:
            try: fetch(src.rstrip("/") + "/" + r, dest, m.get("size"), quiet=quiet); return
            except Exception as e: last = e
        raise last
    for r in rel:
        p = os.path.join(src, r)
        if os.path.isfile(p): os.makedirs(os.path.dirname(dest), exist_ok=True); shutil.copyfile(p, dest + ".part"); os.replace(dest + ".part", dest); return
    raise FileNotFoundError(f"{m['name']} not found in {src}")


def main(argv=None):
    ap = argparse.ArgumentParser(description="Download / check the DroneForge models.")
    ap.add_argument("--check", action="store_true"); ap.add_argument("--list", action="store_true"); ap.add_argument("--optional", action="store_true")
    ap.add_argument("--force", action="store_true"); ap.add_argument("--from", dest="src"); ap.add_argument("--dir"); ap.add_argument("--only"); ap.add_argument("--manifest", default=os.path.join(HERE, "models_manifest.json"))
    a = ap.parse_args(argv)
    man = json.load(open(a.manifest)); base = os.path.abspath(a.dir or os.path.join(ROOT, man["root"]))
    items = [dict(m, dest=os.path.join(base, m["path"]), kind="file") for m in man["models"]]
    if a.optional or (a.only and any(n in a.only.split(",") for n in [o["name"] for o in man.get("optional", [])])):
        items += [dict(o, dest=os.path.expanduser(o["dest"]), kind="zip") for o in man.get("optional", [])]
    if a.only: items = [m for m in items if m["name"] in a.only.split(",")]
    if a.list:
        print(f"models folder: {base}")
        for m in items + ([] if a.only else [dict(o, dest=os.path.expanduser(o["dest"]), kind="zip", optional=True) for o in man.get("optional", []) if not a.optional]):
            print(f"  {m['name']:34s} {human(m['size']):>9s}{'  (optional)' if m.get('optional') or m['kind'] == 'zip' else ''}\n      {m['what']}\n      from {m['url']}")
        for m in man.get("manual", []): print(f"  {m['name']:34s} (manual)\n      {m['what']}")
        return 0
    bad = 0; todo = 0
    for m in items:
        if m["kind"] == "zip": path = os.path.join(m["dest"], m["check_file"]); why = "" if os.path.isfile(path) else "missing"
        else: path = m["dest"]; why = verify(path, m.get("size"), m.get("sha256"))
        if not why and not a.force: print(f"[ok]       {m['name']}  ({human(m['size'])})"); continue
        if a.check: print(f"[{'MISSING' if why == 'missing' else 'BAD':8s}] {m['name']}: {why}  ->  {path}"); bad += 1; continue
        todo += 1; print(f"[get]      {m['name']}  ({human(m['size'])})  {m['what']}")
        try:
            tmp = (os.path.join(base, ".dl_" + m["name"] + ".zip") if m["kind"] == "zip" else path)
            if a.src: from_source(a.src, m, tmp, False)
            else: fetch(m["url"], tmp, m.get("size"))
            if m["kind"] == "zip":
                os.makedirs(m["dest"], exist_ok=True)
                with zipfile.ZipFile(tmp) as z: z.extractall(m["dest"])
                os.remove(tmp)
            else:
                why = verify(path, m.get("size"), m.get("sha256"))
                if why: os.remove(path); raise IOError("downloaded file failed the check: " + why)
            print(f"[done]     {m['name']}  -> {path}")
        except Exception as e:
            bad += 1; print(f"[FAILED]   {m['name']}: {e}")
            if not a.src: print("           no internet? copy the file from another machine and run again with  --from <folder>")
    if not a.only:
        for m in man.get("manual", []):
            print(f"[manual]   {m['name']}: {'present' if os.path.isfile(os.path.join(base, m['path'])) else 'not present (optional)'}")
    print("all required models are in place." if not bad else f"{bad} problem(s).")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
