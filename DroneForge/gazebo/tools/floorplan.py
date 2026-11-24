#!/usr/bin/env python3
"""Floor plan of the generated house world, sliced at flight height (default 0.30-0.80 m).
Used offline to choose room-to-room waypoints; the drone itself navigates by camera. Writes house_plan.npz and house_plan.png."""
import re, sys, math, xml.etree.ElementTree as ET
import numpy as np
HERE = __import__("os").path.dirname(__import__("os").path.abspath(__file__)); ROOT = HERE + "/.."
MODELS = ROOT + "/third_party/small_house_src/models/"
def rotz(y): c, s = math.cos(y), math.sin(y); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def mesh_tris(path):
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    root = ET.parse(path).getroot()
    up = (root.findtext("c:asset/c:up_axis", default="Z_UP", namespaces=ns) or "Z_UP").strip()
    unit = float(root.find("c:asset/c:unit", ns).get("meter", "1")) if root.find("c:asset/c:unit", ns) is not None else 1.0
    out = []
    # node transforms (composed down the scene graph) per geometry id
    xf = {}
    def walk(node, M):
        mm = node.find("c:matrix", ns)
        if mm is not None: M = M @ np.array(mm.text.split(), float).reshape(4, 4)
        for ig in node.iterfind("c:instance_geometry", ns): xf.setdefault(ig.get("url")[1:], []).append(M)
        for ch in node.iterfind("c:node", ns): walk(ch, M)
    for vs in root.iterfind(".//c:visual_scene", ns):
        for nd in vs.iterfind("c:node", ns): walk(nd, np.eye(4))
    for geom in root.iterfind(".//c:geometry", ns):
        mesh = geom.find("c:mesh", ns)
        srcs = {}
        for s_ in mesh.iterfind("c:source", ns):
            fa = s_.find("c:float_array", ns); acc = s_.find(".//c:accessor", ns)
            if fa is not None and acc is not None and int(acc.get("stride", "3")) == 3: srcs[s_.get("id")] = np.array(fa.text.split(), float).reshape(-1, 3)
        vmap = {v.get("id"): v.find("c:input[@semantic='POSITION']", ns).get("source")[1:] for v in mesh.iterfind("c:vertices", ns)}
        for t in list(mesh.iterfind("c:triangles", ns)) + list(mesh.iterfind("c:polylist", ns)):
            inputs = t.findall("c:input", ns); stride = max(int(i.get("offset")) for i in inputs) + 1
            vin = [i for i in inputs if i.get("semantic") == "VERTEX"][0]; off = int(vin.get("offset"))
            pos = srcs[vmap[vin.get("source")[1:]]]
            mats = xf.get(geom.get("id"), [np.eye(4)]); n0 = len(out)
            idx = np.array(t.findtext("c:p", namespaces=ns).split(), int).reshape(-1, stride)[:, off]
            if t.tag.endswith("polylist"):
                vc = list(map(int, t.findtext("c:vcount", namespaces=ns).split())); k = 0
                for n in vc:
                    for j in range(1, n - 1): out.append(pos[[idx[k], idx[k + j], idx[k + j + 1]]])
                    k += n
            else:
                for j in range(0, len(idx), 3): out.append(pos[idx[j:j + 3]])
            tri = np.array(out[n0:]); del out[n0:]
            for M in mats:
                h = np.concatenate([tri, np.ones(tri.shape[:2] + (1,))], -1) @ M.T
                out.extend(list(h[..., :3]))
    tr = np.array(out) * unit
    if up == "Y_UP": tr = tr[..., [0, 2, 1]] * np.array([1, -1, 1])
    return tr
def main(z0=0.45, z1=0.80, res=0.05):
    s = open(ROOT + "/worlds/droneforge_house.sdf").read()
    x0, x1, y0, y1 = -11, 11, -7.5, 7.5
    W, H = int((x1 - x0) / res), int((y1 - y0) / res)
    grid = np.zeros((H, W), np.uint8)
    cache = {}
    for m in re.finditer(r"<model name='([^']+)'>\s*<static>.*?model://aws_robomaker_residential_([A-Za-z0-9_]+)</uri>.*?<pose[^>]*>([^<]+)</pose>", s, re.S):
        name, kind, pose = m.groups()
        if kind in ("RoomCeiling_01", "FloorB_01", "Carpet_01", "Chandelier_01", "Curtain_01") or kind.startswith(("Portrait", "DeskPortrait", "Light")): continue
        p = [float(v) for v in pose.split()]
        sdf = open(MODELS + "aws_robomaker_residential_" + kind + "/model.sdf").read()
        for u in re.findall(r"<collision.*?<uri>model://aws_robomaker_residential_[^/]+/(meshes/[^<]+)</uri>", sdf, re.S):
            key = (kind, u)
            if key not in cache:
                try: cache[key] = mesh_tris(MODELS + "aws_robomaker_residential_" + kind + "/" + u)
                except Exception as e: print("skip", kind, u, e); cache[key] = None
            tr = cache[key]
            if tr is None: continue
            tw = tr @ rotz(p[5]).T + np.array(p[:3])
            zmin, zmax = tw[..., 2].min(1), tw[..., 2].max(1)
            for t in tw[(zmax >= z0) & (zmin <= z1)]:
                xs = ((t[:, 0] - x0) / res).astype(float); ys = ((t[:, 1] - y0) / res).astype(float)
                a, b = int(max(0, xs.min())), int(min(W - 1, xs.max())); c, d = int(max(0, ys.min())), int(min(H - 1, ys.max()))
                if (b - a) * (d - c) > 400000: continue
                gx, gy = np.meshgrid(np.arange(a, b + 1) + .5, np.arange(c, d + 1) + .5)
                (ax, ay), (bx, by), (cx, cy) = zip(xs, ys)
                den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
                if abs(den) < 1e-9:  # degenerate in plan (vertical wall face): draw the segment
                    for q in np.linspace(0, 1, 60):
                        for u0, u1 in ((0, 1), (1, 2)):
                            px = xs[u0] + (xs[u1] - xs[u0]) * q; py = ys[u0] + (ys[u1] - ys[u0]) * q
                            if 0 <= px < W and 0 <= py < H: grid[int(py), int(px)] = 1
                    continue
                l1 = ((by - cy) * (gx - cx) + (cx - bx) * (gy - cy)) / den; l2 = ((cy - ay) * (gx - cx) + (ax - cx) * (gy - cy)) / den
                inside = (l1 >= -.05) & (l2 >= -.05) & (1 - l1 - l2 >= -.05)
                grid[c:d + 1, a:b + 1][inside] = 1
    np.savez(ROOT + "/worlds/house_plan.npz", grid=grid, x0=x0, y0=y0, res=res)
    try:
        import cv2; img = np.where(grid[::-1] > 0, 0, 255).astype(np.uint8); cv2.imwrite(ROOT + "/worlds/house_plan.png", cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
    except Exception as e: print("png:", e)
    print("occupied cells:", int(grid.sum()), "of", grid.size)
main()
