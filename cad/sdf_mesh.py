"""COLLADA (.dae) okuma, düzlem kesiti alma ve loft için kesit hazırlama.

build_tiltrotor_cad.py tarafından kullanılır. Yalnızca numpy gerektirir.
"""
import math
import numpy as np
import xml.etree.ElementTree as ET

NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}

def load_dae_points(path):
    root = ET.parse(path).getroot()
    pts = []
    tris = []
    for geo in root.iter():
        if not geo.tag.endswith('}mesh'):
            continue
        srcs = {}
        for s in geo:
            if s.tag.endswith('}source'):
                fa = s.find('c:float_array', NS)
                if fa is None:
                    continue
                srcs['#'+s.get('id')] = np.fromstring(fa.text, sep=' ').reshape(-1, 3)
        # vertices element maps an id to a POSITION source
        vmap = {}
        for s in geo:
            if s.tag.endswith('}vertices'):
                inp = s.find("c:input[@semantic='POSITION']", NS)
                vmap['#'+s.get('id')] = srcs[inp.get('source')]
        for prim in geo:
            tag = prim.tag.split('}')[1]
            if tag not in ('triangles', 'polylist', 'polygons'):
                continue
            inputs = prim.findall('c:input', NS)
            stride = max(int(i.get('offset')) for i in inputs) + 1
            vin = [i for i in inputs if i.get('semantic') == 'VERTEX'][0]
            off = int(vin.get('offset'))
            _s = vin.get("source"); arr = vmap[_s] if _s in vmap else srcs[_s]
            p = prim.find('c:p', NS)
            idx = np.fromstring(p.text, sep=' ', dtype=np.int64).reshape(-1, stride)[:, off]
            base = len(pts)
            pts.append(arr)
            if tag == 'triangles':
                tris.append(idx.reshape(-1, 3) + base)
            else:
                vc = np.fromstring(prim.find('c:vcount', NS).text, sep=' ', dtype=int)
                k = 0
                fan = []
                for n in vc:
                    f = idx[k:k+n]; k += n
                    for j in range(1, n-1):
                        fan.append([f[0], f[j], f[j+1]])
                tris.append(np.array(fan, dtype=np.int64) + base)
    V = np.vstack(pts)
    T = np.vstack(tris)
    return V, T

def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp,   cp*sr,          cp*cr]])

def chain_loops(segs, snap=2e-4, force_close=False):
    """Segmentleri uç uca ekleyip kapalı çevrimlere böl (düğüm indeksli graf)."""
    if not segs:
        return []
    nodes = []
    index = {}

    def nid(p):
        k = (round(p[0] / snap), round(p[1] / snap))
        if k not in index:
            index[k] = len(nodes)
            nodes.append(p)
        return index[k]

    adj = {}
    edges = []
    for a, b in segs:
        ia, ib = nid(a), nid(b)
        if ia == ib:
            continue
        e = len(edges)
        edges.append((ia, ib))
        adj.setdefault(ia, []).append(e)
        adj.setdefault(ib, []).append(e)

    used = [False] * len(edges)
    loops = []
    for e0, (ia, ib) in enumerate(edges):
        if used[e0]:
            continue
        used[e0] = True
        chain = [ia, ib]
        cur = ib
        while True:
            nxt = None
            for e in adj.get(cur, []):
                if used[e]:
                    continue
                u, v = edges[e]
                other = v if u == cur else u
                used[e] = True
                nxt = other
                break
            if nxt is None:
                break
            chain.append(nxt)
            cur = nxt
            if cur == chain[0]:
                break
        if len(chain) > 3 and chain[0] == chain[-1]:
            loops.append(np.array([nodes[i] for i in chain[:-1]]))
        elif force_close and len(chain) > 5:
            # Mesh'te kapanmayan kenar (pervane uçlarında görülüyor):
            # zinciri uçlarını birleştirerek kapat.
            loops.append(np.array([nodes[i] for i in chain]))
    return loops

def slice_axis(V, T, axis, val, snap=2e-4, force_close=False):
    """axis: 0=x,1=y,2=z. Kesit düzlemi axis=val. Diğer iki koordinatı döndürür."""
    keep_axes = [a for a in (0, 1, 2) if a != axis]
    P = V[T]
    d = P[:, :, axis] - val
    sel = ~((d > 0).all(1) | (d < 0).all(1))
    segs = []
    for tri in P[sel]:
        pts = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            da, db = tri[a, axis] - val, tri[b, axis] - val
            if da == db:
                continue
            if (da <= 0 <= db) or (db <= 0 <= da):
                t = da / (da - db)
                p = tri[a] + t * (tri[b] - tri[a])
                pts.append((p[keep_axes[0]], p[keep_axes[1]]))
        if len(pts) >= 2:
            a, b = np.array(pts[0]), np.array(pts[1])
            if np.hypot(*(a - b)) > 1e-9:
                segs.append((a, b))
    return chain_loops(segs, snap=snap, force_close=force_close)

def dedupe_loops(loops, tol=1e-4):
    """Çift yüzlü mesh yüzünden ikizlenen çevrimleri tekilleştir."""
    out = []
    for l in loops:
        sig = (round(len(l), 0), np.round(l.min(0), 4).tobytes(), np.round(l.max(0), 4).tobytes())
        if any(s == sig for s in [o[1] for o in out]):
            continue
        out.append((l, sig))
    return [o[0] for o in out]

def biggest_loop(loops):
    return max(loops, key=lambda l: abs(_area(l)))

def _area(l):
    x, y = l[:, 0], l[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)

def resample(loop, n=72, start_by='maxx'):
    """Çevrimi yay uzunluğuna göre n noktaya yeniden örnekle, CCW ve ortak
    başlangıç noktası (firar kenarı) ile hizala — loft yüzeyinin burulmaması için."""
    l = np.asarray(loop, float)
    if _area(l) < 0:
        l = l[::-1]
    i0 = int(np.argmax(l[:, 0])) if start_by == 'maxx' else int(np.argmax(l[:, 1]))
    l = np.roll(l, -i0, axis=0)
    l = np.vstack([l, l[:1]])
    seg = np.hypot(*(np.diff(l, axis=0).T))
    s = np.concatenate([[0], np.cumsum(seg)])
    if s[-1] <= 0:
        return None
    su = np.linspace(0, s[-1], n, endpoint=False)
    return np.column_stack([np.interp(su, s, l[:, 0]), np.interp(su, s, l[:, 1])])

def stations(V, T, axis, vals, n=72, start_by='maxx', snap=2e-4, fallback=False):
    """Her istasyonda en büyük kapalı çevrimi alıp yeniden örnekle.

    fallback=True: sıkı zincirleme bir istasyonda hiç kapalı çevrim bulamazsa
    (pervane uçlarında mesh kapanmıyor) o istasyon için toleransı gevşetip
    zinciri zorla kapatır. Gevşek kipi her istasyonda kullanmak kesitleri
    bozup loft'u geçersiz katıya çeviriyor; bu yüzden yalnızca boş kalan
    istasyonlarda devreye girer.
    """
    out = []
    for v in vals:
        loops = dedupe_loops(slice_axis(V, T, axis, v, snap=snap))
        if not loops and fallback:
            loops = dedupe_loops(slice_axis(V, T, axis, v, snap=5e-4, force_close=True))
        if not loops:
            continue
        r = resample(biggest_loop(loops), n, start_by)
        if r is None:
            continue
        if out:
            r = align_to(out[-1][1], r)
        out.append((v, r))
    return out


def align_to(prev, cur):
    """Kesitin başlangıç noktasını bir öncekine en çok benzeyecek şekilde
    kaydır. Sabit bir başlangıç ölçütü (ör. en büyük x) kesit yuvarlaklaştıkça
    istasyondan istasyona zıplıyor ve loft yüzeyini buruyor; pervane göbeğinde
    bu, katıyı geçersiz kılıyordu."""
    n = len(cur)
    best, bk = None, 0
    for k in range(n):
        d = float(np.sum((np.roll(cur, -k, axis=0) - prev) ** 2))
        if best is None or d < best:
            best, bk = d, k
    return np.roll(cur, -bk, axis=0)
