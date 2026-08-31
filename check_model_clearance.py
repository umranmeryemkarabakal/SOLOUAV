#!/usr/bin/env python3
"""
Tiltrotor SDF geometri denetimi: pervane diskleri ile TUM govde yapilari
arasindaki en kucuk mesafeyi, her rotorun TAM tilt supurmesi boyunca olcer.

NEDEN VAR (2026-08-30). Kullanici SITL'de kuyruk pervanesinin arka kanadin
icinden gectigini gordu. Elle bakmak yerine olcunce UC ayri kesisme cikti --
ve biri (dikey stabilize) tilt=0'da, yani hover'in TAMAMINDA. SDF'in kendi
yorumu "0.11 m clearance ... through the tilt sweep" diyordu; o yalnizca
tilt=0 icin dogruydu, yorumu yazan supurmeyi hic hesaplamamisti.

YONTEM: her rotor diski, tilt ekseni (y) etrafinda 0..90 deg dondurulur.
Disk, merkez + yaricap boyunca birkac halka olarak orneklenir; her ornek
noktasinin eksen-hizali kutuya en kisa mesafesi alinir. 0 = temas/nufuz.

KULLANIM
    python3 check_model_clearance.py                 # mevcut SDF
    python3 check_model_clearance.py --aday           # onerilen degisiklikle
"""
import re
import sys
import os
import math
import numpy as np


# --- Model geometrisi: SDF'TEN OKUNUR -------------------------------------
# ELLE YAZILMAZ. Ilk surumde kutular elle girilmisti ve bu, SDF degistiginde
# denetimin sessizce BAYATLAMASI demekti -- yani araci degersiz kilan sey.
SDF = 'tiltrotor_tailplane_model.sdf'

def _load_sdf():
    txt = open(SDF).read()
    rot, struct = {}, {}
    # rotor/motor link pozlari + yaricap
    for nm in ('motor_0','rotor_0','motor_1','rotor_1','motor_2','rotor_2'):
        m = re.search(r"<link name='%s'>(.*?)</link>" % nm, txt, re.S)
        if not m: continue
        pose = [float(x) for x in re.search(r"<pose>([^<]+)</pose>", m.group(1)).group(1).split()[:3]]
        rad = re.search(r"<radius>([^<]+)</radius>", m.group(1))
        struct[nm] = (pose, float(rad.group(1)) if rad else None)
    for i in (0,1,2):
        mp, _ = struct['motor_%d'%i]; rp, rr = struct['rotor_%d'%i]
        # TILT ARALIGI SDF'TEN OKUNUR. Ilk surum her rotoru 0..90 tariyordu ve
        # bu, motor_2_joint'in <upper> = 0.349 (20 deg) ile sinirlandigi
        # (adim 133) gercegini GORMUYORDU -- yani ulasilamayan acilarda sahte
        # carpisma raporluyordu. Eklem ne diyorsa o taranir.
        jm = re.search(r"<joint name='motor_%d_joint'.*?</joint>" % i, txt, re.S)
        up = re.search(r"<upper>([^<]+)</upper>", jm.group(0)) if jm else None
        tmax = float(up.group(1)) if up else 1.5708
        rot['rotor_%d'%i] = dict(m=tuple(mp), r=tuple(rp), R=rr,
                                 tilt_max_deg=int(round(np.degrees(tmax))))
    # base_link visual kutulari
    mb = re.search(r"<link name='base_link'>(.*?)</link>", txt, re.S).group(1)
    boxes = {}
    for m in re.finditer(r"<visual name='([^']+)'>(.*?)</visual>", mb, re.S):
        nm, b = m.group(1), m.group(2)
        po = re.search(r"<pose>([^<]+)</pose>", b); sz = re.search(r"<size>([^<]+)</size>", b)
        if not (po and sz): continue
        c = [float(x) for x in po.group(1).split()[:3]]
        L = [float(x) for x in sz.group(1).split()]
        boxes[nm] = (c, L)
    # hareketli yuzey link'leri
    for nm in ('left_elevator','right_elevator','rudder'):
        m = re.search(r"<link name=[\'\"]%s[\'\"]>(.*?)</link>" % nm, txt, re.S)
        if not m: continue
        po = re.search(r"<pose>([^<]+)</pose>", m.group(1))
        sz = re.search(r"<size>([^<]+)</size>", m.group(1))
        if not (po and sz): continue
        c = [float(x) for x in po.group(1).split()[:3]]
        L = [float(x) for x in sz.group(1).split()]
        boxes[nm] = (c, L)
    return rot, boxes

# --- (eski, elle yazilmis tablolar asagida referans olarak duruyor) --------
# Rotorlar: (ad, motor_pose, rotor_pose, yaricap). Tilt ekseni y, motor
# link'inin orijininden gecer; rotor onun COCUGU, yani tilt ile birlikte doner.

def box(cx, cy, cz, lx, ly, lz):
    return np.array([[cx-lx/2, cx+lx/2], [cy-ly/2, cy+ly/2], [cz-lz/2, cz+lz/2]])

_rot_raw, _box_raw = _load_sdf()
ROTORS = {k: v for k, v in _rot_raw.items()}
STRUCT = {k: box(c[0], c[1], c[2], L[0], L[1], L[2]) for k, (c, L) in _box_raw.items()}
# Govde/kanat: gorsel bir MESH (x8_wing.dae), kutu degil. SDF'in KENDI collision
# kutusu kullaniliyor -- tek gercek kaynak orasi.
STRUCT['govde/kanat'] = box(-0.01, 0.0, -0.135, 0.55, 2.144, 0.05)

# base_link visual parcalari + hareketli yuzeyler.
# NOT: govde (x8_wing.dae) bir MESH; kaba bir kutu ile temsil ediliyor.
# Kutu MESH'ten buyukse yanlis alarm, kucukse kacirma olur -- bu yuzden
# kanat kutusu SDF'teki collision kutusuyla ayni tutuldu (0.55 x 2.144 x 0.05).

# --- Onerilen aday (Adim 132) ---------------------------------------------
def apply_candidate():
    """Adim 132 adayi. PERVANE CAPI DEGISMEZ (R = 0.10 her yerde) -- boylece
    ROTOR_KF / ROTOR_TMAX / thrustToNormalized() hic dokunulmadan kalir.

    ON: pilonlari kisalt. Pilon SALT YAPIDIR -- ne aero modelinde (LiftDrag)
    ne kontrol modelinde (etkinlik matrisi) yeri var, yani KONTROL BEDELI SIFIR.
    Olculdu: 0.05 -> 0.03 uzunluk + merkez 0.22 -> 0.205, aciklik 0 -> 25 mm.
    Motoru yukari almak alternatifiydi ve COK daha kotu: +9.5 cm'de ancak 10 mm.

    KUYRUK: fin/rudder geriye (fin cakismasi tilt=0'da, yani hover boyunca --
    tilt sinirlamak bu yuzden CARE DEGIL), pilon +9 cm, cubugun rotor
    altindaki bolumu 4 -> 2 cm inceltilmis."""
    STRUCT['sag_motor_pylon'] = box(0.205, -0.25, 0.040, 0.03, 0.03, 0.06)
    STRUCT['sol_motor_pylon'] = box(0.205,  0.25, 0.040, 0.03, 0.03, 0.06)
    STRUCT['vertical_stab']   = box(-0.86, 0.0, 0.120, 0.16, 0.02, 0.20)
    STRUCT['rudder']          = box(-0.95, 0.0, 0.120, 0.06, 0.02, 0.18)
    STRUCT['tail_boom']       = box(-0.50, 0.0, 0.000, 0.62, 0.04, 0.02)
    # TARIHSEL (Adim 132 adayi): degerler O GUNE ait -- kuyruk motoru artik
    # -0.55'te ve gercek pervane yaricapi 0.1294. Bu fonksiyon yalnizca --aday
    # ile calisir ve o gunku karsilastirmayi yeniden uretmek icindir; ANA kapi
    # yolunda DEGILDIR. Guncel geometri her zaman SDF'ten okunur.
    ROTORS['rotor_2 (kuyruk)'] = dict(m=(-0.65, 0.0, 0.135), r=(-0.65, 0.0, 0.16), R=0.10)


def disc_box_dist(c, n, R, BOX, n_theta=180, rings=(1.0, 0.75, 0.5, 0.25, 0.0)):
    """Disk (merkez c, normal n, yaricap R) ile eksen-hizali kutu arasi en kisa mesafe."""
    n = n / np.linalg.norm(n)
    a = np.array([1.0, 0, 0])
    if abs(n @ a) > 0.9:
        a = np.array([0, 1.0, 0])
    u = np.cross(n, a); u /= np.linalg.norm(u)
    v = np.cross(n, u)
    t = np.linspace(0, 2*np.pi, n_theta, endpoint=False)
    best = np.inf
    for f in rings:
        P = c + (f*R) * (np.outer(np.cos(t), u) + np.outer(np.sin(t), v))
        q = np.clip(P, BOX[:, 0], BOX[:, 1])
        best = min(best, float(np.linalg.norm(P - q, axis=1).min()))
    return best


def sweep(rotor, tilt_max_deg=90, step=2):
    """Bir rotorun tilt supurmesi boyunca her yapiya en kucuk mesafesi (mm)."""
    m = np.array(rotor['m']); r = np.array(rotor['r']); R = rotor['R']
    off = r - m                      # rotorun tilt eksenine gore ofseti
    out = {k: (np.inf, 0) for k in STRUCT}
    for dd in range(0, tilt_max_deg + 1, step):
        d = np.radians(dd)
        # y ekseni etrafinda donus
        c = m + np.array([off[0]*np.cos(d) + off[2]*np.sin(d),
                          off[1],
                          -off[0]*np.sin(d) + off[2]*np.cos(d)])
        nrm = np.array([np.sin(d), 0.0, np.cos(d)])   # tilt=0 -> +z, tilt=90 -> +x
        for k, BOX in STRUCT.items():
            v = disc_box_dist(c, nrm, R, BOX)
            if v < out[k][0]:
                out[k] = (v, dd)
    return {k: (v*1000, dd) for k, (v, dd) in out.items()}


def rotor_rotor(step=2):
    """Rotor diskleri BIRBIRINE degiyor mu (tum tilt kombinasyonlari)."""
    names = list(ROTORS)
    worst = {}
    for i in range(len(names)):
        for j in range(i+1, len(names)):
            A, B = ROTORS[names[i]], ROTORS[names[j]]
            best = np.inf
            for da in range(0, 91, 10):
                for db in range(0, 91, 10):
                    ca = _center(A, da); cb = _center(B, db)
                    # iki disk arasi ALT SINIR: merkez mesafesi - iki yaricap
                    d = np.linalg.norm(ca - cb) - A['R'] - B['R']
                    best = min(best, d)
            worst[f"{names[i][:8]} <-> {names[j][:8]}"] = best*1000
    return worst


def _center(rot, dd):
    m = np.array(rot['m']); off = np.array(rot['r']) - m
    d = np.radians(dd)
    return m + np.array([off[0]*np.cos(d) + off[2]*np.sin(d), off[1],
                         -off[0]*np.sin(d) + off[2]*np.cos(d)])




def collision_audit():
    """COLLISION DENETIMI (adim 137). Pervane supurmesinden AYRI bir soru:
    collision kutulari dogru yerde ve dogru yonelimde mi?

    NEDEN VAR: adim 133'te bes yuzeye collision eklendi ve elevon kutusu
    MESH'in donmus pozunu (rpy = pi/2, 0, pi) miras aldigi icin yanlis eksende
    kaldi -- 34 cm'lik boyut DIKEY duruyordu, yani kanattan sarkan bir levha.
    Gorsel denetimle gorulmez; ancak donusu uygulayip dunya boyutunu olcunce
    ortaya cikti."""
    import xml.etree.ElementTree as ET
    txt = open(SDF).read()
    print("\n### COLLISION DENETIMI ###")
    bad = 0
    for lm in re.finditer(r"<link name=['\"]([^'\"]+)['\"]>(.*?)</link>", txt, re.S):
        lnm, body = lm.group(1), lm.group(2)
        for cm in re.finditer(r"<collision name='([^']+)'>(.*?)</collision>", body, re.S):
            cnm, cb = cm.group(1), cm.group(2)
            sz = re.search(r"<size>([^<]+)</size>", cb)
            po = re.search(r"<pose>([^<]+)</pose>", cb)
            if not sz:
                continue
            L = np.array([float(v) for v in sz.group(1).split()])
            if po:
                v = [float(x) for x in po.group(1).split()]
                rpy = v[3:6] if len(v) >= 6 else [0, 0, 0]
            else:
                rpy = [0, 0, 0]
            cr, sr = np.cos(rpy[0]), np.sin(rpy[0])
            cp_, sp_ = np.cos(rpy[1]), np.sin(rpy[1])
            cy, sy = np.cos(rpy[2]), np.sin(rpy[2])
            R = np.array([[cy*cp_, cy*sp_*sr-sy*cr, cy*sp_*cr+sy*sr],
                          [sy*cp_, sy*sp_*sr+cy*cr, sy*sp_*cr-cy*sr],
                          [-sp_,   cp_*sr,          cp_*cr]])
            w = 2 * (np.abs(R) @ (L/2))
            # Ince yuzeyler (elevon/elevator/rudder) dunyada DA ince olmali.
            thin = min(L) < 0.02
            if thin and min(w) > 0.05:
                print(f"   ⛔ {cnm}: yerel {L} -> dunya {np.round(w,3)}  "
                      f"(ince yuzey KALIN cikti -- yonelim hatasi)")
                bad += 1
            else:
                print(f"   OK {cnm:<28} dunya {np.round(w,3)}")
    print(f"   yonelim hatasi: {bad}")
    return bad


# --------------------------------------------------------------------------
# MESH TABANLI DENETIM (2026-08-31, Adim 143)
#
# NEDEN VAR. Yukaridaki sweep(), acikligi SDF'in COLLISION kutularina karsi
# olcer. Ama base_link'in collision kutusu (0.55 x 2.144 x 0.05) gercek
# x8_wing.dae mesh'ini TEMSIL ETMIYOR: mesh z = -0.100..+0.208 arasinda,
# kutu ise yalnizca z = -0.160..-0.110. Yani GOVDE PODU (yarim genislik
# 0.20 m, x = +0.288'e kadar one uzanan fairing) collision'da HIC YOK.
#
# Sonuc: y = +-0.25'teki rotorlar 72-88 derece tiltte podun ICINDEN geciyordu
# ama bu betik "65.6 mm acik" diyordu. Hata kutuda, olcumde degil. CAD katilari
# mesh'ten loft edildigi icin girisim URETIMDE de gercekti.
#
# Bu fonksiyon ayni taramayi GERCEK UCGEN MESH'e karsi tekrarlar. Kutu ile
# mesh ayrisirsa gercek olan mesh'tir.
# --------------------------------------------------------------------------
MESH_DIR = os.environ.get(
    'VTOL_MESH_DIR',
    os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models/standard_vtol/meshes'))
WING_VIS_POSE = ([0.53, -1.072, -0.1], [1.5707963268, 0.0, 3.1415926536])
WING_Y_SHIFT = -0.00351
DISC_HALF_T = 0.0053          # pervane disk yari kalinligi (10.6 mm / 2)


def _wing_mesh(n_sample=600000):
    """x8_wing.dae'yi govde cercevesinde dondurur; yoksa None.

    KOSE NOKTASI YETMEZ (2026-08-31, Adim 143b). Ilk surum yalnizca mesh'in
    KOSE noktalarini donduruyordu. Kose araligi bu mesh'te ~21 mm; disk yuzeye
    kosuler ARASINDAN girdiginde en yakin koseye olan mesafe acikligi 30 mm'ye
    kadar FAZLA gosteriyordu. Adim 143'un ilk konum secimi bu yuzden yanlis
    cikti: "32.8 mm temiz" denen konumda cad/dogrula.py hala 18.7 cm^3 nufuz
    buluyordu ve dogru olan oydu.
    Cozum: ucgenler uzerinde ALANLA ORANTILI ornekleme (~1.5 mm yogunluk).
    """
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cad'))
    try:
        from sdf_mesh import load_dae_points
    except ImportError:
        return None
    path = os.path.join(MESH_DIR, 'x8_wing.dae')
    if not os.path.exists(path):
        return None
    V, _T = load_dae_points(path)
    V = V * 0.001
    (px, py, pz), (rr, rp, ry) = WING_VIS_POSE
    cr, sr = math.cos(rr), math.sin(rr)
    cp, sp = math.cos(rp), math.sin(rp)
    cy, sy = math.cos(ry), math.sin(ry)
    R = (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
         @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
         @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
    V = V @ R.T + np.array([px, py, pz])
    V[:, 1] += WING_Y_SHIFT
    if _T is None or len(_T) == 0:
        return V
    T = np.asarray(_T)
    A, B, C = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    area = 0.5 * np.linalg.norm(np.cross(B - A, C - A), axis=1)
    tot = area.sum()
    if tot <= 0:
        return V
    rng = np.random.default_rng(0)          # sabit tohum: kapi tekrarlanabilir
    idx = rng.choice(len(T), size=n_sample, p=area / tot)
    u = rng.random(n_sample)
    v = rng.random(n_sample)
    m = u + v > 1.0
    u[m] = 1.0 - u[m]
    v[m] = 1.0 - v[m]
    S = A[idx] + u[:, None] * (B[idx] - A[idx]) + v[:, None] * (C[idx] - A[idx])
    return np.vstack([S, V])


def mesh_audit():
    V = _wing_mesh()
    print("\n### GERCEK KANAT MESH'INE KARSI (collision kutusu DEGIL) ###")
    if V is None:
        print("   x8_wing.dae bulunamadi -- atlandi (VTOL_MESH_DIR ayarlayin)")
        return 0
    bad = 0
    for rname, rot in ROTORS.items():
        if rname == 'rotor_2':
            continue                      # kuyruk rotoru kanattan uzak
        Rr = rot['R']
        mx, my, mz = rot['m']
        off = rot['r'][2] - mz
        worst = (1e9, 0, None)
        for dd in range(0, int(rot.get('tilt_max_deg', 90)) + 1):
            d = math.radians(dd)
            c = np.array([mx + off * math.sin(d), my, mz + off * math.cos(d)])
            n = np.array([math.sin(d), 0.0, math.cos(d)])
            rel = V - c
            perp = rel @ n
            rad = np.linalg.norm(rel - np.outer(perp, n), axis=1)
            # disk sifir kalinlikta degil: gorsel mesh 10.6 mm (dogrula.py)
            pa = np.maximum(np.abs(perp) - DISC_HALF_T, 0.0)
            ra = np.maximum(rad - Rr, 0.0)
            dist = np.sqrt(pa**2 + ra**2)
            k = int(np.argmin(dist))
            if dist[k] < worst[0]:
                worst = (float(dist[k]), dd, V[k].copy())
        mm, dd, pt = worst
        mm *= 1000.0
        flag = ' ⛔ TEMAS' if mm < 1 else (' ⚠ DAR' if mm < 20 else '')
        if mm < 20:
            bad += 1
        print(f"   {rname} <-> kanat mesh  {mm:7.1f} mm  (tilt {dd:2d} deg, "
              f"nokta {np.round(pt, 3)}){flag}")
    return bad


def xml_comment_audit():
    """XML yorumlarinda kacak '--' arar.

    NEDEN VAR. SDF'e gerekce yazarken '--' kullanmak bu oturumda DORT kez
    dosyayi bozdu: XML yorumlari '--' icermez, ayrastirici hemen reddeder.
    Hata mesaji satir/sutun verir ama nedenini soylemez, o yuzden her seferinde
    yeniden teshis edildi. Kapiya bagli bir kontrol bunu bir daha yasatmaz.
    """
    src = open(SDF).read()
    bad = [c for c in re.findall(r'<!--.*?-->', src, re.S) if '--' in c[4:-3]]
    print("\n### XML YORUM DENETIMI ###")
    if not bad:
        print("   kacak '--' yok")
        return 0
    for c in bad:
        print(f"   ⛔ yorumda kacak '--': {c[:70].strip()}...")
    return len(bad)


_WING_PTS = None


def _touches_wing_mesh(box, tol=0.001):
    """Kutu, gercek kanat mesh'inin herhangi bir noktasini iceriyor mu?"""
    global _WING_PTS

    if _WING_PTS is None:
        V = _wing_mesh(n_sample=200000)
        _WING_PTS = V if V is not None else np.zeros((0, 3))

    if len(_WING_PTS) == 0:
        return False

    P = _WING_PTS
    inside = ((P[:, 0] >= box[0][0] - tol) & (P[:, 0] <= box[0][1] + tol)
              & (P[:, 1] >= box[1][0] - tol) & (P[:, 1] <= box[1][1] + tol)
              & (P[:, 2] >= box[2][0] - tol) & (P[:, 2] <= box[2][1] + tol))
    return bool(inside.any())


def connectivity_audit():
    """Her yapisal parca KOKE (govde/kanat) kadar baglantili mi?

    NEDEN VAR (2026-08-31, Adim 147). Bu betigin butun denetimleri CAKISMA
    ariyordu; hicbiri BOSLUK aramiyordu. Sonuc: dikey stabilizator 20 mm
    havada asili duruyordu (alt kenari z=+0.020, cubugun ust yuzeyi z=0.000)
    ve hicbir kapi gormedi -- kullanici GUI'de gozle yakaladi.

    NEDEN "EN AZ BIR PARCAYA DEGIYOR MU" YETMEZ (olculdu): ilk surum oyleydi
    ve ayni hatayi TEKRAR yakalayamadi, cunku fin kendi RUDDER'ina degiyor.
    Fin+rudder birlikte havada asili bir CIFT olusturuyor ve ikisi de "bagli"
    sayiliyordu. Dogru soru koke ulasilabilirlik: parcalar bir GRAF olusturur
    ve govde/kanat'tan genislik-oncelikli gezinme yapilir.
    """
    print("\n### BAGLANTI DENETIMI (koke ulasilabilirlik) ###")
    TOL = 0.001                       # 1 mm: temas sayilir
    names = list(STRUCT.keys())
    ROOT = 'govde/kanat'

    adj = {n: set() for n in names}

    for ia, n in enumerate(names):
        a = STRUCT[n]

        for m in names[ia + 1:]:
            b = STRUCT[m]
            ov = [min(a[i][1], b[i][1]) - max(a[i][0], b[i][0]) for i in range(3)]

            if all(o >= -TOL for o in ov):
                adj[n].add(m)
                adj[m].add(n)

        # KUTU YETMEZ: base_link'in collision kutusu gercek kanadi temsil
        # etmiyor (mesh_audit notu), o yuzden kanada oturan parcalar kutuya
        # gore kopuk gorunur. Gercek MESH'e karsi da bak.
        if n != ROOT and _touches_wing_mesh(a):
            adj[n].add(ROOT)
            adj[ROOT].add(n)

    seen = {ROOT}
    queue = [ROOT]

    while queue:
        cur = queue.pop()

        for nxt in adj[cur]:
            if nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)

    orphan = [n for n in names if n not in seen]

    for n in orphan:
        near = ', '.join(sorted(adj[n])) or 'hicbir sey'
        print(f"   ⛔ {n:<22} GOVDEYE BAGLI DEGIL (yalnizca sunlara degiyor: {near})")

    if not orphan:
        print(f"   {len(names)} parcanin hepsi govdeye kadar bagli")

    return len(orphan)


def main():
    print(f"\n### {SDF} GEOMETRISI ###")

    total_bad = 0
    for rname, rot in ROTORS.items():
        tmax = rot.get('tilt_max_deg', 90)
        res = sweep(rot, tilt_max_deg=tmax)
        bad = {k: v for k, v in res.items() if v[0] < 20.0}
        print(f"\n{rname}   (R = {rot['R']:.3f} m, tilt 0-{tmax} deg)")
        for k, (mm, dd) in sorted(res.items(), key=lambda x: x[1][0]):
            if mm >= 200:      # uzak olanlari kisalt
                continue
            flag = ' ⛔ TEMAS' if mm < 1 else (' ⚠ DAR' if mm < 20 else '')
            print(f"   {k:<18} {mm:7.1f} mm  (tilt {dd:2d} deg){flag}")
        total_bad += len(bad)

    collision_audit()

    total_bad += mesh_audit()
    total_bad += xml_comment_audit()
    total_bad += connectivity_audit()

    print("\nROTOR <-> ROTOR (disk merkezleri arasi alt sinir):")
    for k, v in rotor_rotor().items():
        print(f"   {k:<24} {v:8.1f} mm" + ('  ⛔' if v < 20 else ''))

    print(f"\n{'='*58}")
    print(f"20 mm'nin ALTINDA kalan rotor-yapi cifti: {total_bad}")
    print("(esik 20 mm: yapisal esneklik ve montaj toleransi icin pay)")


if __name__ == '__main__':
    main()
