#!/usr/bin/env python3
"""tiltrotor_indi Gazebo modelinden parametrik CAD (STEP) üretir.

Kaynak       : Tools/simulation/gz/models/tiltrotor_indi/model.sdf
Mesh kaynağı : models/standard_vtol/meshes/*.dae
Çıktı        : cad/step/tiltrotor_assembly.step + cad/step/parts/*.step  (mm)

Koordinatlar SDF gövde çerçevesi: +x burun, +y sol kanat, +z yukarı.
Modelin <pose> z=0.246 yer ofseti UYGULANMAZ; başlangıç base_link'tir.
"""
import os
import sys
import numpy as np
import cadquery as cq
from cadquery import Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sdf_mesh import load_dae_points, rpy, stations, slice_axis, dedupe_loops, biggest_loop, resample

MM = 1000.0  # m -> mm
MESH_DIR = os.environ.get(
    'VTOL_MESH_DIR',
    os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models/standard_vtol/meshes'))
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'step')

# Asama 2'nin uzerine yazdigi govdelerin listesi ORADA tanimlidir; burada
# ikinci bir kopyasini tutmuyoruz (bu deponun ayni sayiyi iki yerde tutmaktan
# gordugu zarar icin bkz. cad/README.md). Asama 1'in isi, o govdelerin
# dokunulmamis halini Asama 2'nin girdi dizinine de yazmak.
from build_mechanism import TURETILEN, SRC as PRISTINE_DIR

# --------------------------------------------------------------------------
# SDF'ten okunan ölçüler (model.sdf, 2026-08-02 sürümü)
# --------------------------------------------------------------------------
WING_VIS_POSE = ([0.53, -1.072, -0.1], [1.5707963268, 0.0, 3.1415926536])
WING_Y_SHIFT = -0.00351   # mesh'i açıklık ortasına getirir (bbox -1.0719/+1.0789)

ELEVON = {
    'left':  ([-0.105,  0.004, -0.034], [1.5707963268, 0.0, 3.1415926536]),
    'right': ([ 0.281, -1.032, -0.034], [1.5707963268, 0.0, 3.1415926536]),
}

BOXES = [  # (ad, merkez, boyut)
    # INCELTILDI 0.04 -> 0.02, merkez 0.010 -> 0.000 (Adim 132): kuyruk rotor
    # diski 90 derecede cubugun ICINDEN geciyordu. model.sdf ile senkron.
    # UZATILDI 0.62 -> 0.76 m (Adim 139): fin -0.86'ya tasininca cubugun
    # ucundan tasiyordu; gercek aracta desteklenmeyen konsol demekti.
    ('tail_boom',           (-0.545, 0.00, -0.010), (0.71, 0.04, 0.02)),
    ('tailplane_strut',     (-0.70,  0.00, -0.0225), (0.05, 0.03, 0.045)),
    # KISALTILDI 0.05 -> 0.03, merkez 0.22 -> 0.205 (Adim 132): TAM TILT'te
    # (90 deg, seyir duruşu) disk kendi pilonunun icinden geciyordu.
    ('pylon_right',         ( 0.2175, -0.35, 0.040), (0.105, 0.03, 0.06)),
    ('pylon_left',          ( 0.2175,  0.35, 0.040), (0.105, 0.03, 0.06)),
    # INIS TAKIMI (Adim 133): uc ayak, uclari govde alt yuzunun 4 cm altinda.
    # SDF'te silindir; burada kutu olarak temsil ediliyor (CAD tarafinda
    # imalat icin yeterli, montaj tablosu MONTAJ.md'de).
    ('leg_front_right',     ( 0.22, -0.25, -0.175), (0.024, 0.024, 0.05)),
    ('leg_front_left',      ( 0.22,  0.25, -0.175), (0.024, 0.024, 0.05)),
    ('leg_tail',            (-0.62,  0.00, -0.175), (0.024, 0.024, 0.05)),
]

# Kanat profili verilen kutu zarfına oturtulan kuyruk yüzeyleri.
FOILS = [  # (ad, merkez, kiriş(x), açıklık, kalınlık, düşey mi)
    ('tailplane',           (-0.70, 0.00, -0.040), 0.16, 0.60, 0.012, False),
    # GERIYE ALINDI -0.74 -> -0.86 (Adim 132): disk tilt=0'da, yani hover'in
    # TAMAMINDA fininin icinden geciyordu.
    ('vertical_stabiliser', (-0.78, 0.00,  0.100), 0.16, 0.20, 0.020, True),
]

FLAPS = [  # (ad, merkez, kiriş, açıklık, kalınlık, düşey mi, menteşe x/z)
    ('elevator_left',  ( -0.79,  0.15, -0.040), 0.05, 0.28, 0.010, False, -0.765),
    ('elevator_right', ( -0.79, -0.15, -0.040), 0.05, 0.28, 0.010, False, -0.765),
    # Fin ile BIRLIKTE geriye (Adim 132); mentese x'i de ayni kadar kaydi.
    ('rudder',         ( -0.87,  0.00,  0.100), 0.05, 0.18, 0.010, True,  -0.845),
]

MOTORS = [  # (ad, merkez, r, L)
    ('motor_0_right', ( 0.27, -0.35, 0.085), 0.02, 0.035),
    ('motor_1_left',  ( 0.27,  0.35, 0.085), 0.02, 0.035),
    # Adim 132'de 0.135'e yukseltilmis, Adim 133'te GERI ALINMISTI: geri
    # gecişte uc rotor da doydu (BIG_M 0 -> 3843). Yerine kuyruk tilt araligi
    # 20 dereceyle sinirlandi; boylece r_z hic degismiyor.
    ('motor_2_tail',  (-0.55,  0.00, 0.045), 0.02, 0.035),
]

ROTORS = [  # (ad, merkez, mesh)
    ('rotor_0_right', ( 0.27, -0.35, 0.110), 'iris_prop_ccw'),
    ('rotor_1_left',  ( 0.27,  0.35, 0.110), 'iris_prop_cw'),
    ('rotor_2_tail',  (-0.55,  0.00, 0.070), 'iris_prop_ccw'),
]


# --------------------------------------------------------------------------
def mesh_in_body(name, pose, scale=1e-3):
    """DAE'yi oku, SDF visual pose'unu uygula, gövde çerçevesinde döndür."""
    V, T = load_dae_points(os.path.join(MESH_DIR, name + '.dae'))
    t, r = pose
    W = (rpy(*r) @ (V * scale).T).T + np.array(t)
    return W, T


def loft_from_stations(st, axis, n_hint=None, spline=False):
    """(konum, 2B kesit) listesini 3B loft katısına çevir.

    `spline=True`: kesit teli poligon yerine kapalı B-spline olur. Poligon
    kesit, 40 düz parçadan oluştuğu için kenarda köşe köşe bir tırtıklanma
    ve yüzeyde boyuna faset bırakıyor. Kanat/elevon loft'ları hassas olduğu
    için varsayılan poligon; şimdilik yalnızca pervane spline kullanıyor.
    """
    wires = []
    for v, r in st:
        pts = []
        for a, b in r:
            if axis == 1:
                p = (a, v, b)
            elif axis == 2:
                p = (a, b, v)
            else:
                p = (v, a, b)
            pts.append(Vector(p[0] * MM, p[1] * MM, p[2] * MM))
        if spline:
            # periodic=True iken ilk nokta TEKRARLANMAZ; kapanışı OCC yapar
            # (tekrarlanırsa BSplCLib::Interpolate ile çöküyor).
            kenar = cq.Edge.makeSpline(pts, periodic=True)
            wires.append(cq.Wire.assembleEdges([kenar]))
        else:
            wires.append(cq.Wire.makePolygon(pts, close=True))
    return cq.Solid.makeLoft(wires, ruled=False)


def build_wing():
    """Kanat gövdesi ve iki winglet, x8_wing.dae kesitlerinden loft.

    Üç ayrı katı döndürür. Kanat + winglet'i boolean ile tek gövdeye
    kaynatmak OCC'de kararsız (sonuç ya şişiyor ya sıfıra çöküyor);
    winglet'ler zaten ayrı üretim parçası olarak modellenebilir.
    Aynalama OCC mirror() ile değil kesit verisi üzerinde yapılır:
    mirror() ters yönlü katı üretip fuse'u bozuyor.
    """
    V, T = mesh_in_body('x8_wing', WING_VIS_POSE)
    V[:, 1] += WING_Y_SHIFT

    # Kök bölgesinde kesit hızlı değişiyor (gövde podu): orada sık örnekle,
    # yoksa pürüzsüz loft zarfın dışına taşıyor. Tek parça (uçtan uca) loft
    # ise kökte şişiyor; iki yarı ayrı loft edilip y=0'da birleştiriliyor.
    ys = (list(np.arange(0.0, 0.16, 0.0125))
          + list(np.arange(0.16, 0.93, 0.035))
          + [0.938, 0.950, 0.962, 0.976, 0.990, 1.002, 1.015])
    st = stations(V, T, axis=1, vals=ys, n=64)
    wing = loft_from_stations(st, axis=1).fuse(
        loft_from_stations([(-y, r) for y, r in reversed(st)], axis=1)).clean()

    # Winglet: uç bölgeyi ayıkla, düşey (z) kesitlerle loft. Taban kesiti
    # z=-0.005'e indirilerek kanat ucunun içine oturtulur.
    tip = T[(V[T][:, :, 1] > 0.93).all(1)]
    zs = list(np.arange(0.018, 0.19, 0.012)) + [0.194, 0.200, 0.2055, 0.2072]
    stw = stations(V, tip, axis=2, vals=zs, n=48)
    stw = [(-0.005, stw[0][1])] + stw
    stw = close_winglet_tip(stw)
    mir = [(z, np.column_stack([r[:, 0], -r[:, 1]])[::-1]) for z, r in stw]
    return wing, loft_from_stations(stw, axis=2), loft_from_stations(mir, axis=2)


def close_winglet_tip(stw, z_env=0.2077, z_kok=0.2055, n_cap=3, kalan=0.10):
    """Winglet ucunu mesh zarfında kapat.

    SORUN: winglet kesitleri sağlamdır (dolgu 0,67-0,77), ama veter son
    7 mm'de 111 -> 68 -> 20 mm'ye çöküyor ve `makeLoft(ruled=False)` pürüzsüz
    yüzeyi son kesitin ÖTESİNE sürüklüyor: gövde 207,2'de bitmesi gerekirken
    211,2 mm'ye uzanıyor. O uzantı kendi kendini kesen ince bir dilim —
    görünürde kanat ucundan fırlayan bir artık.

    Ölçüm (üst 8 mm'de dilim genişlikleri):
      taşmalı : 0,0 111,8 0,0 0,0 0,0 110,4 109,3 0,0   <- kopuk, dejenere
      kapaklı : 111,7 110,4 107,8 101,0 79,2 69,7 63,7 7,0

    Boolean ile kırpmak denendi, `Bnd_Box is void` ile çöktü (README'nin
    "boolean kırpma bu gövdede kararsız" notu). Onun yerine son istasyon
    atılıp z_kok..z_env arasına çeyrek elips kapak konuyor, böylece loft'un
    ekstrapole edecek yeri kalmıyor ve gövde tam zarfta bitiyor.
    """
    st = [s for s in stw if s[0] <= z_kok]
    if len(st) < 3:
        return stw
    kok = np.asarray(st[-1][1], float)
    m0 = kok.mean(axis=0)
    for i in range(1, n_cap + 1):
        u = i / float(n_cap)
        s = max(float(np.sqrt(max(1.0 - u * u, 0.0))), kalan)
        st.append((z_kok + (z_env - z_kok) * u, m0 + (kok - m0) * s))
    return st


def build_elevon(side):
    V, T = mesh_in_body('x8_elevon_' + side, ELEVON[side])
    y0, y1 = V[:, 1].min(), V[:, 1].max()
    vals = np.linspace(y0 + 1e-4, y1 - 1e-4, 14)
    st = stations(V, T, axis=1, vals=vals, n=40)
    return loft_from_stations(st, axis=1)


def foil_section(chord, thick, n=60):
    """NACA 4 haneli simetrik profilin yarı kalınlık bağıntısı."""
    t = thick / chord
    x = (1 - np.cos(np.linspace(0, np.pi, n))) / 2
    yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x**2
                  + 0.2843 * x**3 - 0.1015 * x**4)
    up = np.column_stack([x, yt])
    lo = np.column_stack([x[::-1], -yt[::-1]])
    p = np.vstack([up, lo[1:-1]]) * chord
    return p


def build_foil(center, chord, span, thick, vertical):
    """Kutu zarfına oturan simetrik profilli yüzey. Burun +x yönünde."""
    sec = foil_section(chord, thick)
    cx, cy, cz = center
    # profil x: burun ileride -> x = cx + chord/2 - s
    pts2 = [(cx + chord / 2 - s, h) for s, h in sec]
    if vertical:
        wp = cq.Workplane('XY').polyline([(x * MM, h * MM) for x, h in pts2]).close()
        sol = wp.extrude(span * MM).translate((0, 0, (cz - span / 2) * MM)).val()
        sol = sol.translate((0, cy * MM, 0))
    else:
        wp = cq.Workplane('XZ').polyline([(x * MM, h * MM) for x, h in pts2]).close()
        sol = wp.extrude(-span * MM).translate((0, (cy - span / 2) * MM, 0)).val()
        sol = sol.translate((0, 0, cz * MM))
    return sol


def build_flap(center, chord, span, thick, vertical, hinge):
    """Yuvarlak hücum kenarlı, sivri firar kenarlı flap kesiti.
    Menteşe ekseni kesitin yuvarlak burnunun merkezinden geçer."""
    r = thick / 2
    xh = hinge                       # menteşe x
    xte = center[0] - chord / 2      # firar kenarı x
    ang = np.linspace(-np.pi / 2, np.pi / 2, 24)
    nose = [(xh + r * np.cos(a), r * np.sin(a)) for a in ang]
    pts2 = nose + [(xte, 0.0)]
    if vertical:
        wp = cq.Workplane('XY').polyline([(x * MM, h * MM) for x, h in pts2]).close()
        sol = wp.extrude(span * MM).translate((0, center[1] * MM, (center[2] - span / 2) * MM)).val()
    else:
        wp = cq.Workplane('XZ').polyline([(x * MM, h * MM) for x, h in pts2]).close()
        sol = wp.extrude(-span * MM).translate((0, (center[1] - span / 2) * MM, center[2] * MM)).val()
    return sol


def build_box(center, size):
    return cq.Solid.makeBox(*[s * MM for s in size],
                            pnt=Vector(*[(c - s / 2) * MM for c, s in zip(center, size)]))


def build_cylinder(center, r, L):
    return cq.Solid.makeCylinder(r * MM, L * MM,
                                 pnt=Vector(center[0] * MM, center[1] * MM, (center[2] - L / 2) * MM))


def _loop_area(r):
    x, y = r[:, 0], r[:, 1]
    return abs(0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def smooth_stations(st, w=2):
    """Kesitleri AÇIKLIK boyunca yumuşat (dalgalanma giderme).

    `stations()` her kesiti aynı nokta sayısına yeniden örnekler ve hepsini
    firar kenarından başlatır, yani i. kesitin k. noktası komşularınınkiyle
    karşılık gelir. Bu yüzden istasyonlar arası hareketli ortalama, palanın
    gerçek biçimini (burulma, veter değişimi) bozmadan yalnızca mesh
    fasetlerinden gelen istasyon-istasyon zıplamasını siler.

    Neden gerekiyor: loft her kesitin TAM içinden geçmek zorundadır; 61
    gürültülü kesit doğrudan 61 dalgalı kaburga demektir. Ölçüm: uca doğru
    veter 17,9 -> 10,4 -> 13,0 mm diye inip çıkıyordu.
    """
    R = [np.asarray(r, float) for _, r in st]
    xs = [x for x, _ in st]
    n = len(R)
    out = []
    for i in range(n):
        lo, hi = max(0, i - w), min(n, i + w + 1)
        grup = [R[j] for j in range(lo, hi) if R[j].shape == R[i].shape]
        out.append((xs[i], np.mean(np.stack(grup), axis=0) if grup else R[i]))
    return out


PROP_MIN_T = 1.2      # mm — dış paladaki asgari kesit kalınlığı (imalat kısıtı)
PROP_DOLGU = 0.35     # bunun altında dolgu oranı olan kesit "katlanmış" sayılır
PROP_TIP_CAP = 18.0   # mm — pala ucu yuvarlatma boyu (4/8/12 denendi, sıçramalı)
PROP_N = 120          # kesit başına nokta (40 kenarda tırtıklanma bırakıyordu)


def _kesit_olcu(r):
    """Kesitin veter/kalınlık/burulma/merkezini ana eksen analiziyle çıkar."""
    c = r - r.mean(axis=0)
    _, _, vt = np.linalg.svd(c, full_matrices=False)
    veter = np.ptp(c @ vt[0])
    kal = np.ptp(c @ vt[1])
    A = abs(0.5 * np.sum(r[:, 0] * np.roll(r[:, 1], -1)
                         - np.roll(r[:, 0], -1) * r[:, 1]))
    dolgu = A / (veter * kal) if veter * kal > 0 else 0.0
    return veter, kal, vt[0], r.mean(axis=0), dolgu


def rebuild_thin_sections(st, n=40):
    """Katlanmış dış pala kesitlerini elips profille yeniden inşa et.

    NEDEN: `iris_prop_cw.dae` bir GÖRSEL mesh'tir, katı model değil. Dış
    palada kabuk 0,2 mm'ye iner; o kadar inceyi dilimleyince kontur kendi
    üstüne katlanır ve loft'un ucu "oval" değil bir dilim olur. Ölçüm: uç
    yüzeyinin alanı kendi sınırlayıcı kutusunun %3-6'sı (profilde ~%65).

    Ölçülen VETER, BURULMA ve KESİT MERKEZİ her istasyonda düzgün değişir
    (veter 20,08 -> 10,25 -> 4,44; burulma -172,8 -> -167,8), yani onlar
    güvenilir ve KORUNUR. Yalnızca konturun kendisi ve kalınlık yeniden
    üretilir: kalınlık, sağlam istasyonlardan gelen eğilime ve imalat için
    PROP_MIN_T alt sınırına oturtulur.

    Elips seçildi çünkü dolgu oranı pi/4 = 0,785 ile hedefe yakın ve kendini
    kesmesi mümkün değil — bu bölgede loft'un geçerli katı vermesi kritik.
    """
    R = [np.asarray(r, float) for _, r in st]
    xs = [x for x, _ in st]
    olc = [_kesit_olcu(r) for r in R]
    # İKİ ölçüt gerekiyor: katlanmış (düşük dolgu) VE jilet inceliğinde
    # (kalınlık < asgari) kesitler yeniden kurulur. Yalnız dolguya bakmak
    # yetmiyordu: en uçtaki istasyonun kalınlığı 0,01 mm ama dolgusu 0,468,
    # yani "sağlam" görünüp geçiyordu ve katının uç yüzeyi dilim kalıyordu.
    def bozuk(o):
        return o[4] < PROP_DOLGU or o[1] < PROP_MIN_T / 1000.0

    saglam = [i for i, o in enumerate(olc) if not bozuk(o)]
    if not saglam:
        return st

    def _ara(i, deger):
        """Sağlam komşular arasında doğrusal ara/dış değer."""
        sol = [j for j in saglam if j <= i]
        sag = [j for j in saglam if j >= i]
        if sol and sag:
            a, b = sol[-1], sag[0]
            if a == b:
                return deger(a)
            return deger(a) + (deger(b) - deger(a)) * (i - a) / float(b - a)
        return deger((sol or sag)[-1 if sol else 0])

    def _alan(j):
        r = R[j]
        return abs(0.5 * np.sum(r[:, 0] * np.roll(r[:, 1], -1)
                                - np.roll(r[:, 0], -1) * r[:, 1]))

    def kalinlik(i, veter):
        """Kalınlığı ALAN sürekliliğinden türet, kalınlık eğiliminden değil.

        Kalınlığı komşulardan taşımak, elips ile ölçülen kamburlu profilin
        dolgu oranları farklı olduğu için geçişte alan çentiği bırakıyordu
        (ölçüm: 38,1 29,9 24,7 30,5 -- komşuları ~30 iken 24,7'ye düşüyor).
        Görünen büyüklük alan olduğu için hedef alanı komşulardan taşıyıp
        elipsin kalınlığını ondan çözüyoruz: A = pi/4 * veter * t.
        """
        hedef = _ara(i, _alan)
        t = 4.0 * hedef / (np.pi * veter) if veter > 0 else PROP_MIN_T / 1000.0
        return max(float(t), PROP_MIN_T / 1000.0)

    # ⛔ GEÇİŞ HARMANI — DENENDİ, GERİ ALINDI (1 Eylül 2026)
    # Sınıra yakın sağlam istasyonları da kendi elipslerine doğru kademeli
    # harmanlamak mantıklı görünüyordu (kamburlu profil -> simetrik elips
    # geçişini yumuşatmak için). Ölçüm tersini söyledi; x=290..340 arasında
    # kesit alanı:
    #   harman yok : 45,5 43,5 38,1 29,9 24,7 30,5   (hafif çentik)
    #   harman = 2 : 45,4 43,7 36,3 19,7 20,4 31,2
    #   harman = 4 : 45,3 32,5 24,7 21,0 22,8 30,7   (belirgin çukur)
    # Kamburlu bir profille elipsi nokta nokta karıştırmak, ikisinin yay
    # uzunluğu dağılımı farklı olduğu için kesiti sıkıştırıyor. BLEND=0.
    BLEND = 0

    def agirlik(i):
        if bozuk(olc[i]):
            return 1.0
        d = min((abs(i - j) for j in range(len(olc)) if bozuk(olc[j])),
                default=None)
        if d is None or d > BLEND:
            return 0.0
        return 1.0 - d / float(BLEND + 1)

    out = []
    for i, (x, r) in enumerate(st):
        veter, kal, ana, mrk, dolgu = olc[i]
        w = agirlik(i)
        if w <= 0.0:
            out.append((x, r))
            continue
        t = kalinlik(i, veter)
        a, b = veter / 2.0, t / 2.0
        # Elipsi ÖNCE yoğun üret, SONRA resample() ile yay uzunluğuna göre
        # 40 noktaya indir. Bu şart: ölçülen kesitler yay uzunluğuna göre
        # örnekleniyor, elips ise açıya göre üretilince noktalar uçlarda
        # yığılıyor (ölçüm: nokta aralığı max/ort 1,55 -- ölçülenlerde 1,01).
        # İki farklı parametrelendirme yan yana gelince loft noktaları
        # yanlış eşleştirip geçiş bölgesinde yüzeyi buruyor; kesitlerin
        # tek tek düzgün olması yetmiyor.
        th = np.linspace(0.0, 2.0 * np.pi, 400, endpoint=False)
        el = np.column_stack([a * np.cos(th), b * np.sin(th)])
        dik = np.array([-ana[1], ana[0]])
        yogun = mrk + el[:, :1] * ana + el[:, 1:2] * dik
        elips = np.asarray(resample(yogun, n=n, start_by='maxx'), float)
        if w >= 1.0 or elips.shape != r.shape:
            out.append((x, elips))
        else:
            out.append((x, (1.0 - w) * np.asarray(r, float) + w * elips))
    return out


def round_blade_tips(st, n_cap=4, kalan=0.12):
    """Pala uçlarını yuvarlayarak kapat.

    Loft son istasyonda bitince OCC ucu DÜZ bir yüzle kapatıyor; pala küt
    kesilmiş görünüyor. Burada her iki uca, çeyrek elips profiliyle küçülen
    birkaç istasyon eklenir: uzaklık u için ölçek = sqrt(1-u^2), yani uç
    yuvarlak bir kapakla biter.

    `kalan`: en son kesit sıfıra indirilmez (dejenere tel loft'u geçersiz
    kılıyor); bu oranda küçük bir yüz bırakılır — uç yarıçapının %12'si,
    gözle görünmez ama katı geçerli kalır.
    """
    if len(st) < 5:
        return st
    xs = [x for x, _ in st]
    R = [np.asarray(r, float) for _, r in st]

    def kapak(uc_i, yon):
        """uc_i: uç istasyonun indeksi. Kapak DIŞARI EKLENMEZ, mevcut ucun
        içine oturtulur — yoksa pervane çapı büyür (ölçüm: 256,7 -> 264,2 mm).

        Kapak boyu uç kesitinin veterinden türetilmiyor: uçta veter 4,4 mm
        ama hemen 18 mm içeride 20 mm, dolayısıyla kısa kapak (2-12 mm)
        alanı bir anda sıçratıyor (ölçüm: 0,0 0,0 8,4 0,0 12,8). 18 mm'lik
        kapak kademeli geçiş veriyor (0,9 3,9 6,7 9,1 11,3 13,1).
        """
        x_uc = xs[uc_i]
        R_cap = PROP_TIP_CAP / 1000.0
        x_bas = x_uc - yon * R_cap
        # kapak bölgesindeki mevcut istasyonlar atılır
        if yon > 0:
            tut = [i for i in range(len(xs)) if xs[i] < x_bas]
        else:
            tut = [i for i in range(len(xs)) if xs[i] > x_bas]
        if not tut:
            return None
        kok = R[tut[-1] if yon > 0 else tut[0]]
        _, _, _, mrk0, _ = _kesit_olcu(kok)
        yeni = []
        for i in range(1, n_cap + 1):
            u = i / float(n_cap)
            s = max(float(np.sqrt(max(1.0 - u * u, 0.0))), kalan)
            yeni.append((x_bas + yon * R_cap * u, mrk0 + (kok - mrk0) * s))
        return tut, yeni

    a = kapak(0, -1.0)
    b = kapak(len(st) - 1, +1.0)
    if a is None or b is None:
        return st
    orta = [i for i in a[0] if i in b[0]]
    return a[1][::-1] + [(xs[i], R[i]) for i in orta] + b[1]


# ⛔ PALA UCU İÇİN ÖNCE DENENEN VE ELENEN YOLLAR (1 Eylül 2026)
#
# Uç dejenerasyonunu ONARMAYA çalışan üç yol da başarısız oldu; hepsi ya
# geçersiz katı verdi ya da eşiğe göre rastgele davrandı:
#
#   1. Dejenere istasyonları atıp yerlerine son sağlam kesitin küçültülmüş
#      kopyasını koymak      -> katı GEÇERSİZ
#   2. Dejenere istasyonları kırpmak (eşik 0,15/0,30/0,45/0,60)
#                             -> yalnızca 0,15 geçerli; eşiğe göre gidip
#                                gelmesi OCC'nin sınırda çalıştığını gösterir
#   3. fallback=False + istasyon aralığını içeri çekmek (0,5..5 mm)
#                             -> çoğu geçersiz; 5 mm'de hacim -723174 cm3
#   4. Katıyı düzlemle kesip sağlam bir kesitte bitirmek
#                             -> geçerli ama uç dolgu oranı 0,06'da kaldı,
#                                çünkü kestiğim yerdeki kesit de bozuktu
#
# Çözüm onarım değil YENİDEN İNŞA oldu (`rebuild_thin_sections`): kesitin
# ölçülen veter/burulma/merkezi korunur, yalnız kontur ve kalınlık yeniden
# kurulur. Sonuç: uç yüzeyi 0,50 -> 8,65 mm2, dolgu oranı 0,07 -> 0,65.


def build_prop(mesh_name, center):
    """Pervaneyi kendi ekseni boyunca (palalar x'te uzanır) alınan gerçek
    mesh kesitlerinden loft eder; pala burulması ve kesit değişimi korunur.

    CCW pervane, CW pervanenin kesitleri y'de aynalanarak üretilir:
    `iris_prop_ccw.dae` kesitleri pala boyunca kendini kesiyor ve loft'u
    geçersiz katıya çeviriyor (`iris_prop_cw.dae` aynı ayarlarla temiz).
    Bir CCW pervane zaten CW'nin dönme eksenini içeren bir düzleme göre
    aynasıdır; aradaki tek fark pala faz açısıdır, o da dönen bir pervanede
    anlamsızdır.

    Üçgenleri tek tek dikmek (sew) de denendi: sonuç katı değil kabuk kalıyor
    ve parça başına ~9 MB STEP üretiyor.
    """
    V, T = load_dae_points(os.path.join(MESH_DIR, 'iris_prop_cw.dae'))  # ölçek 1 (m)
    x0, x1 = V[:, 0].min(), V[:, 0].max()
    # PROP_N=120: kesit teli poligon olduğu için 40 nokta kenarda gözle
    # görülür tırtıklanma bırakıyordu. Spline tel denendi, OCC'de çöktü
    # (BSplCLib::Interpolate, sonra BRep_API: command not done -- kesitlerde
    # çok yakın ardışık noktalar var). Nokta sayısını artırmak aynı sonucu
    # güvenle veriyor: yüz sayısı 42 -> 122, hacim farkı %1, açıklık aynı.
    st = stations(V, T, axis=0, vals=np.linspace(x0 + 5e-4, x1 - 5e-4, 61),
                  n=PROP_N, fallback=True)
    # SIRA ÖNEMLİ: önce yumuşat, SONRA yeniden inşa et. Tersi durumda
    # yumuşatma, yeni kurulan elipsi hâlâ katlanmış komşularıyla ortalayıp
    # katlanmayı geri getiriyor (ölçüm: uç dolgu oranı 0,32'de takılıyor).
    st = smooth_stations(st)
    st = rebuild_thin_sections(st, n=PROP_N)
    st = round_blade_tips(st)
    if mesh_name.endswith('ccw'):
        st = [(x, np.column_stack([r[:, 0], -r[:, 1]])[::-1]) for x, r in st]
    return loft_from_stations(st, axis=0).translate(tuple(c * MM for c in center))


# --------------------------------------------------------------------------
def main():
    os.makedirs(os.path.join(OUT, 'parts'), exist_ok=True)
    os.makedirs(PRISTINE_DIR, exist_ok=True)
    parts = {}

    print('kanat + winglet ...', flush=True)
    parts['wing'], parts['winglet_left'], parts['winglet_right'] = build_wing()

    print('elevonlar ...', flush=True)
    for s in ('left', 'right'):
        parts['elevon_' + s] = build_elevon(s)

    print('kuyruk ve gövde ...', flush=True)
    for name, c, s in BOXES:
        parts[name] = build_box(c, s)
    for name, c, ch, sp, th, vert in FOILS:
        parts[name] = build_foil(c, ch, sp, th, vert)
    for name, c, ch, sp, th, vert, hinge in FLAPS:
        parts[name] = build_flap(c, ch, sp, th, vert, hinge)

    print('motorlar ve pervaneler ...', flush=True)
    for name, c, r, L in MOTORS:
        parts[name] = build_cylinder(c, r, L)
    for name, c, mesh in ROTORS:
        parts[name] = build_prop(mesh, c)

    asm = cq.Assembly(name='tiltrotor_indi')
    for name, shp in parts.items():
        try:
            vol = shp.Volume() / 1e3
        except Exception:
            vol = float('nan')
        bb = shp.BoundingBox()
        print(f'  {name:22s} hacim={vol:9.2f} cm^3  '
              f'x[{bb.xmin:8.1f},{bb.xmax:8.1f}] y[{bb.ymin:8.1f},{bb.ymax:8.1f}] z[{bb.zmin:7.1f},{bb.zmax:7.1f}]')
        cq.exporters.export(cq.Workplane(obj=shp), os.path.join(OUT, 'parts', name + '.step'))
        if name in TURETILEN:
            # Asama 2 bu govdenin uzerine yazacak; girdisi olarak kullanacagi
            # dokunulmamis kopya burada ayrilir.
            cq.exporters.export(cq.Workplane(obj=shp),
                                os.path.join(PRISTINE_DIR, name + '.step'))
        asm.add(cq.Workplane(obj=shp), name=name)

    asm.save(os.path.join(OUT, 'tiltrotor_assembly.step'), 'STEP')
    print('\nyazıldı:', OUT)


if __name__ == '__main__':
    main()
