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


def loft_from_stations(st, axis, n_hint=None):
    """(konum, 2B kesit) listesini 3B loft katısına çevir."""
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
    mir = [(z, np.column_stack([r[:, 0], -r[:, 1]])[::-1]) for z, r in stw]

    # Not: pürüzsüz loft winglet ucunda mesh zarfını ~3.5 mm aşıyor
    # (zmax 211.2 mm, mesh 207.7 mm). Boolean kırpma bu gövdede kararsız
    # çalıştığı için taşma bırakıldı; sonuç açıklığın %0.16'sı mertebesinde.
    return wing, loft_from_stations(stw, axis=2), loft_from_stations(mir, axis=2)


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
    st = stations(V, T, axis=0, vals=np.linspace(x0 + 5e-4, x1 - 5e-4, 61),
                  n=40, fallback=True)
    if mesh_name.endswith('ccw'):
        st = [(x, np.column_stack([r[:, 0], -r[:, 1]])[::-1]) for x, r in st]
    return loft_from_stations(st, axis=0).translate(tuple(c * MM for c in center))


# --------------------------------------------------------------------------
def main():
    os.makedirs(os.path.join(OUT, 'parts'), exist_ok=True)
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
        asm.add(cq.Workplane(obj=shp), name=name)

    asm.save(os.path.join(OUT, 'tiltrotor_assembly.step'), 'STEP')
    print('\nyazıldı:', OUT)


if __name__ == '__main__':
    main()
