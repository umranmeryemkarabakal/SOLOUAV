#!/usr/bin/env python3
"""Mekanizma parçalarını üretir: tilt grubu, iniş takımı bağlantısı, kumanda
yüzeyi menteşe/tahrik donanımı, kuyruk çubuğu desteği.

Kaynak  : model.sdf (menteşe eksenleri) + cad/step/parts/*.step (ana gövdeler)
Çıktı   : cad/step/parts/*.step  (yeni parçalar + menteşe boşluğu açılmış ana gövdeler)

NEDEN AYRI BİR BETİK: build_tiltrotor_cad.py aerodinamik gövdeleri `.dae`
mesh'lerinden loft eder ve mesh dizini olmadan koşamaz. Buradaki parçaların
hiçbiri mesh'e ihtiyaç duymaz — hepsi SDF'ten türeyen eksenler etrafında kutu
ve silindirdir, menteşe boşlukları da mevcut STEP gövdelerinden kesilir.
Böylece mesh'siz bir makinede de mekanizma yeniden üretilebilir.

MENTEŞE SAYILARI BURAYA YAZILMAZ. fusion_02_joints.py'nin ayrıştırıcısı
ödünç alınır; bu deponun aynı sayıyı birden çok yerde tutmaktan gördüğü
zarar için bkz. cad/README.md. Buradaki sabitler yalnızca TASARIM
seçimleridir (yatak ölçüsü, plaka kalınlığı, servo gövdesi) — SDF'te
karşılığı olan hiçbir değer tekrarlanmaz.

Koordinatlar SDF gövde çerçevesi, mm: +x burun, +y sol kanat, +z yukarı.
"""
import os
import re
import sys
import math

import cadquery as cq
from cadquery import Solid, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'step', 'parts')
SDF_CANDIDATES = [
    os.environ.get('TILTROTOR_SDF', ''),
    os.path.join(os.path.dirname(HERE), 'tiltrotor_tailplane_model.sdf'),
    os.path.join(os.path.expanduser('~'), 'SOLOUAV', 'tiltrotor_tailplane_model.sdf'),
]

# --------------------------------------------------------------------------
# TASARIM SABİTLERİ — SDF'te karşılığı yok, mühendislik seçimi
# --------------------------------------------------------------------------
BRG_OD, BRG_ID, BRG_W = 10.0, 5.0, 4.0    # MR105ZZ
YOKE_ARM_T = 5.0                          # çatal kolu kalınlığı
YOKE_ARM_IN = 23.0                        # kol iç yüzünün eksenden y uzaklığı
YOKE_PLATE_T = 6.0
BOSS_R = 10.0                             # yatak bossu dış yarıçapı
MOTOR_R, MOTOR_H = 20.0, 35.0             # SDF motor silindiri (süpürme payı için)
CRADLE_IR, CRADLE_OR, CRADLE_H = 20.0, 23.0, 12.0
CRANK_R = 14.0
SERVO_L, SERVO_W, SERVO_H = 23.0, 12.0, 22.0
PUSHROD_R = 1.2
HINGE_PIN_R = 2.0
CLEAR = 1.0                               # genel kaçıklık payı

# Pylon/direk kutuları — build_tiltrotor_cad.py'deki BOXES ile aynı kaynak
# (oradan okunur; okunamazsa bu yedek kullanılır ve uyarı basılır).
PYLON_FALLBACK = {
    'pylon_right': ((217.5, -350.0, 40.0), (105.0, 30.0, 60.0)),
    'pylon_left':  ((217.5,  350.0, 40.0), (105.0, 30.0, 60.0)),
}
TAIL_POST_Z = (0.0, 30.0)                 # kuyruk direği: boom üstünden yoke altına

CHILD_PARENT = {          # kumanda yüzeyi -> bağlı olduğu ana gövde
    'elevon_left': 'wing', 'elevon_right': 'wing',
    'elevator_left': 'tailplane', 'elevator_right': 'tailplane',
    'rudder': 'vertical_stabiliser',
}
TILT_PARENT = {           # tilt grubu -> üzerine oturduğu gövde
    'motor_0_right': 'pylon_right', 'motor_1_left': 'pylon_left',
    'motor_2_tail': 'tail_pylon',
}


# --------------------------------------------------------------------------
def load_joints():
    """fusion_02_joints.py'nin ayrıştırıcısını ödünç al — ikinci bir kopya yazma."""
    sdf = next((p for p in SDF_CANDIDATES if p and os.path.isfile(p)), None)
    if sdf is None:
        raise SystemExit('model.sdf bulunamadi. TILTROTOR_SDF ile yol verin.\n  ' +
                         '\n  '.join(p for p in SDF_CANDIDATES if p))
    kod = open(os.path.join(HERE, 'fusion_02_joints.py'), encoding='utf-8').read()
    kod = re.sub(r'^import adsk\..*$', '', kod, flags=re.M)
    ns = {'__name__': 'fusion_02_joints_lib'}
    exec(compile(kod, 'fusion_02_joints.py', 'exec'), ns)
    return sdf, ns['parse_sdf_joints'](sdf)


def pylon_boxes():
    try:
        sys.path.insert(0, HERE)
        import build_tiltrotor_cad as b
        out = {}
        for name, c, s in b.BOXES:
            if name.startswith('pylon'):
                out[name] = (tuple(v * 1000.0 for v in c), tuple(v * 1000.0 for v in s))
        if out:
            return out
    except Exception as e:                                   # noqa: BLE001
        print('  uyari: build_tiltrotor_cad okunamadi ({}), yedek olculer'.format(e))
    return PYLON_FALLBACK


# --- geometri yardimcilari ------------------------------------------------
def bx(x0, x1, y0, y1, z0, z1):
    return Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, Vector(x0, y0, z0))


def cy(p1, p2, r):
    d = Vector(*p2) - Vector(*p1)
    return Solid.makeCylinder(r, d.Length, Vector(*p1), d.normalized())


def uni(*shapes):
    out = shapes[0]
    for s in shapes[1:]:
        out = out.fuse(s)
    return out.clean()


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def perp_frame(D, aft_hint=(-1.0, 0.0, 0.0)):
    """Eksene dik iki birim vektör: `aft` (gövde arkası) ve n = aft x D."""
    d = sum(aft_hint[i] * D[i] for i in range(3))
    aft = unit(tuple(aft_hint[i] - d * D[i] for i in range(3)))
    n = (aft[1] * D[2] - aft[2] * D[1],
         aft[2] * D[0] - aft[0] * D[2],
         aft[0] * D[1] - aft[1] * D[0])
    return aft, n


def along(A, D, t):
    return (A[0] + D[0] * t, A[1] + D[1] * t, A[2] + D[2] * t)


def offset(p, v, s):
    return (p[0] + v[0] * s, p[1] + v[1] * s, p[2] + v[2] * s)


def rot_about(shape, A, D, deg):
    return shape.rotate(Vector(*A), Vector(*along(A, D, 1.0)), deg)


def axis_radius(shape, A, D):
    """Gövdenin menteşe eksenine dik azami uzaklığı (köşelerden)."""
    rmax = 0.0
    for v in shape.Vertices():
        w = (v.X - A[0], v.Y - A[1], v.Z - A[2])
        t = sum(w[i] * D[i] for i in range(3))
        perp = tuple(w[i] - t * D[i] for i in range(3))
        rmax = max(rmax, math.sqrt(sum(c * c for c in perp)))
    return rmax


def axial_span(shape, A, D):
    ts = []
    for v in shape.Vertices():
        w = (v.X - A[0], v.Y - A[1], v.Z - A[2])
        ts.append(sum(w[i] * D[i] for i in range(3)))
    return min(ts), max(ts)


def lower_surface_z(shape, x, y, half=14.0):
    """Gövdenin (x,y) civarındaki en alt z'si — iniş takımı bağlantısı için."""
    bb = shape.BoundingBox()
    probe = bx(x - half, x + half, y - half, y + half, bb.zmin - 5, bb.zmax + 5)
    cut = shape.intersect(probe)
    return cut.BoundingBox().zmin


def load_part(name):
    p = os.path.join(OUT, name + '.step')
    if not os.path.isfile(p):
        raise SystemExit('parca yok: ' + p)
    return cq.importers.importStep(p).val()


# --------------------------------------------------------------------------
# 1) TILT GRUBU
# --------------------------------------------------------------------------
def build_tilt(tag, A, post_bot, post_top):
    """A = menteşe noktası (mm). Eksen y boyunca; direk `post_bot`..`post_top`."""
    XA, YC, ZA = A
    S = -1.0 if YC <= 0 else 1.0
    def Y(off):
        return YC + S * off
    parts = {}

    # motor pucunun eksen etrafında süpürdüğü yarıçap
    sweep_r = math.sqrt(MOTOR_R ** 2 + (MOTOR_H / 2.0) ** 2) + 1.4

    ai, ao = YOKE_ARM_IN, YOKE_ARM_IN + YOKE_ARM_T
    yoke = bx(XA - 38, XA, min(Y(-ao), Y(ao)), max(Y(-ao), Y(ao)),
              ZA - 15, ZA - 15 + YOKE_PLATE_T)
    for a, b in ((ai, ao), (-ao, -ai)):
        yoke = uni(yoke,
                   bx(XA - 18, XA, min(Y(a), Y(b)), max(Y(a), Y(b)), ZA - 9, ZA),
                   cy((XA, Y(a), ZA), (XA, Y(b), ZA), BOSS_R))
    yoke = yoke.cut(cy((XA, Y(-ai), ZA), (XA, Y(ai), ZA), sweep_r))       # puc boşluğu
    yoke = yoke.cut(cy((XA, Y(ai - 1), ZA), (XA, Y(ao + 1), ZA), BRG_OD / 2))
    yoke = yoke.cut(cy((XA, Y(-ao - 1), ZA), (XA, Y(-ai + 1), ZA), BRG_OD / 2))
    parts['tilt_yoke_' + tag] = yoke

    for lbl, (a, b) in (('out', (ai, ai + BRG_W)), ('in', (-ai - BRG_W, -ai))):
        brg = cy((XA, Y(a), ZA), (XA, Y(b), ZA), BRG_OD / 2)
        brg = brg.cut(cy((XA, Y(a - 1), ZA), (XA, Y(b + 1), ZA), BRG_ID / 2))
        parts['tilt_bearing_{}_{}'.format(tag, lbl)] = brg

    cradle = cy((XA, YC, ZA - CRADLE_H / 2), (XA, YC, ZA + CRADLE_H / 2), CRADLE_OR)
    cradle = cradle.cut(cy((XA, YC, ZA - CRADLE_H), (XA, YC, ZA + CRADLE_H), CRADLE_IR))
    for a, b in ((ai - 1, ao + 1), (-ao - 1, -ai + 1)):
        cradle = uni(cradle, cy((XA, Y(a), ZA), (XA, Y(b), ZA), BRG_ID / 2))
    parts['tilt_cradle_' + tag] = cradle

    ck = bx(XA - 3, XA + 3, min(Y(ao + 1), Y(ao + 5)), max(Y(ao + 1), Y(ao + 5)),
            ZA - CRANK_R, ZA)
    ck = uni(ck, cy((XA, Y(ao + 1), ZA), (XA, Y(ao + 5), ZA), BRG_OD / 2))
    parts['tilt_crank_' + tag] = ck

    # servo, direğin içinde dikeyde ortalanır (pylon 60 mm, kuyruk direği 30 mm)
    sz0 = post_bot + (post_top - post_bot - SERVO_H) / 2.0
    sz1 = sz0 + SERVO_H
    sv = bx(XA - 36, XA - 13, min(Y(3), Y(15)), max(Y(3), Y(15)), sz0, sz1)
    shx = XA - 24
    sv = uni(sv, cy((shx, Y(15), (sz0 + sz1) / 2), (shx, Y(19), (sz0 + sz1) / 2), 3.0))
    parts['tilt_servo_' + tag] = sv
    hz = (sz0 + sz1) / 2
    parts['tilt_horn_' + tag] = bx(shx - 2, shx + 2, min(Y(19), Y(23)), max(Y(19), Y(23)),
                                   hz, hz + CRANK_R)
    parts['tilt_pushrod_' + tag] = cy((shx, Y(21), hz + CRANK_R),
                                      (XA, Y(ao + 3), ZA - CRANK_R), PUSHROD_R)
    return parts


# --------------------------------------------------------------------------
# 2) İNİŞ TAKIMI BAĞLANTISI
# --------------------------------------------------------------------------
def build_gear(wing, boom):
    parts = {}
    for tag, YC in (('right', -250.0), ('left', 250.0)):
        leg = load_part('leg_front_' + tag)
        top = leg.BoundingBox().zmax
        zs = lower_surface_z(wing, 220.0, YC)      # kanadın o istasyondaki alt yüzeyi
        s = uni(bx(208, 232, YC - 7, YC + 7, top, zs + 5.0),
                bx(190, 250, YC - 20, YC + 20, zs - 6.0, zs))
        parts['leg_front_{}_strut'.format(tag)] = s

    leg = load_part('leg_tail')
    top = leg.BoundingBox().zmax
    bb = boom.BoundingBox()
    # Dikmenin TAMAMI tailplane hücum kenarının önünde kalmalı. İlk sürümde
    # x1 = tp_le + 5 idi ve dikme (x1-20 .. x1) geriye taşıp tailplane'i kesiyordu;
    # dikme boyu kadar pay bırakılıyor.
    tp_le = load_part('tailplane').BoundingBox().xmax
    x1 = tp_le + 25.0
    t = uni(bx(x1 - 20, x1, -10, 10, top, bb.zmin),
            bx(x1 - 5, x1 + 55, -25, 25, bb.zmin - 6, bb.zmin),
            bx(x1 - 5, x1 + 55, -25, -20, bb.zmin, bb.zmax),
            bx(x1 - 5, x1 + 55, 20, 25, bb.zmin, bb.zmax))
    parts['leg_tail_strut'] = t
    return parts


# --------------------------------------------------------------------------
# 3) KUMANDA YÜZEYİ: menteşe boşluğu + tahrik
# --------------------------------------------------------------------------
def hinge_relief(surf, parent, A, D, lim_deg):
    """Yüzeyin TÜM sapma aralığında ana gövdeye giren malzemesini kapsayan
    silindirik boşluk. Dönme yarıçapı koruduğu için silindir yeterlidir."""
    rmax = 0.0
    for deg in (-lim_deg, -lim_deg / 2, 0.0, lim_deg / 2, lim_deg):
        moved = rot_about(surf, A, D, deg)
        try:
            hit = parent.intersect(moved)
        except Exception:                                     # noqa: BLE001
            continue
        if hit.Volume() > 1e-3:
            rmax = max(rmax, axis_radius(hit, A, D))
    if rmax <= 0.0:
        return None, 0.0
    R = rmax + CLEAR
    t0, t1 = axial_span(surf, A, D)
    return cy(along(A, D, t0 - 10), along(A, D, t1 + 10), R), R


def build_actuation(tag, A, D, s25, s50, s75):
    """Menteşe pimleri + korna. Servo yerleşimi yüzeye göre değişir:
    elevon kanadın içine sığar, elevatör/rudder sığmaz (tailplane 9,9 mm,
    fin 16,5 mm; 22 mm'lik servo girmez) — onlar boom'a gömülür."""
    aft, n = perp_frame(D)
    parts = {}
    pins = None
    for s in (s25, s75):
        c = along(A, D, s)
        p = cy(along(c, D, -12), along(c, D, 12), HINGE_PIN_R)
        pins = p if pins is None else uni(pins, p)
    parts['hinge_pins_' + tag] = pins

    if tag.startswith('elevon'):
        # korna yüzeyin ortasında, servo kanadın içinde menteşenin önünde
        hc = offset(along(A, D, s50), aft, 6.0)
        parts['horn_' + tag] = bx_oriented(offset(hc, n, 9.0), aft, D, 8.0, 3.0, 18.0)
        sc = offset(along(A, D, s50), aft, -45.0)
        sv = bx_oriented(sc, aft, D, SERVO_L, SERVO_W, SERVO_H)
        sv = uni(sv, cy(offset(sc, D, 6), offset(sc, D, 10), 3.0),
                 bx_oriented(offset(offset(sc, D, 8.0), n, 7.0), aft, D, 4.0, 3.0, CRANK_R))
        parts['servo_' + tag] = sv
        parts['pushrod_' + tag] = cy(offset(offset(sc, D, 8.0), n, CRANK_R),
                                     offset(hc, n, 18.0), PUSHROD_R)
    return parts, aft, n


def bx_oriented(c, ld, wd, L, W, H):
    """Merkezi c olan, ld/wd eksenlerine oturmuş kutu (H = ld x wd yönünde).

    cq.Plane kullanılıyor: normal=hd, xDir=ld verildiğinde yDir = hd x ld = wd
    olur (ld ile wd birim ve dik olduğu sürece), yani kutunun L/W/H kenarları
    tam olarak ld/wd/hd yönlerine oturur.
    """
    hd = (ld[1] * wd[2] - ld[2] * wd[1],
          ld[2] * wd[0] - ld[0] * wd[2],
          ld[0] * wd[1] - ld[1] * wd[0])
    pl = cq.Plane(origin=Vector(*c), xDir=Vector(*ld), normal=Vector(*hd))
    return cq.Workplane(pl).box(L, W, H).val()


def build_tail_actuation(boom, J):
    """Elevatör ve rudder servoları kuyruk çubuğuna gömülür; rudder'ın mil
    ekseni z olduğu için 22 mm servo 20 mm boom'a sığmaz, altına karina eklenir."""
    bb = boom.BoundingBox()
    hw = bb.ymax                       # boom yarı genişliği
    parts = {}
    # Servolar tailplane_strut'ın (x -725..-675) ARKASINDA kalmalı; ilk sürüm
    # dx=+20/+50 ile strut'a giriyordu (dogrula_mekanizma [5] yakaladı).
    for tag, S, dx in (('right', -1.0, -35.0), ('left', 1.0, -7.0)):
        A = J['elevator_' + tag]['point']
        XS = A[0] + dx
        ys0, ys1 = (-18.0, 4.0) if S < 0 else (-4.0, 18.0)
        sv = uni(bx(XS, XS + SERVO_L, ys0, ys1, A[2] + 23, A[2] + 35),
                 cy((XS + 11.5, ys0 if S < 0 else ys1, A[2] + 29),
                    (XS + 11.5, S * (hw + 10), A[2] + 29), 3.0),
                 bx(XS + 8, XS + 15, min(S * (hw + 13), S * (hw + 9)),
                    max(S * (hw + 13), S * (hw + 9)), A[2] + 29, A[2] + 43))
        parts['servo_elevator_' + tag] = sv
        parts['horn_elevator_' + tag] = uni(
            bx(A[0] - 4, A[0] + 4, min(S * (hw + 17), S * (hw + 13)),
               max(S * (hw + 17), S * (hw + 13)), A[2] - 18, A[2]),
            cy((A[0], S * (hw + 17), A[2]), (A[0], S * (hw + 13), A[2]), 3.0))
        parts['pushrod_elevator_' + tag] = cy(
            (XS + 11.5, S * (hw + 11), A[2] + 43), (A[0], S * (hw + 15), A[2] - 16), PUSHROD_R)

    A = J['rudder']['point']
    parts['servo_fairing_rudder'] = bx(A[0] - 17, A[0] + 14, -14, 14, bb.zmin - 6, bb.zmin + 2)
    rs = uni(bx(A[0] - 13, A[0] + 10, -11, 11, bb.zmin - 4, bb.zmin + 18),
             cy((A[0] - 1.5, 0, bb.zmin + 18), (A[0] - 1.5, 0, bb.zmin + 34), 3.0),
             bx(A[0] - 5, A[0] + 2, 3, 17, bb.zmin + 30, bb.zmin + 37))
    parts['servo_rudder'] = rs
    parts['horn_rudder'] = uni(bx(A[0] - 4, A[0] + 4, 3, 21, A[2] - 84, A[2] - 76),
                               cy((A[0], 0, A[2] - 80), (A[0], 6, A[2] - 80), 3.0))
    parts['pushrod_rudder'] = cy((A[0] - 1.5, 15, bb.zmin + 33.5),
                                 (A[0], 19, A[2] - 80), PUSHROD_R)
    return parts


# --------------------------------------------------------------------------
def main():
    sdf, J = load_joints()
    print('SDF :', sdf)
    print('eklem:', len(J))
    os.makedirs(OUT, exist_ok=True)

    wing = load_part('wing')
    boom = load_part('tail_boom')
    parts = {}

    # --- kuyruk direği (SDF'te yok; tilt grubunu taşımak için gerekli) ---
    tj = J['motor_2_tail']
    XA, YC, ZA = tj['point']
    parts['tail_pylon'] = bx(XA - 38, XA, -15, 15, TAIL_POST_Z[0], TAIL_POST_Z[1])

    # Direklere motor pucunun süpürme boşluğu. Bu OLMADAN motor tilt boyunca
    # kendi pilonunun içinden geçer (dogrula_mekanizma [2] bunu yakalar).
    sweep_r = math.sqrt(MOTOR_R ** 2 + (MOTOR_H / 2.0) ** 2) + 1.4
    for mpart, pname in TILT_PARENT.items():
        A = J[mpart]['point']
        cutter = cy((A[0], A[1] - YOKE_ARM_IN, A[2]),
                    (A[0], A[1] + YOKE_ARM_IN, A[2]), sweep_r)
        base = parts[pname] if pname in parts else load_part(pname)
        parts[pname] = base.cut(cutter).clean()

    # --- tilt grupları ---
    PB = pylon_boxes()
    for part, tag in (('motor_0_right', 'right'), ('motor_1_left', 'left'),
                      ('motor_2_tail', 'tail')):
        A = J[part]['point']
        if part == 'motor_2_tail':
            post_bot, post_top = TAIL_POST_Z
        else:
            c, s = PB[TILT_PARENT[part]]
            post_bot, post_top = c[2] - s[2] / 2.0, c[2] + s[2] / 2.0
        parts.update(build_tilt(tag, A, post_bot, post_top))
        print('  tilt {:5s} eksen ({:7.1f},{:7.1f},{:6.1f})  limit {:+.1f}..{:+.1f} deg'.format(
            tag, A[0], A[1], A[2],
            math.degrees(J[part]['lower']), math.degrees(J[part]['upper'])))

    # --- iniş takımı ---
    parts.update(build_gear(wing, boom))

    # --- kuyruk çubuğu desteği ---
    for tag, S in (('right', -1.0), ('left', 1.0)):
        a = (-178.0, S * 110.0, -8.0)
        b = (-430.0, S * 16.0, -10.0)
        parts['boom_brace_' + tag] = uni(
            cy(a, b, 5.0),
            cy((-170.0, S * 110.0, -8.0), (-186.0, S * 110.0, -8.0), 8.0),
            cy((-430.0, S * 20.0, -10.0), (-430.0, S * 8.0, -10.0), 8.0))

    # --- kumanda yüzeyleri: menteşe boşluğu ana gövdelere işlenir ---
    parents = {'wing': wing, 'tailplane': load_part('tailplane'),
               'vertical_stabiliser': load_part('vertical_stabiliser')}
    for surf, pname in CHILD_PARENT.items():
        A = J[surf]['point']
        D = unit(J[surf]['axis'])
        lim = math.degrees(J[surf]['upper'])
        sb = load_part(surf)
        cutter, R = hinge_relief(sb, parents[pname], A, D, lim)
        if cutter is None:
            print('  {:16s} bosluk gerekmedi'.format(surf))
            continue
        parents[pname] = parents[pname].cut(cutter).clean()
        print('  {:16s} -> {:20s} mentese boslugu R={:5.2f} mm'.format(surf, pname, R))
        t0, t1 = axial_span(sb, A, D)
        p, _, _ = build_actuation(surf, A, D, t0 + (t1 - t0) * 0.25,
                                  t0 + (t1 - t0) * 0.5, t0 + (t1 - t0) * 0.75)
        parts.update(p)

    # elevatör + rudder servoları kuyruk çubuğunda
    parts.update(build_tail_actuation(boom, J))
    parts.update(parents)

    # --- yaz ---
    print()
    for name, shp in sorted(parts.items()):
        bb = shp.BoundingBox()
        print('  {:26s} hacim={:9.2f} cm^3  x[{:8.1f},{:8.1f}] z[{:7.1f},{:7.1f}]'.format(
            name, shp.Volume() / 1e3, bb.xmin, bb.xmax, bb.zmin, bb.zmax))
        cq.exporters.export(cq.Workplane(obj=shp), os.path.join(OUT, name + '.step'))
    print('\nyazildi:', OUT, '({} parca)'.format(len(parts)))


if __name__ == '__main__':
    main()
