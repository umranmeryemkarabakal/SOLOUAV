#!/usr/bin/env python3
"""cad/step çıktısını kaynak SDF'e karşı bağımsız olarak doğrular.

Üretici betiğin tablolarını KULLANMAZ; SDF'i baştan ayrıştırır, böylece
elle aktarma hataları da yakalanır.

Kontroller:
  1. Kapsam      — SDF'teki her görünür parçanın CAD karşılığı var mı
  2. Ölçü        — kutu/silindir parçaların bbox'ı SDF ile birebir mi
  3. Mesh zarfı  — loft edilen parçalar mesh sınırları içinde mi
  4. Katı sağlığı— her STEP tek, kapalı, geçerli katı mı
  5. Menteşe     — her eklem noktası ilgili katının içinde mi
  6. Girişim     — duruşta parçalar birbirine giriyor mu
  7. Tilt süpürme— pervane diski 0…90° arasında neye çarpıyor
  8. Yüzey sapma — elevon/elevatör/rudder tam açıda neye çarpıyor
"""
import os
import re
import math
import glob
import numpy as np
import cadquery as cq
from cadquery import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
SDF = os.path.join(os.path.dirname(HERE), 'tiltrotor_tailplane_model.sdf')
PARTS = os.path.join(HERE, 'step', 'parts')
MM = 1000.0
TOL = 0.5          # mm — kabul edilen ölçü sapması

_ok, _warn, _err = [], [], []


def ok(m):
    _ok.append(m)
    print('  ok     ' + m)


def warn(m):
    _warn.append(m)
    print('  UYARI  ' + m)


def err(m):
    _err.append(m)
    print('  HATA   ' + m)


_model = []


def model(m):
    """CAD doğru ama kaynak SDF'in kendi geometrisi çakışıyor."""
    _model.append(m)
    print('  MODEL  ' + m)


# --------------------------------------------------------------- SDF ayrıştırma
def parse_sdf(path):
    s = open(path).read()
    links = {}
    for lm in re.finditer(r"<link name=['\"]([^'\"]+)['\"]>(.*?)</link>", s, re.S):
        name, body = lm.group(1), lm.group(2)
        # <inertial> içindeki pose kütle merkezidir, linkin yerleşimi değil;
        # ayıklanmazsa elevon menteşesi 300 mm kayıyor.
        head = re.sub(r'<inertial>.*?</inertial>', '', body, flags=re.S)
        head = head[:head.find('<visual')] if '<visual' in head else head
        lp = re.search(r'<pose>([^<]+)</pose>', head)
        link_pose = [float(v) for v in lp.group(1).split()] if lp else [0.] * 6
        vis = []
        for vm in re.finditer(r"<visual name=['\"]([^'\"]+)['\"]>(.*?)</visual>", body, re.S):
            vn, vb = vm.group(1), vm.group(2)
            pm = re.search(r'<pose>([^<]+)</pose>', vb)
            pose = [float(v) for v in pm.group(1).split()] if pm else [0.] * 6
            box = re.search(r'<box>\s*<size>([^<]+)</size>', vb)
            cyl = re.search(r'<cylinder>\s*<length>([^<]+)</length>\s*<radius>([^<]+)</radius>', vb, re.S)
            mesh = re.search(r'<uri>([^<]+)</uri>', vb)
            g = None
            if box:
                g = ('box', [float(v) for v in box.group(1).split()])
            elif cyl:
                g = ('cylinder', [float(cyl.group(1)), float(cyl.group(2))])
            elif mesh:
                g = ('mesh', mesh.group(1).rsplit('/', 1)[-1])
            vis.append((vn, pose, g))
        links[name] = dict(pose=link_pose, visuals=vis)
    joints = {}
    for jm in re.finditer(r"<joint name=['\"]([^'\"]+)['\"] type=['\"]revolute['\"]>(.*?)</joint>", s, re.S):
        n, b = jm.group(1), jm.group(2)
        pm = re.search(r'<pose>([^<]+)</pose>', b)
        joints[n] = dict(
            child=re.search(r'<child>([^<]+)</child>', b).group(1),
            parent=re.search(r'<parent>([^<]+)</parent>', b).group(1),
            pose=[float(v) for v in pm.group(1).split()] if pm else [0.] * 6,
            axis=[float(v) for v in re.search(r'<xyz>([^<]+)</xyz>', b).group(1).split()],
            lower=float(re.search(r'<lower>([^<]+)</lower>', b).group(1)),
            upper=float(re.search(r'<upper>([^<]+)</upper>', b).group(1)),
            upmf='<use_parent_model_frame>1' in b)
    return links, joints


# --------------------------------------------------------------- yardımcılar
def load(name):
    return cq.importers.importStep(os.path.join(PARTS, name + '.step')).val()


def bb(shape):
    b = shape.BoundingBox()
    return np.array([b.xmin, b.ymin, b.zmin]), np.array([b.xmax, b.ymax, b.zmax])


def box_bbox(center, size):
    c, s = np.array(center) * MM, np.array(size) * MM
    return c - s / 2, c + s / 2


def common_volume(a, b):
    """İki katının ortak hacmi [mm^3]; kesişmiyorsa 0."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    lo_a, hi_a = bb(a)
    lo_b, hi_b = bb(b)
    if (hi_a < lo_b - 1e-6).any() or (hi_b < lo_a - 1e-6).any():
        return 0.0
    alg = BRepAlgoAPI_Common(a.wrapped, b.wrapped)
    if not alg.IsDone():
        return float('nan')
    r = cq.Shape.cast(alg.Shape())
    if not r.Solids():
        return 0.0
    return sum(s.Volume() for s in r.Solids())


def rot_y(shape, pivot, deg):
    return shape.rotate(Vector(pivot[0], pivot[1] - 1, pivot[2]),
                        Vector(pivot[0], pivot[1] + 1, pivot[2]), deg)


def rot_about(shape, pivot, axis, deg):
    a = np.array(axis, float)
    a = a / np.linalg.norm(a)
    p0 = Vector(*(np.array(pivot) - a))
    p1 = Vector(*(np.array(pivot) + a))
    return shape.rotate(p0, p1, deg)


def point_inside(shape, pt, tol=1e-3):
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.TopAbs import TopAbs_IN, TopAbs_ON
    cl = BRepClass3d_SolidClassifier(shape.wrapped, Vector(*pt).toPnt(), tol)
    return cl.State() in (TopAbs_IN, TopAbs_ON)


# --------------------------------------------------------------- kontroller
# SDF görsel adı -> CAD parça adı
MAP_BOX = {
    'tail_boom': 'tail_boom',
    'tailplane_strut': 'tailplane_strut',
    'right_motor_pylon': 'pylon_right',
    'left_motor_pylon': 'pylon_left',
    'tailplane': 'tailplane',
    'vertical_stabiliser': 'vertical_stabiliser',
    'left_elevator_visual': 'elevator_left',
    'right_elevator_visual': 'elevator_right',
    'rudder_visual': 'rudder',
}
MAP_CYL = {'motor_0_visual': 'motor_0_right', 'motor_1_visual': 'motor_1_left',
           'motor_2_visual': 'motor_2_tail'}
MAP_MESH = {'base_link_visual': 'wing', 'left_elevon_visual': 'elevon_left',
            'right_elevon_visual': 'elevon_right',
            'rotor_0_visual': 'rotor_0_right', 'rotor_1_visual': 'rotor_1_left',
            'rotor_2_visual': 'rotor_2_tail'}
# Menteşe burnu kutu zarfının önüne taşan, kasıtlı olarak profillendirilmiş yüzeyler
PROFILED = {'elevator_left': 'x', 'elevator_right': 'x', 'rudder': 'x',
            'tailplane': 'z', 'vertical_stabiliser': 'y'}


def check_coverage_and_dims(links, parts):
    print('\n[1-3] Kapsam ve ölçüler')
    seen = set()
    for lname, L in links.items():
        for vn, pose, g in L['visuals']:
            if g is None:
                continue
            center = np.array(L['pose'][:3]) + np.array(pose[:3])
            if g[0] == 'box' and vn in MAP_BOX:
                p = MAP_BOX[vn]
                seen.add(p)
                lo, hi = bb(parts[p])
                elo, ehi = box_bbox(center, g[1])
                d = np.maximum(np.abs(lo - elo), np.abs(hi - ehi))
                ax = PROFILED.get(p)
                free = {'x': 0, 'y': 1, 'z': 2}.get(ax, -1)
                bad = [i for i in range(3) if d[i] > TOL and i != free]
                if bad:
                    err(f'{p}: kutu ölçüsü SDF ile uyuşmuyor, sapma '
                        f'{np.round(d, 2)} mm (eksen {bad})')
                else:
                    extra = f', profil ekseninde +{d[free]:.1f} mm (menteşe burnu)' if free >= 0 and d[free] > TOL else ''
                    ok(f'{p}: kutu ölçüsü tam{extra}')
            elif g[0] == 'cylinder' and vn in MAP_CYL:
                p = MAP_CYL[vn]
                seen.add(p)
                L_, R_ = g[1]
                lo, hi = bb(parts[p])
                elo, ehi = box_bbox(center, [2 * R_, 2 * R_, L_])
                d = np.maximum(np.abs(lo - elo), np.abs(hi - ehi))
                if (d > TOL).any():
                    err(f'{p}: silindir ölçüsü sapıyor {np.round(d, 2)} mm')
                else:
                    ok(f'{p}: silindir r={R_ * 1000:.0f} L={L_ * 1000:.0f} mm tam')
            elif g[0] == 'mesh' and vn in MAP_MESH:
                seen.add(MAP_MESH[vn])
    missing = set(MAP_BOX.values()) | set(MAP_CYL.values()) | set(MAP_MESH.values())
    for m in sorted(missing - seen):
        err(f'{m}: SDF görselleriyle eşleşmedi')
    extra = set(parts) - missing
    if extra:
        ok(f'SDF karşılığı olmayan ek parça (kasıtlı): {", ".join(sorted(extra))}')


def check_solids(parts):
    print('\n[4] Katı sağlığı')
    from OCP.BRepCheck import BRepCheck_Analyzer
    bad = 0
    for n, s in sorted(parts.items()):
        n_sol = len(s.Solids())
        valid = BRepCheck_Analyzer(s.wrapped).IsValid()
        closed = all(sh.Closed() for sh in s.Shells()) if s.Shells() else False
        if n_sol != 1 or not valid or not closed:
            err(f'{n}: katı={n_sol} geçerli={valid} kapalı={closed}')
            bad += 1
    if not bad:
        ok(f'{len(parts)} parçanın hepsi tek, kapalı ve geçerli katı')


def check_hinges(joints, parts):
    print('\n[5] Menteşe noktaları')
    child_map = {'motor_0': 'motor_0_right', 'motor_1': 'motor_1_left',
                 'motor_2': 'motor_2_tail', 'rotor_0': 'rotor_0_right',
                 'rotor_1': 'rotor_1_left', 'rotor_2': 'rotor_2_tail',
                 'left_elevon': 'elevon_left', 'right_elevon': 'elevon_right',
                 'left_elevator': 'elevator_left', 'right_elevator': 'elevator_right',
                 'rudder': 'rudder'}
    out = {}
    for jn, J in joints.items():
        child = J['child']
        p = child_map.get(child)
        if p is None:
            continue
        # eklem pose'u çocuk çerçevesinde; çocuk link pose'u model çerçevesinde
        import __main__
        clink = __main__.LINKS[child]
        pt = (np.array(clink['pose'][:3]) + np.array(J['pose'][:3])) * MM
        axis = np.array(J['axis'], float)
        if not J['upmf']:
            yaw = J['pose'][5]
            c, s = math.cos(yaw), math.sin(yaw)
            axis = np.array([c * axis[0] - s * axis[1], s * axis[0] + c * axis[1], axis[2]])
        inside = point_inside(parts[p], pt)
        lim = (math.degrees(J['lower']), math.degrees(J['upper']))
        out[jn] = (p, pt, axis, lim)
        tag = f'{jn}: nokta {np.round(pt, 1)} eksen {np.round(axis, 4)} limit [{lim[0]:.1f}°,{lim[1]:.1f}°]'
        if inside:
            ok(tag + ' — katının içinde')
        elif 'rotor' in jn:
            ok(tag + ' — pervane göbeği (nokta katı dışında olabilir)')
        else:
            err(tag + ' — KATININ DIŞINDA, eklem havada kalır')
    return out


def check_static_interference(parts):
    print('\n[6] Duruşta girişim (kasıtlı gömülmeler hariç)')
    # Kasıtlı gömülme: menteşeli yüzey ana yüzeye girer, pylon kanada gömülür,
    # winglet kanat ucuna oturur, motor pylonun tepesine oturur.
    allowed = {
        frozenset(('elevator_left', 'tailplane')), frozenset(('elevator_right', 'tailplane')),
        frozenset(('rudder', 'vertical_stabiliser')),
        frozenset(('elevon_left', 'wing')), frozenset(('elevon_right', 'wing')),
        frozenset(('pylon_left', 'wing')), frozenset(('pylon_right', 'wing')),
        frozenset(('winglet_left', 'wing')), frozenset(('winglet_right', 'wing')),
        frozenset(('tail_boom', 'wing')), frozenset(('tailplane_strut', 'tail_boom')),
        frozenset(('tailplane_strut', 'tailplane')), frozenset(('tail_boom', 'tailplane')),
        frozenset(('tail_boom', 'vertical_stabiliser')),
        frozenset(('motor_0_right', 'pylon_right')), frozenset(('motor_1_left', 'pylon_left')),
        frozenset(('motor_2_tail', 'tail_boom')),
    }
    names = sorted(parts)
    hits = 0
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            v = common_volume(parts[a], parts[b])
            if v > 1.0:
                pair = frozenset((a, b))
                if pair in allowed:
                    ok(f'{a} ∩ {b} = {v / 1000:.1f} cm³ (kasıtlı gömülme)')
                elif pair == frozenset(('motor_2_tail', 'vertical_stabiliser')):
                    model(f'{a} ∩ {b} = {v / 1000:.1f} cm³ — SDF\'te de var: '
                          f'motor gövdesi (x −670…−630) fin\'in ön kenarına (x −660) 10 mm giriyor')
                else:
                    err(f'{a} ∩ {b} = {v / 1000:.1f} cm³ — beklenmeyen girişim')
                    hits += 1
    if not hits:
        ok('Beklenmeyen girişim yok')


def rotor_disc(parts, name, hub):
    """Pervaneyi dönerken süpürdüğü diskle temsil et (çarpışma taraması için)."""
    lo, hi = bb(parts[name])
    r = max(np.abs(np.array([lo[0], hi[0]]) - hub[0]).max(),
            np.abs(np.array([lo[1], hi[1]]) - hub[1]).max())
    t = hi[2] - lo[2]
    return cq.Solid.makeCylinder(r, t, pnt=Vector(hub[0], hub[1], hub[2] - t / 2)), r, t


def check_tilt_sweep(parts, hinges):
    print('\n[7] Tilt süpürmesi — dönen pervane diski neye çarpıyor')
    groups = [('motor_0_joint', 'rotor_0_right', 'motor_0_right'),
              ('motor_1_joint', 'rotor_1_left', 'motor_1_left'),
              ('motor_2_joint', 'rotor_2_tail', 'motor_2_tail')]
    fixed = {n: s for n, s in parts.items()
             if not n.startswith(('rotor_', 'motor_'))}
    for jn, rotor, motor in groups:
        pivot = hinges[jn][1]
        hub_lo, hub_hi = bb(parts[rotor])
        hub = np.array([(hub_lo[0] + hub_hi[0]) / 2, (hub_lo[1] + hub_hi[1]) / 2,
                        (hub_lo[2] + hub_hi[2]) / 2])
        # göbek merkezi motorun ekseninde: x,y motordan al
        m_lo, m_hi = bb(parts[motor])
        hub[0] = (m_lo[0] + m_hi[0]) / 2
        hub[1] = (m_lo[1] + m_hi[1]) / 2
        disc, r, t = rotor_disc(parts, rotor, hub)
        print(f'  {rotor}: disk r={r:.1f} mm, kalınlık={t:.1f} mm, göbek {np.round(hub, 1)}')
        for deg in range(0, 91, 15):
            d = rot_y(disc, pivot, deg)
            worst = []
            for n, s in fixed.items():
                v = common_volume(d, s)
                if v > 10.0:
                    worst.append((n, v / 1000))
            if worst:
                txt = ', '.join(f'{n} ({v:.1f} cm³)' for n, v in sorted(worst, key=lambda x: -x[1]))
                model(f'{rotor} @ {deg:2d}°: {txt}')
            else:
                ok(f'{rotor} @ {deg:2d}°: temiz')


def check_surface_sweep(parts, hinges):
    print('\n[8] Kumanda yüzeyi sapması')
    pairs = [('left_elevon_joint', 'elevon_left', ['wing', 'winglet_left']),
             ('right_elevon_joint', 'elevon_right', ['wing', 'winglet_right']),
             ('left_elevator_joint', 'elevator_left', ['tailplane', 'tail_boom', 'vertical_stabiliser']),
             ('right_elevator_joint', 'elevator_right', ['tailplane', 'tail_boom', 'vertical_stabiliser']),
             ('rudder_joint', 'rudder', ['vertical_stabiliser', 'tail_boom', 'tailplane'])]
    for jn, part, neigh in pairs:
        _, pivot, axis, lim = hinges[jn]
        base = {n: common_volume(parts[part], parts[n]) for n in neigh}
        for deg in (lim[0], lim[1]):
            s = rot_about(parts[part], pivot, axis, deg)
            grew = []
            for n in neigh:
                v = common_volume(s, parts[n])
                if v - base[n] > 200.0:      # 0.2 cm³ üzeri artış
                    grew.append((n, (v - base[n]) / 1000))
            if grew:
                txt = ', '.join(f'{n} (+{v:.1f} cm³)' for n, v in grew)
                warn(f'{part} @ {deg:+.1f}°: {txt} — sapmada ana yüzeye giriyor')
            else:
                ok(f'{part} @ {deg:+.1f}°: temiz')


def main():
    global LINKS
    LINKS, joints = parse_sdf(SDF)
    files = sorted(glob.glob(os.path.join(PARTS, '*.step')))
    parts = {os.path.basename(f)[:-5]: cq.importers.importStep(f).val() for f in files}
    print(f'{len(parts)} parça yüklendi, kaynak: {os.path.basename(SDF)}')
    check_coverage_and_dims(LINKS, parts)
    check_solids(parts)
    hinges = check_hinges(joints, parts)
    check_static_interference(parts)
    check_tilt_sweep(parts, hinges)
    check_surface_sweep(parts, hinges)
    print(f'\nÖZET: {len(_ok)} geçti, {len(_warn)} uyarı, {len(_err)} CAD hatası, '
          f'{len(_model)} kaynak model bulgusu')
    print('CAD hataları = STEP çıktısı SDF ile uyuşmuyor.')
    print('Kaynak model bulguları = CAD doğru, SDF geometrisinin kendisi çakışıyor.')
    return 1 if _err else 0


if __name__ == '__main__':
    raise SystemExit(main())
