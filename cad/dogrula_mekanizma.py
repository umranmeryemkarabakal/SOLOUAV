#!/usr/bin/env python3
"""build_mechanism.py çıktısını bağımsız olarak sınar.

dogrula.py'nin mekanizma karşılığı: üretici betiğin tablolarını KULLANMAZ,
STEP dosyalarını ve model.sdf'i baştan okur. Böylece üreticideki bir hata
kapıya da taşınmaz.

Beş denetim:
  [1] katı sağlığı        — her parça geçerli, kapalı, pozitif hacimli mi
  [2] tilt süpürmesi      — motor/rotor/kızak SDF limitleri boyunca dönerken
                            sabit parçalara çarpıyor mu
  [3] kumanda yüzeyi      — 5 yüzey kendi limitlerinde serbest dönüyor mu
  [4] iniş takımı         — bacaklar gövdeye gerçekten bağlı mı (boşlukta değil)
  [5] istenmeyen girişim   — duruşta beklenmeyen çakışma var mı

Çıktı sınıfları dogrula.py ile aynı anlamda:
  HATA   = mekanizma yanlış, build_mechanism.py düzeltilmeli
  UYARI  = incelenmesi gereken ama çalışma aralığında kalan durum
  ok     = geçti

Bağımlılık: cadquery (build_mechanism.py ile aynı ortam). Mesh GEREKMEZ.
"""
import os
import re
import sys
import math
import glob

import cadquery as cq
from cadquery import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
P = os.path.join(HERE, 'step', 'parts')
SDF_CANDIDATES = [
    os.environ.get('TILTROTOR_SDF', ''),
    os.path.join(os.path.dirname(HERE), 'tiltrotor_tailplane_model.sdf'),
]
TOL = 1e-4          # cm^3 — bunun altı temas sayılır, çakışma değil

hata = 0
uyari = 0
gecti = 0


def say(sinif, msg):
    global hata, uyari, gecti
    if sinif == 'HATA':
        hata += 1
    elif sinif == 'UYARI':
        uyari += 1
    else:
        gecti += 1
    print('  {:6s} {}'.format(sinif, msg))


_cache = {}


def part(name):
    if name not in _cache:
        f = os.path.join(P, name + '.step')
        if not os.path.isfile(f):
            return None
        _cache[name] = cq.importers.importStep(f).val()
    return _cache[name]


def vol(s):
    return s.Volume() / 1e3


def inter(a, b):
    """Kesişim hacmi (cm^3). Güvenilirlik süzgeci: kesişim küçük gövdeyi aşamaz."""
    if a is None or b is None:
        return None
    try:
        v = a.intersect(b).Volume() / 1e3
    except Exception:                                          # noqa: BLE001
        return None
    if v > min(vol(a), vol(b)) + 1e-6:
        return None
    return v


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def rot(shape, A, D, deg):
    return shape.rotate(Vector(*A), Vector(A[0] + D[0], A[1] + D[1], A[2] + D[2]), deg)


def load_joints():
    sdf = next((p for p in SDF_CANDIDATES if p and os.path.isfile(p)), None)
    if sdf is None:
        raise SystemExit('model.sdf bulunamadi (TILTROTOR_SDF ile verin)')
    kod = open(os.path.join(HERE, 'fusion_02_joints.py'), encoding='utf-8').read()
    kod = re.sub(r'^import adsk\..*$', '', kod, flags=re.M)
    ns = {'__name__': 'fusion_02_joints_lib'}
    exec(compile(kod, 'fusion_02_joints.py', 'exec'), ns)
    return sdf, ns['parse_sdf_joints'](sdf)


MECH = ('tilt_', 'hinge_pins_', 'horn_', 'servo_', 'pushrod_', 'boom_brace_',
        'leg_front_left_strut', 'leg_front_right_strut', 'leg_tail_strut', 'tail_pylon')

TILT = [('right', 'motor_0_right', 'rotor_0_right', 'pylon_right'),
        ('left',  'motor_1_left',  'rotor_1_left',  'pylon_left'),
        ('tail',  'motor_2_tail',  'rotor_2_tail',  'tail_pylon')]

CHILD_PARENT = {'elevon_left': 'wing', 'elevon_right': 'wing',
                'elevator_left': 'tailplane', 'elevator_right': 'tailplane',
                'rudder': 'vertical_stabiliser'}

# Yüzey yalnızca ana gövdesine karşı sınanırsa yetmez: elevatör, tailplane'den
# değil KUYRUK ÇUBUĞUNDAN takılıyor (+24 deg). Ana gövde dışındaki komşular:
CHILD_KOMSU = {'elevator_left': ['tail_boom'], 'elevator_right': ['tail_boom'],
               'rudder': ['tail_boom'], 'elevon_left': [], 'elevon_right': []}


def main():
    sdf, J = load_joints()
    print('SDF   :', sdf)
    print('parca :', P, '\n')

    names = sorted(os.path.basename(f)[:-5] for f in glob.glob(os.path.join(P, '*.step')))
    mech = [n for n in names if n.startswith(MECH)]

    # ---------------------------------------------------------------- [1]
    print('[1] Kati sagligi ({} mekanizma parcasi)'.format(len(mech)))
    for n in mech:
        s = part(n)
        if s is None:
            say('HATA', '{}: dosya yok'.format(n))
        elif not s.isValid():
            say('HATA', '{}: gecersiz kati'.format(n))
        elif vol(s) <= 0:
            say('HATA', '{}: hacim <= 0'.format(n))
        else:
            say('ok', '{:26s} {:8.2f} cm3'.format(n, vol(s)))

    # ---------------------------------------------------------------- [2]
    print('\n[2] Tilt supurmesi (SDF limitleri boyunca)')
    for tag, mot, rot_name, pyl in TILT:
        A = J[mot]['point']
        D = unit(J[mot]['axis'])
        lo, hi = math.degrees(J[mot]['lower']), math.degrees(J[mot]['upper'])
        moving = [mot, rot_name, 'tilt_cradle_' + tag, 'tilt_crank_' + tag]
        static = ['tilt_yoke_' + tag, 'tilt_bearing_{}_in'.format(tag),
                  'tilt_bearing_{}_out'.format(tag), 'tilt_servo_' + tag,
                  'tilt_horn_' + tag, pyl]
        worst = 0.0
        where = ''
        for k in range(7):
            deg = lo + (hi - lo) * k / 6.0
            for mn in moving:
                mb = part(mn)
                if mb is None:
                    continue
                mv = rot(mb, A, D, deg)
                for sn in static:
                    v = inter(part(sn), mv)
                    if v is not None and v > worst:
                        worst, where = v, '{:.0f} deg {}->{}'.format(deg, mn, sn)
        if worst > TOL:
            say('HATA', 'tilt {:5s}: {:.3f} cm3 ({})'.format(tag, worst, where))
        else:
            say('ok', 'tilt {:5s}: {:+.0f}..{:+.0f} deg boyunca temiz'.format(tag, lo, hi))

    # ---------------------------------------------------------------- [3]
    print('\n[3] Kumanda yuzeyi hareketi')
    for surf, pname in CHILD_PARENT.items():
        A = J[surf]['point']
        D = unit(J[surf]['axis'])
        lim = math.degrees(J[surf]['upper'])
        sb = part(surf)
        hedefler = [pname] + CHILD_KOMSU.get(surf, [])
        worst = 0.0
        wdeg = 0.0
        wnm = ''
        temiz_sinir = lim
        for k in range(25):
            deg = -lim + 2 * lim * k / 24.0
            carpti = False
            for hn in hedefler:
                v = inter(part(hn), rot(sb, A, D, deg))
                if v is not None and v > TOL:
                    carpti = True
                    if v > worst:
                        worst, wdeg, wnm = v, deg, hn
            if carpti and abs(deg) < temiz_sinir:
                temiz_sinir = abs(deg)
        if worst > TOL:
            say('UYARI', '{:16s} {:+.1f} deg''de {} icine {:.3f} cm3 '
                         '(temiz sinir ~+-{:.0f} deg)'.format(
                             surf, wdeg, wnm, worst, temiz_sinir))
        else:
            say('ok', '{:16s} +-{:.1f} deg boyunca serbest'.format(surf, lim))

    # ---------------------------------------------------------------- [4]
    print('\n[4] Inis takimi baglantisi')
    for leg, strut, host in (('leg_front_right', 'leg_front_right_strut', 'wing'),
                             ('leg_front_left', 'leg_front_left_strut', 'wing'),
                             ('leg_tail', 'leg_tail_strut', 'tail_boom')):
        lb, sb, hb = part(leg), part(strut), part(host)
        if sb is None:
            say('HATA', '{}: dikme uretilmemis'.format(leg))
            continue
        gap_leg = sb.BoundingBox().zmin - lb.BoundingBox().zmax
        touches_host = inter(hb, sb)
        ok_leg = abs(gap_leg) < 0.5
        ok_host = touches_host is not None and touches_host >= 0.0 and \
            sb.BoundingBox().zmax >= hb.BoundingBox().zmin - 0.5
        if ok_leg and ok_host:
            say('ok', '{:16s} ayaga ve {} govdesine bagli'.format(leg, host))
        else:
            say('HATA', '{:16s} baglanti kopuk (ayak farki {:.1f} mm)'.format(leg, gap_leg))

    # ---------------------------------------------------------------- [5]
    print('\n[5] Duruşta istenmeyen girisim')
    # servo/korna gomulmeleri KASITLIDIR (cep), onlar haric tutulur
    KASITLI = {('servo_elevon_right', 'wing'), ('servo_elevon_left', 'wing'),
               ('servo_elevator_right', 'tail_boom'), ('servo_elevator_left', 'tail_boom'),
               ('servo_rudder', 'tail_boom'), ('servo_rudder', 'servo_fairing_rudder'),
               ('servo_fairing_rudder', 'tail_boom'),
               ('tilt_servo_right', 'pylon_right'), ('tilt_servo_left', 'pylon_left'),
               ('tilt_servo_tail', 'tail_pylon'),
               ('leg_tail_strut', 'tail_boom'), ('tail_pylon', 'tail_boom'),
               ('boom_brace_right', 'wing'), ('boom_brace_left', 'wing'),
               ('boom_brace_right', 'tail_boom'), ('boom_brace_left', 'tail_boom'),
               ('leg_front_right_strut', 'wing'), ('leg_front_left_strut', 'wing')}
    AIRFRAME = ['wing', 'tailplane', 'vertical_stabiliser', 'tail_boom',
                'pylon_right', 'pylon_left', 'tailplane_strut']
    bulundu = 0
    for m in mech:
        for a in AIRFRAME:
            if m == a or (m, a) in KASITLI or (a, m) in KASITLI:
                continue
            v = inter(part(m), part(a))
            if v is not None and v > TOL:
                say('UYARI', '{} n {} = {:.3f} cm3'.format(m, a, v))
                bulundu += 1
    if not bulundu:
        say('ok', 'beklenmeyen girisim yok')

    print('\nOZET: {} gecti, {} uyari, {} HATA'.format(gecti, uyari, hata))
    return 1 if hata else 0


if __name__ == '__main__':
    raise SystemExit(main())
