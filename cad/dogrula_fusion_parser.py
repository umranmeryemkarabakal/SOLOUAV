#!/usr/bin/env python3
"""fusion_02_joints.py'nin SDF ayristiricisini dogrulama raporuna karsi sinar.

Neden ayri bir kapi: fusion_02_joints.py, Fusion 360 icinde calistigi icin
cadquery/numpy kullanamaz ve SDF'i KENDI regex'leriyle okur. Yani ayristirma
mantiginin ikinci bir kopyasi vardir. Bu kapi o kopyanin dogrulanmis tabloyla
ayni sonucu verdigini garanti eder.

Referans: cad/dogrulama_raporu.txt [5] bolumu (dogrula.py uretir).
Bagimlilik yok -- yalnizca standart kutuphane, sistem python3'u yeter.

Yakaladigi iki gercek hata (1 Eylul 2026, ikisi de sessizdi):
  * link adlari CIFT tirnakli, eklem adlari TEK tirnakli -> elevator/rudder
    menteşesi 790-870 mm kaydi
  * <inertial> pose'u kutle merkezidir, link yerlesimi degil -> elevon
    menteşesi tam 300 mm kaydi
"""
import os
import re
import sys
import math

HERE = os.path.dirname(os.path.abspath(__file__))
RAPOR = os.path.join(HERE, 'dogrulama_raporu.txt')

TOL_NOKTA = 1e-6   # mm
TOL_EKSEN = 1e-4   # rapor ekseni 4 haneye yuvarliyor
TOL_LIMIT = 0.05   # derece; rapor limiti 1 haneye yuvarliyor

SATIR = re.compile(
    r"(\S+_joint): nokta \[([^\]]+)\] eksen \[([^\]]+)\] limit \[([-\d.]+)°,([-\d.]+)°\]")


def rapordan_oku(path):
    """[5] Menteşe noktalari bolumunu ayristir."""
    out = {}
    icinde = False
    for line in open(path, encoding='utf-8'):
        if line.startswith('[5]'):
            icinde = True
            continue
        if icinde and line.startswith('['):
            break
        m = SATIR.search(line)
        if icinde and m:
            out[m.group(1)] = dict(
                point=[float(v) for v in m.group(2).split()],
                axis=[float(v) for v in m.group(3).split()],
                lower=float(m.group(4)), upper=float(m.group(5)))
    return out


def main():
    if not os.path.isfile(RAPOR):
        print('HATA: {} yok. Once dogrula.py calistirin.'.format(RAPOR))
        return 2

    sys.path.insert(0, HERE)
    kod = open(os.path.join(HERE, 'fusion_02_joints.py'), encoding='utf-8').read()
    ns = {'__name__': 'fusion_02_joints_test'}   # run() otomatik tetiklenmesin
    # adsk yalnizca run() icinde kullaniliyor; import satirlarini atla
    kod = re.sub(r'^import adsk\..*$', '', kod, flags=re.M)
    exec(compile(kod, 'fusion_02_joints.py', 'exec'), ns)

    sdf = ns['find_sdf']()
    if sdf is None:
        print('HATA: model.sdf bulunamadi (fusion_02_joints.find_sdf).')
        return 2
    tablo = ns['parse_sdf_joints'](sdf)
    ref = rapordan_oku(RAPOR)

    print('referans {} eklem, betik {} eklem'.format(len(ref), len(tablo)))
    ad = {v['joint']: (k, v) for k, v in tablo.items()}
    kotu = 0
    for jn in sorted(ref):
        if jn not in ad:
            print('  EKSIK  {}: betik bu eklemi uretmedi'.format(jn))
            kotu += 1
            continue
        part, T = ad[jn]
        R = ref[jn]
        dp = max(abs(a - b) for a, b in zip(T['point'], R['point']))
        da = max(abs(a - b) for a, b in zip(T['axis'], R['axis']))
        sorun = []
        if dp > TOL_NOKTA:
            sorun.append('nokta {:.3f} mm'.format(dp))
        if da > TOL_EKSEN:
            sorun.append('eksen {:.5f}'.format(da))
        if not T['continuous']:
            dl = max(abs(math.degrees(T['lower']) - R['lower']),
                     abs(math.degrees(T['upper']) - R['upper']))
            if dl > TOL_LIMIT:
                sorun.append('limit {:.3f}°'.format(dl))
        if sorun:
            print('  FARK   {} ({}): {}'.format(jn, part, ', '.join(sorun)))
            kotu += 1
        else:
            print('  ok     {} -> {}'.format(jn, part))

    if kotu:
        print('\n{} eklemde FARK var. fusion_02_joints.py ayristiricisi '
              'dogrulanmis tabloyla uyusmuyor.'.format(kotu))
        return 1
    print('\n✅ {} eklemin hepsi dogrulanmis tabloyla birebir.'.format(len(ref)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
