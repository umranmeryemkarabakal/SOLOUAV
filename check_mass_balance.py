#!/usr/bin/env python3
"""
Kutle dagilimi ve agirlik merkezi denetimi.

NEDEN VAR (2026-08-31). Bu oturumda geometri epey degisti (fin 12 cm geriye,
cubuk uzadi, pilonlar kisaldi, inis takimi eklendi) ama AGIRLIK MERKEZI hic
olculmedi -- yalnizca fin kaymasinin CG'ye etkisi kabaca tahmin edildi.
CG, bu aracta dogrudan kontrol sabitlerine giriyor: rotor kollari
(p.rotor.pos), aero cp'ler ve tum moment hesaplari CG'ye GORE tanimli.
CG kayarsa etkinlik matrisi sessizce yanlislasir.

⚠ SINIR: SDF yalnizca base_link icin TEK bir <inertial> blogu tasiyor
(kutle 5 kg, CG = <pose> ile verilen nokta). Govde parcalari (kanat, cubuk,
fin, pilonlar, ayaklar) GORSEL/COLLISION olarak var ama AYRI KUTLELERI YOK --
hepsi o tek inertial'in icinde varsayiliyor. Yani bu betik gercek CG'yi
OLCEMEZ; yaptigi sey, parcalarin kutlelerini makul yogunluklarla TAHMIN edip
"geometri degisiklikleri CG'yi ne kadar kaydirir" sorusunu yanitlamak.
Gercek CG, gercek aracta TARTILARAK bulunmali (donanim kontrol listesi B2).
"""
import re
import sys
import numpy as np

SDF = 'tiltrotor_tailplane_model.sdf'

# Kaba yogunluklar (kg/m^3). Kompozit/kopuk yapi icin efektif degerler --
# dolu malzeme degil, ici bos/sandvic yapiyi temsil eder.
RHO = {
    'default': 300.0,     # kompozit sandvic
    'boom': 800.0,        # karbon tup (ici bos ama duvar yogun)
    'leg': 1200.0,        # karbon cubuk
}


def load():
    s = open(SDF).read()
    parts = []

    # base_link'in beyan edilen inertial'i
    m = re.search(r"<link name='base_link'>(.*?)</link>", s, re.S)
    body = m.group(1)
    im = re.search(r"<inertial>(.*?)</inertial>", body, re.S)
    mass = float(re.search(r"<mass>([^<]+)</mass>", im.group(1)).group(1))
    ip = re.search(r"<pose>([^<]+)</pose>", im.group(1))
    cg = [float(v) for v in ip.group(1).split()[:3]] if ip else [0, 0, 0]

    # gorsel parcalar (kutu/silindir olanlar) -- hacimden kutle TAHMINI
    for vm in re.finditer(r"<visual name='([^']+)'>(.*?)</visual>", body, re.S):
        nm, vb = vm.group(1), vm.group(2)
        po = re.search(r"<pose>([^<]+)</pose>", vb)
        sz = re.search(r"<size>([^<]+)</size>", vb)
        cy = re.search(r"<radius>([^<]+)</radius>.*?<length>([^<]+)</length>", vb, re.S)
        if not po:
            continue
        p = [float(v) for v in po.group(1).split()[:3]]
        if sz:
            L = [float(v) for v in sz.group(1).split()]
            vol = L[0] * L[1] * L[2]
        elif cy:
            vol = np.pi * float(cy.group(1))**2 * float(cy.group(2))
        else:
            continue
        rho = RHO['boom'] if 'boom' in nm else (RHO['leg'] if 'leg' in nm else RHO['default'])
        parts.append((nm, p, vol * rho))

    # motorlar ve rotorlar: ayri link, kendi inertial'leri var
    for lm in re.finditer(r"<link name='((?:motor|rotor)_\d)'>(.*?)</link>", s, re.S):
        nm, lb = lm.group(1), lm.group(2)
        po = re.search(r"<pose>([^<]+)</pose>", lb)
        ms = re.search(r"<mass>([^<]+)</mass>", lb)
        if po and ms:
            parts.append((nm, [float(v) for v in po.group(1).split()[:3]], float(ms.group(1))))
    return mass, cg, parts


def main():
    mass, cg, parts = load()
    print(f"\n### KUTLE DENGESI ({SDF}) ###\n")
    print(f"SDF'in BEYAN ETTIGI base_link:")
    print(f"   kutle = {mass:.3f} kg   CG = ({cg[0]:+.3f}, {cg[1]:+.3f}, {cg[2]:+.3f}) m")
    print(f"   ⚠ Bu TEK bir blok -- govde parcalarinin ayri kutlesi YOK.\n")

    print(f"{'parca':<26} {'x':>8} {'y':>8} {'z':>8} {'kutle(g)':>10}")
    tot = 0.0
    mom = np.zeros(3)
    for nm, p, m in sorted(parts, key=lambda a: -a[2]):
        print(f"  {nm:<24} {p[0]:+8.3f} {p[1]:+8.3f} {p[2]:+8.3f} {m*1000:10.1f}")
        tot += m
        mom += np.array(p) * m
    print(f"  {'TOPLAM (tahmini)':<24} {'':>8} {'':>8} {'':>8} {tot*1000:10.1f}")

    if tot > 0:
        cg_est = mom / tot
        print(f"\nTAHMINI CG (yalnizca modellenen parcalar):")
        print(f"   ({cg_est[0]:+.4f}, {cg_est[1]:+.4f}, {cg_est[2]:+.4f}) m")
        print(f"   beyan edilen CG'ye gore kayma: "
              f"({cg_est[0]-cg[0]:+.4f}, {cg_est[1]-cg[1]:+.4f}, {cg_est[2]-cg[2]:+.4f}) m")

    print(f"\n=== SIMETRI (y ekseni) ===")
    ymom = sum(p[1] * m for _, p, m in parts)
    print(f"   y momenti = {ymom*1000:+.2f} g.m   "
          f"{'OK -- simetrik' if abs(ymom) < 1e-6 else 'ASIMETRIK'}")

    print(f"\n=== KONTROL SABITLERIYLE TUTARLILIK ===")
    print(f"   rotor kollari (p.rotor.pos) CG'ye GORE tanimli:")
    print(f"     kanat  x = +0.27   kuyruk x = -0.55")
    print(f"   Bu degerler SDF'teki rotor POZLARIYLA ayni, yani ikisi de")
    print(f"   MODEL ORIJININI CG kabul ediyor. Beyan edilen CG "
          f"({cg[0]:+.3f}, {cg[1]:+.3f}, {cg[2]:+.3f}) bunu {'DOGRULUYOR' if max(abs(np.array(cg)))<1e-9 else 'DOGRULAMIYOR -- ofset var'}.")

    print(f"\n=== BU OTURUMDAKI GEOMETRI DEGISIKLIKLERININ CG ETKISI ===")
    changes = [
        ('fin -0.74 -> -0.86', 0.16*0.02*0.20*RHO['default'], -0.12),
        ('rudder -0.83 -> -0.95', 0.06*0.02*0.18*RHO['default'], -0.12),
        ('cubuk 0.62 -> 0.76 m', (0.76-0.62)*0.04*0.02*RHO['boom'], -0.38),
        ('on pilonlar kisaldi (x2)', -2*(0.02*0.03*0.06)*RHO['default'], +0.205),
        ('inis takimi (3 ayak)', 3*np.pi*0.012**2*0.05*RHO['leg'], -0.14),
        # Adim 143: motorlar disa+geri alindi (pod girisimi). Kutle sabit,
        # yalnizca x kolu degisti -- bu yuzden delta-kutle degil delta-moment.
        ('motorlar x 0.22 -> 0.27', 2*0.055, +0.050),
    ]
    net = 0.0
    for nm, dm, x in changes:
        d = dm * x / mass
        net += d
        print(f"   {nm:<28} {dm*1000:+7.1f} g @ x={x:+.3f} -> CG {d*1000:+6.2f} mm")
    print(f"   {'NET':<28} {'':>7}   {'':>10}    CG {net*1000:+6.2f} mm")
    print(f"\n   Karsilastirma: rotor kolu 0.22 m; {abs(net)*1000:.1f} mm kayma "
          f"pitch kolunu %{100*abs(net)/0.55:.2f} degistirir.")
    print(f"   {'-> IHMAL EDILEBILIR' if abs(net) < 0.005 else '-> ONEMLI, kontrol sabitleri gozden gecirilmeli'}\n")


if __name__ == '__main__':
    main()
