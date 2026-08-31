#!/usr/bin/env python3
"""Spawn visual-only reference geometry into the running Gazebo world.

WHY THIS EXISTS (2026-08-28). A GUI run of this project shows a grey vehicle
on a grey background and the flight is unreadable -- a 40 m descent and a 4 m
descent look identical. Three measured causes, and "no scenery" is only one:

  1. THE LANDING HAPPENS OFF THE EDGE OF THE DRAWN GROUND. `ground_plane` in
     worlds/default.sdf has an INFINITE collision plane but its VISUAL is only
     100x100 m (default.sdf:190-196), i.e. +-50 m. The full mission flies 578 m
     east and lands there -- 528 m past anything that is drawn. There is no
     ground under the vehicle at touchdown, so there is nothing to descend
     against.
  2. The camera never closes distance (`/gui/follow/offset` is a no-op in this
     gz build) -- see gz_chase_cam.py, which is the working replacement.
  3. No reference objects, and <grid> is false (default.sdf:167).

This script addresses (1) and (3). Pair it with gz_chase_cam.py for (2).

DESIGN CONSTRAINTS, all measured rather than assumed:

  * VISUAL ONLY, NO COLLISION, static. This vehicle has no horizontal position
    loop in some phases and drifts hundreds of metres; a collidable prop in the
    corridor would produce a crash that reads as a control regression. Without
    <collision> there is no contact, no mass and no inertia -- the physics is
    bit-for-bit unchanged.
  * ONE model, MANY links. Each `gz service .../create` call costs ~0.1-0.2 s;
    spawning ~40 separate entities would stall the launch. A single model with
    many links is one call.
  * PRIMITIVES ONLY (box/cylinder). The GUI here runs on software rendering at
    ~250% CPU (libEGL cannot create a dri2 screen), and default.sdf contains
    zero meshes. Keep it that way.

FRAMES -- the easy mistake. PX4 local position is NED (x north, y east, z down)
but Gazebo world is ENU (x east, y north, z up). The mission's measured track is
NED y: 0 -> 578 m, i.e. EAST, which in Gazebo is +X. The corridor is therefore
along Gazebo +X, only +-7 m wide in Gazebo Y. Everything below is Gazebo ENU.

Usage:
    python3 gz_scene.py [world]
"""

from __future__ import annotations

import subprocess
import sys

WORLD = sys.argv[1] if len(sys.argv) > 1 else "default"
MODEL_NAME = "flight_reference"

# --- corridor, from the measured mission footprint (ULog 17_25_53) ----------
# Gazebo ENU: +X is the direction of travel. NED y 0..578 -> Gazebo x 0..578.
# 8 kosumun ayak izi (2026-08-29): Gazebo X -0.3 .. 739.2, Y -30.3 .. +70.6.
# Ilk surumde X 650'de bitiyordu ve arac zeminin 89 m otesine cikti ("uçak alan
# dışına çıktı"); yanal sapma da +-7 m sanilmisti, gercekte 10 kat fazla.
# Asagidaki degerler olculen azamiye ~%60 pay birakir. Zemin TEK bir kutu
# oldugu icin buyutmenin render maliyeti yok denecek kadar az.
TRACK_END = 1200.0     # m, irtifa merdiveni/direk dizisinin kapsadigi mesafe
CLEAR = 25.0           # m, nesneler bu Y'nin disinda. Carpisma YOK (gorsel
                       # only), yani nadir bir yanal sapmada arac iclerinden
                       # gecerse tek sonuc gorsel gariplik olur.

GROUND_LEN = 1400.0    # m along X, covers -150 .. 1250
GROUND_WID = 600.0     # m along Y  (+-300)
STRIPE_PITCH = 50.0    # m between cross stripes
TOWER_H = 40.0         # m, matches CLIMB_M in run_mission_test.py
BAND_H = 10.0          # m per colour band -> reads as a 10/20/30/40 ladder


def link(name, x, y, z, geom, rgb, yaw=0.0, spec=0.1):
    """One static visual link. No <collision> -- see module docstring.

    `spec` matters more than it looks: the stock ground_plane uses
    ambient/diffuse/SPECULAR all 0.8 (default.sdf:197-201). A sheet that
    matches its colour but not its specular still reads as a different
    surface under the same light, so the seam at +-50 m stays visible.
    """
    r, g, b = rgb
    return f"""
    <link name="{name}">
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <visual name="v">
        <geometry>{geom}</geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>{spec} {spec} {spec} 1</specular>
        </material>
      </visual>
    </link>"""


def box(sx, sy, sz):
    return f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>"


def cyl(radius, length):
    return f"<cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder>"


def build_sdf():
    links = []

    # 1) Ground sheet. The whole point: give the touchdown a surface to happen
    #    against. Sits at z = -0.02, i.e. BELOW the stock 100x100 visual, so the
    #    two never z-fight in the overlap; beyond +-50 m only this one is drawn.
    # z = +0.005: ustu 0.015 m'de, yani stok 100x100 levhanin (z=0) USTUNDE.
    # Once -0.02'ye, altina konmustu ve renk esitlemesi denendi; ikisi de
    # yetmedi -- kalkis bolgesi ayri bir yuzey olarak okunmaya devam etti
    # ("beyaz alan duruyor", 2026-08-29). Dogru cozum eslemek degil ORTMEK:
    # tek levha gorunur, dikis kaybolur. Gorsel oldugu icin temas hala stok
    # z=0 duzleminde olur; arac 1.5 cm gomulu gorunur, farkedilmez.
    links.append(link("ground", GROUND_LEN / 2 - 150.0, 0.0, 0.005,
                      box(GROUND_LEN, GROUND_WID, 0.02), (0.62, 0.64, 0.60), spec=0.3))

    # 2) Cross stripes every 50 m -- distance scale AND the motion cue that a
    #    uniform plane cannot give. Slightly above the sheet, still below stock.
    #    RENK STOK ZEMINLE AYNI (0.8 gri, default.sdf:197-201). Ilk denemede
    #    bu levha 0.44 koyu gri idi ve stok 100x100 m'lik acik gri levhanin
    #    disinda kaliyordu: kalkis bolgesi BEYAZ, otesi koyu, aralarinda +-50
    #    m'de sert bir sinir -- "ilk kalktigi yer beyaz gozukuyor, butunluk
    #    yok" (2026-08-29). Ayni griyle iki levha tek bir zemin gibi okunur.
    #    NARROW (4 m) and LOW CONTRAST on purpose: 25 m wide half-and-half bands
    #    made the ground flash light/dark as the camera tracked along it
    #    (reported 2026-08-28). A thin line reads as a distance mark; a wide
    #    band reads as the ground changing colour.
    n = 0
    x = 0.0
    while x <= TRACK_END:
        links.append(link(f"stripe_{n}", x, 0.0, 0.017,
                          box(4.0, GROUND_WID, 0.02), (0.52, 0.54, 0.50), spec=0.3))
        x += STRIPE_PITCH
        n += 1

    # 3) Altitude ladders at BOTH ends of the track. The landing is at x=578,
    #    not at the launch point, so a ladder only at the origin would not show
    #    the descent at all.
    # Inis noktasi kosumdan kosuma degisiyor (578 m, 739 m ...), bu yuzden
    # merdiven tek bir yere degil koridor boyunca 200 m'de bir konur.
    for tag, tx in [(f"m{int(v)}", float(v)) for v in range(0, int(TRACK_END) + 1, 200)]:
        for side, sy in (("l", CLEAR), ("r", -CLEAR)):
            for i in range(int(TOWER_H / BAND_H)):
                shade = (0.85, 0.25, 0.15) if i % 2 == 0 else (0.95, 0.95, 0.92)
                links.append(link(f"tower_{tag}_{side}_{i}", tx, sy,
                                  i * BAND_H + BAND_H / 2,
                                  cyl(0.35, BAND_H), shade))

    # 4) Distance posts down the corridor, every 100 m, outside the clear zone.
    for i in range(1, int(TRACK_END // 100) + 1):
        for side, sy in (("l", CLEAR), ("r", -CLEAR)):
            links.append(link(f"post_{i}_{side}", i * 100.0, sy, 1.5,
                              cyl(0.25, 3.0), (0.95, 0.75, 0.10)))

    # 5) Distant blocks for depth. Far outside the corridor; they stay put in
    #    frame and give the eye something to judge motion against.
    for i, (bx, by, h) in enumerate([
        (60.0, 150.0, 18.0), (180.0, -165.0, 26.0), (320.0, 155.0, 14.0),
        (430.0, -150.0, 30.0), (540.0, 160.0, 20.0), (680.0, -155.0, 24.0),
        (820.0, 150.0, 22.0), (980.0, -160.0, 28.0), (1120.0, 145.0, 16.0),
    ]):
        links.append(link(f"block_{i}", bx, by, h / 2,
                          box(22.0, 22.0, h), (0.58, 0.60, 0.64)))

    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{MODEL_NAME}">
    <static>true</static>{''.join(links)}
  </model>
</sdf>"""


def spawn(sdf: str) -> bool:
    """One EntityFactory call. Returns False on any failure -- a missing
    backdrop must never fail a flight."""
    req = 'sdf: "{}" allow_renaming: true'.format(
        sdf.replace("\\", "\\\\").replace('"', '\\"').replace("\n", " "))
    try:
        r = subprocess.run(
            ["gz", "service", "-s", f"/world/{WORLD}/create",
             "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
             "--timeout", "5000", "--req", req],
            capture_output=True, text=True, timeout=20.0)
        return "true" in r.stdout.lower()
    except (OSError, subprocess.TimeoutExpired):
        return False


def main():
    sdf = build_sdf()
    n_links = sdf.count("<link ")
    ok = spawn(sdf)
    print(f"  gorsel referans: {n_links} parca "
          f"({'eklendi' if ok else 'EKLENEMEDI -- ucus etkilenmez'})")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
