#!/usr/bin/env python3
"""Drive the Gazebo GUI camera to chase the model -- a REPLACEMENT for /gui/follow.

WHY THIS EXISTS (2026-08-04). Every GUI run of this project showed the vehicle as
a handful of pixels, and it was blamed on the camera being "far". It is not a
distance setting: `/gui/follow/offset` reports success and does nothing in this
gz build. The camera keeps re-AIMING at the target (the vehicle stays dead centre
in every frame, measured) while never closing the distance, so at 40 m altitude
and 15 m/s the aircraft is ~4 px across and the tilt angle -- the one thing this
project's flights exist to show -- is invisible.

Two earlier fixes did not help and both are worth recording so they are not
retried: `<follow_pgain>1.0</follow_pgain>` in the world SDF (step 41), and the
same tag added to ~/.gz/sim/8/gui.config after discovering the user config
carries its OWN CameraTracking plugin and takes precedence. The gain was never
the problem -- the offset simply is not applied.

WHAT THIS DOES INSTEAD: reads the model pose from /world/default/dynamic_pose/info and
commands an absolute camera pose via /gui/move_to/pose, at ~10 Hz. The offset is
expressed in the model's yawed frame, so the view stays a side/quarter shot as
the vehicle turns.

Usage:
    python3 gz_chase_cam.py [model] [dx dy dz]     # gz FLU: +x fwd, +y LEFT
    python3 gz_chase_cam.py tiltrotor_indi_0 0 -6 1.5
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
import time

MODEL = sys.argv[1] if len(sys.argv) > 1 else "tiltrotor_indi_0"
OFF = [float(v) for v in sys.argv[2:5]] if len(sys.argv) >= 5 else [0.0, -6.0, 1.5]
WORLD = "default"
RATE_HZ = 10.0

# DYNAMIC_POSE, /world/*/pose/info DEGIL (2026-08-29). Belirti: kamera
# tirmanista ~21 m'de takili kaldi, ucak 40 m'ye cikip 621 m doguya gitti --
# "kamera asagida kaldi ucak ucup cikti". Kamera DONMAMISTI, GECMISI
# OYNATIYORDU: setup_gui_view'in gorsel zemini 115 statik parca spawn ediyor,
# bunlarin hepsi pose/info'ya giriyor ve konu 1.36 MB/s'ye cikiyor. Her tikte
# bu tamponu ayristirmak + bir `gz service` sureci dogurmak 10 Hz'e yetismiyor,
# boru yukarida tikaniyor ve okunan veri saniyeler-dakikalar bayatliyor.
# dynamic_pose yalnizca HAREKET EDEN varliklari yayar: olculdu, 157 KB/s
# (8.7 kat kucuk), arac yine ~47 Hz. Blok bicimi birebir ayni (name/position/
# orientation), yani parse_pose degismeden calisir.
# Konu yoksa (eski gz) otomatik olarak eskisine doner.
POSE_TOPIC = os.environ.get("INDI_GZ_POSE_TOPIC") or f"/world/{WORLD}/dynamic_pose/info"

# --- smoothing (2026-08-28) ------------------------------------------------
# Without this the view swings left and right and the whole world appears to
# slide under the aircraft. The offset is applied in the model's YAWED frame,
# and this airframe oscillates in yaw by a few degrees continuously (its yaw
# authority is the weakest axis). At an 18 m camera arm, 3 deg of yaw is ~0.9 m
# of lateral camera travel, reversing several times a second -- unwatchable.
# Filtering the yaw kills the swing while still turning the view when the
# vehicle genuinely changes heading. Position is filtered lightly on top, which
# also hides the jitter from sampling the pose topic through a subprocess.
YAW_TAU = 3.0   # s, heavy on purpose: real heading changes take much longer
POS_TAU = 1.2   # s, 0.5 -> 1.2: govdenin 5 Hz ustu titremesi 0.5 s'lik
                # filtreden bir miktar sizip kamerayi hala sarsiyordu

# Ignore the model's heading entirely and hold the offset in the WORLD frame.
# Useful for the straight-line mission profile, where a fixed side-on bearing
# is steadier than anything referenced to the airframe.
WORLD_FRAME = os.environ.get("INDI_GZ_CAM_WORLD", "0") not in ("", "0", "no", "false")


def _lp(prev, new, tau, dt):
    """One-pole low pass. Returns `new` on the first call (prev is None)."""
    if prev is None:
        return new
    a = dt / (tau + dt)
    return prev + a * (new - prev)


def _lp_angle(prev, new, tau, dt):
    """Low pass on an angle, wrapping correctly across +-pi."""
    if prev is None:
        return new
    d = (new - prev + math.pi) % (2 * math.pi) - math.pi
    a = dt / (tau + dt)
    return prev + a * d


class PoseStream:
    """One long-lived `gz topic -e` reader.

    WHY (2026-08-28). The first version spawned `gz topic -e -n 1` AND a
    `gz service` call every cycle -- 20 processes a second. On this machine the
    GUI already runs on software rendering at ~250% CPU, and the extra churn
    was the one measurable difference between GUI runs (vehicle stalled 1.17 m
    up, 20774 deg of yaw) and headless runs (landed, 2119 deg worst case). Even
    though lockstep should isolate the physics, spawning is not free and the
    correlation was too strong to leave in place. Reading one persistent stream
    halves the process count and removes the per-call startup cost.

    Falls back to the old one-shot call if the stream cannot be opened, so a
    camera problem still never fails a flight.
    """

    def __init__(self):
        self.proc = None
        self.fd = -1
        self.buf = ""
        self.topic = POSE_TOPIC
        self._fell_back = False
        self.last_data = time.monotonic()
        self._open()

    def fall_back(self):
        """dynamic_pose ise bir kereligine /world/*/pose/info'ya don. True =
        donuldu, tekrar denemeye deger; False = zaten eski konudayiz."""
        if self._fell_back or self.topic.endswith("/pose/info"):
            return False
        self.stop()
        self.topic = f"/world/{WORLD}/pose/info"
        self._fell_back = True
        self.buf = ""
        self.last_data = time.monotonic()   # yeni konuya temiz 5 s tani
        self._open()
        return self.proc is not None

    def _open(self):
        try:
            # BINARY, not text. A non-blocking TEXT pipe raises inside the codec
            # ("can't concat NoneType to bytes") the moment there is nothing to
            # read, which is most ticks -- the decoder cannot represent "no data
            # yet". Reading raw fds and decoding here sidesteps that entirely.
            self.proc = subprocess.Popen(
                ["gz", "topic", "-e", "-t", self.topic],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            self.fd = self.proc.stdout.fileno()
            os.set_blocking(self.fd, False)
        except OSError:
            self.proc = None

    def latest(self):
        """Newest complete pose_v text block seen so far, or None."""
        if self.proc is None or self.proc.poll() is not None:
            return None
        chunk = ""
        try:
            while True:
                b = os.read(self.fd, 65536)
                if not b:
                    break
                chunk += b.decode("utf-8", "ignore")
        except BlockingIOError:
            pass          # drained: normal, most ticks end here
        except (OSError, ValueError):
            return None
        # BAYAT TAMPON = ZOMBI SURECI (2026-08-29). Bu fonksiyon eskiden kosulsuz
        # self.buf donduruyordu; tampon bir kez dolunca ICINDE HEP GECERLI bir
        # blok kaliyor, parse_pose son bilinen pozu sonsuza dek uretiyor, main'in
        # `misses > 50` korumasi hic tetiklenmiyordu. Sonuc: SITL kapandiktan
        # sonra da yasayan bir kamera. Iki kez ust uste olcuIdu -- 10:44
        # kosumundan kalan surec 38 dk sonra hala ayaktaydi ve YENI kosumun
        # kamerasiyla ayni kamerayi cekistiriyordu. Yeni veri gelmiyorsa tamponu
        # gecersiz say: hem zombi olur, hem bayat poz komut edilmez.
        if chunk:
            self.last_data = time.monotonic()
            self.buf += chunk
        elif time.monotonic() - self.last_data > 5.0:
            return None
            # keep only the tail; the topic repeats every entity at ~250 Hz and
            # the buffer would otherwise grow without bound over a 5 min flight
            #
            # 400/200 KB -> 120/60 KB (2026-08-29). Tampon buyudukce her tikteki
            # split() de pahalilasiyor; bu, tikin 0.1 s'yi asmasina ve borunun
            # yukarida tikanip verinin bayatlamasina KATKIDA BULUNAN ikinci
            # etkendi (birincisi POSE_TOPIC notu). dynamic_pose 157 KB/s akitir,
            # yani 60 KB'lik kuyruk hala birkac tam donguyu kapsar -- parse_pose
            # sondan tarar, bir blok bulmasi icin fazlasiyla yeter.
            if len(self.buf) > 120_000:
                self.buf = self.buf[-60_000:]
        return self.buf or None

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()


def parse_pose(out):
    """(x, y, z, yaw) of MODEL from a pose_v text dump, or None.

    The topic carries every entity in the world, and the model's own pose is the
    entry whose name is exactly MODEL -- link poses appear under their own names
    and are relative, so matching a prefix would silently pick up a nacelle.
    Scans from the END so the newest sample in the buffer wins.
    """
    if not out:
        return None

    # text protobuf: repeated `pose { name: "..." position { x: .. } orientation { .. } }`
    blocks = out.split("pose {")[::-1]
    for b in blocks:
        if f'name: "{MODEL}"' not in b:
            continue

        def num(key, blk):
            i = blk.find(key)
            if i < 0:
                return None
            return float(blk[i + len(key):].split("\n", 1)[0].strip())

        pos_i = b.find("position {")
        ori_i = b.find("orientation {")

        if pos_i < 0 or ori_i < 0:
            continue

        pos = b[pos_i:ori_i]
        ori = b[ori_i:]
        x, y, z = num("x:", pos), num("y:", pos), num("z:", pos)
        qx, qy = num("x:", ori), num("y:", ori)
        qz, qw = num("z:", ori), num("w:", ori)

        if None in (x, y, z, qx, qy, qz, qw):
            continue

        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return x, y, z, yaw

    return None


def move_camera(px, py, pz, yaw, pitch):
    """Point the GUI camera at (yaw, pitch) from (px, py, pz).

    THE QUATERNION MUST BE THE REAL COMPOSITION (fixed 2026-08-29). The first
    version wrote qy = sin(pitch/2), qz = sin(yaw/2), qx = 0 -- dropping both
    cross terms and leaving the result unnormalised. That is only correct when
    yaw or pitch is zero. In flight neither is: the vehicle tracks east
    (yaw ~ 90 deg) while the camera looks down at it (pitch ~ 0.25 rad), so the
    camera aimed somewhere other than the aircraft and the view sat below it --
    "kamera aşağıda kalıyor, uçak yukarıda, göremiyorum". Hand-commanding a
    correct pose framed the aircraft immediately, which is what isolated it.

    Yaw about Z then pitch about the new Y (ZYX):
      qw = cy*cp,  qx = -sy*sp,  qy = cy*sp,  qz = sy*cp
    """
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    qw, qx, qy, qz = cy * cp, -sy * sp, cy * sp, sy * cp
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    req = (f"pose: {{ position: {{ x: {px:.3f} y: {py:.3f} z: {pz:.3f} }} "
           f"orientation: {{ x: {qx:.6f} y: {qy:.6f} z: {qz:.6f} w: {qw:.6f} }} }}")
    subprocess.run(["gz", "service", "-s", "/gui/move_to/pose",
                    "--reqtype", "gz.msgs.GUICamera", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "300", "--req", req],
                   capture_output=True, text=True)


def main():
    print(f"chase cam: {MODEL}, offset {OFF} (gz FLU), {RATE_HZ:.0f} Hz, "
          f"{'world' if WORLD_FRAME else 'model'} frame")
    misses = 0
    yaw_f = None
    cx = cy = cz = None
    stream = PoseStream()
    t_prev = time.monotonic()
    x_prev = y_prev = z_prev = None
    vx_f = vy_f = vz_f = None

    while True:
        p = parse_pose(stream.latest())

        if p is None:
            misses += 1

            if misses > 50:
                # dynamic_pose yoksa/bos ise bir kez eski konuya don, sonra birak.
                if stream.fall_back():
                    print(f"dynamic_pose bos -- {stream.topic} konusuna donuluyor")
                    misses = 0
                    time.sleep(1.0 / RATE_HZ)
                    continue
                print("model pose unavailable for 5 s -- stopping")
                stream.stop()
                return

            time.sleep(1.0 / RATE_HZ)
            continue

        misses = 0
        x, y, z, yaw = p

        # dt OLCULUR, VARSAYILMAZ (2026-08-29). Eskiden dt = 1/RATE_HZ sabitti.
        # Ama dongu 10 Hz'e YETISMIYOR: her tikte tampon ayristirmasi + bir
        # `gz service` sureci var. Dongu gercekte ~1 Hz donunce alcak-geciren
        # filtrenin katsayisi dt/(tau+dt) on kat kucuk kaliyor ve 1.2 s'lik
        # zaman sabiti fiilen 12 s+ oluyor. Belirti bu: kamera tirmanista
        # "uçağı görüyorum ama aşağıdan" (uçak z=39.7, kamera z=24.3 -- 15 m
        # geride), seyirde ise "uçak uçtu gitti ben kalkış yerinde kaldım"
        # (13 m/s'ye hic yetisemiyor). Gercek dt ile filtre, dongu hizi ne
        # olursa olsun soz verdigi zaman sabitini tutar.
        # Ust sinir: bir tik cok gecikirse (GUI takilmasi) filtre bir anda
        # zipla-sifirla yapmasin diye 0.5 s'de kirpilir.
        t_now = time.monotonic()
        dt = min(max(t_now - t_prev, 1e-3), 0.5)
        t_prev = t_now

        # Filter the heading BEFORE it is turned into a camera position: this is
        # where the left/right swing comes from (see YAW_TAU above).
        yaw_f = 0.0 if WORLD_FRAME else _lp_angle(yaw_f, yaw, YAW_TAU, dt)
        c, s = math.cos(yaw_f), math.sin(yaw_f)
        # offset in the (filtered) model frame, or the world frame
        tx = x + OFF[0] * c - OFF[1] * s
        ty = y + OFF[0] * s + OFF[1] * c
        tz = z + OFF[2]

        cx = _lp(cx, tx, POS_TAU, dt)
        cy = _lp(cy, ty, POS_TAU, dt)
        cz = _lp(cz, tz, POS_TAU, dt)

        # HIZ ILERI-BESLEMESI (2026-08-29). Konuma alcak-geciren filtre uygulamak
        # titresimi bastirir AMA sabit hizda hiza orantili KALICI geri kalma
        # uretir: hata = POS_TAU * v. Seyirde 13 m/s x 1.2 s = 15.6 m, yani
        # 2.5 m'lik kadrajda arac tamamen cerceve disinda -- "uçak uçtu gitti
        # ben kalkış yerinde kaldım". Filtreyi zayiflatmak titresimi geri
        # getirirdi (bkz. POS_TAU notu), o yuzden gecikmeyi filtreyi bozmadan
        # TELAFI ediyoruz: sabit hizda tam olarak POS_TAU*v kadar ileri bak.
        # Hiz iki poz farkindan kestirilip ayni tau ile suzuluyor -- ham fark
        # ornekleme gurultusunu dogrudan kameraya gecirirdi.
        if x_prev is not None:
            vx_f = _lp(vx_f, (x - x_prev) / dt, POS_TAU, dt)
            vy_f = _lp(vy_f, (y - y_prev) / dt, POS_TAU, dt)
            vz_f = _lp(vz_f, (z - z_prev) / dt, POS_TAU, dt)
        x_prev, y_prev, z_prev = x, y, z

        ffx = cx + POS_TAU * (vx_f or 0.0)
        ffy = cy + POS_TAU * (vy_f or 0.0)
        ffz = cz + POS_TAU * (vz_f or 0.0)

        # aim back at the model
        dx, dy, dz = x - ffx, y - ffy, z - ffz
        cam_yaw = math.atan2(dy, dx)
        cam_pitch = -math.atan2(dz, math.hypot(dx, dy))
        move_camera(ffx, ffy, ffz, cam_yaw, cam_pitch)
        # dt ARTIK OLCULEN gecen sure; onu uyumak periyodu ikiye katlardi.
        # Beklemek istedigimiz sabit tik araligi, RATE_HZ.
        time.sleep(1.0 / RATE_HZ)


if __name__ == "__main__":
    main()
