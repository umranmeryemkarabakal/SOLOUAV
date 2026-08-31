"""Shared helpers for driving/observing the mc_indi_tiltrotor SITL test rig
(gz_tiltrotor_indi airframe, ~/PX4-Autopilot).

There is no MAVLink bridge for the custom tiltrotor_indi_setpoint uORB topic
-- external MAVLink clients (pymavlink/MAVSDK) cannot publish arbitrary uORB
topics into a running PX4 instance. Instead this shells out to the
`px4-mc_indi_tiltrotor test_sp` / `px4-commander` / `px4-listener` client
binaries, which run as lightweight clients of the *same* px4 process (this
is standard for POSIX SITL -- `px4-listener` works the same way). This
mirrors run_hover_gust_test.m / run_transition_test.m's role: a scripted
setpoint timeline driving a closed loop, except the loop here is the real
PX4 flight stack + Gazebo physics instead of the MATLAB plant model.

Requires: `export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH`
and a running gz_tiltrotor_indi SITL instance (see README.md in this
folder for launch instructions).
"""

from __future__ import annotations

import math
import re
import subprocess
import sys
import time
from dataclasses import dataclass, field


class Px4Client:
    def __init__(self, bin_dir: str = None, gz_world: str = "default", gz_model: str = "tiltrotor_indi_0"):
        self.bin_dir = bin_dir
        self.gz_world = gz_world
        self.gz_model = gz_model

    def _bin(self, name: str) -> str:
        if self.bin_dir:
            return f"{self.bin_dir}/px4-{name}"
        return f"px4-{name}"

    def _run(self, name: str, args: list[str], timeout: float = 3.0) -> str:
        cmd = [self._bin(name)] + args
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return result.stdout + result.stderr

    def arm(self, force: bool = True) -> str:
        args = ["arm"] + (["-f"] if force else [])
        return self._run("commander", args)

    def disarm(self, force: bool = True) -> str:
        args = ["disarm"] + (["-f"] if force else [])
        return self._run("commander", args)

    def set_setpoint(self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
                      fx: float = 0.0, z_sp: float = 0.0,
                      leso_enable: tuple[bool, bool, bool] = (True, True, False),
                      pos_hold: bool = False) -> str:
        """One-shot publish of tiltrotor_indi_setpoint via the module's
        `test_sp` custom_command.

        pos_hold (2026-07-29, step 28) turns on the horizontal position loop
        (report item (N)). While it is on the loop OWNS roll/pitch -- the roll/
        pitch arguments are ignored -- and it also supplies fx (the item (P)
        trim), so the fx argument is ignored too. The hold target is captured
        on the false->true edge, so a caller never needs its own coordinates.
        Without it the vehicle drifts away at ~7 m/s and any "hover" test is
        really a cruise test (report step 16).
        """
        # A non-finite setpoint is not a bad test, it is a BRICKED controller:
        # the NaN propagates into e_att -> dtau -> G/nu_des -> du, every actuator
        # command becomes NaN and the vehicle produces no thrust at all (measured
        # 2026-07-29 -- a nan yaw_sp from a silently-dropped quaternion parse
        # cost a whole probe flight, and the px4 log looked like a WLS failure).
        for name, val in (("roll", roll), ("pitch", pitch), ("yaw", yaw),
                          ("fx", fx), ("z_sp", z_sp)):
            if not math.isfinite(val):
                raise ValueError(f"refusing to publish non-finite setpoint: {name}={val}")

        lr, lp, ly = (1 if v else 0 for v in leso_enable)
        args = ["test_sp", f"{roll}", f"{pitch}", f"{yaw}", f"{fx}", f"{z_sp}",
                f"{lr}", f"{lp}", f"{ly}", "1" if pos_hold else "0"]
        return self._run("mc_indi_tiltrotor", args)

    def tiltceil(self, deg: float) -> str:
        """Wing tilt ceiling (deg) -- the allocator box upper bound on delta0/delta1.

        Step 31 / back-transition phase 1. 90 = off (default, behaviourally
        neutral). The tail rotor is deliberately not limited. This is a hard box
        constraint, not an objective term: the tilt runaway at cruise is driven
        by the ALTITUDE channel (Ws_Fz = 20) and cannot be outbid by Fx
        (Ws_Fx = 0.05), so only a constraint can retract the tilts.
        """
        if not math.isfinite(deg):
            raise ValueError(f"refusing non-finite tilt ceiling: {deg}")
        return self._run("mc_indi_tiltrotor", ["tiltceil", f"{deg}"])

    def listener(self, topic: str) -> str:
        return self._run("listener", [topic, "-n", "1"], timeout=3.0)

    def status(self) -> dict:
        return parse_named_floats(self.listener("tiltrotor_indi_status"))

    def local_position(self) -> dict:
        return parse_named_floats(self.listener("vehicle_local_position"))

    def estimator_status_flags(self) -> dict:
        """EKF2's own aiding/fault flags (cs_baro_hgt, cs_gps_hgt, cs_fake_hgt, ...).

        Needed to answer WHICH aiding source is keeping an estimate alive during
        a failsafe injection. Step 34 tried to drop the vertical estimate twice
        by guessing at EKF2_* params and failed both times without ever seeing
        which source was still being fused; sampling this topic turns that into
        an observation. Republished ~1 Hz, so poll it, do not expect a stream.
        """
        return parse_named_floats(self.listener("estimator_status_flags"))

    def attitude_euler_deg(self) -> tuple[float, float, float]:
        """roll, pitch, yaw in degrees, from vehicle_attitude's quaternion."""
        att = parse_named_floats(self.listener("vehicle_attitude"))
        q = att.get("q", None)
        if not q or len(q) != 4:
            return (float("nan"),) * 3
        return quat_to_euler_deg(q)

    def arming_state(self) -> int:
        out = self.listener("vehicle_status")
        m = re.search(r"arming_state:\s*(\d+)", out)
        return int(m.group(1)) if m else -1

    def preflight_check_ok(self) -> bool:
        """Cross-check against commander's own health/arming checks (independent
        of mc_indi_tiltrotor's internal ekf_healthy gate) -- see M7."""
        out = self._run("commander", ["check"], timeout=5.0)
        return "Preflight check: OK" in out

    def ekf2_status(self) -> str:
        return self._run("ekf2", ["status"], timeout=5.0)

    # --- estimate-degradation injection (blocker B1 / step 32 failsafe test) ---
    # The graded failsafe branches on INPUT VALIDITY, not on its own reported
    # level (see the FsLevel note in TiltrotorIndiParams.hpp), so a hook that
    # merely forced _fs_level would leave the branch bodies unexercised: the
    # altitude branch tests alt_ok, the position loop tests pos_ok. These
    # injections therefore take the ESTIMATE away for real, which is the only
    # way to test detection and response together -- and needs no rebuild.
    def param_set(self, name: str, value) -> str:
        return self._run("param", ["set", name, str(value)])

    def param_save(self) -> str:
        """Flush params to rootfs/parameters.bson NOW.

        PX4's autosave is delayed, so a `param set` followed promptly by
        kill_sitl() never reaches disk. That is fine for an injection (it is
        meant to be temporary) but fatal for the RESTORE at the end of a probe:
        measured 2026-07-30 (step 36) -- the restore ran, the process died, and
        the file kept the injected values, so the NEXT SITL refused to boot with
        `Preflight Fail: Yaw estimate error`. Always param_save() + sleep before
        killing, when the point was to leave the environment clean.
        """
        return self._run("param", ["save"], timeout=5.0)

    def param_get(self, name: str) -> float:
        out = self._run("param", ["show", name])
        m = re.search(re.escape(name) + r"\s*\[\d+,\d+\]\s*:\s*(-?[\d.]+)", out)
        return float(m.group(1)) if m else float("nan")

    def stop_module(self, name: str) -> str:
        """Stop a PX4 module. `ekf2` is the interesting one: it is the sole
        publisher of vehicle_attitude AND vehicle_local_position, so stopping it
        strands every outer loop at once while vehicle_angular_velocity -- the
        topic that SCHEDULES the module's Run() -- keeps ticking.

        That asymmetry is what the graded failsafe was built to exploit. Step 35
        established its LIMIT: it holds for the altitude and position loops, but
        not for attitude, because rate damping alone was measured not to fly
        (run_rate_only_test.m). So stopping ekf2 now tests the opposite thing --
        that attitude loss produces a prompt, logged CUT rather than one tick of
        a flight mode the airframe cannot fly."""
        return self._run(name, ["stop"], timeout=5.0)

    def lockdown(self, on: bool = True) -> str:
        """commander lockdown -> actuator_armed.lockdown, i.e. the COMMANDED
        kill that terminationCommanded() still answers with NaN out. Cutting
        the motors here is the correct response, not a regression."""
        return self._run("commander", ["lockdown", "on" if on else "off"], timeout=5.0)

    def gz_truth_z(self) -> float:
        """Ground-truth altitude (m, up-positive) straight from Gazebo.

        Needed because every PX4 altitude signal flows through the estimator we
        are deliberately breaking: once ekf2 is stopped, vehicle_local_position
        stops updating and the ulog's last z is frozen at the moment of the
        injection. Without this, what the vehicle ACTUALLY does after an
        estimator failure is unobservable -- which is how step 34's crash first
        passed unnoticed.
        """
        try:
            out = subprocess.run(["gz", "model", "-m", self.gz_model, "-p"],
                                 capture_output=True, text=True, timeout=5.0).stdout
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return float("nan")
        # `gz model -p` prints a [x y z] triple then r p y on the following line.
        m = re.search(r"\[(-?[\d.eE+-]+)\s+(-?[\d.eE+-]+)\s+(-?[\d.eE+-]+)\]", out)
        return float(m.group(3)) if m else float("nan")

    # --- Gazebo disturbance injection ---
    # ApplyLinkWrench is topic-driven, not service-driven (both `default.sdf`
    # and `windy.sdf` load gz-sim-apply-link-wrench-system). Empirically
    # (verified live): the entity must be the *model* (type: MODEL), not a
    # link -- a link-scoped EntityWrench silently no-ops, model-scoped
    # applies immediately. `/wrench/persistent` holds the last value every
    # physics step until `/wrench/clear`; `/wrench` alone is a single
    # one-step impulse (negligible at world dt=4ms unless huge).
    def set_persistent_torque(self, tx: float = 0.0, ty: float = 0.0, tz: float = 0.0) -> None:
        entity = f'entity: {{name: "{self.gz_model}", type: MODEL}}'
        wrench = f'wrench: {{torque: {{x: {tx}, y: {ty}, z: {tz}}}}}'
        cmd = ["gz", "topic", "-t", f"/world/{self.gz_world}/wrench/persistent",
               "--msgtype", "gz.msgs.EntityWrench", "-p", f"{entity}, {wrench}"]
        subprocess.run(cmd, capture_output=True, text=True, timeout=3.0)

    def clear_wrench(self) -> None:
        entity = f'entity: {{name: "{self.gz_model}", type: MODEL}}'
        cmd = ["gz", "topic", "-t", f"/world/{self.gz_world}/wrench/clear",
               "--msgtype", "gz.msgs.EntityWrench", "-p", entity]
        subprocess.run(cmd, capture_output=True, text=True, timeout=3.0)


def quat_to_euler_deg(q: list[float]) -> tuple[float, float, float]:
    """[w,x,y,z] (Hamilton, body->earth, PX4 convention) -> (roll,pitch,yaw) deg."""
    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


_FLOAT_RE = re.compile(r"^\s*([a-zA-Z_][a-zA-Z0-9_]*):\s*(-?\d+\.?\d*(?:[eE][+-]?\d+)?)\s*(?:\(.*\))?\s*$")
# The trailing `(...)` is px4-listener's human-readable annotation, e.g.
#   q: [0.705, 0.001, -0.002, 0.709] (Roll: -0.0 deg, Pitch: -0.2 deg, Yaw: 90.3 deg)
# It must be optional here or the whole field is silently DROPPED -- which is
# what happened to `q` (and therefore to every attitude_euler_deg() call, which
# returned nan). _FLOAT_RE below already tolerated it; this one did not.
# Found 2026-07-29: a nan yaw_sp published from that nan poisoned the entire
# controller (nu_des/G/du all nan, no thrust, vehicle never left the ground).
_ARRAY_RE = re.compile(r"^\s*([a-zA-Z_][a-zA-Z0-9_]*):\s*\[(.*?)\]\s*(?:\(.*\))?\s*$")
_BOOL_RE = re.compile(r"^\s*([a-zA-Z_][a-zA-Z0-9_]*):\s*(True|False)\s*$")


def parse_named_floats(text: str) -> dict:
    """Best-effort parse of px4-listener's `field: value` text dump into a
    dict of floats / lists of floats / bools. Good enough for polling
    tiltrotor_indi_status / vehicle_local_position during a test run --
    not a general uORB deserializer."""
    out: dict = {}
    for line in text.splitlines():
        m = _ARRAY_RE.match(line)
        if m:
            name, body = m.group(1), m.group(2)
            vals = []
            for tok in body.split(","):
                tok = tok.strip()
                if tok in ("True", "False"):
                    vals.append(tok == "True")
                else:
                    try:
                        vals.append(float(tok))
                    except ValueError:
                        vals.append(tok)
            out[name] = vals
            continue
        m = _BOOL_RE.match(line)
        if m:
            out[m.group(1)] = (m.group(2) == "True")
            continue
        m = _FLOAT_RE.match(line)
        if m:
            out[m.group(1)] = float(m.group(2))
    return out


@dataclass
class ScenarioEvent:
    t: float  # seconds since scenario start
    action: str  # human-readable label, for logging
    fn_name: str = ""


def wait_until(predicate, timeout: float, poll_interval: float = 0.2) -> bool:
    """Poll predicate() until it returns truthy or timeout elapses."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(poll_interval)
    return False


PX4_ROOT = "/home/umran/PX4-Autopilot"
GZ_BRIDGE_DIR = f"{PX4_ROOT}/build/px4_sitl_default/src/modules/simulation/gz_bridge"
PX4_BIN = f"{PX4_ROOT}/build/px4_sitl_default/bin/px4"
SOCK = "/tmp/px4-sock-0"


def kill_sitl() -> None:
    """Best-effort teardown of any running px4/gz sim instance (mirrors the
    pkill/rm sequence in docs/gz_tiltrotor/RUNBOOK.md)."""
    for pattern in ("bin/px4 -d", "gz sim"):
        subprocess.run(["pkill", "-9", "-f", pattern], capture_output=True)
    time.sleep(1.0)
    try:
        import os
        if os.path.exists(SOCK):
            os.remove(SOCK)
    except OSError:
        pass


def gui_enabled() -> bool:
    """True when the caller asked for a visible Gazebo window (INDI_SITL_GUI=1)."""
    import os
    return os.environ.get("INDI_SITL_GUI", "0") not in ("", "0", "no", "false")


def follow_model(model: str = "tiltrotor_indi_0") -> None:
    """Lock the GUI camera onto the model. MANDATORY in GUI runs: this vehicle
    drifts (item (N)) and accelerates to cruise, so without follow it leaves the
    view within seconds and the window shows nothing. No-op if the /gui/follow
    service is absent (headless run) -- never fails a flight for a camera.

    The view defaults to `front` (2026-08-03): a 45 deg front-quarter shot, so
    the TILT ANGLE is readable while watching. The old `chase` view sat directly
    behind the vehicle, where the nacelles are edge-on and the whole manoeuvre
    this project spends its flights on -- the ceiling ramping 45 -> 9 -> 20 deg --
    is invisible. Override per run with INDI_GZ_CAM=front|nose|side|chase, or
    give it a literal "x y z" offset in the gz FLU model frame (+x fwd, +y LEFT).
    `side` reads the angle most directly; `nose` is the cleanest framing but
    cannot separate intermediate angles. See gz_follow.sh."""
    import os
    script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gz_follow.sh")
    view = (os.environ.get("INDI_GZ_CAM") or "nose").split()
    try:
        r = subprocess.run(["bash", script, model, *view], capture_output=True, text=True, timeout=40.0)
        print(("  " + r.stdout.strip()) if r.returncode == 0 else
              f"  UYARI: kamera kilitlenemedi ({r.stderr.strip()[:120]})")
    except (OSError, subprocess.TimeoutExpired) as e:
        print(f"  UYARI: gz_follow calistirilamadi: {e}")


def setup_gui_view(model: str = "tiltrotor_indi_0") -> None:
    """GUI-only: give the flight a background to happen against, and a camera
    that actually closes distance. Best-effort throughout -- a missing backdrop
    must never fail a flight, so every step swallows its own errors.

    WHY (2026-08-28). A GUI run was unreadable: the mission's touchdown happens
    578 m east of the launch point, but the stock ground VISUAL is only
    100x100 m (worlds/default.sdf:190-196), so the landing occurred over blank
    background with nothing to descend against. On top of that the camera never
    closes distance -- /gui/follow/offset reports success and does nothing in
    this gz build, leaving the vehicle ~4 px wide (see gz_chase_cam.py).

    Two pieces, both already written and reused here rather than reinvented:
    gz_scene.py spawns visual-only static geometry, gz_chase_cam.py drives the
    camera by absolute pose at 10 Hz.

    The chase cam REPLACES follow_model(): both steer the same camera, so
    running them together makes them fight. Set INDI_GZ_CHASE=0 to fall back to
    the old /gui/follow path. The chase cam exits on its own once the model
    pose stops arriving, so kill_sitl() needs no extra bookkeeping."""
    import os
    here = os.path.dirname(os.path.abspath(__file__))

    # INDI_GZ_SCENE=0 ile kapatilabilir. Kapatma anahtari bir kolaylik degil,
    # BIR OLCUM ARACI: kilitlenme GUI kosularinda 1/3 tekrarliyor, headless'ta
    # hic olmuyor; sahnenin render yukunun bu farka katkisi olup olmadigini
    # ayirt etmenin tek yolu sahneyi kapatip ayni testi kosmak (2026-08-28).
    if os.environ.get("INDI_GZ_SCENE", "1") not in ("", "0", "no", "false"):
        try:
            r = subprocess.run([sys.executable, os.path.join(here, "gz_scene.py")],
                               capture_output=True, text=True, timeout=40.0)
            print(r.stdout.rstrip() or "  gorsel referans: cikti yok")
        except (OSError, subprocess.TimeoutExpired) as e:
            print(f"  UYARI: gorsel referans eklenemedi: {e}")
    else:
        print("  gorsel referans: KAPALI (INDI_GZ_SCENE=0)")

    # VARSAYILAN ACIK, ve bu bir konfor tercihi DEGIL. Stok /gui/follow hedefi
    # her karede kadrajin tam ortasinda tutar (olculdu, bkz. gz_chase_cam.py).
    # Bu govde seyirde 5 Hz ustunde titriyor -- pitch enerjisinin %37-40'i orada,
    # tepe-tepe 1.1 rad/s, ve kaynagi belgeli: motorlari hizla cevirmenin tepki
    # torku (0.542 Nm, aktif momentin %115'i, WLS raporu Adim 94). Kamera bu
    # titremeyi bire bir takip edince ARAC ekranda sabit durur ve DUNYA onun
    # etrafinda sallanir -- "sanki dünya komple titriyor gibi" (2026-08-29).
    # Chase cam'in yaw/konum alcak-geciren filtreleri titremeyi kameraya
    # gecirmez. INDI_GZ_CHASE=0 ile stok follow'a donulur (dunya yine sallanir).
    if os.environ.get("INDI_GZ_CHASE", "1") in ("", "0", "no", "false"):
        follow_model(model)
        return

    # 18 m out, 6 m up: keeps the whole airframe and the ground it is descending
    # towards in one frame. Chosen by watching (2026-08-28) -- a 6 m arm shows
    # the nacelles larger but loses the ground reference, which is exactly what
    # this work set out to restore. For a close look at the nacelle angle set
    # INDI_GZ_CAM="0 -6 2"; the value is "x y z" in the gz FLU model frame
    # (+x fwd, +y LEFT), so any framing is one variable away.
    view = (os.environ.get("INDI_GZ_CAM") or "1 -4 1").split()

    # YERLI TAKIP, /gui/track konusu (2026-08-29). Chase cam'in tamamini
    # gereksiz kilar ve uc yil suren "kamera yetismiyor" hikayesini bitirir.
    #
    # NEDEN CHASE CAM YETMIYORDU: /gui/move_to/pose ISINLAMAZ, ANIMASYON yapar.
    # 10 Hz'de mutlak poz komut edince her komut yeni bir animasyon baslatiyor
    # ve kamera hicbirine varamiyor; sonuc, hedefi ne olursa olsun sabit bir
    # hiz tavani. Olculdu: kamera 6.6 m/s, arac 11-15 m/s, seyirde geride kalma
    # 335 m'ye ulasti ("uçak ilerledi ve ben kaldım"). Bu bir ayar sorunu
    # degil, o API'nin mimari siniri -- dt ve ileri-besleme duzeltmeleri
    # (gz_chase_cam.py) filtreyi duzeltti ama bu tavani kaldiramaz.
    #
    # gz.msgs.CameraTrack follow_target + follow_offset + follow_pgain'i TEK
    # mesajda tasir. Eski yolun kirik parcasi olan ayri /gui/follow/offset
    # servisi (basarili der, hicbir sey yapmaz) devreden cikar. Takibi gz
    # kendisi render hizinda yapar: tik basina surec yok, zombi yok, gecikme
    # yok. Canli olculdu -- geride kalma 335 m -> 41 m -> offset'e oturdu,
    # kamera-arac mesafesi 5.88 m (hedef 5.6 m).
    #
    # Varsayilan gorus ARKA-CAPRAZ (-5 arkada, 2 sagda, 1.5 yukarida): gidis
    # yonune bakar. Onceki yan gorus (0 -18 6) araci gidis yonune degil alanin
    # sagina baktiriyordu -- kullanici gozlemi, 2026-08-29.
    off = {"x": 0.0, "y": 0.0, "z": 0.0}
    try:
        off["x"], off["y"], off["z"] = (float(v) for v in view[:3])
    except (ValueError, IndexError):
        print(f"  UYARI: INDI_GZ_CAM okunamadi ({view!r}) -- varsayilan kullaniliyor")
        off = {"x": 1.0, "y": -4.0, "z": 1.0}

    # track_mode 4 = FOLLOW_LOOK_AT: kamera offset'i korurken HER ZAMAN araca
    # nisan alir. Duz FOLLOW (2) ile aracin onune gecen bir offset araca degil
    # baska yone bakabiliyor.
    #
    # pgain 1.0 -> 5.0 (2026-08-29, olculdu). gui.config'de follow_pgain zaten
    # 1.0'di, yani kazanc "varsayilan 0.01" sorunu COZULMUSTU; buna ragmen
    # takibin kendi yumusatmasi kaliyordu: hizlanirken geride, yavaslarken
    # ONDE. Seyirden frene gecerken kamera araci 37.7 m GECMISTI. pgain 5 ile
    # ayni anda mesafe 4.6 m (hedef 4.2). NOT: bu olcum inis fazinda, dusuk
    # hizda alindi -- 15 m/s seyirde davranisi ayrica dogrulanmali.
    req = (f'track_mode: 4, follow_target: {{name: "{model}"}}, '
           f'track_target: {{name: "{model}"}}, '
           f'follow_offset: {{x: {off["x"]} y: {off["y"]} z: {off["z"]}}}, '
           f'follow_pgain: 5.0, track_pgain: 5.0')
    try:
        r = subprocess.run(["gz", "topic", "-t", "/gui/track",
                            "-m", "gz.msgs.CameraTrack", "-p", req],
                           capture_output=True, text=True, timeout=20.0)
        if r.returncode == 0:
            print(f"  yerli takip: offset {off['x']} {off['y']} {off['z']} (gz FLU)")
            return
        print(f"  UYARI: /gui/track reddetti ({r.stderr.strip()[:120]})")
    except (OSError, subprocess.TimeoutExpired) as e:
        print(f"  UYARI: /gui/track yayinlanamadi: {e}")

    # Yedek: eski chase cam. Yerli takibin olmadigi bir gz surumunde hala
    # calisir (yavas, ama hic kameradan iyidir).
    try:
        subprocess.Popen([sys.executable, os.path.join(here, "gz_chase_cam.py"), model, *view],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"  chase cam (yedek): offset {' '.join(view)} (gz FLU)")
    except OSError as e:
        print(f"  UYARI: chase cam baslatilamadi ({e}) -- /gui/follow'a donuluyor")
        follow_model(model)


def launch_sitl(model: str, log_path: str, world: str = "default", timeout: float = 90.0,
                attempts: int = 3) -> subprocess.Popen:
    """Launch gz_tiltrotor_indi (or any gz_* model) and wait for
    'Ready for takeoff'. Caller must eventually call kill_sitl().

    Headless by default. Set INDI_SITL_GUI=1 to get the Gazebo window on
    DISPLAY (default :1) and the camera locked onto the model; the flight
    itself is identical, only rendering is added. Two consequences that are
    handled here: the GUI's first render makes the cold start slower (the
    ready timeout is raised), and the model-spawn race the retry loop below
    exists for gets MORE likely, not less.

    RETRIES ON PURPOSE (2026-07-29). A COLD gz start regularly loses the race
    for the model-spawn service and dies with

        ERROR [gz_bridge] Service call timed out. Check GZ_SIM_RESOURCE_PATH...
        ERROR [init] gz_bridge failed to start and spawn model

    while leaving an ORPHAN `gz sim` behind that poisons the next attempt --
    which is why each retry starts with kill_sitl() rather than reusing the
    process. Observed 3 times in a row before one succeeded, then the very next
    launch worked first try, so it is a race and not a broken environment. The
    old 25 s single-shot default turned that race into a spurious test failure;
    the failure mode being retried here is a LAUNCH failure, never a flight
    result, so retrying cannot mask a control regression.
    """
    import os

    last_tail = ""
    gui = gui_enabled()
    if gui:
        timeout = max(timeout, 150.0)

    for attempt in range(1, attempts + 1):
        kill_sitl()
        log = open(log_path, "w")
        env = {"PX4_SIM_MODEL": model, "PX4_GZ_WORLD": world}
        if gui:
            env["DISPLAY"] = os.environ.get("DISPLAY") or ":1"
            # GPU'YA TASI (2026-08-29). Bu makinede bir RTX 4070 var ama PRIME
            # "on-demand" modunda, yani uygulamalar acikca istemedikce Intel
            # iGPU'da kosuyor: gz once EGL'i deniyor, `libEGL warning: egl:
            # failed to create dri2 screen` ile dusuyor ve YAZILIMSAL render'a
            # geriliyor -- tek basina ~%250 CPU. Bu yalnizca bir hiz sorunu
            # degil: kilitlenme SADECE GUI kosularinda tekrarliyor (9 kosumda
            # 3, headless 6 kosumda 0) ve govde dusuk hizda yaw'da marjinal
            # kararli, yani render yukunun urettigi zamanlama sarsintisi
            # dengeyi devirebiliyor. Uc degisken EGL uyarisini sifirliyor.
            env.setdefault("__NV_PRIME_RENDER_OFFLOAD", "1")
            env.setdefault("__GLX_VENDOR_LIBRARY_NAME", "nvidia")
            env.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES",
                           "/usr/share/glvnd/egl_vendor.d/10_nvidia.json")
        else:
            env["HEADLESS"] = "1"
        full_env = dict(os.environ)
        full_env.update(env)
        proc = subprocess.Popen([PX4_BIN, "-d"], cwd=GZ_BRIDGE_DIR, stdout=log, stderr=subprocess.STDOUT,
                                env=full_env)

        def ready():
            try:
                with open(log_path) as f:
                    return "Ready for takeoff" in f.read()
            except FileNotFoundError:
                return False

        def spawn_failed():
            try:
                with open(log_path) as f:
                    return "failed to start and spawn model" in f.read()
            except FileNotFoundError:
                return False

        if wait_until(lambda: ready() or spawn_failed(), timeout=timeout, poll_interval=0.5) and ready():
            if gui:
                setup_gui_view()
            return proc

        try:
            with open(log_path) as f:
                last_tail = "".join(f.readlines()[-5:])
        except OSError:
            pass

        print(f"  SITL launch attempt {attempt}/{attempts} failed, retrying...")

    kill_sitl()
    raise RuntimeError(f"SITL did not reach 'Ready for takeoff' in {attempts} attempts "
                       f"({timeout}s each) -- see {log_path}\n{last_tail}")
