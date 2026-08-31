#!/usr/bin/env python3
"""
gz <-> PX4 HITL koprusu: Gazebo'nun sensorlerini MAVLink HIL mesajlarina
cevirip SERI PORT uzerinden gercek karta gonderir, kartin aktuator
ciktilarini geri alip Gazebo'ya uygular.

NEDEN BU DOSYA VAR (2026-08-31, adim 140)
-----------------------------------------
PX4'un HITL yolu MAVLink HIL mesajlarina dayanir ve simulatorun bunlari
SERI/USB uzerinden karta gondermesini bekler. Gazebo Classic bunu hazir yapar;
BIZIM kullandigimiz gz (Harmonic) YAPAMAZ: `gz_bridge` bir PX4 MODULUDUR ve
veriyi `gz::transport` ile alir, yani otopilotun USTUNDE kosar. Gercek kartta
o modul yoktur ve gz-transport seri hat uzerinden konusamaz. Olculdu:
`Tools/simulation/gz/` ve `gz_bridge/` icinde `hil_mode`/`serialEnabled`/
`HIL_SENSOR` gecen TEK satir yok.

Classic'e gecmek DEGERLENDIRILDI VE ELENDI: bu deponun butun fizik olcumleri
(Adim 94'un tilt reaksiyon torku M_TILT = 0.01684, Adim 11'in itki eslemesi,
Adim 12'nin km isareti) gz motorundan turedi. Classic farkli bir motordur ve
hepsinin yeniden dogrulanmasi gerekirdi. Bu kopru fizigi DEGISTIRMEZ -- ayni
gz dunyasi, ayni model, ayni olculmus sabitler; yalnizca veri yolu degisir.

DOGRULAMA DURUMU -- her satir isaretli
  [OLCULDU]  SITL'de dogrulandi (bu dosyanin --dry-run modu).
             2026-08-31 kosumu: IMU 250 Hz, baro 50 Hz, GPS 30 Hz, 3/3 abone.
  [YAZILDI]  kod var, GERCEK KARTTA HIC KOSMADI
  [EKSIK]    yapilmadi; ne gerektigi yazili

⚠ KART ELDE OLMADAN "HAZIR" DENEMEZ. Bu projenin tekrar odedigi bedel tam
budur (Adim 110: olculmeden yazilan land-detector kapisi hic tetiklenmedi;
Adim 84b: SURF_ENABLE bayragi yuzunden butun FW yasasi aylarca sessizce
atildi). Bu yuzden asagida tahmin edilen HICBIR sey yok -- konu adlari, mesaj
tipleri ve alan adlari CALISAN BIR SITL'den okundu.

⚠ PROTOBUF: gz'nin python baglantilari eski protoc ile uretilmis; sistemdeki
yeni protobuf onlari reddediyor ("Descriptors cannot be created directly").
Cozum, saf-python ayristiriciyi zorlamak:
    export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
run_hitl_check.sh bunu zaten ayarliyor.

KULLANIM
    # kart YOK -- gz tarafini dogrula (SITL kosarken):
    ./run_hitl_check.sh
    PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 gz_hil_bridge.py --dry-run --duration 10
    # kart VAR:
    python3 gz_hil_bridge.py --dev /dev/ttyACM0 --baud 921600
"""
import argparse
import math
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

try:
    from gz.transport13 import Node
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.fluid_pressure_pb2 import FluidPressure
    from gz.msgs10.navsat_pb2 import NavSat
    from gz.msgs10.actuators_pb2 import Actuators
    from gz.msgs10.double_pb2 import Double
    GZ_OK, _GZ_ERR = True, None
except ImportError as e:                       # pragma: no cover
    GZ_OK, _GZ_ERR = False, e

# --- HIL_SENSOR fields_updated maskesi ---
# Kaynak: PX4 mavlink_receiver.cpp SensorSource. Bit setlenmezse PX4 o alani
# YOK SAYAR -- eksik bir bit, sessizce eksik bir sensor demektir.
BIT_ACCEL = 0b0000000000111
BIT_GYRO  = 0b0000000111000
BIT_MAG   = 0b0000111000000
BIT_BARO  = 0b0001101000000

WORLD, MODEL = 'default', 'tiltrotor_indi_0'


class GzHilBridge:
    def __init__(self, dev, baud, world=WORLD, model=MODEL, dry_run=False):
        self.world, self.model, self.dry = world, model, dry_run
        self.t0 = time.time()
        self.imu = self.baro = self.gps = None
        self.n = {'imu': 0, 'baro': 0, 'gps': 0, 'sent': 0, 'act': 0}

        if not GZ_OK:
            sys.exit(f"gz python baglantilari yok: {_GZ_ERR}")
        self.node = Node()

        self.mav = None
        if not dry_run:
            if mavutil is None:
                sys.exit("pymavlink yok: pip install pymavlink")
            self.mav = mavutil.mavlink_connection(dev, baud=baud, source_system=1)
            print(f"[bridge] seri: {dev} @ {baud}")

        # [OLCULDU] Aktuator yayincilari. Konu adlari ve mesaj tipleri calisan
        # bir SITL'den okundu (gz topic -i):
        #   /model/<m>/command/motor_speed  -> gz.msgs.Actuators
        #   /model/<m>/servo_<0..7>         -> gz.msgs.Double
        #
        # ⚠ DRY-RUN'DA GORULEN TUZAK (2026-08-31): bu advertise cagrilari,
        # kart olmasa bile gz'de KONU YARATIR. Ilk dogrulama kosumunda
        # `gz topic -l` ciktisinda hem /model/<m>/command/motor_speed (gercek,
        # gz_bridge'inki) hem /<m>/command/motor_speed (benimki) gorundu --
        # yani bir an yanlis konuya yayin yaptigim sanildi. Yanilti degil,
        # ama ayirt edilmesi gerekiyor: gercek olan /model/ ON EKLI olandir.
        # Kart baglandiginda yayinin GERCEKTEN gz_bridge'in dinledigi konuya
        # dustugu `gz topic -e` ile dogrulanmali.
        self.pub_motor = self.node.advertise(
            f"/model/{model}/command/motor_speed", Actuators)
        self.pub_servo = [
            self.node.advertise(f"/model/{model}/servo_{i}", Double)
            for i in range(8)
        ]

    # ---------------- gz -> ic durum ----------------
    def _on_imu(self, msg):
        self.imu = msg
        self.n['imu'] += 1

    def _on_baro(self, msg):
        self.baro = msg
        self.n['baro'] += 1

    def _on_gps(self, msg):
        self.gps = msg
        self.n['gps'] += 1

    def subscribe(self):
        """[OLCULDU] Konu adlari calisan SITL'den alindi (gz topic -l).

        ⚠ MAGNETOMETRE KONUSU YOK, ve bu bir eksiklik degil bir OLGU: SITL'de
        manyetometreyi Gazebo degil PX4'un KENDI `sensor_mag_sim` modulu
        uretiyor (airframe: SENS_EN_MAGSIM 1). Gercek kartta o modul
        CALISMAYACAK, cunku kart gercek bir manyetometre tasiyor -- ama HITL'de
        gercek manyetometre SITL'in dunyasindan habersizdir ve yanlis bir
        heading verir. Bu, kart baglandiginda cozulmesi gereken ILK
        sorulardan biridir; bkz. asagidaki [EKSIK] notu."""
        base = (f"/world/{self.world}/model/{self.model}"
                f"/link/base_link/sensor")
        subs = [
            (f"{base}/imu_sensor/imu", IMU, self._on_imu),
            (f"{base}/air_pressure_sensor/air_pressure", FluidPressure, self._on_baro),
            (f"{base}/navsat_sensor/navsat", NavSat, self._on_gps),
        ]
        ok = 0
        for topic, typ, cb in subs:
            if self.node.subscribe(typ, topic, cb):
                ok += 1
            else:
                print(f"[bridge] ⚠ abone olunamadi: {topic}")
        print(f"[bridge] {ok}/{len(subs)} konuya abone")
        return ok

    # ---------------- ic durum -> MAVLink ----------------
    def send_hil_sensor(self):
        """[YAZILDI] Kartta HIC kosmadi.

        ⚠ CERCEVE DONUSUMU: gz FLU (x-on, y-SOL, z-YUKARI) -> PX4 FRD
        (x-on, y-SAG, z-ASAGI), yani (x, -y, -z). Bu projenin en pahali
        hatalarindan ikisi (Adim 12 ROTOR_KM isareti, engelleyici B4 ROTOR_PY)
        TAM OLARAK bu donusumun atlanmasindan dogdu. Burada atlanirsa kart ters
        isaretli ivme gorur ve derhal devrilir."""
        if self.imu is None or self.mav is None:
            return False
        us = int((time.time() - self.t0) * 1e6)
        a, g = self.imu.linear_acceleration, self.imu.angular_velocity
        fields = BIT_ACCEL | BIT_GYRO
        pres = 0.0
        if self.baro is not None:
            pres = self.baro.pressure / 100.0      # Pa -> hPa
            fields |= BIT_BARO
        self.mav.mav.hil_sensor_send(
            us,
            a.x, -a.y, -a.z,                       # FLU -> FRD
            g.x, -g.y, -g.z,
            0.0, 0.0, 0.0,                         # mag: bkz. subscribe() notu
            pres, 0.0, 0.0, 15.0,
            fields, 0)
        self.n['sent'] += 1
        return True

    def send_hil_gps(self):
        """[YAZILDI] Kartta HIC kosmadi."""
        if self.gps is None or self.mav is None:
            return False
        us = int((time.time() - self.t0) * 1e6)
        g = self.gps
        vn, ve, vu = g.velocity_north, g.velocity_east, g.velocity_up
        self.mav.mav.hil_gps_send(
            us, 3,
            int(g.latitude_deg * 1e7), int(g.longitude_deg * 1e7),
            int(g.altitude * 1e3),
            100, 100,
            int(math.hypot(ve, vn) * 100),
            int(vn * 100), int(ve * 100), int(-vu * 100),   # up -> down
            65535, 12)
        return True

    # ---------------- kart -> gz ----------------
    def pump_actuators(self):
        """[EKSIK] Kartin HIL_ACTUATOR_CONTROLS mesajini gz'ye uygulamak.

        YAYIN TARAFI HAZIR VE OLCULDU (bkz. __init__): konu adlari ve mesaj
        tipleri calisan SITL'den dogrulandi. Eksik olan, MAVLink'ten gelen
        `controls[16]` dizisinin bu iki konuya DOGRU esleneceginin
        dogrulanmasi:
          controls[0..2]  -> motor_speed (Actuators.velocity, rad/s)
          controls[3..10] -> servo_0..7  (Double, rad)
        ⚠ OLCEK BILINMIYOR: PX4 normalize (0..1) gonderir; gz motor modeli
        rad/s bekler (SIM_GZ_EC_MIN/MAX = 10/1500). Bu donusum
        thrustToNormalized()'in tersidir ve YANLIS OLCEK, dogrudan yanlis itki
        demektir -- Adim 11'in tam olarak odedigi bedel.
        Bu yuzden TAHMINLE YAZILMADI: kart baglandiginda ilk is, karttan gelen
        gercek `controls` degerlerini LOGLAYIP olcegi olcmektir."""
        raise NotImplementedError(
            "pump_actuators: kart baglandiginda olculecek (bkz. dosya notu)")

    # ---------------- ana dongu ----------------
    def run(self, rate_hz=250.0, duration=None):
        self.subscribe()
        mode = 'DRY-RUN (kart yok, yalnizca gz okunur)' if self.dry else 'CANLI'
        print(f"[bridge] {mode} @ {rate_hz:.0f} Hz")
        dt = 1.0 / rate_hz
        t_start = t_report = time.time()
        try:
            while True:
                if duration and time.time() - t_start > duration:
                    break
                if not self.dry:
                    self.send_hil_sensor()
                    if self.n['sent'] % 25 == 0:       # GPS ~10 Hz
                        self.send_hil_gps()
                if time.time() - t_report >= 2.0:
                    t_report = time.time()
                    print(f"  imu={self.n['imu']:6d} baro={self.n['baro']:5d} "
                          f"gps={self.n['gps']:4d} gonderilen={self.n['sent']:6d}")
                time.sleep(dt)
        except KeyboardInterrupt:
            pass
        el = time.time() - t_start
        print(f"\n[bridge] {el:.1f} s. imu={self.n['imu']} "
              f"({self.n['imu']/max(el,1e-9):.0f} Hz)  "
              f"baro={self.n['baro']}  gps={self.n['gps']}  "
              f"gonderilen={self.n['sent']}")
        # DRY-RUN OLCUTU: IMU gercekten akiyor mu ve hizi yeterli mi?
        # PX4'un HIL_SENSOR'u en az ~200 Hz bekler (EKF2 tick'i).
        hz = self.n['imu'] / max(el, 1e-9)
        if self.dry:
            print(f"[dry-run] IMU {hz:.0f} Hz  "
                  f"{'OK (>=200 Hz)' if hz >= 200 else '⚠ DUSUK -- HITL icin yetersiz'}")
        return self.n['imu'] > 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dev', default='/dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--world', default=WORLD)
    ap.add_argument('--model', default=MODEL)
    ap.add_argument('--rate', type=float, default=250.0)
    ap.add_argument('--duration', type=float, default=None)
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()
    b = GzHilBridge(a.dev, a.baud, a.world, a.model, a.dry_run)
    sys.exit(0 if b.run(a.rate, a.duration) else 1)


if __name__ == '__main__':
    main()
