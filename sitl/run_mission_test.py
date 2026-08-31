#!/usr/bin/env python3
"""TAM OTONOM GOREV testi: kalkis MC -> seyir (tilt) -> geri gecis -> inis MC.

Adim 42 (2026-08-03). Gereksinim: profilin tamami otonom olmali; pilot bu
profilin surucusu degil, yalnizca mudahale yolu.

BU TESTIN ONCEKILERDEN FARKI: hicbir evre ELLE surulmuyor.
`run_transition_test.py` ileri gecisi bir `fx` RAMPASI gondererek yapiyordu --
yani manevrayi test betigi ucuruyordu, arac degil. Burada betik yalnizca IKI
BAYRAK kaldirip indiriyor (`ft_enable`, `bt_enable`); rampayi, tilt secimini,
emniyetleri ve devri aracin kendi durum makineleri yurutuyor. Bir yetenegin var
olmasi ile ona otonom erisilebilmesi ayri seylerdir ve bu testin olctugu ikincisi.

DIZI
  1) arm + tirmanis (z_sp), pos_hold ile hover
  2) ft_enable = 1  -> RAMP -> CRUISE  (fx'i MAKINE rampalar, tilti WLS secer)
  3) seyirde bekle
  4) ft_enable = 0, bt_enable = 1 -> RETRACT -> BRAKE -> HANDOFF -> pos_hold
  5) kademeli inis, yerde disarm

GECME OLCUTLERI
  1. ft_state CRUISE'a (2) ulasti, IPTAL yok
  2. seyirde kanat tilti >= 30 deg VE v_h >= FT_CRUISE_V (gercekten sabit
     kanat rejimine gecti mi -- "drone konseptiyle ucuyor" olmamali)
  3. ileri gecis boyunca irtifa sapmasi <= FT_ALT_BAND
  4. bt_state HANDOFF'a (3) ulasti ve pos_hold DEVRALDI
  5. son v_h < 1.0 m/s
  6. havada itki doyumu %0 ve BIG_M = 0

Kullanim:
    export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH
    INDI_SITL_GUI=1 INDI_GZ_CAM=side python3 run_mission_test.py
"""

from __future__ import annotations

import math
import os
import shutil
import sys
import time

import numpy as np
from pyulog import ULog

import indi_sitl_common as sc

# RUZGARLI KOSUM (2026-08-31, adim 135): INDI_WORLD=windy_tiltrotor
# Varsayilan "default" -- bu depodaki butun esikler SIFIR RUZGARDA kalibre
# edildi, dolayisiyla ruzgar OPT-IN olmali, varsayilan degil.
WORLD = os.environ.get("INDI_WORLD", "default")

CLIMB_M = 40.0        # m, FT_MIN_ALT = 20 m'nin rahat ustunde
SETTLE_S = 12.0
FT_TIMEOUT_S = 60.0   # betigin bekleyecegi sure (modulun kendi timeout'u 30 s)
# CRUISE_S is overridable because 15 s is a CRITERION duration, not an
# OBSERVATION one: it is long enough to prove the phase was entered and held,
# and far too short both to watch (the tilt sits at 40 deg for 12 s of a 210 s
# flight) and to let the cruise pitch trim settle (tau ~ 20-24 s). Raise it with
# INDI_CRUISE_S for either purpose; the pass/fail criteria are unaffected.
CRUISE_S = float(os.environ.get("INDI_CRUISE_S", "15.0"))
BT_TIMEOUT_S = 200.0
HOLD_CHECK_S = 15.0

# --- SABIT KANAT FAZI (2026-08-29) ---
# FwState degerleri: TiltrotorIndiParams.hpp:754
FW_IDLE, FW_GLIDE, FW_ACTIVE, FW_RETURN = 0, 1, 2, 3
# INDI_FW=0 ile kapatilir; kapaliyken betik 2026-08-29 oncesi davranisina doner.
FW_PHASE = os.environ.get("INDI_FW", "1") not in ("", "0", "no", "false")
# GLIDE 0 -> 90 deg'i 20 deg/s ile suruyor (FW_TILT_RAMP_RATE), yani seyir
# tiltinden ~2-5 s. 45 s giris kapisinin acilmasini beklemek icin de bol pay.
FW_TIMEOUT_S = 45.0
FW_CRUISE_S = float(os.environ.get("INDI_FW_CRUISE_S", "20.0"))
FW_RETURN_TIMEOUT_S = 45.0
FT_CRUISE_V = 8.0
FT_ALT_BAND = 5.0
CRUISE_TILT_MIN = 30.0

LOG_TOPICS_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "logger_topics_shadow.txt")
LOG_TOPICS_DST_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/logging"
LOG_TOPICS_DST = os.path.join(LOG_TOPICS_DST_DIR, "logger_topics.txt")
ULOG_DIR = "/home/umran/PX4-Autopilot/build/px4_sitl_default/rootfs/log"


def newest_ulog() -> str:
    best, best_m = "", 0.0
    for root, _d, files in os.walk(ULOG_DIR):
        for f in files:
            if f.endswith(".ulg"):
                p = os.path.join(root, f)
                if os.path.getmtime(p) > best_m:
                    best, best_m = p, os.path.getmtime(p)
    return best


def main() -> int:
    px4 = sc.Px4Client()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(out_dir, "px4_mission.log")

    os.makedirs(LOG_TOPICS_DST_DIR, exist_ok=True)
    shutil.copyfile(LOG_TOPICS_SRC, LOG_TOPICS_DST)

    print("=== SITL baslatiliyor ===")
    sc.launch_sitl("gz_tiltrotor_indi", log_path, world=WORLD)

    try:
        if not sc.wait_until(px4.preflight_check_ok, timeout=20.0, poll_interval=0.5):
            print("UYARI: preflight temizlenmedi")

        px4.arm(force=True)
        time.sleep(0.5)
        z0 = px4.local_position().get("z", 0.0)
        z_sp = z0 - CLIMB_M

        yaw0_deg = float("nan")
        for _ in range(20):
            _, _, yaw0_deg = px4.attitude_euler_deg()
            if math.isfinite(yaw0_deg):
                break
            time.sleep(0.5)
        if not math.isfinite(yaw0_deg):
            raise RuntimeError("heading okunamadi")
        yaw0 = math.radians(yaw0_deg)
        print(f"arm heading = {yaw0_deg:.2f} deg")

        # fw = 13. argüman (MulticopterIndiTiltrotor.cpp:1941). 2026-08-29'a
        # kadar hic gonderilmiyordu: bu betik 12 argumanla duruyordu, C++ da
        # eksikse false sayiyor. Yani SABIT KANAT MODU (butun rotorlar kesilir,
        # tilt TILT_MAX'a slew eder) depodaki hicbir Python testinde HIC
        # calismamisti -- kod bozulmus degildi, komut hic gelmiyordu.
        def send(pos_hold=False, bt=False, ft=False, fw=False, z=None, yaw=None):
            args = ["test_sp", "0.0", "0.0", f"{yaw0 if yaw is None else yaw}", "0.0",
                    f"{z_sp if z is None else z}", "1", "1", "0",
                    "1" if pos_hold else "0", "1" if bt else "0", "1" if ft else "0",
                    "1" if fw else "0"]
            return px4._run("mc_indi_tiltrotor", args)

        def snap():
            lp = px4.local_position()
            st = px4.status()
            # NOTE: the length test below is `>= 6`, not `== 6`. It was `== 6`
            # until 2026-08-04 and had been silently FALSE since step 45 widened
            # the allocator to 11 actuators, so every live `tilt=` readout in
            # this script (and six others) printed nan for two weeks of flights.
            # Nothing failed, because the pass/fail criteria read u_actual from
            # the ULOG, which was never length-checked -- the blind path was the
            # one a human watches. A width assumption is exactly the kind of
            # constant that must not be duplicated into a comparison.
            u = st.get("u_actual", [float("nan")] * 6)
            return {"vh": math.hypot(lp.get("vx", float("nan")), lp.get("vy", float("nan"))),
                    "z": lp.get("z", float("nan")),
                    "tilt": max(math.degrees(u[3]), math.degrees(u[4])) if len(u) >= 6 else float("nan"),
                    # T0/T1/T2 -- GLIDE'da ucu de sifir olmali; "motorlar durdu"
                    # iddiasinin gozle degil OLCUMLE dogrulanmasi icin.
                    "thr": [u[0], u[1], u[2]] if len(u) >= 6 else [float("nan")] * 3,
                    "fw_state": int(st.get("fw_state", -1)),
                    "fw_tilt": math.degrees(st.get("fw_tilt", float("nan")))}

        print(f"1) kalkis + hover, {CLIMB_M:.0f} m (MULTIKOPTER)")
        t0 = time.monotonic()
        while time.monotonic() - t0 < SETTLE_S + CLIMB_M / 1.5:
            send(pos_hold=True)
            time.sleep(1.0)
        s = snap()
        print(f"   hover: z={s['z']:.1f} m  v_h={s['vh']:.2f} m/s  tilt={s['tilt']:.1f} deg")

        print("2) ft_enable = 1 -- ILERI GECIS, elle hicbir sey surulmuyor")
        t0 = time.monotonic()
        last = -99.0
        while time.monotonic() - t0 < FT_TIMEOUT_S:
            t = time.monotonic() - t0
            send(ft=True)
            s = snap()
            if t - last >= 5.0:
                print(f"   t={t:5.1f}  v_h={s['vh']:6.2f}  z={s['z']:7.2f}  tilt={s['tilt']:5.1f} deg")
                last = t
            if s["vh"] >= FT_CRUISE_V:
                print(f"   -> seyir hizi ({FT_CRUISE_V} m/s) t = {t:.1f} s'de")
                break
            time.sleep(0.5)

        print(f"3) seyirde {CRUISE_S:.0f} s (SABIT KANAT / tilt motor)")
        t0 = time.monotonic()
        while time.monotonic() - t0 < CRUISE_S:
            send(ft=True)
            time.sleep(1.0)
        s = snap()
        print(f"   seyir: v_h={s['vh']:.2f} m/s  tilt={s['tilt']:.1f} deg  z={s['z']:.2f}")

        # --- 3b) SABIT KANAT (2026-08-29) -------------------------------------
        # Kullanici gozlemi: "hep drone gibi gitti, motorlar kapanip tilt
        # senaryosu goremedim". Dogruydu -- FT CRUISE tilti 45 deg'de birakir ve
        # rotorlar doner. Tam tilt + rotor kesme AYRI bir mod (FwState), ve
        # fw_enable hic gonderilmiyordu.
        #
        # ft=True KALMALI: giris kapisi _ft_state == FtState::CRUISE istiyor
        # (MulticopterIndiTiltrotor.cpp:943). Kapinin geri kalani: v_fwd >= 7,
        # irtifa >= 30 m, |roll| < 10 deg, |yaw hizi| < 10 deg/s. Reddedilirse
        # C++ PX4_WARN basar ("fixed-wing mode REFUSED: ..."), px4 loguna bakin.
        if FW_PHASE:
            print("3b) fw_enable = 1 -- SABIT KANAT (GLIDE: butun rotorlar kesilir)")
            t0 = time.monotonic()
            last = -99.0
            reached = False
            while time.monotonic() - t0 < FW_TIMEOUT_S:
                t = time.monotonic() - t0
                send(ft=True, fw=True)
                s = snap()
                if t - last >= 2.0:
                    print(f"   t={t:5.1f}  fw_state={s['fw_state']}  fw_tilt={s['fw_tilt']:5.1f} deg"
                          f"  tilt={s['tilt']:5.1f}  T=[{s['thr'][0]:.2f} {s['thr'][1]:.2f} {s['thr'][2]:.2f}]"
                          f"  v_h={s['vh']:5.2f}  z={s['z']:7.2f}")
                    last = t
                if s["fw_state"] == FW_ACTIVE:
                    print(f"   -> FwState ACTIVE (tam tilt) t = {t:.1f} s'de")
                    reached = True
                    break
                time.sleep(0.5)

            if not reached:
                print("   UYARI: FwState ACTIVE'e ulasilamadi -- px4 logunda "
                      "'fixed-wing mode REFUSED' satirini arayin")
            else:
                print(f"   sabit kanatta {FW_CRUISE_S:.0f} s")
                t0 = time.monotonic()
                while time.monotonic() - t0 < FW_CRUISE_S:
                    send(ft=True, fw=True)
                    s = snap()
                    print(f"   fw_tilt={s['fw_tilt']:5.1f} deg  "
                          f"T=[{s['thr'][0]:.2f} {s['thr'][1]:.2f} {s['thr'][2]:.2f}]  "
                          f"v_h={s['vh']:5.2f}  z={s['z']:7.2f}")
                    time.sleep(2.0)

                # DONUS: bt_enable, FwState ACTIVE -> RETURN gecisini tetikler
                # (TiltrotorIndiControl.hpp:919). Tilt ayni hizla geri iner ve
                # IDLE'a doner -- yani her ucusun basladigi normal hover durumu.
                print("3c) bt_enable = 1 -- FwState RETURN, tilt geri iniyor")
                t0 = time.monotonic()
                while time.monotonic() - t0 < FW_RETURN_TIMEOUT_S:
                    send(bt=True)
                    s = snap()
                    if s["fw_state"] == FW_IDLE:
                        print(f"   -> FwState IDLE, tilt {s['fw_tilt']:.1f} deg "
                              f"(t = {time.monotonic()-t0:.1f} s)")
                        break
                    time.sleep(0.5)

        print("4) ft_enable = 0, bt_enable = 1 -- GERI GECIS")
        t0 = time.monotonic()
        last = -99.0
        while time.monotonic() - t0 < BT_TIMEOUT_S:
            t = time.monotonic() - t0
            s = snap()
            # ZOOM TIRMANISINI GERI ISTEME (2026-08-29). FwState RETURN
            # MOTORSUZDUR, yani arac hizini irtifaya cevirir: olculdu, 16.0 m/s
            # ile 43.5 m'den 46.9 m'ye zoom (ULog 10_00_17, t=86.7-88.9).
            # Devir aninda arac z_sp'nin 6.9 m USTUNDE ve hala 10.5 m/s.
            # Sabit z_sp gondermek "hemen alcal" demek olur; kanat o hizda bol
            # tasima urettigi icin dogru cevap SIFIR ITKI olur ve kuyruk rotoru
            # 3.5 s boyunca alt raya iner (olculdu: 813 doyum ornegi, T2=0.00,
            # t=92.1-95.6). Kilitlenme DEGIL -- T0/T1 4-5 N'de rahat ve rejim
            # kendi kendine duzeliyor -- ama sifir itkideki rotorun pitch
            # otoritesi de sifirdir, ki bu deponun kendi "sifir itki bir
            # ucurumdur" ilkesinin kacinmayi soyledigi durumdur.
            #
            # Cozum enerjiyi zorla degil, DOGAL yoldan harcamak: arac hizliyken
            # mevcut irtifayi hedef yap (NED'de min() daha YUKSEK irtifadir),
            # yavaslayinca gercek z_sp'ye don. Frenleme zaten irtifayi geri
            # veriyor; bu yalnizca ayni seyi iki kez istememizi onluyor.
            # HIZ ESIGI KALDIRILDI (2026-08-29, ikinci tur). Once bu bir hiz
            # esigiyle (5 m/s) kosullanmisti ve bu BASAMAK uretiyordu: esik
            # gecildigi anda hedef 43 m'den 40 m'ye atliyor, ani alcalma talebi
            # yine sifir itkiye gidiyordu. Olculdu: BIG_M 813 -> 33'e dustu ama
            # SIFIRLANMADI ve kalan 33'un tamami t=96.4-96.5 s'de, yani tam
            # anahtarlama aninda (ULog 10_15_25).
            #
            # Kosulsuz min() basamagi tamamen kaldirir ve kendi kendine kapanir:
            # NED'de min() DAHA YUKSEK irtifadir, yani "bulundugun yerin altini
            # isteme, ama z_sp'nin de altina inme". Arac frenleyip alcaldikca
            # hedef onu takip eder ve z_sp'ye PURUZSUZ yakinsar. Inis (faz 5)
            # zaten bulundugu irtifadan 1 m'lik kademelerle basliyor, dolayisiyla
            # z_sp'ye burada donmenin bir islevi yok.
            z_hold = min(z_sp, s["z"]) if math.isfinite(s["z"]) else None
            send(bt=True, z=z_hold)
            if t - last >= 5.0:
                print(f"   t={t:5.1f}  v_h={s['vh']:6.2f}  z={s['z']:7.2f}  tilt={s['tilt']:5.1f} deg"
                      f"{'  [irtifa serbest]' if z_hold is not None else ''}")
                last = t
            if s["vh"] < 1.0:
                print(f"   -> v_h < 1.0 m/s, t = {t:.1f} s")
                break
            time.sleep(0.5)

        t0 = time.monotonic()
        while time.monotonic() - t0 < HOLD_CHECK_S:
            # Ayni min() kurali burada da gecerli, yoksa yukarida kacinilan
            # basamak bu dongunun ILK tikinda geri gelirdi.
            z_now_h = px4.local_position().get("z", float("nan"))
            send(bt=True, z=min(z_sp, z_now_h) if math.isfinite(z_now_h) else None)
            time.sleep(1.0)

        print("5) kademeli inis (MULTIKOPTER), sonra yerde disarm")
        z_now = px4.local_position().get("z", z_sp)
        stalled = 0
        # BUTCE CLIMB_M'E BAGLI (2026-08-28). Alcalma hizi ~0.45 m/s ile
        # sinirli: setpoint her adimda yalnizca 1 m asagida oldugu icin
        # irtifa dongusu bundan hizli inemiyor. Sabit 60 iterasyon (90 s)
        # 40 m'lik gorev icin TAM SINIRDA idi ve tukendi -- arac 5.67 m'de
        # havada disarm edilip dustu. Adim buyuklugu 1.0 m KALMALI:
        # 1.5 m'lik kademeler inis fazinda 13 BIG_M uretti (RUNBOOK (O)).
        max_iter = int(2.0 * (CLIMB_M / 0.45) / 1.5) + 20
        # SON METRE: SETPOINT YERIN ALTINA SURULUR (2026-08-28). Eski profilin
        # tabani -0.3 m idi, yani araca hicbir zaman "in" denmiyordu, "30 cm'de
        # asili kal" deniyordu. Sonuc olculdu: arac yer etkisi bolgesinde
        # takiliyor ve 1 m altinda tahsisat kanat motorlarini raylar arasinda
        # gidip getirmeye basliyor (T0=0/T1=1, sonra ters) -- yaw farki ancak
        # boyle kurulabiliyor cunku o irtifada ortalama itkinin ALT payi
        # tukenmis oluyor. Olculen sonuc: yaw kacisi, 3 kosumun 1'inde 2119 deg,
        # GUI'li kosumda 20774 deg (57 tur) ve arac hic inemedi.
        # Duzeltme: FLARE_ALT altinda hedef yerin 0.5 m ALTI yapilir; boylece
        # irtifa dongusu temasa kadar alcalmayi surdurur ve arac yerde oturur.
        # 1.0 m'lik kademe kurali korunur (1.5 m 13 BIG_M uretmisti, RUNBOOK (O)).
        FLARE_ALT = 1.5      # m, bu irtifanin altinda temas komut edilir
        # 0.5 m COK FAZLAYDI: temastan sonra da bastirmayi surdurup araci
        # zemine gommeye calisiyordu ("zeminin altina girmeye calisiyor",
        # 2026-08-29). Gercek donanimda bu, pervaneleri ve inis takimini itkiyle
        # yere bastirmak demek. 0.15 m temasi garanti etmeye yetiyor.
        TOUCH_Z = 0.15       # m, yer datumunun ALTI -- temasi zorlar

        # ACISAL HIZ TEMAS OLCUTU DEGILDIR -- DENENDI, GERI ALINDI (2026-08-29).
        # Hipotez: yerde oturunca zemin kisiti p,q'yu sifira cakar, dolayisiyla
        # "hizlar cakili" = "yerde". YANLIS CIKTI ve SESSIZ ARIZA uretti:
        # ULog 10_23'te arac 1.29 m'de YER ETKISINDE ASILI kalirken |w| 0.0010
        # rad/s olculdu, betik "TEMAS" ilan etti, temas "dogrulandi" yazdi ve
        # motorlar 1.28 m'de kesildi -- yani arac dustu ve test GECTI dedi.
        # Bu, onceki yazarin bilerek kaldirdigi tam o arizadir ("hareketsizlik
        # ARTIK basari degil"); sahte bir "indi" raporu, "inemedi" raporundan
        # daha kotudur. Asil olcut IRTIFA + vz'dir ve oyle kaldi.
        #
        # Fonksiyon TESHIS AMACLI duruyor: takilma uyarisinda |w| yazdirmak,
        # "asili mi yoksa oturmus mu" sorusunu sonradan loga bakarak
        # ayirt etmeyi kolaylastiriyor. KARAR VERMEZ.
        # AGIRLIK: SDF base_link kutlesi 5.0 kg. Fizik olcutu bunu esik
        # olarak kullanir; SDF degisirse burasi da degismelidir.
        WEIGHT_N = 5.0 * 9.81
        GROUND_THRUST_FRAC = 0.5      # agirligin bu kesrinin altindaysa yerde

        def on_ground_by_thrust():
            """FIZIK OLCUTU (2026-08-31, Adim 150): dikey itki agirligin cok
            altindayken arac DUSMUYORSA, onu tutan sey zemindir.

            NEDEN GEREKLI: yukaridaki olcutlerin hepsi IRTIFA sinyaline
            dayaniyor ve yanilan sinyal tam olarak o. Olculdu (bu oturum,
            6 kosum): arac yerde otururken datum tabanli agl 0.64-1.15 m
            gosterdi, betik "temas dogrulanamadi" deyip motorlari kesti.
            Iki kosumda arac gercekten 0.8 m'de asili kaldi ve kesme sonrasi
            DUSTU -- olculen carpma 3.0 ve 5.8 m/s. Gercek arac bunu kaldirmaz.

            Bu olcut irtifadan TAMAMEN bagimsizdir. Olculen ayrisma:
              yerde oturmus : 5.2, 12.3, 13.1 N
              asili/alcalan : 34.2, 42.3, 50.4, 50.6 N
            Esik 0.5*agirlik = 24.5 N tam ortadan geciyor.

            |vz| kosulu SART: itki dusuk AMA arac dusuyorsa bu serbest
            dusustur, temas degil. Ikisi birlikte "bir sey onu tutuyor" der.

            NOT: vehicle_land_detected KULLANILMADI -- bu gövde icin guvenilir
            olmadigi Adim 110'da olculmustu (14 s gec) ve bu oturumda da
            dogrulandi: arac acikca yerdeyken landed %0 kaldi.
            """
            st = sc.parse_named_floats(px4.listener("tiltrotor_indi_status"))
            ua, du_ = st.get("u_actual"), st.get("du")
            if not ua or not du_ or len(ua) < 6 or len(du_) < 6:
                return False, float("nan")
            try:
                thr = [float(ua[i]) + float(du_[i]) for i in range(3)]
                tlt = [float(ua[i + 3]) + float(du_[i + 3]) for i in range(3)]
            except (TypeError, ValueError):
                return False, float("nan")
            ctz = sum(thr[i] * math.cos(tlt[i]) for i in range(3))
            return ctz < GROUND_THRUST_FRAC * WEIGHT_N, ctz

        def rates_pinned():
            w = sc.parse_named_floats(px4.listener("vehicle_angular_velocity"))
            xyz = w.get("xyz", None)
            if not xyz or len(xyz) < 2:
                return False, float("nan")
            m = max(abs(xyz[0]), abs(xyz[1]))
            return m < 0.01, m

        for _ in range(max_iter):
            # Bu kapi GERCEKTEN ulasiliyor: temiz bir iniste arac 0.03-0.10 m'ye
            # kadar iniyor (ULog 09_52_47). Kilitlenme rejiminde ise 0.6-1.3 m'de
            # takiliyor ve cikis asagidaki `stalled` dalindan yapiliyor.
            #
            # `z0` TABANLI (2026-08-31, Adim 148) -- eskiden mutlak `-0.25` idi,
            # yani EKF yerel orijininin zeminde oldugunu varsayiyordu. Olculdu:
            # arm anindaki `z` kosumdan kosuma -1.013 ile +0.903 m arasinda
            # degisiyor. Bu dosya AGL'yi ZATEN `z0 - z_now` diye hesapliyor
            # (Adim 117); burasi o sozlesmeye uyduruldu.
            if z_now >= z0 - 0.25:
                break
            # *** SUREKLI RAMPA DENENDI VE GERI ALINDI (2026-08-30, adim 134) ***
            # Adim 114'un teshisi, bu kosumda da tekrar uretilen bir ORTUSMEYE
            # dayaniyordu: salinim periyodu 2.40 s, 1 m kademe / 0.394 m/s =
            # 2.54 s. Cok ikna ediciydi ve YANLIS cikti.
            # OLCUM (kademe -> rampa, ayni senaryo):
            #   alcalma hizi   0.394 -> 0.240 m/s   (kademe kalkti, hiz DUSTU)
            #   baskin frekans 0.417 -> 0.419 Hz    (DEGISMEDI)
            #   pitch tepe-tepe  7.96 -> 11.00 deg  (KOTULESTI)
            # Frekans, uyaranin hizindan BAGIMSIZ cikti -- yani salinim
            # profilin uyarmasi degil, sistemin KENDI modu. Merdiven onu
            # yalnizca besliyordu, yaratmiyordu.
            # Bu yuzden 1 m kademe KALIYOR (1.5 m ayrica 13 BIG_M uretmisti,
            # RUNBOOK (O)) ve salinimin gercek kaynagi ayri bir is.
            # HEDEF `z0`'A GORE (2026-08-31, Adim 148). Eskiden `0.0 + TOUCH_Z`
            # idi: "yerin 15 cm alti" demek isteniyordu ama fiilen "EKF
            # orijininin 15 cm alti" deniyordu. OLCULDU (6 kosum, tam ayrisma):
            #   arm'daki z  +0.903, +0.506  -> arac 0.75-0.87 m'de ASILI KALDI
            #   arm'daki z  -1.013 .. +0.023 -> temiz indi
            # Orijin zeminin 0.9 m ustunde kuruldugunda komut "yerin 0.75 m
            # USTUNDE dur" anlamina geliyordu ve irtifa dongusu hatayi
            # sifirlayip duruyordu (fz_sp ~ -50 N, yani agirligi tasimaya devam).
            # Sonuc: betik takilmayi gorup motorlari kesiyor ve arac 0.8 m'den
            # DUSUYOR -- olculen carpma 3.0 ve 5.8 m/s. Gercek arac bunu kaldirmaz.
            z_cmd = z0 + TOUCH_Z if z_now > z0 - FLARE_ALT else z_now + 1.0
            # YAW HEDEFI SON METREDE SERBEST BIRAKILIR (2026-08-28). Olculdu:
            # yere yakin yaw RMS'i 20.1x artiyor (roll 4.1x, pitch 3.1x), tepe
            # 6.44 rad/s. Sebep, kovalanamayan bir yaw hatasi: temas/yer etkisi
            # araci dondurmeye basliyor, bu airframe'in yaw otoritesi yapisal
            # olarak en zayif eksen (yaw rate sp limiti 0.5 vs roll/pitch 3.0,
            # tiltrotor_params.m:279), ve yaw torku kanat itki FARKINDAN
            # uretildigi icin kontrolcu farki buyutuyor -> ortalama itkinin alt
            # payi tukenmis oldugundan bir motor tabana yapisiyor (T0=0/T1=1,
            # sonraki cevrimde ters). Hedefi olculen yaw'a esitlemek hatayi
            # sifirda tutar ve tahsisat itki dengesini bozmayi birakir.
            # NOT: Ws_yaw'i dogrudan degistirmek bu projede DENENDI ve GERI
            # ALINDI (Adim 7, indi_attitude_controller.m:232) -- bu yol ucus
            # yazilimina hic dokunmuyor.
            # TUM INIS BOYUNCA, esige bagli DEGIL (2026-08-28, olcum sonrasi
            # duzeltme). Ilk yazimda yalnizca FLARE_ALT=1.5 m altinda
            # uygulaniyordu; GUI'li bir kosumda kilit 1.92 m'de basladi, yaw
            # kacisi 2.18 m'de, kilitli ornekler 2.50 m'ye kadar cikti -- yani
            # tehlike bolgesi esigin USTUNDE baslayabiliyor ve fix hic
            # devreye girmiyordu. Dikey inis boyunca yon tutmanin gorev degeri
            # olmadigi icin dogru cozum esigi yukseltmek degil, KALDIRMAK.
            _, _, yd = px4.attitude_euler_deg()
            yaw_hold = math.radians(yd) if math.isfinite(yd) else None
            send(pos_hold=True, z=z_cmd, yaw=yaw_hold)
            time.sleep(1.5)
            z_new = px4.local_position().get("z", z_now)
            stalled = stalled + 1 if abs(z_new - z_now) < 0.05 else 0
            z_now = z_new
            # Hareketsizlik ARTIK basari degil: temas komut edildigi halde
            # arac inemiyorsa bu bir ARIZA belirtisidir (yukaridaki kilitlenme).
            # Yerdeyse zaten yukaridaki -0.25 kapisindan cikilmis olur.
            # ALCALMANIN DURMASI TEMASIN TA KENDISI OLABILIR (2026-08-29).
            # Bu dal eskiden kosulsuz ARIZA sayiyordu; oysa "temas komut
            # edildigi halde inmiyor" ile "indi ve yerde duruyor" ayni olcume
            # sahip. Ikisini acisal hiz ayiriyor. Yanlis tarafa dusmek pahaliydi:
            # dongu bastirmayi surdurunce INDI artimlari yere karsi SARIYOR
            # (nu_achieved sonsuza dek 0), T1 sifir rayina cakiliyor ve tek
            # calisan egik rotor yaw kacisini baslatiyor -- olculdu, 6793 deg.
            if stalled >= 3:
                _, wmax = rates_pinned()
                print(f"   UYARI: alcalma durdu, agl {z0 - z_now:.2f} m "
                      f"(ham irtifa {-z_now:.2f} m, |w| {wmax:.4f} rad/s)")
                break
        print(f"   son agl = {z0 - z_now:.2f} m (ham irtifa {-z_now:.2f} m)")

        # TEMASI DOGRULAMADAN MOTOR KESME (2026-08-29). Eskiden dongu biter
        # bitmez 2 s beklenip disarm ediliyordu; dongu TAKILARAK bittiginde
        # (kilitlenme rejimi) arac 1-2.5 m'de oluyor ve motorlar orada
        # kesilince serbest dusuyor -- olculdu: disarm 5.67 m'de, ardindan
        # vz +8.78 m/s. Gercek bir IHA'da bu, kirilan bir inis takimi demek.
        # Simdi once yere oturmasi beklenir: irtifa < 0.30 m VE |vz| < 0.15 m/s
        # ust uste 3 kez. Saglanmazsa yine disarm edilir (arac havada sonsuza
        # kadar tutulamaz) ama bu ACIKCA arıza olarak yazilir, sessizce degil.
        # 0.30 m ESIGI DOGRUDUR -- bir ara bunun "imkansiz" oldugu one suruldu
        # ve YANLISTI (2026-08-29). Yanlisin kaynagi: 09_27_25 logunda acisal
        # hizlarin sifir oldugu 0.596-0.805 m'lik bir pencere olculup "oturma
        # yuksekligi" sanildi; oysa o, TAKLA ATMIS aracin kanadi uzerinde
        # durdugu andi. Temiz bir iniste govde datumu 0.03 m'ye kadar iner
        # (ULog 09_52_47) ve bu olcut rahatca saglanir.
        #
        # Temas dogrulanamiyorsa sebep esik degil, aracin ASAGI INEMEMESIDIR:
        # INDI artimlari yere karsi sariyor, T1 sifir rayina cakiliyor ve tek
        # calisan egik rotor yaw kacisini baslatiyor (olculdu: 6793 deg).
        # PENCERE 20 -> 60 ORNEK (10 s -> 30 s), 2026-08-31 adim 137.
        # OLCULEN SEBEP: son inis kosumunda arac 0.32 m'ye kadar indi ve
        # olcutu (agl<0.55, |vz|<0.15) SAGLADI -- ama betik o ana gelmeden
        # pencereyi tuketmisti. Arac 0.6-1.0 m bandinda ~25 s salinip
        # (yer etkisi: vz surekli isaret degistiriyor) sonra oturuyor;
        # 10 s o salinimi bile kapsamiyordu. Yani "temas dogrulanamadi",
        # aracin inememesi degil, betigin BEKLEMEMESI demekti.
        settled = 0
        for _ in range(60):
            lp_now = px4.local_position()
            z_c, vz_c = lp_now.get("z", 0.0), lp_now.get("vz", 0.0)
            pinned, wmax = rates_pinned()
            # IKI YOL, VE IRTIFA OLANI ASIL OLANDIR (2026-08-29, duzeltildi).
            # Once yalnizca `pinned` kullanildi ve bu YANLISTI: temiz bir inis
            # olculdu (ULog 09_52_47) -- arac 0.03-0.10 m'ye indi, T0/T1
            # 18.8/19.4'te dengeli kaldi, 60 s'de toplam yaw 6 deg -- ama
            # acisal hizlar CAKILI DEGILDI (|p| 0.024, |q| 0.10 ort). Cunku
            # arac yere degse de hala ~19 N/rotor ile AKTIF KONTROL altinda;
            # atil degil. 0.01 esigi yalnizca kalkis oncesi gercekten
            # hareketsiz araci yakaliyor (olculdu: |p| 0.0002, |q| 0.0009).
            # Irtifa+vz olcutu bu inisi dogru onaylardi; `pinned` onaylamadi.
            # DATUMA GORE AGL, HAM -z DEGIL (2026-08-29, Adim 117). `z0` arm'dan
            # 0.5 s sonra, arac YERDE otururken okundu -- yani zeminin ta
            # kendisi. Olcut eskiden ham `z_c > -0.30` idi ve bu ayni datum
            # hatasina tabiydi: 116'da olculen ofset (init) + 117'de olculen
            # ucus ici kayma toplami ~1 m'yi bulunca, arac yere OTURSA BILE
            # olcut saglanamiyordu. Uc kosumda da "temas dogrulanamadi" yazildi,
            # oysa ucunde de temas darbesi ve hiz donmasi ulog'da olculdu.
            # Olcutun gecmiste dogru gorunmesi (ULog 09_52_47) o kosumun
            # datumunun -0.28 m olmasindandi -- sans, tasarim degil.
            agl_c = z0 - z_c
            # ESIK 0.30 -> 0.55 (2026-08-30, adim 133). SEBEP MODELDE DEGISTI:
            # inis takimi eklendi (uc ayak, uclari govde alt yuzunun 4 cm
            # altinda). Arac artik AYAKLARIN uzerinde duruyor, yani govde
            # datumu asla 0.30 m'ye inemez -- olculdu: agl 0.47 m'de toplam
            # itki 52 -> 19.5 N'e dustu (agirlik 49.1 N), yani zemin araci
            # tasiyordu ve arac YERDEYDI, ama olcut onu goremiyordu.
            # 0.55 m: ayak yuksekligi (0.04) + govde yari kalinligi (0.025) +
            # olculen oturma payi. Eski 0.30 esigi INIS TAKIMSIZ modele aitti.
            # IKI BAGIMSIZ YOL, VEYA ile (Adim 150). Irtifa yolu KORUNDU;
            # fizik yolu yalnizca EKLENIR, yani tespit kotulesemez.
            ok_alt = agl_c < 0.55 and abs(vz_c) < 0.15
            low_thrust, ctz_c = on_ground_by_thrust()
            # FIZIK YOLUNDA |vz| KOSULU YOK, VE BU BILEREK (Adim 150).
            # Ilk yazimda vardi; kayitli loglara karsi sinandi ve aracin
            # ACIKCA yerde oldugu bir kosumu (dikey itki 5.2 N) KACIRDI,
            # cunku EKF'in vz'si o sirada 0.263 m/s okuyordu. vz, yanilan
            # irtifa sinyaliyle AYNI kestirimciden geliyor; onu kapi yapmak
            # kotu sinyali geri sokmak olurdu.
            # Yerine SUREKLILIK: 5.2 N itkiyle havada olsaydi arac
            # (1 - 5.2/49.05)*g = 8.8 m/s^2 ile duserdi. Asagidaki
            # `settled >= 3` (3 x 0.5 s = 1.5 s) suresince duşuş 11 m eder;
            # inis fazi 2 m'de basladigina gore boyle bir sey imkansizdir.
            # Yani SUREKLI dusuk itki tek basina temas kanitidir.
            ok_phys = low_thrust
            ok = ok_alt or ok_phys
            settled = settled + 1 if ok else 0
            if settled >= 3:
                yol = "irtifa" if ok_alt else "FIZIK (dikey itki)"
                print(f"   temas dogrulandi [{yol}] (agl {agl_c:.2f} m, ham irtifa "
                      f"{-z_c:.2f} m, vz {vz_c:+.2f}, dikey itki {ctz_c:.1f}/"
                      f"{WEIGHT_N:.1f} N, |w| {wmax:.4f} rad/s) -> disarm")
                break
            time.sleep(0.5)
        else:
            lp_now = px4.local_position()
            z_f = lp_now.get('z', 0.0)
            _, ctz_f = on_ground_by_thrust()
            print(f"   ARIZA: temas dogrulanamadi, agl {z0 - z_f:.2f} m "
                  f"(ham irtifa {-z_f:.2f} m, dikey itki {ctz_f:.1f}/{WEIGHT_N:.1f} N) "
                  f"-- motorlar YINE DE kesiliyor")
            if ctz_f == ctz_f and ctz_f < GROUND_THRUST_FRAC * WEIGHT_N:
                print(f"   ⚠ ama dikey itki agirligin altinda: arac MUHTEMELEN "
                      f"YERDEYDI ve |vz| kosulu tutmadi -- logu inceleyin")
        px4.disarm(force=True)
        time.sleep(2.0)

    finally:
        sc.kill_sitl()
        try:
            os.remove(LOG_TOPICS_DST)
        except FileNotFoundError:
            pass

    ulog = newest_ulog()
    print(f"\nulog: {ulog}\npx4 log: {log_path}\n")
    ok = analyze(ulog, log_path)
    print()
    import check_output_cuts
    check_output_cuts.check(ulog)
    return 0 if ok else 1


def analyze(path: str, px4_log: str) -> bool:
    u = ULog(path, ["tiltrotor_indi_status", "vehicle_local_position"])
    d = {m.name: m.data for m in u.data_list}
    st = d["tiltrotor_indi_status"]
    ts = np.asarray(st["timestamp"], float)
    keep = np.concatenate(([True], np.diff(ts) > 1e-6))
    t = ts[keep] / 1e6
    ft = np.asarray(st["ft_state"], float)[keep] if "ft_state" in st else np.zeros_like(t)
    bt = np.asarray(st["bt_state"], float)[keep]
    pha = (np.asarray(st["pos_hold_active"], float)[keep] > 0.5
           if "pos_hold_active" in st else np.zeros_like(t, dtype=bool))
    sat = np.column_stack([np.asarray(st["sat_flag[%d]" % i], float)[keep] for i in range(6)])
    ua = np.column_stack([np.asarray(st["u_actual[%d]" % i], float)[keep] for i in range(6)])
    tilt = np.degrees(np.maximum(ua[:, 3], ua[:, 4]))

    lp = d["vehicle_local_position"]
    tl = np.asarray(lp["timestamp"], float) / 1e6
    z = np.asarray(lp["z"], float)
    vh = np.hypot(np.asarray(lp["vx"], float), np.asarray(lp["vy"], float))

    with open(px4_log) as f:
        log = f.read()
    n_abort = log.count("forward transition ABORTED")

    print("=== TAM OTONOM GOREV ANALIZI ===")
    print(f"  ulog: {path}")

    # Evre dizisi -- pencereler ft_state/bt_state'ten, olculen sinyalden DEGIL.
    print("\n  evre dizisi:")
    for name, series, val in (("FT RAMP", ft, 1), ("FT CRUISE", ft, 2),
                              ("BT RETRACT", bt, 1), ("BT BRAKE", bt, 2), ("BT HANDOFF", bt, 3)):
        m = np.abs(series - val) < 0.1
        if m.any():
            a, b = t[m][0], t[m][-1]
            mm = (tl >= a) & (tl <= b)
            print(f"    {name:<11} {a:6.1f}-{b:6.1f} s ({b - a:5.1f} s)  "
                  f"v_h {vh[mm][0]:5.2f} -> {vh[mm][-1]:5.2f}  tilt max {tilt[m].max():5.1f} deg")
        else:
            print(f"    {name:<11} HIC GIRILMEDI")

    c1 = bool((np.abs(ft - 2) < 0.1).any()) and n_abort == 0
    print(f"\n  1) FT CRUISE'a ulasti : {bool((np.abs(ft - 2) < 0.1).any())}, "
          f"iptal x{n_abort} ................ {'GECTI' if c1 else 'KALDI'}")

    m_cr = np.abs(ft - 2) < 0.1
    tilt_cr = float(tilt[m_cr].max()) if m_cr.any() else float("nan")
    if m_cr.any():
        mm = (tl >= t[m_cr][0]) & (tl <= t[m_cr][-1])
        vh_cr = float(vh[mm].mean())
    else:
        vh_cr = float("nan")
    c2 = (tilt_cr >= CRUISE_TILT_MIN) and (vh_cr >= FT_CRUISE_V)
    print(f"  2) gercekten SEYIR    : tilt {tilt_cr:.1f} deg (>= {CRUISE_TILT_MIN:.0f}), "
          f"v_h ort {vh_cr:.2f} m/s (>= {FT_CRUISE_V:.0f}) . {'GECTI' if c2 else 'KALDI'}")

    # SABIT KANAT FAZI BU OLCUTUN DISINDA (2026-08-29). Olcut ILERI GECISIN
    # irtifa tutusunu sinar ve FW'den once yazildi. FW sirasinda ft_state
    # CRUISE'da KALIR (giris kapisi oyle istiyor, MulticopterIndiTiltrotor.cpp:943),
    # dolayisiyla pencere FW'yi de yutuyordu ve olcut FW'nin irtifa salinimini
    # ileri gecise fatura ediyordu: 6.91 / 9.84 / 10.95 m gibi degerler.
    # PENCERE DARALTILDI, ESIK DEGISTIRILMEDI -- ve FW salinimi GIZLENMIYOR,
    # hemen asagida AYRI bir satir olarak raporlaniyor. Bir olcutu gecirmek
    # icin esigi gevsetmek ile yanlis pencereyi duzeltmek ayri seylerdir.
    fw_a = np.asarray(st["fw_state"], float)[keep] if "fw_state" in st else np.zeros_like(t)
    m_ft = (ft > 0.5) & (fw_a < 0.5)
    if m_ft.any():
        t_ft = t[m_ft]
        mm = (tl >= t_ft[0]) & (tl <= t_ft[-1]) & \
             (np.interp(tl, t, fw_a) < 0.5)
        band = float(z[mm].max() - z[mm].min()) if mm.any() else float("nan")
    else:
        band = float("nan")
    c3 = math.isfinite(band) and band <= FT_ALT_BAND
    print(f"  3) FT irtifa sapmasi  : {band:.2f} m (<= {FT_ALT_BAND:.0f}) "
          f".................... {'GECTI' if c3 else 'KALDI'}")

    # FW salinimi: olcut DEGIL, gorunurluk. GLIDE ucu de kestigi icin arac
    # once duser, kanat rotorleri yeniden yaninca yukselir -- gercek ve
    # beklenen bir salinim, ama buyuklugu takip edilmeli.
    m_fw = fw_a > 0.5
    if m_fw.any():
        t_fw = t[m_fw]
        mf = (tl >= t_fw[0]) & (tl <= t_fw[-1])
        if mf.any():
            print(f"     FW fazi irtifa salinimi: {float(z[mf].max()-z[mf].min()):.2f} m "
                  f"(olcut degil, bilgi)")

    c4 = bool((np.abs(bt - 3) < 0.1).any()) and bool(pha.any())
    print(f"  4) BT HANDOFF + hold  : {bool((np.abs(bt - 3) < 0.1).any())} / {bool(pha.any())} "
          f"........................ {'GECTI' if c4 else 'KALDI'}")

    tail = tl >= tl[-1] - 30.0
    air = z < -3.0
    idx_air = np.nonzero(air)[0]
    vh_fin = float(vh[idx_air[-20:]].mean()) if idx_air.size > 20 else float("nan")
    c5 = math.isfinite(vh_fin) and vh_fin < 1.0
    print(f"  5) son v_h (havada)   : {vh_fin:.2f} m/s (< 1.0) "
          f"....................... {'GECTI' if c5 else 'KALDI'}")

    z_st = np.interp(t, tl, z)
    airborne = z_st < -3.0
    big = int(sat[airborne, :3].any(axis=1).sum())
    sat_pc = 100.0 * float(sat[airborne, :3].mean())
    c6 = (big == 0) and (sat_pc == 0.0)
    print(f"  6) havada doyum/BIG_M : %{sat_pc:.2f} / {big} "
          f"............................. {'GECTI' if c6 else 'KALDI'}")

    ok = all([c1, c2, c3, c4, c5, c6])
    print(f"\n  SONUC: {'GECTI' if ok else 'KALDI'}")
    return ok


if __name__ == "__main__":
    sys.exit(main())
