#!/usr/bin/env bash
# gz_follow.sh — Gazebo GUI kamerasini tiltrotor modeline kilitler.
#
# SITL'i GUI'li baslattiktan sonra (HEADLESS=1 VERMEDEN) calistirin; kamera
# modeli takip eder, boylece arac ucup uzaklassa da (bu kontrolcude yatay
# konum donusu YOK, arac yuzlerce metre surüklenebilir) gorus alaninda kalir.
#
# Kullanim:
#   ./gz_follow.sh                              # varsayilan: front (on-capraz)
#   ./gz_follow.sh tiltrotor_indi_0 side        # hazir gorus adi
#   ./gz_follow.sh tiltrotor_indi_0 -5 0 2.5    # model + offset (x y z, metre)
#
# GORUS ADLARI -- hepsi TILTI GORMEK icin secildi (2026-08-03, Adim 39;
# mesafeler Adim 41'de YAKINLASTIRILDI, asagidaki nota bakin):
#   nose   (+1.8,  0.0, +0.25) VARSAYILAN. Tam karsidan ve yakin.
#   front  (+1.6, -1.6, +0.35) on-capraz 45 deg: burun + nasel acisi birlikte.
#   side   ( 0.0, -2.2, +0.30) tam yandan. Nasel acisi DOGRUDAN okunur (tilt
#          govde X-Z duzleminde doner), yani 45->9->20 deg rampasi icin en iyisi.
#   far    (+3.2, -3.2, +1.0)  genel kadraj; manevranin tamamini gormek icin.
#   chase  (-2.0,  0.0, +0.8)  eski varsayilan, arkadan takip. Tilt gorunmez.
#
# gz FLU model cercevesi: +x ileri, +y SOL, +z yukari. Yani -y = SAGDAN bakis.
#
# NEDEN YAKIN OLMALI (2026-08-03, Adim 41 -- kullanici gozlemi): offset MODEL
# cercevesindedir, yani arac yaw'da donunce kamera o yaricapta SAVRULUR. Pilot
# testinde yaw cubugu heading'i 223 deg dondurdu ve 3.2 m'lik offset kamerayi
# aracin etrafinda genis bir yayda gezdirdi -- izleyene "kamera uzaklasti" gibi
# gorunuyor. Yaricapi kucultmek hem savrulmayi hem de araci kadrajda kucuk
# birakmayi ayni anda cozer. (follow_pgain zaten 1.0, yani takip kazanci
# degil GEOMETRI sorunuydu.)
#
# HANGI TESTTE TILT GERCEKTEN HAREKET EDER: yalnizca geri gecis
# (`run_backtrans_test.py`) -- tavan 45 -> 9 -> 20 deg gider. Pilot testinde
# (`run_pilot_input_test.py`) arac hover rejiminde kalir ve tilt yalnizca
# 1-16 deg arasi oynar (yaw trimi delta0'i ~9-10 deg'de tutar, madde (P));
# orada "tilt gorunmuyor" bir kamera sorunu DEGIL, senaryonun kendisidir.
#
# NOT: gz'nin follow modu kamerayi MODELIN cercevesinde tasir. Bu arac yaw
# ekseninde donuyorsa (bkz. WLS_LOCKUP_INVESTIGATION_REPORT.md Adim 12) kamera
# da onunla doner ve donus "arac sabit, yer donuyor" gibi gorunur. Gercek yaw
# davranisini GORSEL olarak degil, her zaman veriden dogrulayin:
#   px4-listener vehicle_angular_velocity   -> xyz[2]
#
# NOT 2 (Adim 15): follow tek basina YETMEZ. Iki ek sey gerekli:
#  a) Kameranin takip kazanci varsayilan 0.01 -- hizli hedefe hic yetismiyor
#     (olculdu: 30 s'de ~170 m geride). worlds/default.sdf'teki
#     CameraTracking eklentisine <follow_pgain>1.0</follow_pgain> eklendi.
#  b) Asil sorun aracin ~10 m/s suruklenmesi ve bu YAPISAL: tum tiltler
#     [0, pi/2] araliginda oldugu icin hover'da net Fx her zaman >= 0.
#     Uzun gozlem kosularinda pitch trim komut edin:
#       px4-mc_indi_tiltrotor test_sp 0 0.061 0 0 <z_sp> 1 1 0
#     (0.061 rad ~ 3.5 deg burun yukari; surukleme 9.4 -> 1.7 m/s)

set -u

MODEL="${1:-tiltrotor_indi_0}"
VIEW="${2:-front}"

case "${VIEW}" in
	nose)  OFF_X=1.8;  OFF_Y=0.0;  OFF_Z=0.25 ;;   # VARSAYILAN: tam karsidan, yakin
	front) OFF_X=1.6;  OFF_Y=-1.6; OFF_Z=0.35 ;;
	side)  OFF_X=0.0;  OFF_Y=-2.2; OFF_Z=0.30 ;;
	far)   OFF_X=3.2;  OFF_Y=-3.2; OFF_Z=1.00 ;;
	chase) OFF_X=-2.0; OFF_Y=0.0;  OFF_Z=0.80 ;;
	*)
		# Ad degilse sayisal offset bekleniyor (geriye donuk uyumluluk).
		OFF_X="${2:-1.8}"
		OFF_Y="${3:-0.0}"
		OFF_Z="${4:-0.25}"
		VIEW="ozel"
		;;
esac

if ! command -v gz >/dev/null 2>&1; then
	echo "HATA: 'gz' bulunamadi (Gazebo kurulu mu?)" >&2
	exit 1
fi

# GUI servisleri, gz sim -g penceresi acilana kadar kayitli olmaz; birkac saniye bekle.
for i in $(seq 1 15); do
	if gz service -l 2>/dev/null | grep -q '^/gui/follow$'; then
		break
	fi
	sleep 1
done

if ! gz service -l 2>/dev/null | grep -q '^/gui/follow$'; then
	echo "HATA: /gui/follow servisi yok. SITL HEADLESS=1 ile mi baslatildi?" >&2
	echo "      GUI icin: DISPLAY=:1 PX4_SIM_MODEL=gz_tiltrotor_indi ../../../../bin/px4 -d" >&2
	exit 1
fi

gz service -s /gui/follow \
	--reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 \
	--req "data: \"${MODEL}\"" >/dev/null || { echo "HATA: follow cagrisi basarisiz" >&2; exit 1; }

gz service -s /gui/follow/offset \
	--reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 3000 \
	--req "x: ${OFF_X}, y: ${OFF_Y}, z: ${OFF_Z}" >/dev/null || { echo "HATA: offset cagrisi basarisiz" >&2; exit 1; }

echo "Kamera '${MODEL}' modeline kilitlendi (gorus: ${VIEW}, offset: ${OFF_X} ${OFF_Y} ${OFF_Z})."
