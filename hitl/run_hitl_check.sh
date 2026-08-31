#!/usr/bin/env bash
# HITL koprusu on-kontrolu: KART OLMADAN kosar.
#
# NE DOGRULAR (2026-08-31, adim 140):
#   1. gz sensor konulari yayinda mi ve hizlari HITL icin yeterli mi
#      (PX4'un HIL_SENSOR'u EKF2 tick'i icin >=200 Hz bekler)
#   2. koprunun abone/yayin arayuzu gercek konu adlariyla eslesiyor mu
#
# NE DOGRULAMAZ: MAVLink gonderimi, cerceve donusumu, aktuator eslemesi.
# Bunlar ancak GERCEK KART baglandiginda olculebilir -- ve olculmeden
# "hazir" denmemelidir.
set -e
cd "$(dirname "$0")"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

if ! pgrep -f "bin/px4" > /dev/null; then
    echo "SITL kosmuyor. Once baslatin:"
    echo "  cd ~/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridge"
    echo "  PX4_SIM_MODEL=gz_tiltrotor_indi HEADLESS=1 ../../../../bin/px4 -d"
    exit 1
fi

echo "=== gz konu envanteri ==="
gz topic -l 2>/dev/null | grep -E "sensor|command|servo" | sed 's/^/  /'
echo
echo "=== kopru dry-run (10 s) ==="
python3 gz_hil_bridge.py --dry-run --duration 10
