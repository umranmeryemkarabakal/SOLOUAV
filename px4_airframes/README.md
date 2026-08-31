# PX4 airframe dosyaları

Kart ve SITL yapılandırması. Bunlar PX4 ağacında yaşar; buradaki kopyalar
**depoyu kendi kendine yeterli** kılmak içindir (`gz_model/` ile aynı gerekçe).

| dosya | nerede yaşar | ne |
|---|---|---|
| `4023_gz_tiltrotor_indi{,.post}` | `init.d-posix/airframes/` | SITL |
| `14002_tiltrotor_indi{,.post}` | `init.d/airframes/` | gerçek kart |
| `14003_tiltrotor_indi_hitl{,.post}` | `init.d/airframes/` | **HITL** (madde H6) |

## 14003 nasıl çalışıyor

14002'yi **kopyalamaz, kaynak gösterir** (`. ${R}etc/init.d/airframes/14002_tiltrotor_indi`).
Airframe betikleri `rc.autostart` tarafından `.` ile source edilir, yani source
zinciri çalışır. Böylece çıkış fonksiyonları, tilt servo ölçekleri ve `.post`
durdurma listesi tek yerde kalır; iki dosyanın ayrışması imkânsızdır.

Üstüne yalnızca HITL farkları gelir: `SYS_HITL 1`, `HIL_ACT_FUNC1..11`,
emniyet devre kesiciler (`CBRK_SUPPLY_CHK`, `CBRK_IO_SAFETY`, `UAVCAN_ENABLE 0`),
`COM_RC_IN_MODE 1`.

## SYS_HITL tek başına yetmez

PX4 kaynağından okundu (tahmin değil): HIL'i fiilen açan şey
`Commander::instantiate()` içindeki `argv[1] == "-h"` kontrolüdür
(`Commander.cpp:2669`), parametre okuması değil. `SYS_HITL > 0`, `rcS`'in
(satır ~342) şu dalını tetikler:

```sh
pwm_out_sim start -m hil     # gerçek PWM yerine HIL kanalları
sensors start -h             # gerçek sensörler yerine HIL_SENSOR
commander start -h           # HIL_STATE_ON -- asıl anahtar
param set GPS_1_CONFIG 0     # gerçek GPS kapatılır
```

## ⚠ Sessiz tuzak: datarate

`Mavlink::set_hil_enabled()` HIL akışlarını **yalnızca `_datarate > 5000`**
ise açar (`mavlink_main.cpp:666`). Bağlantı yavaşsa HIL sessizce açılmaz —
hata da vermez.

- USB (`/dev/ttyACM*`): datarate 100000'e sabitlenir → sorun yok
- UART: `datarate = baudrate / 20`. 921600 → 46080 ✅, **57600 → 2880 ❌**

Yani HITL'i düşük baud'lu bir telemetri linki üzerinden kurmayın.

## Kullanım

```bash
# kartta
param set SYS_AUTOSTART 14003
param save
reboot

# bilgisayarda (SITL gz koşarken)
cd hitl && python3 gz_hil_bridge.py --dev /dev/ttyACM0 --baud 921600
```

## Hâlâ açık

`pump_actuators()` **bilerek yazılmadı**: kanal eşlemesi artık biliniyor
(`gz_hil_bridge.py`, `HIL_CHANNELS`) ama **ölçek** bilinmiyor — PX4 normalize
(0..1) gönderir, gz motor modeli rad/s bekler. Tahminle yazmak Adım 11'in
bedelini tekrarlamak olur. Kart bağlandığında ilk iş `log_actuator_scale()`
ile ham `controls[]` değerlerini kaydedip ölçeği **ölçmek** (madde K4).
