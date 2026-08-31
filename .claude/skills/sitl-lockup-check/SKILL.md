---
name: sitl-lockup-check
description: Runs the mc_indi_tiltrotor SITL hover+climb repro test (arm, 6m climb, ≥25s observed) to check for the known WLS actuator lockup / yaw runaway issue. Use whenever a control-constant change (Wu, Ws, Kp, ROTOR_*, LESO settings) needs SITL validation, or when asked to check/test/reproduce the T0/T1 lockup or yaw divergence bug, or before claiming any fix to this issue is confirmed.
---

# SITL kilitlenme/yaw-savrulma kontrolü

Bu skill, `mc_indi_tiltrotor` airframe'inde (PX4 SITL, `gz_tiltrotor_indi`)
bilinen WLS aktüatör kilitlenmesi / yaw savrulması sorununu (bkz.
`sitl/RUNBOOK.md` §4, `sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md`) standart,
güvenli bir prosedürle test eder.

## Neden bu skill var

Bu testi elle çalıştırmak bu projede defalarca şu tuzaklara düştü:
- **Sandbox modu px4/gz sürecini sessizce engelliyor** — `dangerouslyDisableSandbox: true` ZORUNLU (yoksa süreç hiç çıktı vermeden exit code 1 ile ölür).
- **`run_in_background` gerekli** — arka plan `&`'ı tek bir Bash çağrısı içinde kullanmak, çağrı bitince süreci öldürür.
- **Komut zincirinin BAŞINDA `pkill` olursa** (eşleşen süreç yoksa exit 1 döner) arka plan sarmalayıcısı TÜM script'i hiç çalıştırmadan "failed" işaretliyor — `pkill ... || true` kullanın ya da pkill'i AYRI, önce çalışan bir foreground adıma koyun.
- **`px4-listener <topic> -n N` (N>1) disarmed'ken takılabilir.**
- **KISA testler (≤10s) YANILTICI** — sorun tipik olarak ~15-25s'de tetikleniyor; bu projede daha önce bir "düzeldi" izlenimi, yalnızca 25s'lik testte gerçek arızanın ortaya çıkmasıyla yanlış çıkmıştı (bkz. Aday çözüm 3).
- **Süreç temizliği `pkill` ile garanti değil** — bazen `pkill` deseni eşleşmiyor (özellikle arka plan sarmalayıcısı farklı bir shell içinde başlattığında); `ps aux | grep -iE "px4_sitl|bin/px4|gz sim"` ile PID'leri bulup `kill -9` ile doğrulayın.

## Prosedür

1. **Ön temizlik** (foreground, `dangerouslyDisableSandbox: true`):
   ```bash
   ps aux | grep -iE "px4_sitl|bin/px4|gz sim" | grep -v grep
   ```
   Eşleşen PID varsa `kill -9 <pid...>`, sonra `rm -f /tmp/px4-sock-0 /tmp/px4_lock-0 /tmp/px4.log`.

2. **PX4'ü arka planda başlat** (`run_in_background: true`,
   `dangerouslyDisableSandbox: true`, pkill YOK bu adımda):
   ```bash
   rm -f /tmp/px4-sock-0 /tmp/px4_lock-0 /tmp/px4.log
   cd ~/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridge
   PX4_SIM_MODEL=gz_tiltrotor_indi HEADLESS=1 ../../../../bin/px4 -d > /tmp/px4.log 2>&1
   ```

   **Kullanıcı uçuşu izlemek isterse** `HEADLESS=1`'i kaldırın ve `DISPLAY=:1`
   ekleyin. Bu durumda "Ready for takeoff"tan sonra **kamerayı modele
   kilitlemek ZORUNLUDUR** — bu kontrolcüde yatay konum döngüsü yok, araç
   yüzlerce metre sürüklenip görüş alanından çıkar (bir koşuda ~170 m ölçüldü):
   ```bash
   "tiltrotor_Matlab files/sitl/gz_follow.sh"   # varsayılan offset: -2 0 0.8
   ```
   Ayrıca **irtifadayken disarm etmeyin** — araç düşüp takla atar ve sonraki
   arm "ters uçuyor" gibi görünen, aslında yerde yatan bir duruma yol açar.
   Önce alçak bir `z_sp` yayınlayıp indirin, sonra disarm edin.
   `gz` follow modu kamerayı modelin çerçevesinde taşıdığı için yaw dönüşü
   ekranda "yer dönüyor" gibi görünebilir; **yaw'ı gözle değerlendirmeyin.**

3. **Hazır olmasını bekle** (foreground):
   ```bash
   for i in $(seq 1 30); do grep -qa 'Ready for takeoff' /tmp/px4.log 2>/dev/null && { echo READY; break; }; sleep 1; done
   ```

4. **Arm et, setpoint yayınla, EN AZ 25 saniye izle**
   (`dangerouslyDisableSandbox: true`, `timeout` en az 40000ms):
   ```bash
   export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH
   px4-commander arm -f
   z0=$(px4-listener vehicle_local_position | grep -m1 '^\s*z:' | awk '{print $2}')
   z_sp=$(python3 -c "print($z0 - 6.0)")
   px4-mc_indi_tiltrotor test_sp 0 0 0 0 $z_sp 1 1 0   # son 3 arg: leso roll/pitch/yaw, gerekirse degistir

   # ZORUNLU: setpoint'in GERCEKTEN ulastigini dogrula (bkz. asagidaki not)
   px4-listener tiltrotor_indi_setpoint

   sleep 26
   px4-commander disarm -f
   ```

   **Setpoint dogrulamasi ATLANMAMALI.** `test_sp` "published test
   setpoint" yazsa bile setpoint kontrolcuye ulasmamis olabilir; o zaman
   `px4-listener tiltrotor_indi_setpoint` **"never published"** doner ve
   kontrolcu sessizce `z_sp = lpos.z` (mevcut irtifayi koru) +
   `roll/pitch/yaw_sp = 0` yedegine duser — yani **tirmanis senaryosunu
   degil, irtifa-koruma senaryosunu** test etmis olursunuz. Bu tuzak
   2026-07-27'de (Adim 11e) bulundu ve o ana kadarki 10 adimlik tum
   kosum gecmisini gecersiz kildi: sebebi `custom_command()`'daki
   fonksiyon-yerel `uORB::Publication`'in donerken `orb_unadvertise()`
   cagirmasiydi (duzeltildi — publication artik `static`). Cikti
   "never published" ise DURUN: kosuyu gecerli saymayin.

   Ek capraz kontrol: gercek bir tirmanis komutu varken `nu_des(4)`
   birkac ON N buyuklugunde NEGATIF olur; ~0'a yakinsa setpoint
   uygulanmiyordur.
   Görev sırasında ~t=10-15s civarında ara ara
   `px4-listener tiltrotor_indi_status | grep -E "u_actual|sat_flag|d_hat|nu_des"`
   ile canlı gözlem faydalı ama zorunlu değil — asıl veri log'dan çıkarılır.

   **YAW'I AÇIDAN ÖLÇMEYİN (2026-07-27, Adım 12b'de öğrenildi).** Birkaç
   saniyede bir `vehicle_attitude`'dan yaw açısı okumak, sürekli bir dönüşü
   (ör. 1.4 rad/s) rastgele açılara dönüştürür ve "sınırlı gezinme" gibi
   gösterir — bu, Adım 11'in sonucunu bir adım boyunca yanlış yönlendirdi.
   Doğrusu **yaw hızı**:
   ```bash
   px4-listener vehicle_angular_velocity | grep -E '^\s+xyz:'   # xyz[2] = yaw hizi
   ```
   Kesin analiz için ulog kullanın (`pyulog` kurulu; loglar
   `~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/<tarih>/*.ulg` altında —
   px4 log satırındaki göreli yol yanıltıcıdır).

5. **Sonuçları çıkar** (`/tmp/px4.log`'dan, eğer `T0dbg`/`T2dbg`/`WRdbg`
   tanı logları `TiltrotorIndiControl.hpp`'de aktifse):
   ```bash
   grep "T0dbg1" /tmp/px4.log | awk -F'nu_des=\\[' '{print NR": "$2}'

   # WRdbg (Adim 11): kanat rotorlerinin kilitlenip kilitlenmedigi + hangi
   # nu_des ekseninin onlari surukledigi. Kilitlenme sayisi 0 olmali:
   grep -c "Wu0=1000000" /tmp/px4.log; grep -c "Wu1=1000000" /tmp/px4.log
   # lo0/lo1 = -u_actual; 0.00'a yaklasmasi rotorun tabana indigini gosterir
   grep "WRdbg0" /tmp/px4.log | sed -n 's/.*lo0=\(-\?[0-9.]*\).*/\1/p' | sort -n | tail -1
   ```
   `WRdbg` satirlarinda `a0`/`a1` dizisi `du`'nun eksen-bazli ayrismasidir
   (sirasiyla roll, pitch, yaw, Fx, Fz) — hangi eksenin o rotoru ittigini
   dogrudan gosterir. Yoksa yalnizca canli `px4-listener` gozlemlerine
   guvenilir.

   **Not:** `PX4_INFO` uzun satirlari sessizce kirpar (Adim 11'in ilk
   kosusunda `Wu1=1000000`'i `Wu1=100` diye kirpmisti) — yeni tani logu
   eklerken kritik alanlari format dizesinin BASINA koyun.

6. **Kapat ve temizle:**
   ```bash
   ps aux | grep -iE "px4_sitl|bin/px4|gz sim" | grep -v grep
   # bulunan PID'leri kill -9 ile öldür, sonra:
   rm -f /tmp/px4-sock-0 /tmp/px4_lock-0
   ```

## ZORUNLU: koşunun ileri hızını kaydedin (2026-07-27, Adım 16)

Bu kontrolcüde **yatay pozisyon döngüsü yok** ve airframe yapısal olarak
kendini ileri itiyor (hover'da net Fx ≥ 0, çünkü tüm tiltler ≥0). Sonuç:
arm'dan ~25 s sonra araç **~10 m/s** ile uçuyor olur. Bu önemli, çünkü
yaw eksenini asıl sönümleyen şey kontrolcü değil, **ileri hızdaki
aerodinamik rüzgâr gülü etkisi**:

| `yaw_sp=+30°` adımı | 11.6 m/s | 2.45 m/s |
|---|---|---|
| yanıt | monoton, ~%13 aşım, oturuyor | **±25° sönümsüz salınım** |

Yani "hover testi geçti" demek, aslında "10 m/s seyirde geçti" demek
olabilir. Her koşuda hızı kaydedin:

```bash
px4-listener vehicle_local_position | grep -E '^\s+(vx|vy):'
```

Gerçek düşük-hız davranışını görmek için pitch trim ile frenleyip
(`test_sp 0 0.061 0 0 <z_sp> 1 1 0`, ~60-90 s) hız <3 m/s'ye indikten
sonra kriteri yeniden ölçün. Ayrıntı → rapor §4 (Q).

## Geçti/Kaldı kriteri

**GEÇTİ (sorun tekrar üretilmedi):** Test boyunca (25s+) hiçbir aktüatör
kalıcı olarak 0 veya tavan (45N) değerinde kilitlenmiyor, yaw açısı
±30°'yi aşmıyor, `vz` `ALT_VZ_MAX=2.0 m/s`'yi belirgin aşmıyor.

**KALDI (sorun hâlâ var):** Yukarıdakilerden herhangi biri ihlal
ediliyor — `sat_flag`'in `True` kaldığı bir aktüatör, `nu_des`'te
kalıcı/sıçramalı büyüme, ya da yaw'ın sürekli tek yönde dönmesi.

## Sonrasında

Sonucu (geçti/kaldı, ham `nu_des` zaman serisi, hangi aktüatör/eksen)
`sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md`'ye yeni bir "Adım N" olarak,
`sitl/RUNBOOK.md`'ye kısa bir özet olarak ekleyin — bu proje her denemeyi
(başarılı/başarısız fark etmeksizin) belgeleme disiplinini benimsemiştir.
