# Donanım testi hazırlık kontrol listesi — `mc_indi_tiltrotor`

**Hazırlanma tarihi:** 2026-07-29 (Adım 28 sonrası; B5 Adım 31'de, B1 Adım
32-35'te güncellendi — son güncelleme 2026-08-03, **Adım 39: madde (S)
KAPATILDI** — `BRAKE → HANDOFF` eşiği ve fren marjı artık hız BÜYÜKLÜĞÜNÜ
değil, yasanın kontrol ettiği ekseni (işaretli gövde ileri hızı) okuyor;
B5 🔴 → 🟠. Madde (T) 🟠 açık ama artık enstrümanlı.)
**Kısa cevap: HAYIR, donanıma geçmeye hazır değil.** Durum 🔴 **NO-GO** —
B1/B2/B3/B4 hâlâ 🔴 ve **hiçbir şey gerçek donanımda uçmadı**.

SITL'de artık her kriter geçiyor (Adım 27-28: kilitlenme yok, yaw kararlı,
duruş 0.06 m, asimetri 1.02×). Ama **SITL'de geçmek, donanımda uçmaya hazır
olmak demek değil** — ve bu projenin kendi geçmişi tam olarak bunun kanıtı:
Adım 11, 12, 21 ve 27'nin dördü de *kontrolcü ile plant arasındaki arayüz*
hatalarıydı ve **saf MATLAB bunları yapısal olarak göremiyordu**. Donanımda
plant GERÇEK; yani bu sınıftaki her hata doğrudan uçuşta ortaya çıkar.

Aşağıdaki listede 🔴 = uçuştan önce **zorunlu**, 🟠 = kısmen çözüldü ama
yalnızca simülasyonda doğrulandı, 🟡 = ilk uçuştan önce güçlü tavsiye,
🟢 = hazır/kapalı.

---

## 0-Z. KART TAKILDIĞINDA YAPILACAKLAR (2026-08-31, adım 141)

Kart olmadan yapılabilecek her şey bitti (bkz. §0-A). Bu bölüm, **Cube Orange
elde olduğu anda** sırayla yapılacak işlerin listesi. Hiçbiri tahminle
yazılamaz — hepsi ölçüm gerektirir.

### K1 — Firmware yükle ve airframe'i seç
```
~/PX4-Autopilot/build/cubepilot_cubeorange_default/cubepilot_cubeorange_default.px4
param set SYS_AUTOSTART 14002      # 14002_tiltrotor_indi
param set SYS_HITL 1               # HITL modu
```
Firmware **güncel** (2026-08-31 derlemesi, flash %84,50, RAM %8,21). Airframe
ve `.post` ROMFS'te doğrulandı.

### K2 — ⚠ ÇIKIŞ KABLOLAMASINI DOĞRULA (en kritik madde)

`14002_tiltrotor_indi` şu dağılımı **varsayıyor**; modül hangi pinin ne
sürdüğünü bilmez, eşlemeyi bu parametreler yapar:

| Cube Orange pini | bağlı olmalı | fonksiyon |
|---|---|---|
| MAIN 1 | sağ kanat rotoru ESC | Motor1 (101) |
| MAIN 2 | sol kanat rotoru ESC | Motor2 (102) |
| MAIN 3 | kuyruk rotoru ESC | Motor3 (103) |
| MAIN 4 | sol elevon | Servo1 (201) |
| MAIN 5 | sağ elevon | Servo2 (202) |
| MAIN 6 | sol elevator | Servo3 (203) |
| MAIN 7 | sağ elevator | Servo4 (204) |
| MAIN 8 | rudder | Servo5 (205) |
| **AUX 1** | **sol kanat tilt servosu** | Servo6 (206) |
| **AUX 2** | **sağ kanat tilt servosu** | Servo7 (207) |
| **AUX 3** | **kuyruk tilt servosu** | Servo8 (208) |

**İki sessiz ve pahalı hata sınıfı:**
1. **Sıra karışması** (sağ/sol rotor ters): araç ters yöne roll yapar.
   Kontrolcü doğru komut verir, tel yanlış motora gider.
2. **Tilt servoları MAIN'e takılırsa:** `PWM_AUX_MINA1/MAXA1 = 0..90`
   ölçeklemesi uygulanmaz, sürücü varsayılan ±57,3°'ye düşer ve tilt komutu
   YANLIŞ ÖLÇEKLENİR — 45° komutu ~28° gerçek açı verir.

Doğrulama (pervanesiz!):
```
px4-actuator_test set -f 101 -v 0.1    # MAIN1 -> sag kanat rotoru
px4-actuator_test set -f 206 -v 0.5    # AUX1  -> sol kanat tilt ~45 deg
```

### K3 — HITL köprüsünün MAVLink yarısı
`hitl/gz_hil_bridge.py`. gz tarafı **ölçüldü** (IMU 250 Hz, 3/3 abone);
MAVLink tarafı `[YAZILDI]` ama kartta hiç koşmadı.
```
./hitl/run_hitl_check.sh                                    # once gz tarafi
python3 hitl/gz_hil_bridge.py --dev /dev/ttyACM0 --baud 921600
```
⚠ **İlk kontrol çerçeve dönüşümü:** gz FLU → PX4 FRD `(x, −y, −z)`. Bu
projenin en pahalı iki hatası (Adım 12 `ROTOR_KM`, engelleyici B4 `ROTOR_PY`)
tam olarak bu dönüşümün atlanmasından doğdu. Kart ters işaretli ivme görürse
derhal devrilir. Doğrulama: araç yatay dururken `az ≈ +9.81` (FRD'de aşağı
pozitif) okunmalı.

### K4 — `pump_actuators()` ölçeğini ÖLÇ
Yayın tarafı hazır ve ölçüldü; eksik olan **ölçek**. PX4 normalize (0..1)
gönderir, gz motor modeli rad/s bekler (`SIM_GZ_EC_MIN/MAX = 10/1500`).
Bu dönüşüm `thrustToNormalized()`'in tersidir ve yanlış ölçek = yanlış itki —
Adım 11'in tam olarak ödediği bedel.

**İlk iş tahmin etmek değil, ölçmek:** karttan gelen `HIL_ACTUATOR_CONTROLS`
mesajının `controls[0..2]` değerlerini logla, hover komutunun hangi sayıya
karşılık geldiğini gör, sonra eşlemeyi yaz.

### K5 — Manyetometre yolunu çöz
gz'de manyetometre konusu **YOK**. SITL'de onu Gazebo değil PX4'ün kendi
`sensor_mag_sim` modülü üretiyor (`SENS_EN_MAGSIM 1`). Gerçek kartta o modül
çalışmayacak — kart gerçek bir manyetometre taşıyor ama o, SITL'in dünyasından
habersiz ve **yanlış heading** verir.

Üç seçenek, hiçbiri ölçülmedi:
(a) gz dünyasına bir magnetometer sensörü eklemek ve köprüden `HIL_SENSOR`'un
    mag bitleriyle göndermek (en temiz, `BIT_MAG` zaten kodda);
(b) kartın gerçek manyetometresini kullanıp SITL'i onun heading'ine hizalamak;
(c) `HIL_STATE_QUATERNION` ile tutumu doğrudan enjekte etmek (EKF'i baypas
    eder, yani HITL'in test ettiği şeyin bir kısmını kaybeder).

### K6 — B3: itki eşlemesini gerçek motor/ESC eğrisinden türet
`thrustToNormalized()` hâlâ **Gazebo sabitlerine** kalibre: `ROTOR_KF = 2e-5`,
`ROTOR_WMAX = 1500`, `ROTOR_WMIN = 10`. Aynı sayıları gerçek donanıma taşımak
yeni bir itki eşleme hatası yaratır (Adım 11).

Airframe dosyasında `PWM_MAIN_MIN/MAX` ve ESC kalibrasyonu **bilerek yok** —
ikisi de bu ölçüme bağlı. Statik itki tezgâhı gerekir: her rotor için
komut → devir → itki eğrisi.

### K7 — B2: fiziksel sabitleri ÖLÇ
Kütle, atalet, rotor konumları, `km` — hepsi bugün SDF'ten geliyor, **hiçbiri
ölçülmedi**. Ağırlık merkezi de: `check_mass_balance.py` bu oturumun geometri
değişikliklerinin CG'yi **16 mm geriye** kaydırdığını hesapladı, ama gerçek CG
ancak tartılarak bulunur (statik pay hesabı %40,7 → %34,2; tipik hedef %5-15,
yani hâlâ fazlasıyla kararlı tarafta).

### K8 — B4: `ROTOR_PY` işaretini fiziksel olarak doğrula
Simülasyon tarafı çözüldü, fiziksel doğrulama duruyor. Araç sabitken tek bir
kanat rotorunu düşük gazda çalıştırıp roll momentinin yönünü gözle.

### K9 — H7: kartta tick jitter ölçümü
Döngü 400 Hz (2,5 ms bütçe). GUI koşumları bu gövdenin zamanlama sarsıntısına
duyarlı olduğunu gösterdi (aynı ikili: GUI'de 112 BIG_M, headless 0).
`perf` çıktısı ve `_loop_perf` sayaçları ile ölç.

### K10 — H8: RC yolu (`RC_CHANNELS_OVERRIDE`)
Adım 40'ta "sınanmadı" diye kayıtlı; tüm pilot testleri MAVLink
`MANUAL_CONTROL` üzerinden yapıldı. VTOL anahtarı → `bt_enable` yolu bu
kanaldan geliyor.

---

## 0-A. HITL KAPISI (2026-08-29, Adım 120-121 — bu bölüm YENİ)

Bu listenin geri kalanı **serbest uçuşu** hedefliyor ve 3 Ağustos'ta (Adım 39)
dondu; çalışma logu ise Adım 121'de. HITL ayrı ve daha yakın bir kapı: plant
hâlâ Gazebo olduğu için **B2/B3/B4 HITL'i bloklamaz** (hepsi plant gerçek
olunca ısırır). HITL'e özgü durum:

| # | madde | durum |
|---|---|---|
| H1 | Modül gerçek kartta derleniyor | 🟢 **KAPANDI** (Adım 120) — `cubepilot_cubeorange`, `CONFIG_MODULES_MC_INDI_TILTROTOR=y` eklendi |
| H2 | `wlsAllocate` yığın taşması | 🟢 **KAPANDI** (Adım 120) — 2184 B > 2048 B sınırı; `WlsScratch` ile modül üyesine taşındı |
| H3 | Flash bütçesi | 🟢 **KAPANDI** (Adım 121-122) — %97,98 → **%84,03**; 13 modül budandı, her turda SITL'de uçurularak doğrulandı. `MC_POS_CONTROL` bilerek KALDI (`mc_hover_thrust_estimator` çalışıyor ve `MPC_THR_HOVER`'ı okuyor) |
| H4 | RAM bütçesi | 🟢 %8,21 (43 KB / 512 KB) — sorun değil |
| H5 | Gerçek kart airframe dosyası + param seti | 🟢 **KAPANDI** (Adım 122) — `14002_tiltrotor_indi` + `.post`, `CMakeLists.txt`'e koşulsuz kaydedildi. Çıkış atamaları SITL sırasıyla birebir; **kablolamaya göre doğrulanmalı** |
| H6 | `SYS_HITL` + sensör köprüsü yapılandırması | 🟡 **KURULDU, KARTTA KOŞMADI** (Adım 149) — `14003_tiltrotor_indi_hitl` airframe'i eklendi ve ROMFS'e girdiği doğrulandı; 14002'yi kopyalamaz, **kaynak gösterir**. `SYS_HITL 1` + `HIL_ACT_FUNC1..11` (14002'nin çıkış sırasıyla birebir) + emniyet devre kesiciler. Kaynaktan okundu: HIL'i açan şey parametre değil `commander -h` (`Commander.cpp:2669`); `SYS_HITL>0` rcS'in o dalını tetikler. ⚠ `set_hil_enabled` **datarate > 5000** ister — USB'de 100000, ama 57600 baud UART'ta 2880 olur ve HIL **sessizce açılmaz**. Köprüye kanal eşlemesi (`HIL_CHANNELS`) ve ölçek kaydedici (`log_actuator_scale`) eklendi; `pump_actuators()` ölçek ölçülene kadar hâlâ açık (K4). |
| H7 | Kartta CPU/tick jitter ölçümü | 🔴 **AÇIK** — döngü 400 Hz (2,5 ms). GUI koşumları bu gövdenin zamanlama sarsıntısına duyarlı olduğunu gösterdi (Adım 121 öncesi ölçüm: aynı ikili, GUI'de 112 BIG_M / headless 0) |
| H8 | RC vericisi ↔ kart (`RC_CHANNELS_OVERRIDE` yolu) | 🔴 **AÇIK** — Adım 40'ta "sınanmadı" diye kayıtlı; tüm pilot testleri MAVLink `MANUAL_CONTROL` üzerinden |
| B0 | Kartta iniş/görev yolu | 🟡 **KISMEN KAPANDI** (Adım 153) — **iniş dizisi MODULE TASINDI**: `landingSequence()` (`TiltrotorIndiControl.hpp`), `LandState` IDLE→DESCEND→FLARE→TOUCHDOWN, tek bir `land_enable` bayrağıyla sürülür — `ft_enable`/`bt_enable` ile aynı kalıp. Sayılar PC betiğinden birebir taşındı (1 m kademe, 1,5 s periyot, flare 1,5 m, touch 0,15 m). Temas ölçütü irtifadan bağımsız: dikey itki < 0,5×ağırlık, 1,5 s kesintisiz. **Modül DISARM ETMEZ** — TOUCHDOWN yalnızca bildirir, karar dışarıda (PX4'ün land_detector/commander ayrımı gibi). SITL'de doğrulandı: 6/6, BIG_M 0, temas agl 0,01 m'de. 🔴 **AÇIK KALAN:** bayrakları (ft/bt/fw/land) kaldıracak bir görev katmanı hâlâ yok, ve kartta bir bayrağı kaldırmak bile RC yoluna bağlı (H8, sınanmadı). Ama artık dışarıdan gereken şey bir YÖRÜNGE değil, bir BOOL. |
| B1 | Pilot girişi + kademeli failsafe | 🔴 açık — HITL'in varlık sebebi; alçak irtifada hızlı link kaybı hâlâ 100+ m sürüklenmeyle bitiyor |

**ESP32 (companion) — ölçülmüş değerlendirme:** yetersizlik CPU'da değil
**flash'ta**. ESP32 iç döngüyü alamaz (400 Hz, 2,5 ms; UART/MAVLink üzerinden
taşınamaz), ama sürü kararı / görüntü-ses işleme / faydalı yük mantığını
üstlenirse karşılık gelen PX4 modülleri karttan düşürülebilir — katkısı
**flash boşaltmak** olur. Adım 121'den sonra bu baskı zaten büyük ölçüde kalktı.

---

## 0. Beş engelleyici (bunlar bitmeden hiçbir güçlü test yapılmamalı)

### 🔴 B1 — Pilot girişi ve failsafe: KOD YAZILDI, ölçüldü, kapsamı DARALTILDI

*(Adım 32-33 kodu yazdı — 2026-07-29; Adım 34 onu ilk kez ÖLÇTÜ ve seviye 3'ün
çalışmadığını buldu; Adım 35 seviye 3'ü ölçüp KALDIRDI — 2026-07-30.)*

**Bugünkü özet:** kademeli failsafe artık yalnızca **dış döngüler** hakkında bir
sav taşıyor (NO_POS ölçülerek doğrulandı, NO_ALT hâlâ tetiklenemedi). Duruş
kestirimi sert ön koşul oldu. **Pilot yolu 2026-08-03'te (Adım 40) İLK KEZ
ÇALIŞTIRILDI** — yedi dalın altısı ilk denemede doğru işledi, ama yedincisi
(link kaybı) savını tutmuyor: **madde (U), aşağıda.** B1 **hâlâ 🔴**.

> ### ✅ Adım 40 (2026-08-03) — pilot yolu çalıştırıldı, ve önce dört gizli taşma düzeltildi
>
> **Ulaşılabilirlik önce UÇMADAN ölçüldü** (`sitl/probe_pilot_link.py`). Modülün
> pilot dalı iki koşula bağlı ve asıl risk bizim gönderdiğimizde değil,
> **commander'ın verdiği** `flag_control_manual_enabled`'daydı: akış yokken
> `False`, MAVLink `MANUAL_CONTROL` akışı başlayınca **`True`** (valid=True,
> source=3, nav_state 4→2). Param değişmedi — `COM_RC_IN_MODE` varsayılanı 3
> zaten joystick kabul ediyor. `manual_control_switches` (VTOL anahtarı →
> `bt_enable`) yalnızca `RC_CHANNELS_OVERRIDE` ile geliyor; **o yol hâlâ
> sınanmadı.**
>
> **🔴 Dört gizli taşma.** Adım 32'nin bulduğu `(now - topic.timestamp) < T`
> deseni (iki `uint64`, `now` = gyro ÖRNEK zamanı) **dört yerde daha duruyordu**;
> Adım 32/33 düzeltmeyi yalnızca ısırdığı iki yere uygulamış. Ölçüldü:
> `estimator_status_flags` bu uçuşlarda hiç taşmıyor (39217 tick'te 0), ama
> `vehicle_attitude` **düzeltme olmasaydı %0.03 taşacaktı** — form hatalı, SITL
> maskeliyor (gz gyro'sunun publish−sample farkı ~0 µs, çünkü lockstep; **gerçek
> sensör hattında o gecikme gerçektir**). Dördü de toplama formuna çevrildi. En
> kritik ikisi: `tilt_aligned` (Adım 35'ten beri `att_ok` false ⇒ çıkış kesilir,
> yani taşma gerçek duruş kaybından ayırt edilemez — **madde (T)'nin imzası;
> (T)'yi kapatmaz ama Adım 38'in düşünmediği üçüncü aday mekanizmayı eler**) ve
> pilot tazeliği (orada topic gerçek zamanla damgalanıyor, yani "topic now'dan
> yeni" normal durum — taşan formda bir çubuk girdisi sessizce yok sayılırdı).
>
> **Uçuş sonucu** (`sitl/run_pilot_input_test.py`, iki bağımsız GUI'li uçuş):
> devralma tam bir kez ve basamaksız ✅; çubuklar ortada → `pos_hold`
> kendiliğinden (%28-30) ✅; **pilot sahipken max |roll| 8.2-8.3°, |pitch|
> 8.4-8.5°** = 0.5 çubuk × `MAN_TILT_MAX` 15° komutunun tam karşılığı ✅; yaw
> çubuğu 120-164° ✅; throttle 13.7-13.9 m ✅; havada NaN 0, BIG_M 0,
> `attitude LOST` 0 ✅. Zorunlu `sitl-lockup-check` geçti (%0.00/0, yaw −8.93°,
> irtifa RMS 0.089 m).
>
> ### 🔴 madde (U) — link kaybı dalı TUTAMADIĞI bir şeyi iddia ediyor
>
> Dal tetikleniyor ve `pilot input LOST -- holding position and descending`
> yazıyor. **Ölçüm: alçalma boyunca `pos_hold_active` hiç 1 olmuyor ve araç
> 4.8-6.1 m/s ile uçarak iniyor, ~100 m ileride yere değiyor** (iki uçuşta da,
> ortalama 5.43 / 5.46 m/s). Zincir tamamen bilinen parçalardan: çubuk
> evrelerinde madde (N)'nin yapısal ileri ivmesi aracı 9.3 m/s'ye çıkarıyor →
> `pos_hold REFUSED: 9.3 m/s > 3.0` → link kaybı dalı `req_pos_hold = true`
> diyor ama bu yalnızca bir **İSTEK**, `POS_ENGAGE_V_MAX` reddetmeye devam
> ediyor → ve dal, aracı yavaşlatabilecek tek mekanizmayı **açıkça kapatıyor**:
> `req_bt = false`. Kontrol listesi bunu Adım 34(d)'de zaten yazmıştı
> (*"Station keeping must be handed to `bt_enable` on recovery — not wired"*),
> ama orada kurtarma yolundaydı; **burada müdahale edecek kimsenin olmadığı
> durumda.** Madde (S) ile aynı yapısal desen: alıcının reddedebildiği bir
> istek, ve reddedilirse yedeği olmayan bir tasarım.
>
> **✅ ADIM 41'DE DÜZELTİLDİ VE DOĞRULANDI (2026-08-03).** `req_bt = false` yerine
> **`req_bt = !_pos_hold_active && (v_h > POS_ENGAGE_V_MAX || _bt_state != IDLE)`**
> — yani hold kabul edemeyecek kadar hızlıysa araç, işi tam olarak "seyirden
> hover'a" olan makineye devredilir. `_bt_state != IDLE` **mandalı** şart:
> yoksa hız eşiğin altına düştüğü anda `req_bt` düşer, makine IDLE'a sıfırlanır,
> tavan bırakılır ve bir tick sonra yeniden başlar — manevra bitmek yerine
> sürekli baştan başlardı. Bırakma koşulu `_pos_hold_active`, çünkü manevranın
> varlık sebebi zaten odur. Alçalma bunun altında bağımsız olarak sürer: geri
> dönecek bir pilot yok, iniş yavaşlamayı BEKLEMEMELİ.
>
> **Ölçüm — düzeltme öncesi/sonrası, aynı test:**
>
> | | link kaybı irtifası | yatay yol (kayıptan temasa) | temas v_h | alçalırken ort. v_h |
> |---|---|---|---|---|
> | öncesi | 7-13 m | **104-110 m** | **4.96-5.39 m/s** | 5.43-5.46 |
> | sonrası | 17.8 m | **13 m** | **0.16 m/s** | **1.33** |
>
> px4 log'unda dizinin tamamı görünüyor: `pilot input LOST at 5.0 m/s -- too fast
> ... handing over to the back-transition` → `state 0 -> 1 -> 2 -> 3` →
> `pos_hold: holding` → `state 3 -> 0`. Yedi ölçütün yedisi de geçti.
>
> **ÖNEMLİ SINIR — düşük irtifada bu düzeltme DEVREYE GİRMEZ ve girmemeli.**
> İlk denemede link kaybı 7.2 m AGL'de oluştu ve log `back-transition REFUSED:
> 7.2 m AGL < 15.0 m minimum` yazdı: 30-40 s'lik bir manevra için yer yok, doğru
> cevap zaten "hemen in". Yani madde (U) **irtifası olan** link kaybı için
> kapandı; **alçak irtifada hızlı link kaybı hâlâ 100+ m sürüklenmeyle biter** ve
> bunun çözümü geri geçiş değil (yatay eksene doğrudan bir fren gerekir).
> Test senaryosu bu yüzden tırmanışı 13 m'den ~30 m'ye çıkardı — düzeltmenin
> hedeflediği rejimi kurmayan bir koşu onu sınamaz (Adım 39'un dersi).

> ### 🔴 madde (V) — İLERİ GEÇİŞ bir kontrol yasası değil; otonom görev bu yüzden eksik
>
> *(2026-08-03, Adım 41'de "pilot komut edemiyor" olarak açıldı; **aynı gün
> gereksinim netleşince 🟠 → 🔴 yükseltildi ve yeniden çerçevelendi.** Hedef tam
> otonom görevdir — kalkış multikopter, seyir sabit kanat/tilt motor, iniş
> multikopter — yani sorun "pilot süremiyor" değil, **hiçbir şey otonom
> süremiyor**: ileri geçişin tek yolu bir hata ayıklama konsoludur.)*
>
> **Eksikti üç parça; biri ADIM 42'DE KAPANDI:**
> 1. ~~İleri geçiş yasası yok~~ → **✅ YAZILDI VE UÇTU (Adım 42, 2026-08-03).**
>    `forwardtrans_loop.m` / `forwardTransition()`, üç durum
>    (IDLE→RAMP→CRUISE), `ft_enable` bayrağı. **Tilt komut ETMEZ** — `fx`'i
>    rampalar, tilti WLS seçer; gerekçe ölçüm: ileri yönde tahsisatın kendi
>    tercihi zaten doğru yöne bakıyor (geri yönde bakmıyordu, orada KUTU KISITI
>    gerekmişti). Pitch her zaman 0 (Adım 29), `pos_hold` bırakılır, iptal
>    **geri geçişi ister** (`fx = 0` işe yaramaz — Adım 30). İki emniyet farklı
>    cinsten: irtifa bandı 5 m (aero-bağımlı) + süre 30 s (aero-bağımsız).
>    **Doğrulama: iki bağımsız otonom görev uçuşu** — kalkış → ileri geçiş
>    (14.4 s) → **tilt 44.1° / 15.2 m/s seyir** → geri geçiş → hover → iniş,
>    elle hiçbir komut yok. İrtifa sapması 0.58-0.80 m, doyum %0.00, BIG_M 0,
>    NaN 0, iptal 0. Mantık testi 16/16, MATLAB regresyonu nötr,
>    `sitl-lockup-check` geçti. Test: `sitl/run_mission_test.py`.
> 2. **Seyirde enerji yönetimi yok** (TECS benzeri bir yasa) — 🔴 AÇIK. Adım 29
>    ölçtü: ~5-6 m/s üstünde burun yukarı bir *tırmanma* komutudur ve irtifa
>    döngüsü kanat taşımasına karşı koyamaz. Şu an seyirde `fx` sabit tutuluyor.
> 3. **Görev dizicisi yok** — 🔴 AÇIK. Evreleri hâlâ dışarıdan gelen iki bayrak
>    zincirliyor. PX4'ün `navigator`/`flight_mode_manager`'ı bu airframe'de
>    kasıtlı durdurulmuş, yani diziyi modül yürütmeli.
>
> **Madde (V) bu yüzden 🔴 kalıyor ama kapsamı daraldı:** eksik olan artık
> geçiş yeteneği değil, onu bir GÖREVE bağlayan katman.
>
> ### 🟠 Adım 48 (2026-08-04) — ileri geçiş İPTAL ETMİYOR; ve iptalin kaçış yolunun sıfır marjlı olduğu bulundu
>
> **GEREKSİNİM:** görev profili tek parçadır ve ileri geçiş kendini kesmez
> (`FT_ALLOW_ABORT = false`). İki emniyet dedektörü **silinmedi** — çalışıyor ve
> `warn_code` ile loglanıyor; yalnızca eylemleri kalktı. `true` yapmak eski
> davranışı birebir geri getirir ve iki mod da test ediliyor (21 denetim; uyarı
> iptalin tetikleneceği **tam aynı anda** çıkıyor, t = 4.56 s).
>
> **🔴 DONANIM İÇİN ÖNEMLİ BULGU (kod okumasıyla, uçuşla DEĞİL):** iptalin
> garantili bir kaçış yolu yoktu. İptal = geri geçişi İSTEMEK (Adım 30 ölçtü:
> `fx_sp = 0` tiltleri geri çekmiyor, araç yavaşlamıyor), ve geri geçiş
> `BT_MIN_ALT` altında başlamayı reddediyor. Sabitler sıfır marj bırakıyordu:
>
> ```
> FT_MIN_ALT (20 m)  −  FT_ALT_BAND (5 m)  =  15 m  =  BT_MIN_ALT
> ```
>
> Yani **alçalarak** iptal eden bir geçiş, tam olarak kaçışının reddedildiği
> irtifada iptal ediyordu — o köşede iptal etmek etmemekten kötüydü (seyir
> hızında, tiltler önde, kimsenin sahiplenmediği araç). Madde (U) ve (R) ile
> aynı kalıp. **İptal ileride geri getirilirse ÖNCE bu marj kapatılmalıdır.**
>
> Doğrulama: PX4 derlemesi temiz, tam otonom görev **6/6 GEÇTİ** (FT
> RAMP→CRUISE tilt 42.5°, BT RETRACT→BRAKE→HANDOFF, iptal ×0, NaN 0,
> doygunluk %0.00).
>
> ### 🔴 Adım 46 (2026-08-03) — 3. deneme MATLAB'da ÇALIŞIYOR, ama PX4'e GİTMEDİ; ve TECS bir ENGELLEYİCİYE dönüştü
>
> Kontrol yüzeyleri artık beş bağımsız aktüatör değil **üç sanal aktüatör**
> (aileron = antisimetrik elevon → Fz ve pitch tam cancel; elevator = simetrik
> servo_2/3, 0.70 m dürüst kuyruk kolu; rudder). Simetrik elevon (flap) bilerek
> aktüatör değil — ağırlık penceresi hesabı onun için **boş** çıkıyor
> (131.2 < wu < 43.5, ters), yani Adım 45/deneme 1 hiçbir ağırlıkla
> kurtarılamazdı. MATLAB'da fx = 10 N / 70 s: tilt **52.1° → 70.6°**, kanat yük
> payı **%98.9 → %106.4**, toplam rotor itkisi **24.0 → 16.1 N**, max |ω|
> 0.021 → 0.013 (iyileşti).
>
> **SITL'e gönderilmedi, çünkü MATLAB kapısı temiz değil:**
> 1. 🔴 **Aileron, elevator olmadan açıkken ıraksıyor** (fx = 10 ve 12'de
>    bağımsız olarak tekrarlandı). Yüzeyler ancak BİRLİKTE devreye alınabilir.
> 2. 🔴 **fx ≥ 12 N'de rejim yüzeyler KAPALIYKEN BİLE marjinal**: doygunluk
>    %13.5-16.1, kanat tilt'i 90° mekanik durakta çakılı. Aracı yavaşlatan şey
>    bir yasa değil o durak; yüzeyler bu kazara korumayı kaldırıyor.
>
> **(2)'nin sonucu: seyirde enerji yönetimi (TECS) artık "eksik" değil,
> yüzeylerin ÖN KOŞULU.** Sıradaki iş yüzey ayarı değil, o katman.
>
> **Aynı adımda MATLAB plant'inde iki gerçek kusur bulundu ve düzeltildi:**
> kanadın taşıma **işareti tersti** (15 m/s'de burun yukarı 43-67 N *aşağı*) ve
> sürükleme ~7 kat fazlaydı. Bu, hem donanım riski açısından hem de metodolojik
> olarak önemli: **bu plantla yapılmış her MATLAB seyir/geçiş değerlendirmesi,
> Adım 46 öncesinde ters işaretli bir kanatla yapılmıştı.** Yeni plant
> (`aero_panels.m`) SITL'de ölçülen kanat yük payını (%41-50) bağımsız olarak
> yeniden üretiyor.
>
> **Aşağıdaki bölüm maddenin ilk (pilot merkezli) yazımıdır — ölçümü geçerli,
> çerçevesi güncellendi.**
>
> ### 🟠 madde (V) ilk yazımı — pilot ILERI geçişi hiç komut edemiyor (2026-08-03, Adım 41)
>
> GUI'li bir izleme sırasında kullanıcı sordu: *"tilti hiç görmedim, drone
> konseptiyle mi uçuyor?"* — ve haklıydı. Ölçüldü: pilot uçuşlarında kanat tilti
> havada yalnızca **3.7-15.1°** arasında oynuyor (yaw triminin δ0'ı tuttuğu
> aralık, madde (P)); araç **saf multikopter** olarak uçuyor.
>
> Sebep kodda tek satır: pilot dalı `fx_sp = 0.0f` **sabitliyor**. Çubuklar
> yalnızca duruş (≤ `MAN_TILT_MAX` = 15°) ve tırmanma hızı komut ediyor. Tek
> geçiş kontrolü VTOL anahtarı → `req_bt`, o da **geri** geçiş. Yani
> **pilot, hiç başlatamayacağı bir geçişi bitirebiliyor.** Seyre çıkmanın tek
> yolu tezgâh komutu `test_sp`'nin `fx` argümanı — yani bir hata ayıklama
> konsolu.
>
> Koddaki yorum *"gives the pilot both halves that blocker B5 called out as
> missing: they can START the manoeuvre and they can CANCEL it"* diyor; bu doğru
> ama **geri geçişin iki yarısı** için — uçuşun iki YÖNÜ için değil. Sav ile
> kapsamın karıştığı bir yer daha (Adım 35'in dersi).
>
> **Donanım açısından:** bir pilot bu araçla seyir uçuşu yapamaz, yani B5'in
> çözdüğü yetenek pilotun erişemediği bir yetenektir. Düzeltme yönü açık (bir
> çubuk/anahtar → `fx_sp` ya da bir ileri-geçiş isteği) ama **tasarım kararı
> gerektiriyor**: `fx_sp`'yi doğrudan pilota vermek, madde (N)'nin yapısal ileri
> ivmesiyle birleşince `pos_hold`'un reddedeceği hızlara çıkmanın en kısa yolu —
> yani madde (U)'nun tetikleyicisi. Ölçülmeden uygulanmamalı.

**Başlangıç durumu (Adım 31 sonu).** Modülde `manual_control_setpoint`,
`rc_channels`, failsafe, geofence, kill-switch **hiçbiri yoktu**. Yalnızca
`vehicle_control_mode.flag_armed` okunuyordu; tek setpoint kaynağı `test_sp`
konsol komutuydu. Yani araç havadayken müdahale yolu yoktu ve her bozulmuş
girdiye tek cevap `publishDisarmed()` — havada **motorları kesmek** — idi.

**Adım 32-33'te yazılanlar.** Dört seviyeli kademeli failsafe (`FsLevel`:
NONE / NO_POS / NO_ALT / RATE_ONLY — her biri yalnızca girdisi kaybolan
döngüyü kapatır; **RATE_ONLY adım 35'te kaldırıldı, aşağıya bakın**),
`manual_control_setpoint` üzerinden pilot girişi (açı
komutu roll/pitch, tazyikli yaw hız komutu + leash, throttle → tırmanma
hızı, çubuklar ortada → `pos_hold`), VTOL geçiş anahtarından `bt_enable`,
link kaybında pozisyon tut + alçal, ve `tiltrotor_indi_status.failsafe_level`
telemetrisi. Tasarımın dayanağı yapısal: **INDI hız döngüsü yalnızca ω ve
ω̇ istiyor**, ikisi de `vehicle_angular_velocity`'den geliyor — yani Run()
çalışıyorsa iç döngünün ihtiyacı tanım gereği mevcut.

Aynı oturumda **gerçek bir hata da bulundu ve düzeltildi**: eski tazelik
testi `(now - att.timestamp) < 50_ms` iki `uint64` üzerinde işaretsiz taşma
yapıyordu (`vehicle_attitude` bu tick'in gyro örneğinden ~4 ms İLERİDE
yayınlanıyor, çıkarma ~1.8e19'a sarıyor) → uçuş başına **7-13 kez havada
tüm motorlara NaN**. Görünmezdi çünkü hiçbir şey loglamıyordu.

**Adım 34'ün ölçümü (`sitl/run_failsafe_test.py`, seviye başına bir uçuş).**
Enjeksiyon, seviyeyi zorlayan bir kanca yerine kestirimi gerçekten alarak
yapıldı (kod her döngüde o döngünün KENDİ girdisine bakıyor, `_fs_level`
yalnızca en-kötü özeti — seviyeyi zorlamak dal gövdelerini çalıştırmazdı):

| Seviye | Enjeksiyon | Sonuç |
|---|---|---|
| 1 NO_POS | `EKF2_GPS_CTRL = 2` | ✅ **GEÇTİ.** 5 s sonra fs=1, motorlar kesilmedi (%0.0 NaN), **irtifa korundu**, açısal hız baseline'ın altında (0.081 vs 0.131), 0 BIG_M, geri alındığında ~3 s'de temizlendi |
| 2 NO_ALT | baro + GPS yükseklik kapalı | ⚠️ **TETİKLENEMEDİ** (iki farklı denemede) |
| 3 RATE_ONLY | `ekf2 stop` | ❌ **ARAÇ DÜŞTÜ** — aşağıya bakın. **Adım 35'te bu seviye KALDIRILDI** |

**❌ Seviye 3, kademeli tasarımın var olma sebebi olan seviye, ÇALIŞMIYOR.**
Modül doğru davrandı ve `failsafe 3 (att=0[cp=0 fresh=0 tilt=0] alt=0 xy=0)
-- degrading, NOT cutting` yazdı. **0.05 s sonra** `actuator_armed.lockdown`
true oldu, `terminationCommanded()` ona NaN'la cevap verdi ve araç 35 m'den
**serbest düştü** (gz yer gerçeği: 34.84 m → 0.10 m, ~2.7 s = tam serbest
düşüş). Yani RATE_ONLY tek bir 4 ms tick'ten uzun hiç çalışmadı.

Sebep, koddaki bir varsayımın ölçümle çürütülmesi. `terminationCommanded()`
yorumu "iki nedeni de *komut edilmiş*, çıkarsanmış değil" diyor; **ikisi de
yanlış**:

```
Commander.cpp:1881          armed.lockdown = (nav_state == NAVIGATION_STATE_TERMINATION)
                                             || HIL || throw_launch
ModeUtil/control_mode.cpp:119  flag_control_termination_enabled = true
                                             <- yine yalnızca nav_state == TERMINATION
```

Commander o duruma **otomatik** giriyor: attitude geçersizken
`FailsafeBase::modeCanRun()` başarısız, yedek mod yok (bu airframe'in `.post`
betiği `flight_mode_manager` ve `mc_pos_control`'ü durduruyor), failsafe
`Action::Terminate`'e yükseliyor. Gerçekten insan kararı olan tek şey
`manual_lockdown` (kill switch). Ayrıca ölçüldü: `gz_bridge`
`actuator_armed`'a hiç bakmıyor — aracı düşüren NaN'ı **modülün kendisi**
yazdı, yani karar bizim tarafta.

**✅ KARAR VERİLDİ VE UYGULANDI — ADIM 35 (2026-07-30): RATE_ONLY KALDIRILDI.**

Adım 34 üç seçenek bırakmıştı (1: commander'ın otomatik sonlandırmasını
görmezden gel; 2: RATE_ONLY'yi dürüstçe geri çek; 3: commander'ı hiç
TERMINATION'a sokma). Karar **ölçümle** verildi, tercihle değil.

**Önce ölçüm — `run_rate_only_test.m`.** Sav bir kontrol-yasası savı olduğu
için doğru yer MATLAB'dı: orada commander yok, yani seviye ilk kez 50 ms'den
uzun çalışabildi. PX4 dallarının birebir karşılığı kuruldu (`att_sp = att`
⇔ `omega_sp = 0`; irtifa döngüsü kapalı, `Fz = -m·g·0.97` açık çevrim;
`fx_sp = 0`), kontrolcüde hiçbir değişiklik gerekmedi. Üç senaryo, çünkü
savın kendisi ("hangi yatışta ise onu korur") başlangıç koşuluna bağlı:

| senaryo | >30° yatış | max \|roll\| | yaw savrulma | çarpma vz |
|---|---|---|---|---|
| 1) temiz hover | 45.3 s | 179.6° | +287° | 11.7 m/s |
| 2) gust + roll bozucusu | 12.2 s | 154.2° | +208° | 23.7 m/s |
| 3) 10° yatık girildi | 1.7 s | 122.1° | +391° | 19.6 m/s |

(35 m'den serbest düşüş referansı: 2.67 s, 26.2 m/s. Süreler bozulma anından
itibaren.) **Üçünün üçü de ters döndü ve yere çarptı.** En temiz durum bile —
bozucu yok, düz girildi — 179.6° roll'e gidip 11.7 m/s ile çarpıyor. Yani
"motorları kesmekten kesinlikle daha iyi" savı nicel olarak sınırda (11.7 vs
26.2 m/s), nitel olarak yanlış: bu kurtarılmış bir uçuş değil, biraz
yavaşlatılmış bir düşüş.

Mekanizma adım 29/30'un bulduğuyla aynı: **hız sönümlemesi duruş tutmak
değildir.** Artık torklar duruşa sınırsız integre olur; araç yattıkça açık
çevrim Fz yer çekimine karşı koymayı bırakır; hız arttıkça kanat momentleri
devreye girer ve süreç hızlanır.

**Tasarımın bir parçası ÇALIŞTI ve korundu:** dikey politika.
`FS_FZ_OPENLOOP = 0.97`, duruş düz tutulduğunda 49.5 s boyunca ortalama
0.71 m/s alçalma verdi — irtifayı neredeyse tuttu. Başarısız olan `omega_sp = 0`.
Açık çevrim itki "z gitti"nin iyi bir cevabı; hız sönümlemesi "duruş gitti"nin
cevabı değil.

**Uygulanan değişiklik (seçenek 2).** `FsLevel` artık NONE / NO_POS / NO_ALT.
Duruş kestirimi kademelendirilebilir bir girdi değil, **sert ön koşul**:
kaybolursa `Run()` çıkışı keser, tek seferlik bir `attitude LOST` hatası basar
ve `resetState()` çağırır (aksi halde LESO/integral/gölge aktüatör durumu
donar ve duruş geri gelirse artımlı yasa gerçeği tarif etmeyen bir gölgeden
devam ederdi). Uygulama detayları:

- `terminationCommanded()`'ın "iki nedeni de komut edilmiş" yorumu düzeltildi
  — adım 34 çürütmüştü, tek insan kararı `manual_lockdown`.
- `FsLevel` değeri **3 bir daha kullanılmayacak**: adım 35 öncesi loglarda
  "duruş gitti ama hâlâ uçuyor" demek. `.msg` dosyasında da böyle işaretlendi.
- `altitudeLoopVz` (orta irtifa kolu) artık **kanıtlanabilir biçimde
  ulaşılamaz**: tek girişi RATE_ONLY'ydi, `EKF2.cpp:1589-1590` yüzünden
  `alt_ok ≡ vz_ok`. Silinmedi, gerekçesiyle bırakıldı (`decelLoop` /
  `TILT_STICTION_BAND` ile aynı disiplin) — ayrı seviye-2 kararına bağlı.

**Bedeli açıkça yazıyorum:** artık *geçici* bir duruş boşluğu (>50 ms, sonra
düzelecek olan) da çıkışı kesiyor; eskiden hız sönümlemeli bir köprü alırdı.
Bu kabul edildi çünkü (a) commander aynı sinyalle ~50 ms sonra zaten
sonlandırıyor — modül onunla yarışmıyor, aynı fikirde olduğunu söylüyor,
(b) ölçüm o köprünün bir yere çıkmadığını gösteriyor.

**SITL doğrulaması (2026-07-30).** `run_failsafe_test.py --level 3` yeniden
yazıldı: ölçütü tersine çevrildi, artık "bozulup uçtu mu" değil "temiz,
kayıtlı ve zamanında kesti mi". Sonuç **GEÇTİ** — son `vehicle_attitude`
örneği t=48.83 s, modülün kesmesi t=48.87 s (**36 ms gecikme**), commander'ın
`lockdown`'ı t=48.93 s (**kesmeden 64 ms SONRA**, yani kararı modül verdi),
kesmeden sonra %100 NaN, ve arm'dan enjeksiyona kadar 11226 örnekte **%0.000
NaN** (adım 32'nin taşma hatası için regresyon koruması). Zorunlu
`sitl-lockup-check` de geçti (itki sat %0.00, BIG_M 0, itki 10.33-19.43 N,
yaw +4.06°, |vz|max 0.517, irtifa RMS 0.059 m) — adım 31 taban çizgisiyle
denk.

**Seviye 2 (NO_ALT) neden tetiklenemedi — ve muhtemelen ÖLÜ DAL.** İlk deneme
GPS 3B hızını açık bıraktı (dikey hız oradan besleniyordu). İkinci deneme tüm
dikey yardımı kapattı, ama `z_valid` 12 s boyunca hiç düşmedi. Yapısal sebep:
`EKF2.cpp:1589-1590`'da `z_valid` ve `v_z_valid` **aynı OR ifadesi**, yani
modülde `alt_ok ≡ vz_ok`. Bunun iki sonucu var: (a) NO_ALT ancak attitude
sağlamken dikey geçerlilik düşerse oluşur, (b) irtifa dalındaki orta kol
(`altitudeLoopVz`, `FS_DESCENT_VZ` ile kontrollü alçalma) `!alt_ok` iken
**asla** koşamaz — yalnızca RATE_ONLY + geçerli lpos durumunda, o da attitude
kaybının lpos'tan bağımsız olmasını gerektirir. `estimator_status_flags` ≥1 Hz
yayınlandığı için `tilt_aligned`'ın 3 s penceresi bu kapıyı da kapatıyor.

**✅ ADIM 36 (2026-07-30) — SEVİYE 2 / `altitudeLoopVz` KARARI VERİLDİ, ve
yukarıdaki paragrafın İKİ İDDİASI DA ÖLÇÜMLE DÜZELTİLDİ.**

**Düzeltme 1: `altitudeLoopVz` ölü kod DEĞİL.** `altitudeLoop()` onu her
`ALT_TS` çevriminde çağırıyor (`TiltrotorIndiControl.hpp:334`) — yani her
uçuşta, nominal yolda koşuyor. Ölü olan şey fonksiyon değil, onun **ikinci
çağıranıydı**: `else if (vz_ok)` failsafe kolu. Adım 34'ün kaydı bu noktada
yanlıştı.

**Düzeltme 2: NO_ALT ulaşılamaz DEĞİL.** Adım 34 iki denemeden sonra
"tetiklenemedi" demişti; ulog'da `fs=2` **görüldü** (`07_05_51.ulg`,
seviye dizisi `[0, 1, 2]`). Doğru enjeksiyon `EKF2_GPS_CTRL = 0` — yani
**tüm** yardımın kesilmesi. Adım 34'ün iki denemesi de en az bir kaynağı ayakta
bırakıyordu.

**Karar ve gerekçe.** Ölü kol **KALDIRILDI** (ulaşılabilir kılınmadı). Kolun
ihtiyaç duyduğu durum "z geçersiz, vz geçerli"; bu durum **üretilemiyor**.
Kaynak: `EKF2.cpp:1588-1590` ikisini de **tek bir OR**'dan türetiyor, üstelik
"bazı tüketiciler ikisinin farklı olmasını doğru işlemiyor" diyen bir TODO ile.
Kolu ulaşılabilir kılmak, kestirimcinin **açıkça yapmayı reddettiği** bir ayrımı
uydurmak olurdu — yani sınanmamış bir failsafe varsayımı, ki adım 35 onun
bedelini ölçtü. `decelLoop`'tan farkı: `decelLoop` **ölçülmüş olumsuz bir
sonucu** kaydediyor (tekrarlanmasın diye saklanır), bu kol ise yalnızca bir
varsayımdı. `FS_DESCENT_VZ` kaldı — tek kullanıcısı artık **link kaybı** yolu.

**`probe_no_alt.py` — üç konfigürasyon, tahmin yerine ölçüm.** (Adım 34 iki kez
tahmin edip iki kez tutturamamıştı; bu betik `estimator_status_flags`'ı
örnekleyerek hangi kaynağın ayakta kaldığını **görüyor**.)

| aşama | param | sonuç |
|---|---|---|
| A | `BARO_CTRL=0`, `GPS_CTRL=1` | `z_valid` **TRUE kaldı**, `z` −20.04'te **DONDU** |
| C | A + `HGT_REF=2` (var olmayan kaynak) | aynı: `z_valid` TRUE, `z` −20.39'da dondu |
| B | `BARO_CTRL=0`, `GPS_CTRL=0` | `fake_hgt` → `z_valid` **anında false**, `fs=2` |

**Sonuç: "z geçersiz, xy geçerli" durumu PX4 param yüzeyinden üretilemiyor.**
Yükseklik yardımı kesilip yatay yardım devam ederse EKF2 dikey kestirimi
**geçerli ilan etmeye devam ediyor**. Seviye 2'ye ancak tüm yardımı keserek
varılıyor, o da xy'yi götürüyor, o da commander'ı TERMINATION'a sokuyor —
ölçüldü: **fs=2 penceresi 0.02 s**, lockdown 20 ms sonra. **RATE_ONLY'yi
öldüren yapısal tavanın aynısı.** Bu yüzden `--level 2` ölçütleri modülün
kontrol ettiği şeye indirgendi (seviye raporlandı mı, o sırada motorları
kesmedi mi — ikisi de ✅); commander'ın sonlandırması başarısızlık değil,
**kaydedilen bir tavan**.

### 🔴 B1-a — YENİ TEHLİKE: kısmi yükseklik kaybında kestirim SESSİZCE YANLIŞ

Yukarıdaki A ve C aşamalarının asıl bulgusu seviye 2 değil. O konfigürasyonlarda
EKF'in `z`'si **donuyor ve `z_valid` TRUE kalıyor**, oysa araç gerçekten
alçalıyor:

| aşama | EKF `z` | gz yer gerçeği | süre |
|---|---|---|---|
| A | −20.04 (sabit) | 19.95 → **10.15 m** | 22 s |
| C | −20.39 (sabit) | 20.01 → **17.89 m** | 21 s |

Yani A'da **10 m'lik gerçek irtifa kaybı** kestirimde hiç görünmedi ve hiçbir
failsafe tetiklenmedi. **Modülün geçerlilik kapısı bunu yapısal olarak
göremez** — `z_valid`/tazelik testlerinin ikisi de "sinyal var ve taze" diyor;
yanlış olan **değerin kendisi**. Altitude döngüsü bir kurguyu uçuruyor.

Bu, adım 32'nin düzelttiği "durmuş kestirim sağlıklı görünüyor" hatasının bir
üst seviyesi: orada sinyal bayattı, burada **taze ve yanlış**. Donanımda baro
tıkanması / GPS yükseklik kaybı bunun gerçek muadilleri.

- [ ] **Çapraz kontrol gerekli**: `z`'yi bağımsız bir kaynakla (baro ham,
      rangefinder, GPS yükseklik) karşılaştırıp sapma eşiği aşılırsa bozulma
      ilan et. Şu an modülde hiçbir tutarlılık kontrolü yok — yalnızca
      geçerlilik bayrağı ve tazelik.
- [ ] İlk uçuşta bunun gerçekten olup olmadığını görmek için ölçüm: log'a
      `z` ile bağımsız yükseklik kaynağının farkı yazılmalı

**Kalan işler:**
- [x] ~~Açık karar (önce MATLAB'da rate-only ölçümü)~~ — **Adım 35'te kapandı:**
      ölçüldü, RATE_ONLY kaldırıldı, SITL'de doğrulandı
- [x] ~~Seviye 2 / `altitudeLoopVz` ölü dal~~ — **Adım 36'da kapandı:** ölü kol
      kaldırıldı, fonksiyon kaldı (ölü değildi), NO_ALT'ın ulaşılabilir ama
      commander tarafından 0.02 s'de kesildiği ölçüldü
- [ ] **Yeni:** B1-a (sessizce yanlış `z`) — yukarıdaki iki kalem
- [ ] Pilot girişi (Adım 33) **hiç çalıştırılmadı**: SITL'de RC yok, yani
      `manual_control_setpoint` hiç yayınlanmıyor. MAVLink `MANUAL_CONTROL`
      mesajı (pymavlink) bu yolu SITL'de sürebilir — yapılmadı.
- [ ] Bağımsız **kill switch** (tercihen ESC/güç seviyesinde, yazılımdan bağımsız)
- [ ] `publishDisarmed()`'ın gerçek ESC'lerde ne yaptığı doğrulanmalı: NaN
      yazıyor; gerçek ESC/servo bunu nasıl yorumluyor? **Adım 35 bunu daha
      önemli hale getirdi**: duruş kaybı artık bu yolu kullanıyor, yani NaN'ın
      donanımdaki anlamı bir failsafe davranışı oldu, yalnızca kill değil.
- [ ] **Geçici duruş boşluğu**: adım 35'ten sonra >50 ms'lik bir
      `vehicle_attitude` kesintisi çıkışı kesiyor. **SITL tarafı ölçüldü
      (Adım 37, `sitl/check_output_cuts.py`): bugünün 8 uçuşunda boşluk max
      8-16 ms, p99 4 ms → `FS_ATT_TIMEOUT_US`'a 3.1-6.2× pay, ve havada
      0 NaN çıkış.** Ama bu yalnızca bir ALT SINIR: gerçek EKF'te (donanım,
      titreşim, GPS glitch) dağılım farklıdır, **yerde/tetherli yeniden
      ölçülmeli** — çıkarsa `FS_ATT_TIMEOUT_US` yeniden değerlendirilir.
      Aynı tarama adım 31'in loglarında (adım 35 öncesi ikili) **uçuş başına
      3 adet tek-tick havada NaN** buldu; bugünkü kodda yok, ama bu ölçüm
      artık her koşuda otomatik yapılıyor çünkü kimse bakmayınca görünmüyor.
- [ ] Bozulma sırasında **yatay sürüklenme kabul edilmiş bir sonuç**: xy
      geçersizken pozisyon döngüsü kapanmak zorunda ve madde (N) yüzünden araç
      kendini ileri iter (ölçüldü: 14 s'de 0.1 → 2.6 m/s). Kestirim geri
      geldiğinde `pos_hold` **REDDEDİLİYOR** (4.8 m/s > `POS_ENGAGE_V_MAX`
      = 3.0), yani failsafe **kendi kendine toparlanmıyor** — geri geçiş
      (`bt_enable`) toparlanma yoluna bağlanmalı

### 🔴 B2 — Her fiziksel sabit SİMÜLASYONDAN geliyor

`TiltrotorIndiParams.hpp`'deki şu değerlerin tamamı `model.sdf`'ten alınma,
hiçbiri gerçek araçtan ölçülme değil:

| Sabit | Değer | Gerekli iş |
|---|---|---|
| `MASS` | 5.0 kg | Aracı **tart** (bataryalı, uçuşa hazır) |
| `I_XX/I_YY/I_ZZ` | 0.2 / 0.25 / 0.25 | Bifilar sarkaç ölçümü (ya da CAD'den, toleransıyla) |
| `ROTOR_PX/PY/PZ` | ±0.22/0.25/0.06, kuyruk −0.65/0/−0.07 | CG'ye göre **fiziksel ölç** |
| `ROTOR_KF` | 2.0e-5 N/(rad/s)² | Statik itki testi |
| `ROTOR_WMAX` | 1500 rad/s | Gerçek motor/ESC/pervane |
| `ROTOR_TMAX` | ~45 N | Statik itki testi (tek rotor) |
| `ROTOR_KM` | ∓0.06 Nm/N | Tork ölçümü; **işaret** Adım 12'de doğrulandı, **büyüklük** değil |

- [ ] Yukarıdaki tablonun tamamı gerçek ölçümle güncellendi
- [ ] Toplam statik itki / ağırlık oranı ≥ 1.8 doğrulandı (WLS'in manevra payı var mı)

> **Not:** `ROTOR_KM`'nin *işareti* donanıma aktarılabilir (Adım 12, gerçek
> dönüş yönlerinden türüyor) — ama 0.06 değeri SDF `momentConstant`'ından
> geliyor. Yaw'ın tek zayıf ekseni olduğu bu araçta bu büyüklük önemli.

### 🔴 B3 — İtki komut eşlemesi Gazebo'ya özgü

`thrustToNormalized()` (Adım 11'in kök-neden düzeltmesi) tam olarak şunu
tersliyor: `MixingOutput` normalize komutu `SIM_GZ_EC_MIN=10…MAX=1500` rad/s'ye
doğrusal ölçekliyor, sonra gz `T = 2e-5·ω²` uyguluyor. **Gerçek ESC/motor/pervane
zincirinin eğrisi bu değil.**

Bu sadece "kalibrasyon" meselesi değil: WLS'in etkinlik matrisi `dT/du = 1.0`
varsayıyor ve bunu doğru kılan şey tam da bu eşleme. Yanlış eğri = **itkiye
bağlı biçimde yanlış G matrisi** — Adım 11'de bu, düşük itkili rotoru tabana
iten pozitif geri besleme yaratmıştı.

- [ ] Gerçek zincirin itki eğrisi ölçüldü (PWM/DShot komutu → itki, tam aralık)
- [ ] `thrustToNormalized()` bu eğrinin tersiyle değiştirildi
- [ ] Ölçülen `dT/du`'nun çalışma aralığı boyunca ne kadar değiştiği belgelendi

### 🔴 B4 — `ROTOR_PY` işaret bulmacası: **SİMÜLASYON TARAFI ÇÖZÜLDÜ**, fiziksel doğrulama duruyor

**Bulmaca çözüldü (2026-07-30, Adım 34): işaret DOĞRU, çelişki bir çerçeve
dönüşümü hatasıydı — SDF'in kendisinde değil, onu okuyan notta.** SDF `<pose>`
değerleri **model çerçevesinde**, yani gz'nin FLU'sunda (x ileri, y **sol**,
z yukarı); `ROTOR_P*` ise FRD. Dönüşüm `(x, −y, −z)`:

| gz link | SDF pose (FLU) | → FRD | `ROTOR_P*` |
|---|---|---|---|
| `rotor_0` | 0.22, **−0.25**, −0.06 | 0.22, **+0.25**, +0.06 | 0.22, **+0.25**, +0.06 ✅ |
| `rotor_1` | 0.22, **+0.25**, −0.06 | 0.22, **−0.25**, +0.06 | 0.22, **−0.25**, +0.06 ✅ |
| `rotor_2` | −0.65, 0, +0.07 | −0.65, 0, **−0.07** | −0.65, 0, **−0.07** ✅ |

FLU'da y=−0.25 zaten **sağ** demek, yani SDF'in "Right wing rotor" yorumu da
doğru. Üç sütun da birebir uyuyor — ve dikkat, sabitler `motor_N` değil
**`rotor_N`** link pozisyonlarına (itkinin uygulandığı yer) uyuyor, ki doğrusu
o. Adım 4'te işaret çevirme denemesinin MATLAB'da regresyona yol açması bu
sonucu **destekliyor**: orijinal değer doğruydu.

`ROTOR_KM` işaretleri de aynı zincirde tutarlı çıktı. gz sürükleme torkunu
rotorun **dönme eksenine** uyguluyor (`−turningDirection·T·km`) ve o eksen
motorla birlikte eğiliyor. rotor_0 (ccw, dir=+1) için
`τ(FLU) = −km·T·(sin δ, 0, cos δ)` → `τ(FRD) = (−km·T·sin δ, 0, +km·T·cos δ)`.
Kontrolcü ise `m_i = km_i·T_i·dir_i`, `dir_i = (sin δ, 0, −cos δ)` ve
`km_0 = −0.06` ile tam aynısını veriyor (`effectiveness_matrix.m:22-26`) —
**tilt bağımlılığı dahil**: eğim büyüdükçe reaksiyon torkunun roll bileşeni
büyüyor, yaw bileşeni küçülüyor. Bu geri geçiş rejiminde (δ → 90°) önemli ve
doğru modellenmiş.

**Yani "MATLAB kendi içinde tutarlı olduğu için görmüyor" argümanına artık
ihtiyaç yok: SITL'de plant (Gazebo) ile kontrolcü (PX4) bu sabiti
PAYLAŞMIYOR** ve araç kararlı uçuyor, duruşunu 0.06 m'de tutuyor — yani
geometri gerçek plant'a karşı zaten doğrulanmış durumda.

**Ama bu bir TÜRETME, donanım ölçümü değil.** Gerçek araçta hangi motorun nerede
olduğu, hangi yöne döndüğü ve kanal eşlemesi hâlâ elle doğrulanmalı — bunlar
Adım 11/12 ile aynı sınıf riskler ve bu projede iki kez gerçekleşti. Aşağıdaki
kutular bu yüzden açık kalıyor. Kodun içindeki eski not
(`TiltrotorIndiParams.hpp:57-68`, "opposite sign from {0.25,-0.25,0}") artık
yanlış; düzeltilmesi gerekiyor.

- [ ] Hangi motorun fiziksel olarak sağda/solda olduğu **elle doğrulandı**
- [ ] Her rotorun dönüş yönü (CW/CCW) **gözle doğrulandı** ve `ROTOR_KM`
      işaretleriyle karşılaştırıldı
- [ ] Motor çıkış kanal sırası (0/1/2) fiziksel motorlarla eşleşiyor
- [ ] Tilt servo kanalları (5/6/7) fiziksel servolarla eşleşiyor ve **yönleri**
      doğru (komut +1 → gerçekten 90°, komut −1 → gerçekten 0°)

### 🟠 B5 — Geri geçiş var ama yalnızca SITL'de; (R) ve (S) kapandı, (T) açık

*(2026-08-03, Adım 39: 🔴'dan 🟠'a İNDİRİLDİ. Madde (S) kapatıldı — eşik ve
fren marjı artık yasanın kontrol ettiği ekseni (gövde ileri hızı) okuyor — ve
düzeltme, rejimi deterministik kuran yeni bir probe ile sınandı. Geriye 🟠
madde (T) ve **B5'in asıl sebebi: manevra hiç gerçek donanımda uçmadı** kalıyor.
`fx_trim` bağımlılığı da (aşağıda) sürüyor.)*

*(2026-07-31, Adım 38: 🟠'dan 🔴'a ÇIKARILMIŞTI. Madde (R) kapandı, ama aynı
doğrulama madde (S)'yi ortaya çıkardı — fren yasası aracı geri yönde
12.8 m/s'ye kaçırabiliyor. Bir engelleyicinin durumu, en ağır açık maddesine
göre belirlenir.)*

*(Adım 31, 2026-07-29 ile daraltıldı; Adım 30'daki hâli 🔴 "seyirden
hover'a dönüş yolu YOK" idi.)*

Adım 30'da araç **tek yönlüydü**: seyre çıkabiliyor, dönemiyordu. Adım 31
bunu çözdü ve engelleyici 🔴'dan 🟠'a indi — **kapanmadı**, çünkü gerçek
donanımda hiç uçmadı.

> **⚠️ GÜNCELLEME 2026-07-30 (Adım 37): Adım 31'in "doğrulandı" sonucu
> MARJİNALMİŞ ve bugün iki kez BAŞARISIZ oldu.** GUI'li yeniden koşumda manevra
> iki bağımsız uçuşta **devir hızının üstünde takıldı** (v_h 3.2-3.5 m/s'de
> 90+ s kararlı denge; `BT_HANDOFF_V = 3.0`'a hiç inemedi, yani `pos_hold` hiç
> istenmedi). Sebep ölçüldü: fren yasası pitch'i hızla söndürürken yenmesi
> gereken ileri kuvvet **sabit** (δ1/δ2 tabanda çakılı → yaw trimi δ0'ı 10-15°'de
> tutuyor → sürekli 3.1-4.1 N). Yalnızca *durmak* için gereken açı
> `asin(fx_trim/(m·g)) = 3.39°`, `BT_PITCH_MAX`'in (4°) hemen altında.
> Adım 31'in iki uçuşu eşiği ancak sürüklenme sayesinde geçmiş — birinde son
> 0.5 m/s **12.1 s** sürmüş.
>
> **Düzeltildi ve doğrulandı:** fren yasası ayrıştırıldı
> (`pitch = BT_TRIM_PITCH + BT_PITCH_MAX·fade`, ilk terim sönmez ve `FX_TRIM`'e
> bağlı), `BT_BRAKE_V_FULL` `BT_HANDOFF_V`'ye bağlandı. İki bağımsız uçuş,
> ikisi de tam dizi: 15.38/14.66 m/s → 0.13/0.07 m/s, BRAKE evresi 92-99 s'lik
> takılmadan **5.7-6.1 s**'ye indi, yaw −3.4/+1.1°, irtifa bandı 1.11-1.14 m,
> itki doyumu %0.00, 0 BIG_M; zorunlu `sitl-lockup-check` geçti. Ayrıntı:
> rapor Adım 37.
>
> **🔶 AYNI OTURUMDA İKİNCİ BİR KUSUR BULUNDU — madde (R).**
> Fren düzeltmesi doğrulandıktan sonraki iki uçuşta manevra bu kez **RETRACT**
> evresinde takıldı: tavan 9°'ye indikten sonra araç oraya asimptotik yaklaşıyor
> ve **terminal hız 8.0-9.3 m/s**, yani `BT_RELEASE_V = 8.0`'ın üstünde —
> dolayısıyla çıkış koşulu hiç sağlanmayabiliyor (bir uçuşta 200 s boyunca
> sağlanmadı). Tamamlanan uçuşlar eşiği ilk yavaşlama geçicisinde geçmiş.
> **Tehlikeli kısmı süre değil, o sırada bulunulan konfigürasyon:** tavan tam
> trim değerinde otururken δ0 çakılı kalıyor ve yaw'ın modülasyon otoritesi
> sıfırlanıyor (adım 31/Faz 2'nin ölçtüğü mekanizma) — 200 s'lik uçuşta yaw
> **+117.7°** sürüklendi. Yani geri geçiş **SITL'de bile garanti tamamlanan bir
> manevra değil: 8 uçuşun 5'i tamamladı.**
>
> **✅ MADDE (R) KAPATILDI — ADIM 38 (2026-07-31).** Adım 37'nin iki seçeneği
> arasında seçim yapılmadı, **ikisi birden** uygulandı, çünkü farklı şeyleri
> garanti ediyorlar: (1) `BT_RELEASE_V` 8.0 → **10.0** — eşik ölçülen terminal
> aralığın dışına taşındı; (2) **`BT_FLOOR_DWELL = 20 s` (yeni)** — tavan tabana
> vardıktan sonra bu süre geçerse hıza **hiç bakmadan** BRAKE'e geçilir. İkincisi
> aero-bağımsızdır ve asıl donanım gerekçesidir: 10.0 sayısı Gazebo aerodinamiğinden
> geldi, gerçek kanat farklı sürüklerse eşik yine dengenin içine düşer — ama süre
> terimi terminal hız ne olursa olsun geçerlidir, yani **sınırsız bir arızayı
> sınırlı bir gecikmeye çevirir**. İptal değil BRAKE seçildi: bekleme,
> RETRACT'ın verebileceği en düşük hızda dolar ve BRAKE tavanı yükselterek
> yaw otoritesini zaten geri verir.
> **Doğrulama:** üç bağımsız normal uçuş, üçünde de tam dizi (RETRACT 16.5-20.9 s
> — eskiden 21.9-25.1 s ve takılan uçuşlarda 111/200 s; son v_h 0.09-0.24 m/s,
> yaw −2.4…−6.0°, irtifa bandı 1.38-1.56 m); artı **kancalı bir build ile
> (`BT_RELEASE_V = 5.0f`) emniyet yolu bilerek tetiklendi** ve 6.6 m/s'de
> `[via FLOOR_DWELL backstop]` olarak çalıştığı ölçüldü. MATLAB tarafında
> plantsız bir durum makinesi testi eklendi (`run_backtrans_sm_test.m`, 6/6),
> regresyon birebir nötr. Ayrıntı: rapor Adım 38.
>
> **⚠️ AMA B5 YİNE KAPANMADI — çünkü doğrulama YENİ BİR KUSUR ORTAYA ÇIKARDI:
> madde (S), aşağıda.**
>
> ### ✅ madde (S) KAPATILDI (Adım 39, 2026-08-03) — eşik ve fren marjı artık gövde ileri hızını okuyor
>
> **Düzeltme iki yerde, tek ilkeden: bir eşik, yasasının KONTROL ETTİĞİ ekseni
> okumalıdır.** (1) `BRAKE → HANDOFF` koşulu **işaretli** gövde ileri hızını
> okur (`v_fwd < BT_HANDOFF_V`); `v_fwd < 0` de geçerli bir çıkıştır, çünkü
> BRAKE'in işi ileri hızı bitirmekti ve bitmiştir — mutlak değer almak aynı
> hatayı tekrarlardı. (2) Frenleme **marjı** da `max(0, v_fwd)` ile söner, yani
> araç geri gitmeye başlarsa marj SIFIRLANIR ve geriye yalnızca duruş trimi
> kalır; **kaçışı fiilen durduran terim budur.** RETRACT bilerek `v_h`
> (büyüklük) okumaya devam ediyor: oradaki soru aerodinamiktir ("kanat hâlâ
> taşıyor mu"), cevabı hava hızının büyüklüğüdür.
>
> **Doğrulama:** iki normal uçuş (15.21/15.76 → 0.21/0.12 m/s, BRAKE 8.5/9.1 s,
> yaw −2.2/−2.6°, irtifa bandı 1.53/1.54 m, %0.00 doyum, 0 BIG_M) + kancalı
> uçuş (`BT_RELEASE_V = 5.0f`, emniyet yolu yine çalıştı: 6.64 m/s'de
> `[via FLOOR_DWELL backstop]`); plantsız durum makinesi testi 9 gruba çıktı
> (13/13, ve test 8 eski büyüklük mantığının aynı izde 300 s takıldığını
> gösteriyor); MATLAB regresyonu birebir nötr; zorunlu `sitl-lockup-check`
> GEÇTİ (yaw −1.37°, irtifa RMS 0.275 m).
>
> **Asıl kanıt kancalı uçuştan gelmedi** — o uçuşta yanal sürüklenme yalnızca
> 1.28 m/s'ye çıktı, yani rejim HİÇ KURULMADI. Rejimi deterministik kuran yeni
> bir probe yazıldı (`sitl/probe_lateral_handoff.py`: frenlerken heading +90°
> çevrilir, ölçülen arızanın geometrisi budur): yanal **4.76 m/s**, HANDOFF
> `|v_h| = 5.04` m/s'de gerçekleşti (`v_fwd = +2.87`) — **eski büyüklük koşulu
> o anda sağlanmıyordu**, manevra 0.17 m/s'de tamamlandı.
>
> **⚠️ VE MADDE (S) SANILDIĞI KADAR EGZOTİK DEĞİLMİŞ (Adım 39b, GUI'li yeniden
> koşum).** `side` kameralı **normal** bir uçuşta yanal sürüklenme hiçbir kanca
> olmadan **3.60 m/s**'ye çıktı — arızayı üreten 3.04'ün üstünde — ve HANDOFF
> `v_h = 3.58`, `v_fwd = 2.99`'da gerçekleşti, yani **eski büyüklük koşulu o
> anda da sağlanmıyordu: düzeltme olmasaydı bu sıradan uçuş da takılırdı.**
> Dört normal uçuşta ölçülen yanal 1.04 / 1.28 / 1.49 / 1.90 / **3.60** m/s,
> yani dağılımın kuyruğu eşiği rahatça aşıyor. Adım 38'de maddenin yalnızca
> kancalı uçuşta görünmesi, arızanın nadir olmasından değil, o uçuşun UZUN
> olmasındandı. Sonuç yine temiz: yeniden-hızlanma 0.00, yaw +0.8°, irtifa
> bandı 1.23 m, %0.00 doyum.
>
> **Kalıntı (kapatılmadı, ÖLÇÜLDÜ):** `POS_ENGAGE_V_MAX` hâlâ bir büyüklük
> kapısı, yani handoff ileri eksende istenip hold tarafından reddedilebiliyor
> ve her tick yeniden deneniyor. Ölçüldü: normal uçuşlarda **0.22 / 0.56 s**,
> ağır yanal sürüklenmeli probe'da **3.23 s** — ve o süre boyunca `v_h` 5.04 →
> 3.00 ile azalmaya devam etti, pitch trimde kaldı, kaçış olmadı. Kapıyı yön
> duyarlı yapmak Adım 29'un güvenlik gerekçesine dokunur ve kendi ölçümünü
> hak eder. Modül >0.5 s'lik her beklemeyi WARN olarak yazıyor.
>
> **Test altyapısı da düzeltildi ve bu ayrı bir bulgu:** madde (S)'yi yakalasın
> diye eklenen 6. ölçüt önce "işaretli ileri hız ≥ −2.0 m/s" idi ve probe'da
> KALDI (−3.95) — oysa araç kaçmamıştı, `v_h` monoton 9.74 → 3.00 iniyordu;
> sebep heading'in aynı pencerede **+186.7°** dönmesi, yani ölçüt DÖNEN bir
> çerçeveye izdüşüme bakıyordu. **Ölçüt, kontrol yasasının yaptığı hatanın
> aynısını yapmıştı.** Şimdi çerçeveden bağımsız: fren penceresinde `v_h`'nin
> koşan minimumundan **yeniden-hızlanması ≤ 1.0 m/s**, ve **gerçek arıza
> log'una karşı doğrulandı** (14_13_11: 10.31 m/s → KALDI; dört yeni uçuş:
> 0.00 → GEÇTİ). Ölçüt penceresi de daraltıldı ("fren yasasının pitch'e sahip
> olduğu" aralık), bunun için `TiltrotorIndiStatus.msg`'e `pos_hold_active`
> eklendi.
>
> **Aşağıdaki bölüm arızanın ÖLÇÜLDÜĞÜ hâli — kayıt olarak duruyor.**
>
> ### ~~🔴 madde (S)~~ — HANDOFF eşiği YANLIŞ SİNYALE bakıyor, fren yasası aracı GERİ kaçırıyor
>
> *(Adım 38, 2026-07-31 — madde (R)'nin SEBEBİ DEĞİL, ondan bağımsız ve önceden
> var olan bir kusur; emniyet yolunu doğrulayan kancalı uçuşta görünür oldu.
> **Adım 39'da kapatıldı, yukarı bakın.**)*
>
> `BRAKE → HANDOFF` koşulu `v_h < BT_HANDOFF_V = 3.0` ve `v_h` bir
> **BÜYÜKLÜK** (`hypot(vx,vy)`). Fren yasası ise yalnızca **ileri** ekseni
> kontrol ediyor. Ölçüm (`sitl/diag_brake_reversal.py`, log 14_13_11):
> `min|v_h| = 3.08 m/s` iken **gövde ileri hızı u = −0.51 m/s, yanal hız
> v = +3.04 m/s** — yani araç ileri yönde çoktan durmuştu ama eşiğin üstünde
> tutan şey **yanal sürüklenmeydi**. Handoff hiç istenmedi, sönmeyen fren
> pitch'i (3.39° trim + 4.0° marj) itmeye devam etti ve araç **geri yönde
> 12.8 m/s'ye kaçtı**. Aynı uçuşta beş kez tekrarlandı (13.4 / 7.9 m/s …).
> Yanal sürüklenmenin kaynağı yapısal: **RETRACT/BRAKE boyunca yanal eksende
> hiçbir kontrol yok**, pozisyon döngüsü ancak HANDOFF'ta devreye giriyor.
>
> **Bu, aynı hastalığın ÜÇÜNCÜ tekrarı** (Adım 37 = otorite sönüyordu,
> madde (R) = eşik dengede oturuyordu, madde (S) = eşik yanlış eksene bakıyor).
> Üç normal uçuşta görülmedi çünkü BRAKE yalnızca 8.9-9.8 s sürdü ve yanal
> sürüklenme birikemedi — **ama emniyet devreye girdiğinde manevra uzuyor, yani
> (R)'nin çözümü (S)'yi daha ERİŞİLEBİLİR kılıyor.** Aday çözüm (ölçülmedi,
> uygulanmadı): eşiği gövde ileri hızına bağlamak ve/veya işaret değişiminde
> pitch'i kesmek. **Donanımda geri geçiş bu madde kapanmadan denenmemelidir.**
>
> ### 🟠 madde (T) — `attitude LOST (tilt=0)` uçuş ortasında tetikleniyor
>
> İki kancalı uçuşta üç kez görüldü (normal uçuşların hiçbirinde yok): Adım 35'in
> sert ön koşulu tetiklenip çıkışı kesti ve geri geçiş durum makinesini
> **sıfırladı** (manevra tavan bırakılmış hâlde baştan başladı). `tilt_aligned`
> hem `estimator_status_flags`'in 3 s'den taze olmasına hem `cs_tilt_align`'a
> bağlı, ve EKF2 bu topic'i **~1 Hz** yayınlıyor. İki mekanizma da mümkün:
> topic bayatladı (sağlıklı EKF'de çıkış kesilir) ya da EKF şiddetli uçuş
> durumunda hizalamayı gerçekten düşürdü.
>
> **ADIM 39 GÜNCELLEMESİ (2026-08-03) — ENSTRÜMAN KURULDU, ama madde
> TEKRARLANMADI, ve tekrarlanmamasının kendisi bir bulgu.** Topic log profiline
> eklendi (`sitl/logger_topics_shadow.txt`). Dört uçuşun hiçbirinde
> `attitude LOST` görülmedi; ölçülen yayın aralığı ortalama **0.93-0.96 s, max
> 1.00 s** (3 s'lik tazelik penceresinin çok altında, >3 s olan hiç yok) ve
> `cs_tilt_align` her örnekte 1. Yani **bu koşullarda "topic bayatladı"
> mekanizması için kanıt yok.** Ama (T)'nin görüldüğü uçuşlar madde (S)'nin
> **geri kaçış** uçuşlarıydı (12.8 m/s), ve o rejim artık oluşmuyor — yani
> tetikleyici koşulun kendisi ortadan kalkmış olabilir. **Madde açık kalıyor**
> (🟠): bir daha görülürse hangi mekanizma olduğu artık tahmin edilmeyecek,
> log'dan okunacak. Tekrarlanmasının en olası yolu şiddetli manevra; kasıtlı
> bir tekrar denemesi yapılmadı.
>
> **Donanım açısından bunun anlamı, B5'i kapatmaya YAKLAŞTIRMAK DEĞİL, tam
> tersi.** `BT_TRIM_PITCH` doğrudan `FX_TRIM = 2.9 N`'den türüyor ve o değer
> *simüle edilmiş* trimin ölçümü (aşağıdaki §4 maddesi zaten bunu açık
> bırakıyor). Gerçek araçta yaw triminin doğurduğu ileri kuvvet farklıysa fren
> yasasının ilk terimi de yanlış olur — üstelik artık manevranın tamamlanıp
> tamamlanmaması buna DOĞRUDAN bağlı. **`fx_trim` ölçümü B5 için de bir
> ön koşul hâline geldi.**

**Ne yapıldı.** Dört durumlu bir geri geçiş modu (`backtrans_loop.m` /
`backTransition()`, `bt_enable` setpoint bayrağı):
RETRACT (kanat tilt tavanı 2 °/s ile 9°'ye iner) → BRAKE (tavan 20°'ye
**yükseltilir**, ≤4° burun yukarı frenleme) → HANDOFF (`pos_hold`
devralır). İki otomatik uçuş, `bt_enable` dışında hiçbir komut yok:
13.5/14.5 m/s → 0.12/0.09 m/s, yaw toplam dönüş −10.1°/−8.1°, irtifa
bandı 0.95 m, itki doyumu %0.00, sıfır BIG_M. `sitl-lockup-check` geçti.

**Adım 30'un teşhisi eksikti, düzeltildi.** "Frenleme otoritesi tilt'tir"
doğru, ama tilt'i ileri süren şey Fx'in zayıflığı değil **irtifa (Fz)
kanalı**: tahsisat her örnek için çevrimdışı yeniden çözülüp atıf
yapıldı — Fz talebini sıfırlamak sürüklenmeyi tersine çeviriyor
(+0.64 → −0.45 °/s), Fx talebini sıfırlamak hiçbir şey değiştirmiyor.
Bu yüzden tilt bir **kutu kısıtıyla** sürülür, ağırlıkla değil
(`Ws_Fz/Ws_Fx = 400`, ve Ws_Fx'i o mertebeye çıkarmak Adım 7'de yaw'ı
bozmuştu).

**Donanım açısından ne değişti, ne değişmedi.** Araç artık tek yönlü
değil — ama bu **yalnızca Gazebo'da** gösterildi. Donanımda seyir hâlâ
**denenmemeli**, üç somut nedenle:
1. Manevranın her eşiği (`BT_RETRACT_RATE`, `BT_CEIL_FLOOR = 9°`,
   `BT_RELEASE_V = 8 m/s`, `BT_BRAKE_CEIL = 20°`) **SITL'de ölçüldü** ve
   hepsi aerodinamiğe bağlı. Gazebo modelinde beş lift-drag yüzeyi var;
   gerçek kanat farklı taşırsa bu eşiklerin hepsi kayar. MATLAB bu
   manevrayı yapısal olarak üretemiyor (tek boylamsal yüzey, 12 m/s'de
   ~25 N), yani **ikinci bir bağımsız doğrulama ortamı yok** — B2/B3 ile
   aynı sınıf risk.
2. `BT_CEIL_FLOOR`'un altına inmek **yaw'ı aç bırakıyor**: taban 9/7/5°
   süpürmesinde yaw sapması +0.0205/+0.0308/+0.0384 rad/s (süpürme sırası
   ters çevrilerek doğrulandı) ve 5°'de araç kendiliğinden kaçtı. Sebep
   yapısal: δ1 `TILT_MIN`'de çakılı olduğu için diferansiyel tam olarak
   tavana eşit, ve δ1 = 0 iken `τ_z = −0.25·Fx` — yaw trim torku ile kalan
   ileri kuvvet **aynı büyüklük**. Gerçek araçta trim farklı çıkarsa
   (bkz. `fx_trim` maddesi) bu sınır da kayar.
3. `bt_enable`'ın tek kaynağı hâlâ `test_sp` konsol komutu — **pilot
   tetikleyemez, iptal edemez** (B1). Geri geçiş 30-40 s sürüyor ve bu
   süre boyunca müdahale yolu yok.

**Kapanma koşulu:** B2+B3 ölçüldükten sonra eşiklerin gerçek aero ile
yeniden türetilmesi, B1'in çözülmesi (pilotun başlatıp iptal edebilmesi),
ve manevranın önce yüksek irtifada, bol paylı bir uçuşta denenmesi.
Şu anki `BT_MIN_ALT = 15 m` SITL'de ölçülen ≤1 m'lik irtifa bandına göre
seçildi; donanımda bu pay çok daha geniş tutulmalı.

---

## 1. Tezgâh testleri (pervanesiz, sonra pervaneli-bağlı)

### Sensör / kestirim
- [ ] 🔴 Magnetometre kalibrasyonu + **yaw kestirim hatası** kontrolü.
      SITL'de EKF yaw'ı gerçek yönelimden **5-10° sapıyordu** (Adım 27); orada
      ground truth vardı, donanımda **yok**. Kontrolcü *kestirilen* yaw'ı
      tuttuğu için bu hata doğrudan fiziksel yönelim hatasına dönüşür.
- [ ] 🔴 IMU/gyro kalibrasyonu; `vehicle_angular_velocity.xyz_derivative`
      gürültüsü ölçüldü — **INDI'nin tüm sönümlemesi bu sinyalden geliyor**
      (SITL'de gecikme 4-8 ms, HF gürültü sinyalin %4'üydü; Adım 20)
- [ ] 🟡 Titreşim (vibe) seviyeleri kabul aralığında — pervane dengesizliği
      `omega_dot` üzerinden doğrudan INDI'yi bozar

### Aktüatör
- [ ] 🔴 **Tilt servolarının gerçek dinamiği ölçüldü**: `TILT_TAU = 0.15 s`
      birinci derece yaklaşım, ve PX4'te **servo pozisyon geri beslemesi YOK** —
      kontrolcü bir *gölge model* kullanıyor. Gerçek servo bu modelden saparsa
      INDI'nin lineerleştirme noktası ve WLS'in G matrisi birlikte kayar.
      SITL'de sapma p99 ≤ 0.55° idi (Adım 18); gerçek servoda ölçülmeli.
- [ ] 🔴 Servo **stiction/ölü bant** ölçüldü. Adım 24'ün uyarısı kritik:
      ölü bandı gölge modele eklemek **kapalı çevrimi kilitledi** (komut
      artışı ölü banttan küçük → gölge donuyor). Yani gerçek servonun ölü
      bandı `TILT_SLEW_BOX_RATE·TS_BOX = 0.012 rad = 0.69°`'den **büyükse**
      benzer bir kilitlenme riski var.
- [ ] 🔴 Tilt mekanik aralığı gerçekten [0°, 90°] mi — ve **0° gerçekten dikey mi**
      (tüm yaw trim mantığı buna dayanıyor)
- [ ] 🟡 Servo besleme akımı/ısınması sürekli tilt düzeltmesi altında kabul edilebilir

### Tahsisat / emniyet
- [ ] 🔴 `leso_enable_yaw = FALSE` doğrulandı. **Yaw ekseninde LESO açmak
      SITL'de aracı 5 saniyede ters çevirdi** (rapor §4 (H)) — bu ayar
      donanımda asla açılmamalı.
- [ ] 🔴 `RATE_SP_LIMIT = [3.0, 3.0, 0.5]` — yaw'daki **0.5** Adım 13'ün
      düzeltmesi ve yaw'ı çözen şeydi; tek skalere geri dönmek savrulmayı
      geri getirir.
- [ ] 🟡 Arm anında `hoverTrim()` tohumlaması gerçek geometriye göre yeniden
      hesaplandı (B2/B4'ten sonra)

---

## 2. Bağlı (tethered) / kısıtlı ilk çalıştırma

- [ ] 🔴 Araç **fiziksel olarak kısıtlı** (test standı / bağ) hâlde ilk arm
- [ ] 🔴 Motor dönüş yönleri ve kanal eşlemesi düşük gazda gözle doğrulandı
- [ ] 🔴 Tilt servolarının arm anında **doğru yöne** gittiği doğrulandı
      (SITL'de disarm hâlinde NaN yazılıyor, servolar 0°'da duruyor; arm anında
      gölge δ0 ≈ 9.4°'ye tohumlanıyor ve gerçek servo 72 ms'de yetişiyordu)
- [ ] 🔴 Açık çevrim yaw reaksiyon torku işareti ölçüldü ve modelin tahminiyle
      karşılaştırıldı — **Adım 12'de kök nedeni bulan ölçüm tam olarak buydu**
      (tahmin +6.2 rad/s², ölçüm +6.45/+6.56)
- [ ] 🟡 Kısa süreli, düşük irtifada (ip boyu ~30 cm) tutma denemesi

---

## 3. İlk serbest uçuş (kademeli)

Her adımda ≥25 s gözlem (kısa testler bu projede defalarca yanılttı) ve
**yaw'ı açıdan değil hızdan** oku (`vehicle_angular_velocity.xyz[2]`).

- [ ] 1 m irtifada 30 s duruş — `pos_hold` **açık**
- [ ] Sürüklenme, yaw hızı RMS, aktüatör doyumu kaydedildi
- [ ] 3 m'de 60 s duruş
- [ ] ±30° yaw adımı, **her iki yön** (madde (P) asimetrisi)
- [ ] Kademeli iniş, **1.0 m'lik kademelerle** (1.5 m SITL'de 13 aktüatör
      yapışması üretti — Adım 27)

Her uçuşta kaydedilecek bağlam: yatay hız platosu, batarya gerilimi, rüzgâr.

---

## 4. SITL senaryo kapsamı

- [x] 🟢 **Rüzgâr/bozucu reddi — KAPATILDI (2026-07-29).** İlk kez *gerçek
      hover'da* (`pos_hold` açık) koşuldu. 0.4 + 0.15·sin(2π·0.3t) N·m sürekli
      roll bozucusu altında araç duruşunu korudu: sürüklenme ≤0.36 m, v_h
      0.01-0.42 m/s, rate RMS p/q ≈ 0.06-0.09 rad/s, **0 BIG_M**, aktüatör
      yapışması yok, EKF %100 sağlıklı.
- [x] 🟢 **Geçiş senaryosu — KAPATILDI (2026-07-29).** Duruş → `pos_hold`
      bırakma → Fx rampası 0→10 N. **Devir temiz:** bırakma anında hız
      aktivitesi artmıyor, azalıyor (p RMS 0.2123 → 0.1503, max|ω| 0.3642 →
      0.2538) — roll/pitch sahipliğinin ve `fx_trim`'in el değiştirmesi basamak
      üretmiyor. Rampada 0→10.86 m/s, itki `sat_flag` %0.0, irtifa 0.4 m
      içinde, **0 BIG_M**.
- [ ] 🟡 **Rüzgârda duruş, tilt büyürken** — hâlâ test edilmedi. `fx_trim`'in
      `gain_schedule_smooth` ile sönümlenme yolu normal geçişte hiç
      tetiklenmiyor (11 m/s'de bile `smooth` yalnızca 0.090'a çıkıyor, ve
      geçişte `pos_hold` zaten kapalı). Bu fade bir güvenlik payı; aktif
      mekanizma değil. Şiddetli rüzgârda pozisyon tutarken tilt büyürse
      devreye girer — o senaryo koşulmadı.
- [ ] 🔴 **Pozisyon döngüsü YALNIZCA HOVER İÇİNDİR — bu bir varsayım değil,
      ölçülmüş bir sınır (Adım 29).** 14.5 m/s'de devreye alındığında pitch
      `POS_TILT_MAX`'te doydu, hız 9.6 m/s'de kilitlendi ve araç 35 s boyunca
      sürekli tırmandı; sonraki frenleme denemesinde SITL'de **düştü**. Sebep:
      `theta_sp = -atan2(ax_b, g)` düz multikopter bağıntısı, oysa bu aracın
      kanadı var — ~5-6 m/s üstünde burun yukarı bir frenleme değil **tırmanış**
      komutudur ve irtifa döngüsü kanat taşımasını yenemez.
      Yazılımda `POS_ENGAGE_V_MAX = 3.0 m/s` kapısıyla engellendi (SITL'de
      doğrulandı: hover'da kabul, 9.4 m/s'de red, irtifa kaçışı yok).
      **Donanımda:** bu kapıya güvenmeden önce gerçek araçta doğrulanmalı ve
      pilot bu sınırı bilmeli.
- [x] 🟠 **Geri geçiş (B5)** — artık bir engelleyici olarak §0'da izleniyor.
      SITL'de otomatik ve doğrulandı (Adım 31); **Adım 37'de (2026-07-30) bu
      doğrulamanın marjinal olduğu bulundu, manevra iki uçuşta tamamlanmadı,
      fren yasası ayrıştırılarak düzeltildi ve iki uçuşla yeniden
      doğrulandı**; donanımda denenmedi.
- [x] 🟢 **Ölçütler artık makineyle kontrol ediliyor (Adım 37).**
      `sitl/analyze_backtrans.py` beş geri-geçiş ölçütünü ulog'dan hesaplıyor ve
      pencereyi bağımsız bir sinyalden (`bt_state`, yeni telemetri) kuruyor;
      `sitl/check_output_cuts.py` her uçuşta havada NaN çıkış + `vehicle_attitude`
      boşluğu tarıyor. İkisi de `run_backtrans_test.py` sonunda otomatik koşuyor.
      Doğrulaması: adım 31'in loglarında kayıtlı sayıları yeniden üretti ve
      bilinen başarısız uçuşu doğru şekilde eledi.
- [ ] 🔴 **Hızlıyken alçalmayın** (Adım 29): 13-21 m/s'de inişe geçen bir uçuşta
      98+51 BIG_M aktüatör yapışması sayıldı (kanat taşıması alçalmaya direndiği
      için irtifa döngüsü itkiyi sert kısıyor). Doğrulanmış iniş **hover'dan,
      1.0 m kademelerle** → 0 BIG_M.
- [ ] 🟡 Pozisyon döngüsü kazançlarının (`POS_KP_P/KP_V/KI_V`) gerçek kütle ve
      atalet altında yeniden değerlendirilmesi — **yalnızca MATLAB/SITL'de
      ayarlandı**, gerçek donanımda hiç uçmadı
- [ ] 🔴 `fx_trim = 2.9 N` gerçek araçta yeniden ölçülmeli: bu değer
      *simüle edilmiş* trim'in ulaşılabilir denge noktası. Gerçek araçta yaw
      trim'inin doğurduğu ileri kuvvet farklı olacak.
      **🟡 → 🔴 yükseltildi (Adım 37, 2026-07-30):** artık yalnızca duruş
      kalitesini değil, **geri geçişin tamamlanıp tamamlanmadığını** belirliyor.
      Fren yasasının sönmeyen terimi `BT_TRIM_PITCH = asin(FX_TRIM/(m·g))`
      buradan türüyor; SITL'de bu kuvvetin 3.1-4.1 N olduğu ve otoritenin
      (4°=3.42 N) tam bu aralığın içinde kaldığı ölçüldü — yani hata payı dar.
- [ ] 🟡 LESO açık/kapalı farkı SITL rüzgâr testinde **belirsiz** çıktı
      (p'de biraz kötü, q'da biraz iyi), oysa MATLAB'da 3× kazandırıyor.
      Engelleyici değil ama açıklanmamış bir fark.

---

## 5. Hazır olanlar 🟢

Bunlar çözülmüş ve iki bağımsız ortamda doğrulanmış durumda:

- 🟢 Aktüatör kilitlenmesi (Adım 11-12) — sıfır BIG_M, çok sayıda uçuşta
- 🟢 Yaw savrulması (Adım 13) — eksen bazlı `RATE_SP_LIMIT`
- 🟢 Düşük hızda yaw salınımı, madde (Q) (Adım 23, 27) — `TILT_SLEW_BOX_RATE = 3.00`
- 🟢 Yatay sürüklenme, madde (N) (Adım 28) — pozisyon döngüsü, 235 m → 0.06 m
- 🟢 Yaw yön asimetrisi, madde (P) (Adım 28) — `fx_trim`, 7.4× → 1.02×
- 🟢 MATLAB ↔ Simulink ↔ PX4 sabit senkronu (Adım 27-28)
- 🟢 İrtifa kontrolü — tutuş RMS 0.05-0.09 m

---

## Özet

**Kontrol yasası tarafı olgun.** Bu oturumların çözdüğü şeyler gerçek ve iki
ortamda doğrulandı. **Eksik olan kontrol yasası değil, donanım entegrasyonu:**
pilot girişi/failsafe yazıldı ama yarısı ölçülmemiş ya da çalışmıyor (B1 —
kademeli failsafe'in 1. seviyesi doğrulandı, 3. seviyesi commander'ın otomatik
sonlandırmasıyla eziliyor ve araç düşüyor, pilot yolu hiç çalıştırılmadı),
fiziksel parametrelerin hiçbiri ölçülmedi (B2),
itki eşlemesi simülatöre özgü (B3) ve bilinen bir işaret belirsizliği donanımda
tehlikeli hâle geliyor (B4). Beşinci engelleyici B5 (seyirden hover'a dönüş)
Adım 31'de **çözüldü ama yalnızca SITL'de** — eşiklerinin hepsi aerodinamiğe
bağlı ve ikinci bir doğrulama ortamı yok (MATLAB bu manevrayı yapısal olarak
üretemiyor), üstelik pilot onu başlatamıyor da iptal edemiyor da (B1).
**Adım 38'de B5 🟠'dan 🔴'a çıkmıştı:** madde (R) (RETRACT'ın çıkışı dengenin
sınırındaydı) iki terimli bir çıkışla kapatıldı ve üç uçuşla doğrulandı, ama
aynı doğrulama **madde (S)**'yi ortaya çıkardı — `BRAKE → HANDOFF` eşiği hız
BÜYÜKLÜĞÜNE bakıyordu, oysa fren yasası yalnızca ileri ekseni kontrol ediyor;
giderilemeyen bir yanal bileşen eşiği bloke edince sönmeyen fren pitch'i aracı
**geri yönde 12.8 m/s'ye kaçırdı**.
**Adım 39'da (2026-08-03) madde (S) KAPATILDI ve B5 🔴 → 🟠 indi:** hem eşik hem
frenleme marjı artık işaretli **gövde ileri hızını** okuyor (RETRACT bilerek
büyüklükte kaldı — orası aerodinamik bir soru), üç uçuş + rejimi deterministik
kuran yeni bir probe (`probe_lateral_handoff.py`) ile doğrulandı, ve yeni 6.
ölçüt **gerçek arıza log'una karşı** sınandı. Geriye 🟠 **madde (T)**
(`attitude LOST` uçuş ortasında tetiklenip durum makinesini sıfırlıyor; artık
topic loglanıyor ama madde tekrarlanmadı, mekanizma hâlâ ayırt edilmedi) ve
B5'in asıl sebebi kalıyor: **manevra gerçek donanımda hiç uçmadı** ve
tamamlanması `fx_trim`'e (🔴, ölçülmemiş) doğrudan bağlı.

**En kısa güvenli yol:** B2 + B4 (ölçüm ve doğrulama, uçuş yok) → B3 (statik
itki testi) → B1 (RC + kill switch entegrasyonu) → tezgâh → bağlı → kademeli
uçuş. B1 olmadan hiçbir serbest uçuş denenmemeli. **Seyir ve geri geçiş,
B2+B3 ölçülüp eşikler gerçek aero ile yeniden türetilene kadar donanımda
denenmemeli** (B5).
