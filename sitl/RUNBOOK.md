# Çalıştırma Talimatları — mc_indi_tiltrotor SITL Test Kampanyası (M4-M6)

> Kurulum arka planı, mimari notlar ve bilinen açık sorunlar için → [README.md](README.md)

Bu dosya `gz_tiltrotor_indi` airframe'ini (MATLAB kontrolcüsünün PX4 C++ portu,
`~/PX4-Autopilot/src/modules/mc_indi_tiltrotor/`) SITL'de başlatıp M4-M6 test
kampanyasını çalıştırmanın adımlarını tarif eder.

---

## 0. Ön koşul: land-detector düzeltmesi derlenmiş olmalı

`mc_indi_tiltrotor`, WLS tahsisini kendi içinde yaptığı için standart
`control_allocator` boru hattını atlar ve `vehicle_thrust_setpoint`'i **hiç
yayınlamaz**. PX4'ün stok `MulticopterLandDetector`'ı "yerde/havada" kararını
tam olarak bu topic'ten (`_vehicle_thrust_setpoint_throttle`) verir — topic
hiç gelmeyince `landed=true` sabitlenip kalır ve Gazebo'da araç fiziksel
olarak ne yaparsa yapsın, PX4 `COM_DISARM_PRFLT` (~10 s) ile aracı kendiliğinden
disarm eder (log'da `Disarmed by auto preflight disarming`).

**Düzeltme** (2026-07-24, doğrulandı): `MulticopterIndiTiltrotor.cpp`, her
tick'te komut edilen ortalama normalize rotor itkisini `vehicle_thrust_setpoint`
olarak da yayınlıyor (yalnızca land-detector telemetrisi — kontrol mantığına
dokunmuyor). Kontrol edin:

```bash
grep -q "_vehicle_thrust_setpoint_pub" \
  ~/PX4-Autopilot/src/modules/mc_indi_tiltrotor/MulticopterIndiTiltrotor.hpp \
  && echo "OK: düzeltme kaynak ağacında" || echo "EKSİK: düzeltme yok, aşağıya bakın"
```

Yoksa veya güncel binary'de değilse derleyin:

```bash
cd ~/PX4-Autopilot && make px4_sitl_default
```

> Bu adım atlanırsa M5/M6 script'leri her seferinde ~10 s'de "auto preflight
> disarming" ile kesilir — LESO açık/kapalı, WLS kalitesi veya EKF sağlığından
> **bağımsız**, tamamen bu eksik topic yüzünden. Ayrıntı → README.md § Bilinen
> davranış/tuzaklar.

---

## 1. SITL'i başlat

```bash
pkill -9 -f 'px4_sitl_default/bin/px4'; pkill -9 -f 'gz sim'; rm -f /tmp/px4-sock-0
cd ~/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridge
PX4_SIM_MODEL=gz_tiltrotor_indi HEADLESS=1 ../../../../bin/px4 -d > /tmp/px4.log 2>&1 &
grep -a 'Ready for takeoff' /tmp/px4.log   # bekleyin
```

`launch_sitl()` (bkz. `indi_sitl_common.py`) bu diziyi M5/M6 script'leri için
otomatik yapar — kendi başınıza çalıştırmanız yalnızca M4 `smoke_test.py`
öncesinde veya elle gözlem yaparken gerekir.

### 1a. Gazebo arayüzüyle izleme + kamerayı araca kilitleme

> **Test betiklerinde tek anahtar (2026-07-30, Adım 37):** `INDI_SITL_GUI=1`.
> `indi_sitl_common.launch_sitl()` bu değişkeni görürse `HEADLESS` vermez,
> `DISPLAY`i (varsayılan `:1`) geçirir ve "Ready for takeoff"tan sonra
> `gz_follow.sh`'i **kendisi** çağırır — yani aşağıdaki elle adımların hepsi
> her betik için otomatik olur. Uçuş birebir aynıdır, yalnızca render eklenir;
> tek yan etki soğuk başlangıcın yavaşlaması (ready timeout'u 150 s'ye çıkar).
>
> ```bash
> export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH   # px4-* istemcileri
> INDI_SITL_GUI=1 python3 run_backtrans_test.py
> ```
>
> `PATH` satırı atlanırsa betik `FileNotFoundError: 'px4-commander'` ile
> **arm etmeden** düşer (kosu boşa gider, 2026-07-30'da bir kez oldu).
>
> **Kamera görüşü — TİLTİ GÖRMEK İÇİN (2026-08-03, Adım 39).** Eski varsayılan
> araca 2 m arkadan bakıyordu ve naseller tam profilden gizleniyordu: bu
> projenin uçuşlarını harcadığı asıl olay, tavanın **45° → 9° → 20°** rampası,
> izlenemiyordu. Artık dört hazır görüş var ve `INDI_GZ_CAM` ile seçiliyor
> (`follow_model()` bunu **bütün** testlere uygular):
>
> | ad | offset (gz FLU: +x ileri, +y SOL) | ne işe yarar |
> |---|---|---|
> | `front` **(varsayılan)** | `+2.2 −2.2 +0.6` | ön-çapraz 45°; burun + nasel açısı birlikte |
> | `nose` | `+3.2 0.0 +0.4` | tam karşıdan; en temiz kadraj ama **ara açıları (9° vs 20°) ayırt ettirmez** |
> | `side` | `0.0 −3.2 +0.4` | tam yandan; **nasel açısı doğrudan okunur** — tavan rampası için en iyisi |
> | `chase` | `−2.0 0.0 +0.8` | eski varsayılan; tilt görünmez |
>
> ```bash
> INDI_SITL_GUI=1 INDI_GZ_CAM=side python3 run_backtrans_test.py
> INDI_GZ_CAM="1.5 -3.0 1.0" ...     # ad yerine ham offset de verilebilir
> ```
>
> **Koşular artık kademeli inip YERDE disarm ediyor** (Adım 39): önceden
> `run_backtrans_test.py` / `probe_lateral_handoff.py` manevra biter bitmez
> 25 m'de disarm ediyordu, yani GUI'de izlenen son şey aracın düşüşüydü.
> Ölçütler etkilenmez — `bt_enable` inişten önce bırakılır, pencere eskisi gibi
> kapanır. (Açık bırakılsaydı 25 m'lik iniş, irtifa bandı ölçütünü düşürürdü.)

Uçuşu elle izlemek için `HEADLESS=1` **verilmez** ve `DISPLAY` ayarlanır:

```bash
cd ~/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridge
DISPLAY=:1 PX4_SIM_MODEL=gz_tiltrotor_indi ../../../../bin/px4 -d > /tmp/px4.log 2>&1 &
```

**"Ready for takeoff" göründükten sonra kamerayı modele kilitleyin — bu adım
her GUI'li uçuşta yapılmalı:**

```bash
"$(dirname "$0")"/gz_follow.sh          # veya: sitl/gz_follow.sh
# elle karşılığı:
#   gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
#     --reptype gz.msgs.Boolean --timeout 3000 --req 'data: "tiltrotor_indi_0"'
#   gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d \
#     --reptype gz.msgs.Boolean --timeout 3000 --req 'x: -2.0, y: 0.0, z: 0.8'
```

**Neden zorunlu:** bu kontrolcüde **yatay konum döngüsü yok** (yalnızca
tutum + irtifa). Araç yaw'da dönerken tilt'ten doğan Fx onu sürükler; bir
koşuda başlangıç noktasından **~170 m** uzaklaştığı ölçüldü. Kamera
kilitlenmezse araç ilk saniyelerde görüş alanından çıkar.

**Kamera takibi tek başına yetmez (Adım 15'te ölçüldü).** İki ek şey:

- `Tools/simulation/gz/worlds/default.sdf`'teki `CameraTracking`
  eklentisine **`<follow_pgain>1.0</follow_pgain>`** eklendi — varsayılan
  0.01 kamerayı hedefe o kadar yavaş yaklaştırıyor ki hızlı hedefe hiç
  yetişmiyor (30 s'de ~170 m geride kalıyordu; 1.0 ile ~62 m'de plato).
- **Uzun gözlem koşularında pitch trim komut edin:**
  ```bash
  px4-mc_indi_tiltrotor test_sp 0 0.061 0 0 $z_sp 1 1 0   # +3.5 deg burun yukari
  ```
  Araç hover'da yapısal olarak ileri kuvvet üretiyor (tüm tiltler ≥0
  olduğundan net Fx ≥ 0; yaw trim'i ~3 N ileri itki doğuruyor — §4 (N)).
  `atan(3/49) ≈ 3.5°` burun yukarı bunu dengeliyor: sürüklenme
  **9.4 → 1.7 m/s**. Kriter koşularında (`sitl-lockup-check`) pitch_sp=0
  kalmalı; bu yalnızca gözlem kolaylığı içindir.

**İki tuzak:**
- `gz` follow modu kamerayı **modelin çerçevesinde** taşır; araç yaw'da
  dönüyorsa kamera da döner ve dönüş "araç sabit, yer dönüyor" gibi görünür.
  **Yaw davranışını gözle değil, her zaman `vehicle_angular_velocity.xyz[2]`
  ile doğrulayın** (bkz. §4, Adım 12b: 5 s'lik açı örneklemesi bir kez
  sürekli dönüşü "sınırlı gezinme" sandırdı).
- **İrtifadayken `disarm` ETMEYİN.** Araç 6 m'den düşüp takla atar ve ters
  yatar; sonra yanlışlıkla yeniden arm edilirse "uçak ters uçuyor" gibi
  görünen ama aslında yerde yatan bir durum oluşur. Önce alçak bir `z_sp`
  ile indirin, sonra disarm edin.

---

## 2. Test sürücülerini çalıştır

```bash
export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH
cd "tiltrotor_Matlab files/sitl"

python3 smoke_test.py             # M4 — arm + hover setpoint + EKF sağlık takibi
python3 run_hover_gust_test.py    # M5 — LESO açık/kapalı roll bozucu karşılaştırması
python3 run_transition_test.py    # M6 — Fx_sp 0->10N rampası, hover->ileri geçiş

# Sonradan eklenenler (Adım 31-35)
python3 run_lockup_check.py       # ZORUNLU regresyon kapısı: sitl-lockup-check
                                  #   kriter koşusu, betikleşmiş hali
python3 run_backtrans_test.py     # B5 — otomatik geri geçiş (hover->seyir->hover)
python3 probe_pilot_link.py       # B1 — pilot yolu ULAŞILABILIR mi (uçuş YOK)
python3 run_pilot_input_test.py   # B1 — pilot girişi kriter koşusu (Adım 40)
python3 run_failsafe_test.py --level 1   # NO_POS: xy kestirimi alınır, uçmaya devam
python3 run_failsafe_test.py --level 2   # NO_ALT: TETİKLENEMİYOR (ölü dal, bkz. rapor)
python3 run_failsafe_test.py --level 3   # DURUŞ KAYBI: çıkış KESİLMELİ (Adım 35 sonrası;
                                  #   eskiden RATE_ONLY idi, ölçülüp kaldırıldı)
```

`--level 3`'ün anlamı **Adım 35'te tersine döndü**: eskiden "bozulup uçtu mu"
diye bakıyordu, artık "duruş kaybında temiz, kayıtlı ve zamanında kesti mi"
diye bakıyor. Ayrıntı → `WLS_LOCKUP_INVESTIGATION_REPORT.md` Adım 35.

**Analiz betikleri (uçuş başlatmaz, ulog okur — Adım 37-38):**

```bash
python3 analyze_backtrans.py [ulog]        # geri geçişin ALTI ölçütü + evre dizisi
python3 check_output_cuts.py [ulog]        # havada NaN çıkış + duruş boşluğu
python3 brake_ceiling_margin.py <ulog>...  # BRAKE'te BT_BRAKE_CEIL payı (Adım 38)
python3 diag_brake_reversal.py <ulog>      # madde (S)/(T): büyüklük vs gövde ileri/yanal
                                           #   hız + estimator_status_flags tazeliği
python3 probe_lateral_handoff.py           # UÇUŞ BAŞLATIR — madde (S) rejimini
                                           #   deterministik kurar (Adım 39)
```

**`probe_lateral_handoff.py` neden var (Adım 39):** madde (S) yanal sürüklenme
gerektirir ve SITL'de yanal sürüklenme RASTLANTISALDIR — kancalı bir uçuş bile
1.28 m/s'de kaldı, yani rejimi kurmadan "geçti" verdi. Probe frenleme sırasında
heading'i +90° çevirerek mevcut hızı gövde çerçevesinde ileriden yanala taşır
(ölçülen arızanın geometrisi budur) ve yanalı 4.76 m/s'ye çıkarır. **Bu bir
kriter koşusu DEĞİLDİR:** 90°'lik dönüşü kendisi komut ettiği için
`analyze_backtrans.py`'nin 4. ölçütünü (toplam yaw dönüşü ≤ 45°) bilerek ihlal
eder. Bakılacak yer probe'un kendi çıktısıdır.

**6. ölçüt = YENİDEN-HIZLANMA, işaretli hız DEĞİL (Adım 39, sert ders):** ilk
yazılışı "işaretli gövde ileri hızı ≥ −2.0 m/s" idi ve probe'da KALDI (−3.95)
— oysa araç kaçmıyordu, `v_h` monoton 9.74 → 3.00 iniyordu; heading aynı
pencerede **+186.7°** dönmüştü ve `v_fwd` DÖNEN bir çerçeveye izdüşümdür. Yani
ölçüt, madde (S)'nin kendi hatasını tekrarlamıştı. Şimdiki hâli çerçeveden
bağımsız: `v_h`'nin koşan minimumundan yükselişi ≤ 1.0 m/s. **Gerçek arıza
log'una karşı doğrulandı** (2026-07-31/14_13_11: 10.31 m/s → KALDI). Bir ölçüt
yazdığınızda onu geçmesi gereken uçuşlarda değil, **KALMASI gereken uçuşta**
sınayın.

**`BT_FLOOR_DWELL` emniyetini SITL'de görmek için kanca gerekir** (Adım 38):
gerçek `BT_RELEASE_V = 10.0f` ile hız koşulu her zaman sağlandığı için emniyet
hiç çalışmaz. `TiltrotorIndiParams.hpp`'de değeri geçici olarak `5.0f` yapıp
yeniden derleyin; px4 log'unda
`state 1 -> 2 ... [via FLOOR_DWELL backstop]` + WARN satırını arayın. **Ölçüm
bitince değeri geri alıp yeniden derlemeyi unutmayın.**

M5/M6 kendi SITL örneğini başlatıp bitirdiğinde kapatır (`kill_sitl()`) —
aralarında adım 1'i elle tekrarlamanıza gerek yok.

Uzatılmış (extended) M5 koşusu için:

```bash
python3 run_hover_gust_test.py --duration 70 --suffix _extended
```

---

## 3. Beklenen çıktı (2026-07-24'te doğrulanan referans)

Düzeltme sonrası, temiz bir çalıştırmada görülmesi gerekenler:

| Test | Beklenen |
|---|---|
| M5 — `Disarmed by …` log satırı | `Disarmed by internal command` (script'in kendi disarm'ı) — **asla** `auto preflight disarming` |
| M5 — `ekf healthy` oranı | `%100 of window` her iki LESO konfigürasyonunda |
| M5 — RMS p/q (LESO açık vs kapalı) | `LESO acik: RMS p=0.5368, RMS q=0.3164` · `LESO kapali: RMS p=0.0154, RMS q=0.0275` — bkz. not aşağıda |
| M6 — `Disarmed by …` log satırı | `Disarmed by internal command` |
| M6 — `EKF healthy` | `%100 of scenario` |
| M6 — ileri hız / ort. tilt (senaryo sonu) | `vx≈5.38 m/s`, `ortalama tilt≈32.5°` |

**Not (M5 RMS ters görünüyor):** LESO açık koşusu, kapalı koşudan **daha
yüksek** RMS p/q veriyor — LESO'nun amacı bozucuyu bastırmak olduğundan bu
beklenenin tersi. Bu, düzeltmeden **bağımsız, ayrı bir bulgu**; henüz kök
nedeni araştırılmadı. Ayrıca M6'da ortalama tilt hâlâ ~32.5° ve senaryo
boyunca azami `|omega|` 4.73 rad/s — README'deki "Bilinen açık sorun (M6)"
bölümünde tarif edilen asimetrik WLS/kuyruk-tilt olgusuyla tutarlı olabilir;
bu bölüm artık disarm hatasından bağımsız olarak yeniden değerlendirilmeli.

---

## 4. Bilinen ciddi sorun: WLS aktüatör kilitlenmesi (ÇÖZÜLDÜ — Adım 11/12/13) + yaw'ın düşük hızda sönümsüzlüğü (ÇÖZÜLDÜ — Adım 23/27)

> **⚠️ GERİ GEÇİŞ: fren yasası Adım 37'de (2026-07-30) DEĞİŞTİ — eskisi
> manevrayı yarıda bırakıyordu.** Geri geçiş testini koşarken beklenen dizi
> RETRACT → BRAKE → **HANDOFF**; BRAKE evresi ~6 s sürer. Eğer araç
> **3-5 m/s civarında takılıp orada kalıyorsa** eski (tek terimli) fren
> yasasıyla koşuyorsunuz demektir: `pitch = BT_PITCH_MAX·v_h/BT_BRAKE_V_FULL`
> hız düştükçe sönüyordu, ama yenmesi gereken ileri kuvvet sönmüyor
> (δ1/δ2 `TILT_MIN`'de çakılı → yaw trimi δ0'ı 10-15°'de tutuyor → 3.1-4.1 N).
> Yeni yasa ayrıştırılmış: `BT_TRIM_PITCH (3.39°, sönmez) + BT_PITCH_MAX·fade`.
> Ölçütleri elle bakmayın — `python3 analyze_backtrans.py [ulog]` beşini de
> ulog'dan hesaplar (`run_backtrans_test.py` sonunda otomatik çağrılır).

> **⚠️ KRİTER KOŞUSU YAPARKEN: `yaw_sp = 0` VERMEYİN.** (Adım 27, 2026-07-29)
> Araç Gazebo'da **+90° heading'de doğuyor** (ground-truth ile ölçüldü),
> dolayısıyla `yaw_sp = 0` bir hover tutuş testi değil **90°'lik dönüş
> komutu** demek. `[arm+3, arm+32]` penceresindeki yaw bandı da o dönüşün
> 3. saniyedeki yerini ölçer — Adım 22-24'te "yaw ❌ 35-37°" diye
> raporlanan şey bu artefakttı, gerçek bir kusur değil. Doğrusu:
> arm anındaki heading'i okuyup `yaw_sp` olarak vermek:
> ```bash
> yaw0=$(px4-listener vehicle_attitude | python3 -c "
> import sys,re,math; s=sys.stdin.read()
> q=[float(x) for x in re.search(r'q:\s*\[([^\]]*)\]',s).group(1).split(',')]
> print(round(math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]**2+q[3]**2)),5))")
> px4-mc_indi_tiltrotor test_sp 0 0 $yaw0 0 $z_sp 1 1 0
> ```
> Ayrıca EKF yaw'ı gerçek heading'den **~5-10° sapıyor** — yaw sayılarını
> fiziksel yönelim sanmayın.
>
> **GERÇEK HOVER TESTİ ARTIK MÜMKÜN (Adım 28, 2026-07-29).** Yatay pozisyon
> döngüsü eklendi (madde (N)). `test_sp`'nin 10. argümanı `pos_hold`:
> ```bash
> px4-mc_indi_tiltrotor test_sp 0 0 $yaw0 0 $z_sp 1 1 0 1
> #                                                     ^ pos_hold = 1
> ```
> Hedef, bayrağın `false→true` kenarında yakalanır — koordinat bilmeniz
> gerekmez. Ölçülen: hedeften sapma ortalama 0.06 m (eskiden 25 s'de 235 m).
> `pos_hold` açıkken roll/pitch'i **döngü sahiplenir** (verdiğiniz roll_sp/
> pitch_sp yok sayılır) ve `fx_sp`'yi de döngü üretir (madde (P) trim'i).
> **Adım 15'in `pitch_sp = 0.061` geçici çözümüne artık gerek yok.**
>
> **Yaw durumu (Adım 27, 2026-07-29):** `TILT_SLEW_BOX_RATE` 1.75 → **3.00**
> dağıtıldı ve doğrulandı. İki bağımsız tutuş uçuşunda kriterin dördü de
> geçti: yaw hata bandı −7.3…+5.9° / −10.3…+10.3°, yaw hızı RMS
> 0.0014-0.0028 rad/s, \|vz\| ≤ 1.74 m/s, irtifa tutuş RMS 0.052-0.072 m,
> sıfır BIG_M, itki `sat_flag` %0.0. Kutunun gerçekten aktif olduğu
> ulog'dan `|du(tilt)| p99.5 / TS_BOX = 3.000 rad/s` ile doğrulandı.
> **İniş uyarısı:** iniş kademeleri **1.0 m** olmalı — 1.5 m'lik kademeler
> iniş fazında 13 BIG_M üretti (madde (O)).
>
> **ÖNCE BUNU OKUYUN.** Aşağıdaki §4 gövdesi Adım 1-10'un canlı araştırma
> logudur ve **tarihsel kayıt** olarak korunmuştur. 2026-07-27'de (Adım 11)
> kök neden bulunup düzeltildi; aşağıdaki birçok ara hipotez ve ölçüm
> artık bu ışıkta yeniden yorumlanmalıdır.
>
> **Kök neden:** `MulticopterIndiTiltrotor.cpp` itki komutunu
> `motors.control[i] = u_cmd(i)/ROTOR_TMAX` ile **doğrusal** gönderiyordu,
> ama gerçek zincir karesel: `MixingOutput` normalize komutu
> `SIM_GZ_EC_MIN=10 … MAX=1500` rad/s'ye doğrusal ölçekliyor, ardından
> gz `MulticopterMotorModel` `T = 2e-5·w²` uyguluyor. İki model yalnızca
> `control=1` (45 N) uç noktasında uyuşuyordu. Sonuçları: (a) gerçek toplam
> itki ağırlığın (49 N) altında kalıyordu — araç kalkamıyordu; (b) WLS'in
> etkinlik matrisi itkiye bağlı biçimde yanlıştı (gerçek `d(T)/du` 5 N'da
> 0.23, 45 N'da 1.99; kontrolcü hep 1.0 sanıyordu), bu da düşük itkili
> rotoru tabana iten pozitif geri besleme yaratıyordu.
> Düzeltme: `thrustToNormalized()` (karekök tersleme).
>
> **İkinci hata:** `test_sp`'nin fonksiyon-yerel `uORB::Publication`'ı
> dönerken `orb_unadvertise()` çağırdığı için **setpoint kontrolcüye hiç
> ulaşmıyordu** (60 ardışık yayından sonra bile topic "never published").
> Yani §4'teki tüm koşular aslında "6 m tırmanış" değil **"mevcut irtifayı
> koru + attitude setpoint'leri 0"** testiydi. Düzeltme: publication
> `static` yapıldı.
>
> **Düzeltme sonrası (2 bağımsız koşu, 25s ve 40s):** 6 m tırmanış ~0.15 m
> hatayla takip ediliyor, **hiçbir aktüatör kilitlenmiyor** (82 örnekte
> sıfır BIG_M sabitlemesi), roll/pitch ≤0.5°, `|vz|` ≤ 0.20 m/s.
> **Kalan tek açık sorun:** yaw. (Adım 11'de "±60° bandında geziniyor"
> denmişti — Adım 12 bunun bir örnekleme artefaktı olduğunu gösterdi,
> aşağıya bakın.)
>
> ---
>
> **GÜNCELLEME — Adım 12 (2026-07-27): ikinci bir kök neden bulundu,
> yaw ölçümü düzeltildi.**
>
> 1. **`ROTOR_KM` işaretleri FRD çerçevesinde tersti** (üç rotorda da).
>    gz gövdeye `tau_z(FLU) = -turningDirection·T·km` uygular; FLU→FRD z
>    çevrimiyle `+km·T` olur, model ise `-km·T` diyordu. Sonuç:
>    `hoverTrim()`'in yaw sıfırlayıcı tilt'i dengesizliği gideriyor değil
>    **artırıyordu**. Ölçümle doğrulandı: arm anındaki tepe yaw ivmesi
>    **+6.5 → +0.47 rad/s² (14×)**. Düzeltildi (MATLAB + Simulink + PX4);
>    MATLAB regresyonu **iyileşti** (RMS p/q 2-5×).
> 2. **"±60° gezinme" ÖLÇÜM ARTEFAKTIYDI.** 5 s aralıklı `px4-listener`
>    açı örneklemesi, sürekli dönüşü (ort. +1.44 rad/s, 40 s'de 5 tam
>    tur) rastgele açılara dönüştürüyordu. **Yaw'ı açıdan değil hızdan
>    ölçün:** `px4-listener vehicle_angular_velocity` → `xyz[2]`, ya da
>    ulog (`vehicle_angular_velocity.xyz[2]`).
> 3. **LESO yaw ekseninde AÇILMAMALI** — düzeltilmiş G altında bile
>    5 saniyede aracı ters çevirdi (`test_sp ... 1 1 1` denemeyin).
> 4. **Kalan dönüşün mekanizması ölçüldü** (`YWdbg` logu): tahsisat
>    talep edilen yaw torkunun **%6.8'ini** üretebiliyor çünkü tilt
>    komutu sürekli `TILT_RATE_MAX·dt = 0.005 rad` slew limitinde;
>    ayrıca araç dönerken heading sarmalandığı için dış döngü yaw hız
>    setpoint'ini ±3 rad/s arasında sürekli ters çeviriyor.
>
> Ayrıntı → `WLS_LOCKUP_INVESTIGATION_REPORT.md` Adım 12.
>
> ---
>
> **GÜNCELLEME — Adım 13 (2026-07-27): YAW ÇÖZÜLDÜ, üç kriter de geçti.**
>
> Çözüm yaw'a otorite eklemek değildi (Adım 6/7/10'da denenen üç yol da
> yanlış soruyu çözüyormuş). Dış attitude döngüsünün hız setpoint limiti
> tek skalerdi (3.0 rad/s); araç dönerken yaw hatası ±180°'de
> sarmalandığı için bu limit `omega_sp(2)`'nin sürekli işaret
> değiştirmesine yol açıyor ve **zamanın yarısında iç döngü dönüşü
> sönümlemek yerine hızlandırıyordu**. Limit eksen bazlı yapıldı:
> **`[3.0, 3.0, 0.5]` rad/s** (`p.ctrl.rate_sp_limit` /
> `sf_indi_rate_law.m` / PX4 `RATE_SP_LIMIT[3]`).
>
> | Ölçüt (t_arm+10…35 s) | Önce | Sonra |
> |---|---|---|
> | Yaw hızı RMS | 1.91 rad/s | **0.058 rad/s** |
> | 25 s'de integre dönüş | 1879° | **44°** |
> | Yaw açısı (yerleşmiş) | ±180° sarmalıyor | **≤1.6°** |
> | Roll/pitch hız RMS | 0.054 / 0.070 | **0.0008 / 0.0021** |
> | İrtifa hata RMS / \|vz\|max | 0.456 m / 8.25 m/s | **0.211 m / 0.17 m/s** |
>
> Yaw artık takip de ediyor: `yaw_sp=+30°` komutu 8 s'de izleniyor,
> salınımsız. **Ders:** zayıf otoriteli bir eksende dış döngü hız limiti,
> o eksenin gerçekten ulaşabileceği hızın ALTINDA tutulmalı.
>
> **ÖNEMLİ DARALTMA — Adım 16:** yukarıdaki tüm sonuçlar araç ~10 m/s
> ileri hızdayken alındı (pozisyon döngüsü yok, araç sürekli hızlanıyor).
> Aynı yaw adımı 2.45 m/s'de **±25°'lik sönümsüz salınım** veriyor. Yaw'ı
> sönümleyen şey kontrolcü değil, ileri hızdaki aerodinamik rüzgâr gülü
> etkisi. **Gerçek, yerinde duran hover'da yaw kriteri sağlanmıyor** —
> bkz. rapor §4 (Q). Test yaparken aracın hızını mutlaka kaydedin.
>
> **Yeni açık maddeler:** yatay sürüklenme 25 s'de 235 m (yaw trim tilt'i
> hover'da kaçınılmaz ileri kuvvet üretiyor, tilt aralığı tek yönlü) ve
> agresif adım-alçalmada rotorlerin ~68°'ye eğilip bir itki kanalının
> sıkışması. Ayrıntı → rapor Adım 13b ve §4 (L/N/O).

### (tarihsel) Adım 1-10 araştırma logu

**Belirti:** Arm edilip bir irtifa/hover setpoint'i verildiğinde, kanat
rotorlarından biri (T0 veya T1) birkaç saniye içinde 0 N'a düşüp orada
kilitleniyor; diğer rotor(ler) telafi etmeye çalışıyor. GUI açıkken (ek CPU
yükü altında) bu durum daha da kötüleşip aracın tamamen ters dönmesine
(roll ≈180°) kadar gidebiliyor. 2026-07-24'te canlı GUI incelemesi sırasında
gözlendi — bkz. bu oturumun geçmişi.

**Kök neden (doğrulandı, karşılaştırmalı testle izole edildi):**

1. MATLAB referansı — hem `wls_allocate.m` (aktif-küme) hem de `sf_wls_alloc.m`
   tarzı büyük-M yöntemi — aynı senaryoda (hover_trim'den başlayıp 6 m tırmanma,
   bozucu yok) **kusursuz**: T0/T1 simetrik kalıyor, roll/pitch hep < 0.02°.
   Bu, WLS algoritmasının kendisinde bir tasarım hatası **olmadığını** kanıtlar
   — `TiltrotorIndiControl.hpp::wlsAllocate()` matematiksel olarak
   `sf_wls_alloc.m` ile birebir aynıdır.
2. Aynı senaryo gerçek SITL'de (GUI'siz bile) T0/T1'den birini ~1 s içinde
   0 N'a çöktürüp orada kilitliyor — MATLAB'da hiç görülmeyen bir davranış.
3. Modüle geçici bir `PC_INTERVAL` performans sayacı eklenip ölçüldü:
   `Run()` çağrıları arası gerçek süre **ortalama ~4 ms (~250 Hz)** —
   MATLAB'ın ve fiziksel aktüatör sabitlerinin varsaydığı **sabit 2.5 ms
   (400 Hz)**'nin **%60 daha yavaşı** — üstelik düzensiz: `min 0 µs, max
   8000 µs, rms ≈540 µs`.
4. `Run()`'daki `dt = constrain((now-_last_run)*1e-6f, 0.000125f, 0.02f)`
   bu gerçek (titreşen) `dt`'yi WLS'in aktüatör **hız-limiti kutusuna**
   (`rate_lo/hi ∝ dt`, bkz. `MulticopterIndiTiltrotor.cpp` ~satır 310-328)
   doğrudan besliyor. `dt` çok küçük geldiği tick'lerde bu kutu daralıyor,
   o tick'te ilgili aktüatör olduğu yerde donuyor; büyük-M çözücü bunu
   doygunluk sayıp o aktüatörün ağırlığını `1e6`'ya sabitliyor. Art arda
   gelen düzensiz `dt` tick'leri bir rotoru adım adım aşağı "mandallayarak"
   0'a kilitleyebiliyor — MATLAB'ın kusursuz sabit `Ts_ctrl` simülasyonunun
   hiç maruz kalmadığı bir durum.

**Durum:** Düzeltilmedi. Aday çözümler:

- ~~Hız-limiti kutusunu gerçek ölçülen `dt` yerine sabit `Ts_ctrl` (1/400 s)
  varsayımına göre hesaplamak~~ — **denendi (2026-07-25), sonuç: ÇÖZMEDİ.**
  Bkz. aşağıdaki "Aday çözüm 1 — sonuç" bölümü; hipotez kısmen çürütüldü.
- `dt`'ye alt sınırı (`0.000125f`) yükseltmek veya kısa vadeli ortalama
  (filtrelenmiş `dt`) kullanmak, tek bir anormal-küçük tick'in kutuyu
  aşırı daraltmasını önlemek için. (Denenmedi.)
- Büyük-M iterasyonunda bir tick içinde doygunlaşan aktüatörü kalıcı
  kilitlemek yerine, birden fazla ardışık tick boyunca doygun kalırsa
  kilitlemek (histerezis). (Denenmedi — not: mevcut `wlsAllocate()`'te
  `Wu_eff`/`du_pref` zaten her `Run()` çağrısında sıfırdan başlıyor, yani
  tick'ler arası kalıcı bir "mandal" durumu **yok**; histerezis eklenecekse
  yeni bir persistent state gerekir.)

**Aday çözüm 1 — sonuç (2026-07-25, ÇÖZMEDİ):**

`TiltrotorIndiParams.hpp`'ye `TS_CTRL = 1/400 s` sabiti eklenip yalnızca
WLS hız-limiti kutusu hesabında (`MulticopterIndiTiltrotor.cpp` ~satır
315-325) gerçek `dt` yerine kullanıldı — entegrasyon/decimasyon için
kullanılan diğer `dt` referansları (satır 171, 247, 274, 396, 402)
dokunulmadan bırakıldı. Derlenip aynı repro senaryosuyla (arm + 6 m
tırmanma) tekrar test edildi:

- `u_actual[0]` yine 10 saniye boyunca kesintisiz `0.00000`, `sat_flag[0]`
  yine sürekli `True` — düzeltmeden önceki davranışla **bit bit aynı**.
- Yaw hâlâ kontrolsüzce dönüyor (10 s'de defalarca ±180°'ye yakın geçiş).
- Sabit kutu ile T0'ın yukarı hareket payı (`du_max(0)`) ~22.5 N
  (45 N tavanın yarısı) — yani kutu T0'ı yukarı çıkmaktan **alıkoymuyor**;
  çözücü her tick'te kendi optimizasyonuyla `du(0) ≤ 0` seçiyor.
- Kod incelemesi: `wlsAllocate()` (`TiltrotorIndiControl.hpp` satır
  139-198) içindeki büyük-M ağırlığı (`Wu_eff`) ve `du_pref`, fonksiyon
  her çağrıldığında (`Run()` her tick'te) sıfırdan başlıyor — tick'ler
  arası kalıcı bir "mandal" durumu tutan bir state **yok**. Bu, "düzensiz
  dt tick'leri T0'ı adım adım aşağı mandallıyor" hipoteziyle çelişiyor;
  gözlenen kilitlenme muhtemelen her tick'te bağımsız olarak yeniden
  üretilen gerçek bir optimizasyon sonucu (örn. kontrolsüz yaw hatasının
  ürettiği büyük/salınımlı tork talebi + `Ws`/`Wu` ağırlıklandırması ile).

  **Sonraki adım için ipucu:** `effectivenessMatrix()`'te T=0 olduğunda
  `dtau_ddelta` (tilt sütunları) sıfırlanıyor (T ile çarpılıyor) ama
  `dtau_dT` (itki sütunu, T0'ın kendisi) **sıfırlanmıyor** — yani T0'ın
  itki sütunu hâlâ tam etkili görünüyor, dejenere/tekil bir G matrisi
  değil.

**Kök neden bulundu (2026-07-25, teşhis logu ile doğrulandı):**

`wlsAllocate()` içine geçici, throttle'lı (`PX4_INFO`, 500ms) bir tanı
logu eklenip (`TiltrotorIndiControl.hpp`, "TEMP DIAGNOSTIC" yorumuyla
işaretli — **kalıcı değil, temizlenecek**) `G` sütun 0, `nu_des`,
`Ws(3)/Ws(4)`, `Wu_eff(0)`, `du(0)`, `du_min(0)/du_max(0)` tick başına
izlendi. Sonuç, §4'ün orijinal "dt titreşimi kutuyu daraltıyor"
hipotezinden **farklı ve daha temel** bir mekanizma ortaya çıkardı:

1. T0'ın etkinlik sütunu sabit ve şu (hover, δ0≈0): `dtau/dT0 =
   [roll: -0.25, pitch: +0.22, yaw: -0.05, Fx: 0.00, Fz: -1.00]`.
2. Arm'dan hemen sonra `nu_des(0)` (INDI'nin istediği roll-tork
   düzeltmesi) küçük ama **kalıcı pozitif ve büyüyen** bir değer alıyor
   (0.01 → 0.06 → 0.10 → 0.15 → 0.20...). Roll açısının kendisi bu sırada
   küçük kalıyor (< 1°) — yani bu bir açı hatası değil, INDI'nin artımlı
   ivme talebi (muhtemelen LESO `d_hat(0)` tahmini üzerinden).
3. Roll ağırlığı (`Ws(0)=200`) diğer eksenlere (`Ws(3)=0.1` Fx,
   `Ws(4)=20` Fz) göre ezici derecede yüksek. En-küçük-kareler çözücü
   için pozitif roll-tork üretmenin "en ucuz" yolu, roll etkinliği en
   güçlü aktüatör olan T0'ı **azaltmak** (`-0.25 × ΔT0<0 → +tork`) —
   T0'ın Fz'ye tam katkısı (-1.00, en güçlü Fz aktörü) olmasına rağmen,
   büyüyen `nu_des(4)` (Fz talebi) T0'ı yukarı çekmiyor çünkü Fz
   ağırlığı (20) roll ağırlığının (200) çok altında kalıyor.
4. T0, tick tick azaltılarak fiziksel tabana (`ROTOR_TMIN=0`) çarpıyor.
   O noktada `du_min(0) = max(ROTOR_TMIN - u_actual(0), rate_lo)` doğal
   olarak `0`'a yaklaşır (aktüatör zaten tabanda, daha fazla azaltılamaz
   — bu kısım doğru/beklenen davranış). Büyük-M o tick'in iç
   iterasyonunda `Wu_eff(0)=1e6` yapıp kutuyu zorluyor; ama bu tick'e
   özel, kalıcı bir mandal **değil**.
5. Asıl sorun: bir sonraki tick'te `nu_des(0)` **hâlâ pozitif**
   olduğundan (roll-tork talebi geçmiyor), çözücü yine aynı sonuca varıp
   `du(0)=0`'da kalmayı "seçiyor" — T0 hiç yukarı çekilmiyor çünkü onu
   yukarı çekmenin roll ekseninde yaratacağı ceza (200² ağırlık),
   sağlayacağı Fz faydasından (20² ağırlık) çok daha ağır basıyor.
   Bu, kendi kendini besleyen bir döngüye benziyor: T0 düştükçe
   `hoverTrim()`'in varsaydığı reaksiyon-tork dengesi bozuluyor,
   bu da daha fazla roll-tork talebi üretiyor, bu da T0'ı daha da
   aşağı çekiyor.

**Sonuç:** Aday çözüm 1 (sabit `TS_CTRL`) bu mekanizmaya dokunmuyor,
çünkü sorun kutu genişliğinde değil, **ağırlıklandırma + WLS'nin roll
düzeltmesini T0 üzerinden çözmeyi tercih etmesinde**. Olası yönler
(henüz denenmedi): (a) `Wu(0..2)` için tabana yakın aktüatörlere ekstra
ceza eklemek (soft anti-drain), (b) roll/yaw düzeltmesi için tilt
farkını (δ) itki farkına tercih edecek şekilde `Wu_tilt` ağırlıklarını
yeniden dengelemek, (c) `nu_des(0)`'ın neden kalıcı pozitif kaldığını
(LESO `d_hat(0)` mü, `hoverTrim()` mü) ayrı olarak izole etmek.

**Aday çözüm 2 — Wu_tilt yeniden dengeleme, sonuç (2026-07-26, ÇÖZMEDİ,
sorunu TAŞIDI):**

Analitik türetme: hover trim itkisinde (`Tw≈18.32N`) roll ekseni icin
`|dtau_ddelta0| = Tw*0.05 ≈ 0.916`, `|dtau_dT0| = 0.25` (sabit) — tilt,
thrust'tan ~3.665x daha etkili. WLS'nin "birim tork başına ceza"
karşılaştırması `Wu_i/|G_i|` olduğundan, tilt'in thrust'a tercih
edilmesi için `wu_tilt < wu_thrust*(0.916/0.25) ≈ 3.665` gerekir. Eski
değer (`WU_TILT_HOVER=8.0`) bu eşiğin üstündeydi. `WU_TILT_HOVER` 8.0'dan
**3.0**'a düşürüldü (`TiltrotorIndiParams.hpp`, `gain_schedule.m`,
`sf_wls_alloc.m`, `sitl/run_transition_test.py` tutarlı biçimde
güncellendi; `tiltrotor_indi.slx` yeniden build edildi).

Doğrulama:
- Saf MATLAB (`run_hover_gust_test.m`, `run_transition_test.m`):
  regresyon yok, LESO açık/kapalı RMS ilişkisi beklenen yönde
  (açık daha düşük RMS), transition testi kararlı (`max|omega|=0.0176
  rad/s`).
- SITL repro (§4'teki komut dizisi, arm + 6m tırmanma): **kilitlenme
  hâlâ oluyor**, ama artık T0 yerine **T1** sıfıra kilitleniyor
  (`u_actual=[34.18, 0.00000, 24.28, 0.255, 0.079, 0.00000]`,
  `sat_flag=[F,T,F,T,F,T]`). 5 saniye boyunca T1 kilitli kalırken T0
  tavana doğru tırmandı (27.9N→38.7N), `d_hat(0)` sürekli büyüdü
  (2.52→3.21), `nu_des(4)` (Fz talebi) 9.47N'dan 11.9N'a çıktı — §4'te
  tarif edilen kendi kendini besleyen döngüyle **aynı imza**, sadece
  hangi rotorun kilitlendiği değişti. ~10s'de sistem durum değiştirdi:
  T1 kısmen toparlandı (11.16N) ama `d_hat` işaret değiştirdi
  (-1.26/-0.46), `nu_des(3)` (Fx) -34.66'ya sıçradı, `δ2` 71°'ye çıktı —
  sönümsüz salınıma girdiğine işaret; test bu noktada disarm edilip
  sonlandırıldı (flip riski, bkz. §4 uyarısı).

**Sonuç:** Wu_tilt yeniden dengeleme TEK BAŞINA kök nedeni çözmüyor —
WLS'in *hangi* aktüatörü tükettiğini değiştiriyor ama nu_des'in kalıcı/
büyüyen tek-işaretli kalma eğilimini (asıl motor) değiştirmiyor. Bu,
§4'ün 2026-07-25 analizindeki (c) şıkkını doğruluyor: sorun ağırlık
oranından önce, **`nu_des`'in neden kalıcı büyüdüğünde** yatıyor (LESO
`d_hat` mi kendi telafisini göremeyip drift ediyor, yoksa `hoverTrim()`
gerçek SITL dinamiğiyle mi uyuşmuyor). Sonraki adım (c)'nin izolasyonu
olmalı; (b) faydasız değil (tilt kullanımını gerçekten artırdı, bkz.
`δ0/δ1` değerleri) ama tek başına yeterli değil — (c) çözülünce (b)'nin
3.0 değeri korunabilir ya da yeniden ayarlanabilir.

**Nasıl yeniden üretilir / kontrol edilir:**

```bash
export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH
px4-commander arm -f
z0=$(px4-listener vehicle_local_position | grep -m1 '^\s*z:' | awk '{print $2}')
z_sp=$(python3 -c "print($z0 - 6.0)")
px4-mc_indi_tiltrotor test_sp 0 0 0 0 $z_sp 1 1 0
sleep 5
px4-listener tiltrotor_indi_status | grep -E "u_actual|sat_flag"   # T0 veya T1 ~0 ise sorun tekrar üretildi
px4-commander disarm -f
```

> ⚠️ Bu sorun düzeltilmeden GUI ile canlı gözlem yaparken aracı **arm edip
> uzun süre gözetimsiz bırakmayın** — CPU yükü arttıkça tam ters dönme riski
> var (gözlendi, bkz. yukarıdaki belirti).

**Aday çözüm 3 — `ROTOR_KM` mekanik düzeltmesi (0.05→0.06), sonuç
(2026-07-26, ÇÖZMEDİ, farklı ve DAHA CİDDİ bir arıza modu ortaya çıkardı):**

Gerçek mekanik/model uyuşmazlığı bulundu ve doğrulandı: kontrolcünün
kullandığı `ROTOR_KM=[0.05,-0.05,0.05]` (`TiltrotorIndiParams.hpp`,
`CA_ROTORi_KM` airframe parametresinden miras), gerçek Gazebo fizik
motorunun (`Tools/simulation/gz/models/tiltrotor_tailplane/model.sdf`,
`gz-sim-multicopter-motor-model-system` plugin) kullandığı
`<momentConstant>0.06</momentConstant>` değeriyle **%20 farklı**. gz-sim
kaynağı (`MulticopterMotorModel.cc`) doğrulandı:
`dragTorque_z = -turningDirection*thrust*momentConstant` — yani
`momentConstant` tam olarak `km` (Nm/N) ile aynı birim/tanım, doğrudan
karşılaştırılabilir. İşaret deseni (rotor0=ccw→−, rotor1=cw→+, rotor2=ccw→−)
mevcut `km=[+,−,+]` işaretleriyle tutarlı — yalnızca büyüklük yanlış.
Bu, `hoverTrim()`'in analitik yaw-nulling düzeltmesinin (`d1_trim`) gerçek
reaksiyon-tork dengesizliğini ~%17 eksik telafi etmesi anlamına geliyor —
kalıcı, gerçek bir uncorrected yaw torku bırakıyor.

`ROTOR_KM` 0.06'ya güncellendi (`TiltrotorIndiParams.hpp`,
`tiltrotor_params.m`, `sf_wls_alloc.m` tutarlı biçimde; saf MATLAB'da
plant+kontrolcü zaten aynı değeri paylaştığından bu yalnızca PX4-gerçek
fizik arasındaki gerçek uyuşmazlığı kapatıyor). MATLAB testleri regresyon
göstermedi.

SITL doğrulaması (aynı repro, ama bu sefer daha UZUN izlendi, ~23s):
- **t+5..13s arası GERÇEK bir iyileşme görüldü:** `d_hat(0)` başta büyüdü
  (-1.34→-2.05) ama sonra kendiliğinden 0'a yakınsadı (-2.05→-0.50),
  T0 kilitli kalsa da (`sat_flag[0]=True` boyunca) T1/T2 düzgünce telafi
  etti, δ2 yumuşak biçimde arttı. Bu, önceki aday çözümlerde hiç
  görülmeyen bir **kendiliğinden yakınsama** — km düzeltmesi gerçek bir
  katkı sağladı.
- **Ama t+14s'ten sonra sistem farklı, daha ciddi bir modda BOZULDU:**
  T0 kısa süreliğine toparlandı (~38N), ama T1 bu sefer çökmeye başladı;
  t+20-23s'de **hem T0 hem T1 sıfıra yakın** (`0.33N`, `0.00N`), kuyruk
  rotoru (T2) **tavana kilitlendi** (45N, `sat_flag[2]=True`) VE δ2
  sürekli arttı (0.44rad→1.47rad, ~25°→84° — neredeyse tam cruise tilt,
  hover'da!), `d_hat` her iki eksende patladı (`[4.24, 8.48]`). **Yaw
  -161.9°'ye savruldu** (att_sp psi=0 iken), `vz=-11.01 m/s` (dikey hız
  hedefin çok üstünde, kontrolsüz iniş). Test bu noktada disarm edilip
  durduruldu (flip/çarpışma sınırındaydı).

**Sonuç:** km düzeltmesi TEK BAŞINA da kök nedeni çözmüyor, ama önceki
denemelerden **farklı bir sinyal** verdi: erken evrede gerçek bir
kendiliğinden-yakınsama gözlendi (bu üç denemeden hiçbirinde daha önce
olmamıştı), sonra ayrı bir mekanizma devreye girip **yaw ekseninde
sınırsız bir savrulmaya** dönüştü. Bu, kuyruk rotorünün (T2) tavana
kilitlenip cruise-tilt'e yaklaşmasıyla zamansal olarak örtüşüyor — kuyruk
rotoru Fz katkısını (cos(δ2) küçülürken) kaybediyor, bu da kanat
rotorlerine (T0/T1) daha fazla Fz yükü biniyor, ki onlar da yaw/roll
düzeltmesi için tüketiliyor olabilir — kendini besleyen çok-eksenli bir
döngü ihtimali var. **Sonraki adım artık netleşti: yaw ekseni
(`nu_des(2)`) izolasyonu** — LESO yaw ekseninde etkin değil
(`leso_enable=[1,1,0]`, `d_hat(2)` hep 0 kalıyor durum çıktısında), yani
yaw'daki bu savrulma LESO drift'i DEĞİL; ya gerçek/kalıcı bir fiziksel
tork dengesizliği (km düzeltmesine rağmen artakalan bir hata, ör.
`effectiveness_matrix`'teki başka bir geometri/işaret hatası) ya da düşük
`Ws_yaw=3` önceliğinin, kuyruk-tilt'in kaçışını (runaway) durduramaması.
`δ2`'nin neden sürekli arttığını (hangi `nu_des` bileşeni onu itiyor —
yaw mı, Fx mi, Fz mi) tick-bazında izlemek gerekiyor.

**Nasıl yeniden üretilir:** §4'teki aynı komut dizisi, ama `sleep 5`
yerine en az 20-25 saniye izlenmeli — sorun yalnızca uzun vadede ortaya
çıkıyor, kısa (5s) testler yanıltıcı biçimde "düzeldi" görünümü verebilir.

**T2dbg tanı logu eklendi (`TiltrotorIndiControl.hpp`, `wlsAllocate()`):**
kuyruk rotorü (T2/δ2) etkinlik sütunları, ağırlıkları ve `nu_des(2)` (yaw)
500ms'de bir `PX4_INFO` ile loglanıyor (§4'teki T0dbg logunun yanında).
25s'lik bir koşuda toplanan veri "Aday çözüm 4"ü doğurdu.

---

**Aday çözüm 4 — `ROTOR_PY` işaret düzeltmesi denemesi (2026-07-26,
GERİ ALINDI — MATLAB'da doğrulanmış regresyona neden oldu):**

**T2dbg verisiyle bulgu:** 25 saniyelik SITL koşusunda `nu_des(2)` (yaw)
**arm anından itibaren hemen -1.19** ve test boyunca neredeyse hiç
azalmadan (~-0.9 ile -1.2 arası) kaldı — bu, zamanla büyüyen bir "drift"
değil, `hoverTrim()`'in hiç düzeltemediği **kalıcı bir başlangıç hatası**
işareti. LESO yaw ekseninde kapalı olduğundan (`d_hat(2)` hep 0), bu drift
LESO kaynaklı olamaz.

**Gerçek bir mekanik uyuşmazlık bulundu:** `model.sdf`, `motor_0`'ı acikca
`<!-- Right wing rotor -->` yorumuyla **Y=-0.25**'te, `motor_1`'i
`<!-- Left wing rotor -->` yorumuyla **Y=+0.25**'te tanımlıyor —
`TiltrotorIndiParams.hpp`/`tiltrotor_params.m`'deki varsayımın
(`ROTOR_PY=[+0.25,-0.25,0]`, rotor0=sağ) **tam tersi**. Hover'da
`dtau_dT`'nin roll bileşeni `-ry` olduğundan (`cross(r,dir)` türetmesi),
bu ters işaret WLS/INDI'nin roll↔thrust etkinlik modelini gerçek fiziğin
işaretçe tersi yapıyor olabilir — tilt farkının roll etkinliği yalnızca
`km`'ye bağlı olduğundan (ry'den bağımsız) etkilenmez, ki bu da aday
çözüm 2'nin (Wu_tilt düşürme, tilt'i tercih ettirme) neden erken evrede
gerçek bir iyileşme sağladığını tutarlı biçimde açıklıyordu.

**Uygulanan düzeltme (DENENDİ):** `ROTOR_PY` `[-0.25,+0.25,0]`'e çevrildi
(`TiltrotorIndiParams.hpp`, `tiltrotor_params.m`, `sf_wls_alloc.m`'de
tutarlı biçimde; `km` değiştirilmedi). `turningDirection→numeric` eşlemesi
ayrıca doğrulandı (gz-sim kaynağı: `ccw=+1, cw=-1`) — bu, aday çözüm 3'teki
işaret-eşleşme varsayımını doğruladı, o taraftan bir hata yoktu.

**Sonuç: CİDDİ REGRESYON, GERİ ALINDI.** Saf MATLAB referans testinde
(`run_hover_gust_test`, plant+kontrolcü bu değişiklikten SONRA da birbiriyle
tutarlıydı) RMS p/q **~0.0065/0.0015'ten 0.34/1.29 rad/s'e** fırladı
(~50-800×). Bu, plant ile kontrolcünün aynı (yeni) modeli paylaştığı,
tamamen kontrollü bir ortamda gerçekleşti — yani basit işaret değişimi
YANLIŞ (ya da eksik) bir düzeltme. Değişiklik üç dosyada da geri alındı;
PX4 tarafına hiç build/deploy edilmedi (regresyon MATLAB'da yakalandığı
için SITL'e hiç gitmedi).

**Açık soru:** SDF'deki "Right wing rotor @ Y=-0.25" gözlemi gerçek ve
açıklanmamış durumda duruyor — ama düzeltmenin nasıl olması gerektiği
belirsiz. Olası yönler (henüz denenmedi): (a) gz-sim'in tam itki
formülünü (yalnızca özet/paraphrase değil, ham kaynağı) satır satır
incelemek — itki hesabına da `turningDirection` çarpanının girdiği
görüldü (`thrust = turningDirection·sign(ω)·ω²·motorConstant`), bu
standart bir pervane modeli için beklenmedik; bu terimin gerçek etkisini
yanlış yorumlamış olabilirim. (b) Y işaretini km işaretiyle BİRLİKTE
(ikisini de veya farklı bir kombinasyonu) değiştirmeyi denemek. (c) SDF'nin
poz referans çerçevesini (model-frame mi, parent-link-relative mi)
`gz model` ile ampirik olarak doğrulamak, yorum satırlarına güvenmek
yerine. **Bu konuya tekrar girmeden önce ayrı, odaklı bir oturumda ele
alınmalı — tahminle deneme yapmak MATLAB referansını bile bozabiliyor.**

**Adım 5-6 (2026-07-26, aynı oturum devamı) özet:** gz-sim ham kaynağı
(`MulticopterMotorModel.cc`) satır satır incelendi — km işaret-eşleşme
analizi doğrulandı, ama pose/frame zinciri statik dosya okumasıyla
çözülemedi; sistemin ilk ~15s kararlı davranması "roll işareti tam ters"
hipoteziyle çelişiyor, bu hat terk edildi. Ardından `leso_enable_yaw=1`
ile (rebuild gerekmeden, yalnızca `test_sp` argümanı) test edildi:
roll/pitch TÜM test boyunca küçük kaldı (önceki 4 denemenin hiçbirinde
görülmemiş bir kararlılık) ama yaw kendisi sıfıra yakınsamadı — ~17s
sabit kaldıktan sonra ani bir geçişle FARKLI bir kalıcı değere (+1.49)
yerleşti. Değişiklik kalıcı yapılmadı (yaw sorunu çözülmediği için).
Tam ayrıntı ve tam `nu_des` zaman serisi → `WLS_LOCKUP_INVESTIGATION_REPORT.md`
Adım 5-6.

**Adım 7-8 özet:** `Ws_yaw` 3→6 denendi (LESO açıkken) — yaw'ın tepki
HIZINI hiç değiştirmedi, Fx talebini -28N'e patlattı, geri alındı (WLS
ağırlıklandırma yolu artık tükendi). Ardından "Gazebo'nun kendisi mi
sorunlu?" sorusu incelendi: kontrolcünün `_u_actual` gölge aktüatör
modelinin Gazebo'dan HİÇ geri besleme okumadığı, tilt servolarının ise
gerçekte basit bir gecikme değil tork-sınırlı (`cmd_max=2`) bir
P=100/D=0 PID + sürtünme ile çalıştığı bulundu. Canlı `gz model -p`
sorgusuyla gölge/gerçek karşılaştırıldı: kuyruk tilt'inde (δ2) 2.6-3.7°
işaret-değiştiren bir sapma ölçüldü (kanatlarda <0.5°) — gerçek ama
muhtemelen tek başına yeterli olmayan bir katkı payı. Ayrıntı →
`WLS_LOCKUP_INVESTIGATION_REPORT.md` Adım 7-8.

**Adım 9-10 özet:** `TILT_RATE_MAX` 3.0→2.0 rad/s düşürüldü (Adım 8'in
devamı, PX4'e özgü, kalıcı yapıldı) — **İLK KEZ net bir iyileşme**:
roll/pitch iki 25s'lik koşuda da sınırlı/küçük kaldı (biri 0.1°/0.0°),
`vz` kontrolsüzlüğü (-11 m/s felaketi) tekrarlanmadı (`vz≈-0.13 m/s`),
yaw sınırsız savrulmak yerine sabit bir platoya oturdu — ama bir kanat
rotorü hâlâ periyodik olarak 0'a kilitleniyor, bu sefer sonucu flip
değil "yetersiz itki, tırmanış duruyor". Ardından yaw `Kp_att[2]`/
`Kp_rate[2]` 1.5/2.0'dan 2.5/3.5'e çıkarıldı (Adım 10) — MATLAB'da
`run_hover_gust_test` RMS'i ~10-100× kötüleştirdi (roll↔yaw limit cycle,
tam da eski kod yorumunun uyardığı gibi), PX4'e hiç gitmeden geri
alındı. **Yaw'a daha fazla otorite vermeye dayalı üç ayrı yaklaşım
(LESO açma, Ws_yaw, Kp) artık tükendi** — tek başarılı yön aktüatör-
dinamiği gerçekçiliği (Adım 8/9). Ayrıntı →
`WLS_LOCKUP_INVESTIGATION_REPORT.md` Adım 9-10.

Tam rapor için → `WLS_LOCKUP_INVESTIGATION_REPORT.md`.

---

## 5. Bilinen tuzaklar

(Land-detector/`COM_DISARM_PRFLT` sorunu için → §0. Zamanlama titreşimi
sorunu için → §4. Kalanlar README.md'den taşındı.)

- **`COM_DISARM_PRFLT` (10 s):** arm edilip gerçek itki komutlanmadan 10 saniye
  geçerse PX4 kendini disarm eder. §0'daki düzeltme sonrası bu yalnızca test
  script'i gerçekten setpoint göndermeyi bırakırsa (örn. çökme/exception)
  tetiklenmeli — sürekli/deterministik biçimde tetikleniyorsa önce §0'ı kontrol
  edin.
- **`estimator_status_flags` ~1 Hz'de yayınlanır:** modüldeki EKF sağlık
  kapısı (`cs_tilt_align`/`cs_yaw_align`, yapışkan bayraklar) 3 saniyelik bir
  tazelik penceresiyle kontrol edilir.
- **`fs_bad_*` bitleri sağlıklı uçuşta bile titreşebilir** — sert per-tick
  kapı yalnızca `cs_tilt_align`/`cs_yaw_align` kullanır, `fs_bad_*` yalnızca
  M7 uçuş-sonrası raporunda değerlendirilir.
- **`px4-listener <topic> -n N` (N>1) disarmed'ken takılabilir** —
  argümansız (`-n 1` veya hiç) çağırın.
- **Setpoint yayınladıktan sonra GERÇEKTEN ulaştığını doğrulayın**
  (2026-07-27'de öğrenildi, Adım 11e): `test_sp` "published test
  setpoint" yazsa BİLE setpoint kontrolcüye ulaşmamış olabilir.
  `px4-listener tiltrotor_indi_setpoint` çalıştırın — çıktı
  **"never published"** ise setpoint uygulanmıyor demektir ve
  kontrolcü sessizce `z_sp = lpos.z` (mevcut irtifayı koru) +
  `roll/pitch/yaw_sp = 0` yedeğine düşer, yani **yanlış senaryoyu**
  test etmiş olursunuz. Bu tuzak, bu araştırmadaki 10 adımlık tüm
  koşum geçmişini geçersiz kıldı.
- **`gz topic -e` sessizce boş döner** — gz-transport docker0'a bağlanır, CLI
  ulaşamaz; doğrulamayı `gz model`/`gz service` gibi servis çağrılarıyla yapın
  (bkz. `~/PX4-Autopilot/docs/gz_tiltrotor/RUNBOOK.md` §4).
- **`param set` KALICI: bir sonraki koşuya sızar** (2026-07-30, Adım 36).
  SITL param'ları `build/px4_sitl_default/rootfs/parameters.bson`'a yazılıyor,
  yani bir failsafe/enjeksiyon betiği `EKF2_*` değiştirip geri almazsa **bir
  sonraki SITL hiç açılmaz** — belirti: `Preflight Fail: Yaw estimate error` ve
  `launch_sitl` 3 denemede de "Ready for takeoff" göremez. Kurtarma:
  ```bash
  rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters*.bson
  ```
  Kalıcı çözüm: param değiştiren her betik `finally` bloğunda geri alsın **ve
  `param save` + 2 s beklesin** — otosave GECİKMELİ, yani geri alıp hemen
  `kill_sitl()` çağırmak diske hiçbir şey yazmaz (`probe_no_alt.py` örnek).
  Bu bir kez gözden kaçtı ve arkasından koşan `sitl-lockup-check` bozuk
  param'larla koşup **sahte bir GEÇTİ** verdi.
- **`parameters*.bson` silindikten sonraki İLK koşu ölçüm sayılmaz** (Adım 36).
  Kalibrasyon/manyetik sapma sıfırdan öğrenildiği için EKF geç yakınsıyor, araç
  `POS_ENGAGE_V_MAX`'i aşıyor, `pos_hold` REDDEDİYOR ve "hover" testi bir seyir
  testine dönüşüyor (ölçüldü: baseline `v_h` 5.8 m/s, itki doygunluğu %16,
  BIG_M 949 — sonraki boot'ta hepsi sıfır). Bir kez boot edip atın.
- **Pilot girişi SITL'de param DEĞİŞTİRMEDEN sürülebilir** (2026-08-03, Adım 40).
  `COM_RC_IN_MODE` varsayılanı 3 ("RC or Joystick, keep first") joystick'i zaten
  kabul ediyor, yani MAVLink `MANUAL_CONTROL` göndermek yeterli — ve param'a
  dokunmamak SITL'in kalıcı-param tuzağını tamamen atlatır. pymavlink ile
  `udpin:127.0.0.1:14540`'a bağlanın, 50 Hz `manual_control_send` + 1 Hz GCS
  heartbeat gönderin. **`flag_control_manual_enabled` akış başlayana kadar
  `False`'tur** (commander verir, biz değil) — pilot dalı ondan önce
  çalışmaz, ve modül DISARMED iken o blok hiç yürütülmez, yani "pilot input:
  none" görmek tek başına bir arıza belirtisi değildir.
  `manual_control_switches` (VTOL anahtarı) bu yoldan GELMEZ; onun için
  `RC_CHANNELS_OVERRIDE` gerekir.
- **`matlab -batch ... | tail` KİLİTLENİR** (2026-08-03, Adım 39). MATLAB
  arkada bir yardımcı süreç bırakıyor ve o süreç pipe'ın yazma ucunu açık
  tutuyor, yani `tail` hiçbir zaman EOF görmüyor: MATLAB işini bitirip çıksa
  bile komut asılı kalır (bir kez 9 dakika beklendi). Çıktıyı **dosyaya
  yönlendirin**, sonra dosyayı okuyun:
  ```bash
  matlab -batch "run_backtrans_sm_test" > /tmp/out.txt 2>&1
  ```
- **`px4-*` istemcileri PATH'te olmalı** — test betikleri `px4-commander` /
  `px4-listener`'ı çıplak adla çağırıyor, yoksa `FileNotFoundError` ile SITL
  başlar başlamaz düşerler:
  ```bash
  export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH
  ```
- **EKF2 `z_valid`, `z`'nin DOĞRU olduğu anlamına gelmez** (2026-07-30, Adım 36).
  Yükseklik yardımı kesilip yatay yardım devam ederse `z` donuyor ve
  `z_valid` TRUE kalıyor — ölçüldü: araç 22 s'de gerçekten 10 m alçalırken EKF
  `z` = −20.04'te sabit. Bir irtifa ölçümünü doğrulamak için **gz yer gerçeğini**
  kullanın (`Px4Client.gz_truth_z()`), ulog'un `z`'sini değil.

---

## 6. Kapatma

```bash
pkill -9 -f 'px4_sitl_default/bin/px4'; pkill -9 -f 'gz sim'; rm -f /tmp/px4-sock-0
```

M5/M6 script'leri bunu `finally` bloğunda kendileri çağırır; script bir
istisna ile yarıda keserse elle çalıştırın.
