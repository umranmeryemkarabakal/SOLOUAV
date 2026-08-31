# SITL test tooling — mc_indi_tiltrotor

> **Çalıştırma adımları (adım adım komutlar + beklenen çıktı)** → [RUNBOOK.md](RUNBOOK.md)
>
> **Güncel kontrol durumu ve açık sorunlar** →
> [WLS_LOCKUP_INVESTIGATION_REPORT.md](WLS_LOCKUP_INVESTIGATION_REPORT.md)
> dosyasının başındaki **"► BURADAN BAŞLAYIN"** bloğu (2026-07-28, Adım 17).
> Kısaca: aktüatör kilitlenmesi ✅ çözüldü, yaw'ın sınırsız dönmesi ✅
> çözüldü, **ama yaw ekseni gerçek (düşük hızlı) hover'da hâlâ sönümsüz**
> — donanım için 🔴 NO-GO.
>
> **Adım 17 (2026-07-28) teşhisi değiştirdi:** saf MATLAB'da aynı yaw adımı
> **salınımsız oturuyor**, üstelik MATLAB plant'i hiçbir hızda aerodinamik
> yaw momenti üretmediği hâlde (`M_aero(3) ≡ 0`). Yani sorun "düşük hızda
> aero sönümlemenin kaybolması" değil — **SITL'e özgü** bir kararsızlaştırıcı
> var. Yeni MATLAB referans testi: `../run_yaw_step_test.m`.
>
> **Adım 18 (2026-07-28) ölçtü ve kökü daralttı:**
> - Gölge `_u_actual` ile gerçek Gazebo eklem açısı arasındaki sapma
>   **p99'da ≤ 0.55°** — aynı pencerede yaw 73.7° bantta salınırken. Yani
>   gölge model **baskın neden DEĞİL** (Adım 17'nin şüphesi elendi).
> - **(Q) yeniden üretildi ve yeniden çerçevelendi:** salınım `yaw_sp = 0`
>   iken oluyor → bir *adım yanıtı* kusuru değil, **denge kararsızlığı**.
> - **En güçlü korelat: δ1'in `TILT_MIN=0` sınırından kalkıp geri çakılması**
>   → **(P) ve (Q) muhtemelen aynı kök: tek yönlü tilt aralığı.**
>
> **Adım 19 (2026-07-28) hız hipotezini de çürüttü:** `yaw_sp = 0` sabit
> tutulup yalnızca ileri hız değiştirilen tek değişkenli A/B'de salınım,
> hız **2.00-2.09 m/s'de sabitken** söndü → **ileri hız salınımı bitiren
> şey değil.** Ayrıca salınım **sönümsüz değil, ~30-35 s'de oturan çok
> zayıf sönümlü bir mod** (iki uçuşta tekrarlandı). Kalan tek nicel boşluk:
> MATLAB aynı adımı 3.1-3.7 s'de oturtuyor → **2-10× sönümleme farkı**,
> sebebi SITL'e özgü ve açıklanmadı. Çıktı: `yaw_airspeed_ab.png`.
>
> **Adım 21 (2026-07-28) — GERÇEK KOD HATASI:**
> `MulticopterIndiTiltrotor.cpp:315-316, 324-325` WLS'in slew kutusunu sabit
> `TS_CTRL = 1/400` ile boyutluyor, ama modül **250 Hz'de** dönüyor (aynı
> fonksiyon `dt`'yi 171. satırda doğru hesaplıyor). Ölçüldü: `|ddelta|` p99 tam
> **0.00500 rad**, tick 4.00 ms, tilt `sat_flag` **%99.4-99.9**, tahsisat yaw
> verimi **%20.6** → **efektif tilt slew tavanı 1.25 rad/s (hedeflenen 2.0'ın
> %62'si)**. Kanat tilt'i yaw'ın tek gerçek aktüatörü.
> ⚠️ **Naif düzeltme zararlı olabilir** — nominal 2.0'ı 2.0 efektif yapar, ki
> Adım 14'te ıraksatan efektif 1.875'in üstündedir. İtki kanalı temiz çıktı
> (sapma ~%0.1).
>
> **Adım 22 (2026-07-28) — KALICI DEĞİŞİKLİK, davranış-nötr, doğrulandı:**
> tahsisat kutusu ile fiziksel servo limiti ayrıştırıldı.
> `TILT_RATE_MAX = 2.0` artık yalnızca gölge modelin fiziksel limiti;
> tahsisat kutusu **`TILT_SLEW_BOX_RATE = 1.25 rad/s` × `TS_BOX = 1/250`**
> (tilt kutusu 1 ULP farkla aynı). SITL doğrulaması: `|ddelta|` p99 hâlâ tam
> 0.00500 rad, itki doyumu %0.0, kilitlenme yok, `|vz|` ≤ 0.78 m/s, irtifa
> hata RMS 0.234 m; yaw hâlâ (Q) nedeniyle kalıyor (37.09°).
> *Not: Adım 21'in "kod hatası" nitelemesi fazla sertti — sabit periyot
> jitter'a karşı kasıtlıydı; eksik olan periyodun gerçek döngü hızıyla
> eşleşmesiydi.*
>
> **⭐ Adım 23 (2026-07-28) — (Q)'NUN MEKANİZMASI BULUNDU.** `slewbox <rad/s>`
> test kancasıyla kutu **uçuş içinde** tarandı (iki koşu, ters sıra, her
> değerde aynı +30° yaw adımı). Adımın son 5 s yaw hızı RMS'i:
> **1.25 → 0.583/0.466 (salınıyor)**, 1.50 → 0.391/0.005 (marjinal),
> **1.75 → 0.0037/0.0051 (sakin)**, **2.00 → 0.0056/0.0055 (sakin)**.
> 1.25 hem 0.86 hem 2.20 m/s'de salınıyor; 1.75/2.00 ise 0.81-3.14 m/s'nin
> tamamında sakin → **hız değil, kutu hızı ayırt edici.** Yani düşük hızdaki
> yaw salınımı **tahsisatın tilt slew'undan aç bırakılmasıymış.**
> Varsayılan **1.25 → 1.75** yapıldı. Doğrulama: kilitlenme ✅, dikey hız ✅
> (0.816 m/s), roll/pitch ✅; yaw hâlâ ❌ (tepe 35.80° vs 37.09°) — kalan
> aşım arm geçicisi, kalıcı salınım yok oldu (RMS ~0.5 → ~0.005).
> Kullanım: `px4-mc_indi_tiltrotor slewbox 1.75`. Aktif değer log'dan
> `|du(3)| p99.5 / TS_BOX` ile geri okunur.
>
> **Adım 24 (2026-07-28) — "gölgeyi gerçek servoya sadık kıl" DENENDİ, GERİ
> ALINDI.** Sadakat açığının tamamı Coulomb sürtünme ölü bandıymış
> (friction/p_gain = **0.573°**); çevrimdışı doğrulamada 1. derece + ölü bant
> **3.5×/8.0×/139×** daha iyi, tam 2. derece model ise DAHA KÖTÜ (atalet
> birkaç ms'de oturuyor, 4 ms tick'in içinde). **Ama kapalı çevrimde
> kilitleniyor:** `du` slew kutusuyla 0.40° ile sınırlı, ölü banttan küçük →
> gölge ve komut birlikte donuyor (SITL: tilt'ler tüm uçuş donuk, yaw bandı
> 238°). Geri alındı. **Ders: açık çevrim replay (`servo_model.py`) kapalı
> çevrim geri besleme tuzağını gösteremez** — komut yolunun içindeki model
> değişikliklerini kapalı çevrimde doğrulayın.
>
> Ölçüm araçları (rebuild gerektirmez): `shadow_vs_real.py`,
> `gz_joint_csv.sh` (Adım 21'den beri rotor hızlarını da yazar),
> `logger_topics_shadow.txt`. Kullanım → `shadow_vs_real.py` docstring'i.
>
> **GUI'li gözlem** için `gz_follow.sh` (kamerayı araca kilitler) —
> zorunlu, çünkü yatay pozisyon döngüsü yok ve araç ~10 m/s sürükleniyor.

MATLAB kontrolcüsünün (`indi_attitude_controller.m` + LESO + WLS +
`altitude_loop.m`) PX4 C++ portu `~/PX4-Autopilot/src/modules/mc_indi_tiltrotor/`
altında; yeni `gz_tiltrotor_indi` airframe'i Model-3'ün (`tiltrotor_tailplane`)
Gazebo modelini yeniden kullanır. Tasarım kararları ve kapsam sınırları için
bkz. `~/.claude/plans/cached-dancing-puppy.md`.

## Başlatma

```bash
pkill -9 -f 'px4_sitl_default/bin/px4'; pkill -9 -f 'gz sim'; rm -f /tmp/px4-sock-0
cd ~/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridge
PX4_SIM_MODEL=gz_tiltrotor_indi HEADLESS=1 ../../../../bin/px4 -d > /tmp/px4.log 2>&1 &
grep -a 'Ready for takeoff' /tmp/px4.log   # bekleyin
```

Kaynak değiştiyse önce derleyin: `cd ~/PX4-Autopilot && make px4_sitl_default`.

## Test sürücüsü

```bash
export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH
cd "tiltrotor_Matlab files/sitl"
python3 smoke_test.py     # M4 doğrulaması: arm + hover setpoint + EKF sağlık takibi
```

`indi_sitl_common.py` — `Px4Client` sınıfı `px4-mc_indi_tiltrotor test_sp`,
`px4-commander`, `px4-listener` client binary'lerini shell'leyerek sürer
(özel `tiltrotor_indi_setpoint` uORB topic'i için MAVLink köprüsü yok —
harici MAVLink istemcileri rastgele uORB topic'i yayınlayamaz; bu yüzden
aynı işlemin client'ları kullanılıyor, `px4-listener`'ın çalışma şekliyle
aynı).

## Bilinen davranış / tuzaklar

- **`COM_DISARM_PRFLT` (10 s):** arm edilip gerçek itki komutlanmadan
  10 saniye geçerse PX4 kendini disarm eder ("auto preflight disarming").
  Test script'leri arm sonrası setpoint'i hemen göndermeli. **2026-07-24'e
  kadar bu, her M5/M6 koşusunda deterministik biçimde tetikleniyordu** —
  gerçek neden setpoint zamanlaması değil, `mc_indi_tiltrotor`'ın
  `vehicle_thrust_setpoint`'i hiç yayınlamamasıydı: PX4'ün stok
  `MulticopterLandDetector`'ı "havada mı" kararını bu topic'ten veriyor,
  topic gelmeyince `landed=true` sabitleniyor ve gerçek Gazebo fiziğinden
  bağımsız olarak COM_DISARM_PRFLT tetikleniyordu. Düzeltildi (modül artık
  komut edilen ortalama itkiyi bu topic'e de yazıyor) ve doğrulandı — bkz.
  [RUNBOOK.md §0](RUNBOOK.md#0-ön-koşul-land-detector-düzeltmesi-derlenmiş-olmalı).
- **`estimator_status_flags` ~1 Hz'de yayınlanır** (sürekli akış değil, durum
  değişince/periyodik). Modüldeki EKF sağlık kapısı bunu 3 saniyelik bir
  tazelik penceresiyle kontrol eder — daha sıkı bir pencere (ilk denemede
  500 ms) `cs_tilt_align`/`cs_yaw_align` gerçekte sabit `True` iken kontrolcü
  çıkışının her iki adımda bir NaN/gerçek değer arasında "titremesine" yol
  açtı (bkz. modül yorumları).
- **`fs_bad_*` (füzyon hata) bitleri sağlıklı uçuşta bile anlık titreşebilir**
  — modülün sert (per-tick) EKF kapısı yalnızca `cs_tilt_align`/`cs_yaw_align`
  (yapışkan/kalıcı bayraklar) kullanır; `fs_bad_*` yalnızca uçuş-sonrası M7
  raporunda değerlendirilir.
- **`px4-listener <topic> -n N` (N>1) disarmed'ken takılabilir** — argümansız
  (`-n 1` veya hiç) çağırın.

## Bilinen açık sorun (M6, çözülmedi)

`run_transition_test.py` şu an WLS tahsisinde gerçek bir hata ortaya
çıkarıyor: `Fx_sp` 0→10N rampalanırken ileri hız pratikte sıfırda kalıyor
ve rotor tilt/itki dağılımı beklenen simetrik-yakın-sıfır-tilt hover
dengesine değil, asimetrik bir konfigürasyona yakınsıyor (sağ kanat rotoru
T0≈0N'a düşüyor, sol kanat tek başına ağırlığın çoğunu taşıyor, **kuyruk
rotoru ~33° tilt taşıyor** — halbuki `CA_SV_TL2_CT=0` mantığı gereği kuyruk
tilt'inin yaw'a katkısı olmaması bekleniyordu ve bu büyüklükte bir tilt hiç
beklenmiyor). Bu durum 14s'lik settle sırasında oluşuyor ve Fx rampası
boyunca değişmeden kalıyor.

**Zaten yapılan/doğrulanmış olan:** Gölge aktüatör durumunun arm anında
naif (eşit itki, sıfır tilt) yerine `hoverTrim()` (hover_trim.m portu) ile
tohumlanması, arm sonrası ~90° yaw sapmasının süresiz büyüyüp çökmeye yol
açmasını düzeltti — sistem artık ~10s'de <2° yaw hatasına yakınsıyor.
Bu düzeltme kalıcı (`TiltrotorIndiControl.hpp:hoverTrim()`,
`MulticopterIndiTiltrotor.cpp` seed bloğu).

**Sonraki adım (yapılmadı):** WLS iterasyonlarını (`wlsAllocate()` içindeki
6 big-M iterasyonu) adım adım loglayıp hangi kutu-kısıtın T0'ı sıfıra
ittiğini ve kuyruk tilt'inin neden büyüdüğünü izole etmek; MATLAB'ın kendi
`hover_trim(p)` çıktısıyla `effectivenessMatrix()`'in sayısal çıktısını
karşılaştırmak (G/nu0 matrisinde bir işaret/indeks hatası olabilir).
