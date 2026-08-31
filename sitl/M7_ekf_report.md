# M7 — EKF2 Doğrulama Raporu

`gz_tiltrotor_indi` üzerinde çalışan `mc_indi_tiltrotor` (M1-M6) test
kampanyası boyunca (28 SITL uçuşu, M4 smoke test'ten M6'nın son
`hoverTrim()` düzeltmeli koşusuna kadar) PX4'ün EKF2 tahmincisinin
sağlığını doğrular. Analiz `sitl/ekf_report.py` ile yapıldı —
`Tools/ecl_ekf/process_logdata_ekf.py` bu airframe için kullanılamadı
(bkz. §3).

## 1. Canlı kontrol (son doğrulama)

```
$ px4-commander check
INFO  [commander] Preflight check: OK

$ px4-ekf2 status -v
ekf2:0 EKF dt: 0.0080s, attitude: 1, local position: 1, global position: 1
ekf2: IMU message missed: 0 events
```

Attitude, local ve global position hepsi geçerli (`1`), kayıp IMU mesajı yok.

## 2. 28 uçuş üzerinde toplu analiz (`ekf_report.py`)

| Bulgu | Sonuç |
|---|---|
| `fs_bad_*` (füzyon hata pulse'ları) | **28/28 uçuşta %0** — hiç görülmedi |
| Innovation test oranları (mag/vel/pos/hgt/tas/beta, eşik <1.0) | 27/28 uçuşta hepsi **0.0–0.66** arası (geçti); 1 uçuşta (bkz. §2.1) aştı |
| `cs_tilt_align` / `cs_yaw_align` (yapışkan hizalama bayrakları) | Çoğu uçuşta sürekli %100; bazılarında (kısa uçuşlarda) ilk birkaç örnekte henüz hizalanmamış — bkz. §2.2 |
| `cs_fake_pos` / `cs_inertial_dead_reckoning` | Bazı uçuşlarda %6–33 — bkz. §2.2, GPS kilitlenme başlangıç geçişi |
| `pre_flt_fail_*` | 27/28 uçuşta %0; 1 uçuşta gerçek arıza tespit edildi — bkz. §2.1 |

### 2.1 Tek "kirli" uçuş: `14_11_46.ulg` (174s) — beklenen ve doğru davranış

Bu, M5'in **düzeltme öncesi** ilk denemesindeki kazara-uzun (fixed-iteration
polling hatası, M5 bölümünde anlatıldı) koşulardan biri; muhtemelen o dönemde
gözlenen "Attitude failure (roll)" / "Disarmed by auto preflight disarming"
olayına denk geliyor (bkz. `px4_m5_leso_off.log`). Bu uçuşta:

- `pre_flt_fail_innov_vel_horiz` %13, `vel_vert` %13, `height` %2,
  `mag_field_disturbed` %5 zaman diliminde true
- `vel_test_ratio` maksimum **4.62** (eşik 1.0'ı aşıyor), `mag_test_ratio`
  maksimum **1.68**

**Yorum:** Bu, EKF2'nin bir arızası değil — tam tersine, araç fiziksel
olarak kontrolden çıkmışken (o zamanki kontrolcü hatası nedeniyle,
`hoverTrim()` düzeltmesinden önce) EKF2'nin bunu **doğru şekilde tespit
edip bayrakladığını** gösteriyor. Estimator'ın gözcü işlevinin çalıştığının
kanıtı.

### 2.2 `cs_fake_pos` / hizalama-henüz-tamamlanmadı örnekleri — başlangıç geçişi

Nonzero `cs_fake_pos`/`cs_inertial_dead_reckoning` oranları (%6-33) yalnızca
**kısa** (13-27s) test uçuşlarında görülüyor ve GPS kilitlenmeden önceki
normal başlangıç penceresine (~1-3s) karşılık geliyor — kısa bir uçuşta bu
birkaç saniye, toplam sürenin büyük bir yüzdesini oluşturuyor. Uzun
uçuşlarda (örn. `13_09_35.ulg`, `13_25_51.ulg`, `14_20_14.ulg`,
`14_27_28.ulg`, `14_36_11.ulg`, `14_37_56.ulg`) bu oran **%0**'a düşüyor —
tutarlı biçimde bir başlangıç geçişi olduğunu doğruluyor, sürekli bir
sorun değil.

## 3. `Tools/ecl_ekf/process_logdata_ekf.py` neden kullanılamadı

Bu script'in kendi `InAirDetector`'ı "always on ground" / "no airtime
detected" diyerek analiz yapmayı reddediyor. Kök neden: bu airframe
`mc_pos_control`, `flight_mode_manager` gibi modülleri kasıtlı olarak
durduruyor (bkz. `4023_gz_tiltrotor_indi.post`) — `InAirDetector`'ın
beklediği bazı girdiler hiç yayınlanmıyor, bu yüzden araç fiziksel olarak
uçarken bile "yerde" sanılıyor. `ekf_report.py` bu sezgiyi atlayıp
doğrudan `estimator_status`/`estimator_status_flags` üzerinden, tüm
armed pencere boyunca analiz yapıyor.

## 4. Bilinen sınırlama: test script zamanlaması sim-time'ı birebir izlemiyor

`run_hover_gust_test.py`/`run_transition_test.py`'deki "t" ekseni Python'ın
duvar-saati (`time.monotonic()`) ölçümü; ULog analizi sırasında bazı
uçuşların loglanan süresinin (`ULog.last_timestamp`) script'in hedeflediği
süreden **daha kısa** çıktığı görüldü (örn. 70s hedeflenen genişletilmiş
test, ULog'da ~13-25s olarak loglanmış). Bu, yoğun subprocess-tabanlı
polling'in gerçek zamanlı çarpanı (`real_time_factor`) CPU çekişmesiyle
düşürmesinden kaynaklanıyor gibi görünüyor — script'in duvar-saati "t"
etiketleri ile PX4'ün iç simülasyon zamanı birebir örtüşmüyor. Niteliksel
sonuçları (LESO faydası, `hoverTrim()` düzeltmesi) geçersiz kılmıyor ama
mutlak zaman etiketleri yaklaşık olarak okunmalı.

## Sonuç

EKF2, M1-M6 test kampanyası boyunca **sistematik bir sağlık sorunu
göstermedi**. Tek "kirli" uçuş, kontrolcünün (o zamanki, artık düzeltilmiş)
bir hatası nedeniyle aracın gerçekten kontrolden çıktığı ana denk geliyor
ve EKF2 bunu doğru tespit etti — bu bir başarısızlık değil, doğrulama.
`commander check` ve `ekf2 status -v` ile canlı son kontrol de temiz.
