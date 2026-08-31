---
name: safe-control-change
description: Safety checklist for changing any control constant in this project (WLS weights Ws/Wu, gain-schedule Kp_att/Kp_rate, rotor geometry ROTOR_PX/PY/PZ/KM, LESO settings) that exists in both the MATLAB reference and the PX4 C++ port. Use before editing any such constant, or when asked to tune/fix/adjust control gains, weights, or airframe geometry constants.
---

# Kontrol sabiti değişikliği güvenlik protokolü

Bu proje, saf MATLAB referansı (`indi_attitude_controller.m` + ilgili
`.m` dosyaları), Simulink portu (`sf_*.m`, `tiltrotor_indi.slx`) ve PX4
C++ portu (`TiltrotorIndiParams.hpp`, `TiltrotorIndiControl.hpp`) olmak
üzere **üç paralel, birbiriyle aynı matematiği uygulaması gereken**
uygulama barındırıyor. Bir kontrol sabitini (WLS ağırlıkları, kazançlar,
rotor geometrisi, LESO ayarları) değiştirirken bu üç dosya grubu
SENKRON kalmalı.

## Neden bu skill var — iki gerçek fiyasko

Bu projede iki kez, teorik olarak makul görünen bir sabit değişikliği
doğrudan PX4/SITL'e uygulanmadan önce **MATLAB'da test edilmeden**
denenseydi ciddi zarar verebilirdi; ikisi de MATLAB kontrolüyle
yakalandı:

1. **`ROTOR_PY` işaret düzeltmesi (2026-07-26):** SDF yorumlarına
   dayanan makul bir "sağ/sol ters" düzeltmesi, saf MATLAB'da (plant +
   kontrolcü KENDİ İÇİNDE tutarlıyken) RMS hatayı **50-800× kötüleştirdi**.
   MATLAB kontrolü olmasaydı bu doğrudan SITL'e, belki de donanıma
   gidebilirdi.
2. **`Ws_yaw` artırma denemesi (2026-07-26):** MATLAB'da "regresyon yok"
   görünmesine rağmen (çünkü MATLAB bu spesifik SITL sorununu hiç
   yeniden üretmiyor), SITL'de Fx talebini -28N'e patlattı — yani
   MATLAB'ın "temiz" geçmesi HER ZAMAN yeterli değildir, ama YİNE DE
   gerekli bir ilk filtredir (gerçek bir regresyonu HER İKİ denemede de
   MATLAB önceden yakalayabilirdi).

Tam anlatım: `sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md` "Aday çözüm 4" ve
"Adım 7".

## Prosedür (SIRAYLA, atlanmadan)

1. **Değişikliği önce MATLAB referansında yapın**, ilgili tüm dosyalarda
   tutarlı biçimde:
   - `gain_schedule.m` (Kp_att, Kp_rate, wu_tilt) VEYA
     `tiltrotor_params.m` (rotor geometrisi, km, kütle/atalet) VEYA
     `indi_attitude_controller.m` (Ws ağırlıkları)
   - **AYNI değeri** `sf_wls_alloc.m`'de de güncelleyin (Simulink
     codegen-safe rewrite, `p.*`'yi DEĞİL literal sabitleri kullanır —
     kolayca unutulur, bkz. bu dosyadaki geçmiş güncellemeler).
   - Değer PX4'te de kullanılıyorsa `TiltrotorIndiParams.hpp`'de aynı anda
     güncelleyin (ama önce yalnızca MATLAB'ı test edin, PX4'ü rebuild
     etmeden — adım 3'e kadar).

2. **MATLAB regresyon testini çalıştırın:**
   ```matlab
   run_hover_gust_test
   run_transition_test
   ```
   Değişiklik öncesi RMS p/q ve transition testinin `max|omega|`/irtifa
   değişimi değerleriyle karşılaştırın (bu konuşmanın geçmişinde ya da
   `hover_gust_test.png`/`transition_test.png`'nin zaman damgalarında
   bulunabilir). **Beklenmedik büyüklükte bir sapma (>2-3×) varsa DURUN
   ve değişikliği gözden geçirin — SITL'e hiç göndermeyin.**

3. **Yalnızca MATLAB temizse** `tiltrotor_indi.slx`'i yeniden build edin
   (`tiltrotor_indi_build`) ve PX4 modülünü derleyin (`make
   px4_sitl_default` — `dangerouslyDisableSandbox` GEREKMEZ, yalnızca
   çalıştırma/simülasyon süreçleri için gerekir).

4. **SITL'de doğrulayın** — `sitl-lockup-check` skill'ini kullanın
   (≥25s izleme, kısa testler yanıltıcı, bkz. o skill'in gerekçesi).

5. **Sonucu ne olursa olsun belgeleyin** —
   `sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md`'ye yeni bir adım olarak
   (başarılı VEYA başarısız), `README.md`/`sitl/RUNBOOK.md`'ye kısa özet.
   Değişiklik SITL'de zarar verdiyse (regresyon/kötüleşme), **üç
   dosyada da (MATLAB + Simulink + PX4) eski değere geri alın**, MATLAB
   testleriyle geri dönüşü doğrulayın, ve neyin denenip neden geri
   alındığını raporda net biçimde işaretleyin ("DENENDİ, GERİ ALINDI").

## Kritik kural

**MATLAB'da "regresyon yok" görmek, SITL'de güvenli olduğu anlamına
GELMEZ** (Adım 7 örneği) — çünkü MATLAB'ın idealize plant modeli bu
projenin SITL'e özgü sorununu hiç yeniden üretmiyor. MATLAB kontrolü
yalnızca "bariz bir matematik hatası yok" garantisi verir; asıl
doğrulama her zaman `sitl-lockup-check` ile yapılmalıdır. Ama MATLAB
kontrolü ATLANMAMALI — çünkü bariz hataları (Aday çözüm 4 gibi) SITL'e
gitmeden, çok daha ucuza yakalar.
