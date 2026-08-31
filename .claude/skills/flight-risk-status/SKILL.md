---
name: flight-risk-status
description: Reports the current flight-safety criticality (GO/NO-GO) of this tiltrotor controller for a given target environment (real hardware, SITL, or pure MATLAB), based on the latest documented open issues. Use when asked "is it safe to fly this", "can we test on hardware", "how critical is this bug", or before any hardware flight attempt is discussed.
---

# Uçuş güvenliği kritiklik durumu

Bu skill, projedeki bilinen açık kontrol sorunlarının (özellikle
`sitl/RUNBOOK.md` §4 ve `sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md`'de
belgelenen WLS aktüatör kilitlenmesi / yaw savrulması sorunu) GÜNCEL
durumunu okuyup hedef ortama (donanım / SITL / MATLAB) göre bir
GO/NO-GO değerlendirmesi verir. Amaç: her seferinde kritiklik analizini
sıfırdan yeniden türetmek yerine, güncel belgelenmiş duruma dayalı hızlı
ve tutarlı bir cevap vermek.

## Prosedür

1. `sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md`'nin başındaki **Durum**
   satırını ve **§1a Uçuş kritiklik değerlendirmesi** bölümünü okuyun —
   bu, en güncel GO/NO-GO tablosunu içerir.
2. `sitl/RUNBOOK.md` §4'ün başlığındaki (ÇÖZÜLMEDİ/ÇÖZÜLDÜ) durumunu
   kontrol edin — rapordan daha güncel olabilir.
3. En son eklenen "Adım N" / "Aday çözüm N" girişini bulun (dosyanın
   sonuna doğru) — sorunun hâlâ açık mı yoksa son oturumda çözüldü mü
   olduğunu teyit edin. **Bu dosyaları okumadan, hafızaya veya önceki
   bir konuşmaya dayanarak GO/NO-GO söylemeyin** — durum sık
   güncelleniyor.
4. Kullanıcının sorduğu hedef ortama göre cevap verin. **Raporun §1a'sı
   birden fazla tablo içeriyor** (Adım 1-10 dönemi, Adım 11, Adım 12,
   Adım 16). **Her zaman "★ EN GÜNCEL DURUM ★" işaretli olanı
   kullanın.** 2026-07-27 (Adım 16) itibarıyla özet:
   - **Gerçek donanım:** **NO-GO.** İki kök neden bulunup düzeltildi
     (Adım 11 itki eşlemesi, Adım 12 `ROTOR_KM` işareti) ve yaw'ın
     sınırsız dönmesi Adım 13'te durduruldu — ama **Adım 16, o
     doğrulamaların hepsinin ~10 m/s ileri hızda yapıldığını gösterdi.**
     Yaw'ı sönümleyen şey kontrolcü değil, ileri hızdaki aerodinamik
     rüzgâr gülü etkisi; 2.45 m/s'de +30° yaw adımı ±25° sönümsüz
     salınım veriyor. Bu kontrolcüde yatay pozisyon döngüsü olmadığı ve
     airframe yapısal olarak kendini ileri ittiği için (net Fx ≥ 0,
     çünkü tüm tiltler ≥0) **gerçek anlamda yerinde duran hover hiç
     test edilmemiş** — ve yaw için en kötü koşul odur. Ayrıca itki
     eşleme düzeltmesi hâlâ **Gazebo SDF sabitlerine özgü**
     (`ROTOR_KF=2e-5`, `WMAX=1500`, `WMIN=10`) — gerçek donanımda
     ölçülmüş motor/ESC eğrisinden yeniden türetilmeli. (`ROTOR_KM`
     işaret düzeltmesi ise donanıma **taşınabilir**: gerçek rotor dönüş
     yönlerinden türer, Gazebo'ya özgü değildir.)
   - **SITL geliştirme/test:** **GO.** Aktüatör kilitlenmesi ve dikey
     hız kontrolsüzlüğü çözüldü (845 örneklik koşuda sıfır BIG_M),
     irtifa/roll/pitch temiz. Her "düzeldi" iddiası ≥25s'lik
     `sitl-lockup-check` ile doğrulanmalı — **ve artık koşunun ileri
     hızı da kaydedilmeli** (o skill'deki zorunlu adım).
   - **Saf MATLAB geliştirme:** **GO** — sorun MATLAB referansında hiç
     gözlenmedi. Adım 11 ve 12 bunun KESİN sebebini buldu: her iki hata
     da plant ile kontrolcünün paylaştığı bir varsayımda olduğu için
     MATLAB'da yapısal olarak görünmez (ikisi birlikte yanlıştır).
5. Varsa, en son denenen ve BAŞARISIZ/GERİ ALINAN yöntemleri kısaca
   listeleyin (kullanıcı "ne denendi" diye sorabilir) — raporun "Şu ana
   kadarki genel çıkarımlar" bölümü bunun için iyi bir özet kaynağıdır.

## Cevap formatı

Kısa bir GO/NO-GO tablosu + 2-3 cümlelik gerekçe + "son güncelleme"
olarak raporun en son Adım/Aday çözüm numarasını belirtin (böylece
kullanıcı bilginin ne kadar güncel olduğunu bilir). Uzun bir teknik
analiz TEKRARLAMAYIN — o zaten raporda var, yalnızca ona işaret edin.
