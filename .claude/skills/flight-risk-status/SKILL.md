---
name: flight-risk-status
description: Reports the current flight-safety criticality (GO/NO-GO) of this tiltrotor controller for a given target environment (real hardware, SITL, or pure MATLAB), based on the latest documented open issues. Use when asked "is it safe to fly this", "can we test on hardware", "how critical is this bug", or before any hardware flight attempt is discussed.
---

# Uçuş güvenliği kritiklik durumu

Bu skill, projedeki bilinen açık kontrol sorunlarının GÜNCEL durumunu
okuyup hedef ortama (donanım / SITL / MATLAB) göre bir GO/NO-GO
değerlendirmesi verir. Amaç: kritiklik analizini her seferinde sıfırdan
türetmek yerine, **belgelenmiş güncel duruma** dayalı hızlı ve tutarlı
bir cevap vermek.

## Kaynak dosyalar — hepsi DEPO KÖKÜNDE

| dosya | ne için |
|---|---|
| `WLS_LOCKUP_INVESTIGATION_REPORT.md` | geliştirme günlüğü; başındaki "► BURADAN BAŞLAYIN" bloğu = güncel durum tablosu |
| `RUNBOOK.md` | §4 başlığında sorunun ÇÖZÜLDÜ/ÇÖZÜLMEDİ durumu |
| `HARDWARE_READINESS_CHECKLIST.md` | **donanım GO/NO-GO için asıl kaynak**; H1-H8 + B0-B5 engelleyici tablosu |

⚠ **`sitl/` altındaki kopyaları OKUMAYIN.** `sitl/RUNBOOK.md` ve
`sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md` 2026-07-28 / Adım 26'da donmuş
eski kopyalardır (kök sürümler Adım 147+). Bu skill daha önce tam olarak
o yolları gösteriyordu ve beş hafta bayat bir cevap üretiyordu.

## Prosedür

1. **`WLS_LOCKUP_INVESTIGATION_REPORT.md`'nin ilk ~50 satırını okuyun** —
   "► BURADAN BAŞLAYIN (son güncelleme: …, Adım N)" bloğu. Güncel durum
   tablosu ve "açık kalan teknik konular" listesi buradadır.
   ⚠ Dosyanın içindeki **`★ EN GÜNCEL DURUM ★` işaretine GÜVENMEYİN** —
   o işaret §1a'daki Adım 16-17 (2026-07-27/28) bloğunda duruyor ve
   artık en güncel tablo O DEĞİL. Baştaki "BURADAN BAŞLAYIN" bloğu kazanır.
2. **`HARDWARE_READINESS_CHECKLIST.md`'yi okuyun** — donanım sorusu
   soruluyorsa asıl cevap buradadır ve rapordan **daha güncel olabilir**.
   Başındaki "Kısa cevap" satırı + H1-H8/B0-B5 tablosu. 🔴 kalan
   maddeleri ada ada sayın (kullanıcı "neden hazır değil" diye sorar).
3. `RUNBOOK.md` §4 başlığındaki (ÇÖZÜLDÜ/ÇÖZÜLMEDİ) durumu doğrulayın.
4. **Bu dosyaları okumadan; hafızaya, önceki bir konuşmaya veya aşağıdaki
   snapshot'a dayanarak GO/NO-GO söylemeyin.** Durum sık güncelleniyor;
   bu skill'in bilinen tek arıza modu budur.
5. Kullanıcı "ne denendi" diye sorarsa: raporda **⛔ DENENDİ, GERİ ALINDI**
   işaretli adımlar ve "Şu ana kadarki genel çıkarımlar" bölümü.

## Yönlenme snapshot'ı — BAĞLAYICI DEĞİL

> Aşağısı yalnızca ne arayacağınızı bilmeniz için; **çelişki halinde
> dosyalar kazanır**. Tarihi geçmişse güncelleyin.

**2026-09-01 (rapor Adım 147, kontrol listesi Adım 154) itibarıyla:**

- **Gerçek donanım: 🔴 NO-GO.** Kart hiç takılmadı, hiçbir şey gerçek
  donanımda uçmadı. Açık engelleyiciler: **H7** (kartta CPU/tick jitter
  ölçülmedi — gövde zamanlama sarsıntısına duyarlı: aynı ikili GUI'de
  112 BIG_M, headless 0), **H8** (RC vericisi ↔ kart yolu sınanmadı),
  **B1** (pilot girişi + kademeli failsafe; link kaybı hâlâ 100+ m
  sürüklenme), **H6** 🟡 (HITL airframe'i kuruldu, kartta koşmadı).
  Ayrıca Adım 11'in itki eşlemesi hâlâ Gazebo sabitlerine kalibre
  (`ROTOR_KF=2e-5`, `WMAX=1500`, `WMIN=10`) — gerçek motor/ESC
  eğrisinden yeniden türetilmeli. (`ROTOR_KM` işaret düzeltmesi ise
  donanıma **taşınabilir**: gerçek rotor dönüş yönlerinden türer.)
- **SITL: 🟢 GO.** Tam görev 6/6 geçiyor (kalkış→ileri geçiş→sabit
  kanat→geri geçiş→hover→iniş), sakin ve rüzgârlı dünyada. Her "düzeldi"
  iddiası ≥25 s'lik `sitl-lockup-check` ile doğrulanmalı, **koşunun
  ileri hızı da kaydedilmeli** (yaw sönümlemesi ileri hızdan geliyor
  olabilir — Adım 16'nın dersi).
- **Saf MATLAB: 🟢 GO.** Sorun MATLAB referansında hiç gözlenmedi;
  Adım 11-12 sebebini buldu: her iki hata da plant ile kontrolcünün
  PAYLAŞTIĞI bir varsayımdaydı, yani MATLAB'da yapısal olarak görünmez.
- **Açık üç teknik konu:** rüzgârlı roll limit çevrimi (gecikme telafisi
  istiyor), BRAKE→HANDOFF pitch sıçraması (+15,2°), aralıklı alçalma
  takılması (0,6-0,8 m yer etkisi).

## Cevap formatı

Kısa bir GO/NO-GO tablosu + 2-3 cümlelik gerekçe + "son güncelleme"
olarak okuduğunuz en son Adım numarasını belirtin (kullanıcı bilginin ne
kadar taze olduğunu bilsin). Uzun teknik analizi TEKRARLAMAYIN — dosyada
zaten var, ona işaret edin.
