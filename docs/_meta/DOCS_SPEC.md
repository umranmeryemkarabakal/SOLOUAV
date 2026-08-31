# DOCS_SPEC — Dokümantasyon Sözleşmesi

Bu dosya, `docs/` altındaki tüm dokümanlar için bağlayıcı kuralları tanımlar.
Hem insan yazarlar hem Claude bu sözleşmeye uyar. Bir doküman bu dosyayla
çelişiyorsa doküman yanlıştır.

---

## 0. Proje yapılandırması

> **Yeni projede ilk iş burayı doldurun.** Aşağısı domain-agnostiktir.

```yaml
project_name: "Tilt-Rotor INDI Uçuş Kontrolü"
project_root: "./"              # kod depo kökünde düz duruyor, src/ yok
docs_root: "docs/"
language: "tr"                  # dokümanların dili
code_languages: ["C++", "MATLAB", "Python"]
domain: "havacılık / uçuş kontrol sistemleri / gömülü yazılım"
primary_audience_L1: "teknik olmayan paydaş, yeni mühendis, yatırımcı"
primary_audience_L2: "projeye katılan deneyimli mühendis"
primary_audience_L3: "alan uzmanı, hakem, akademik okuyucu"
citation_style: "IEEE"
```

> **Kod grupları** (Faz 1 haritalaması bunları tarar):
> PX4 uçuş yazılımı — `MulticopterIndiTiltrotor.{hpp,cpp}`,
> `TiltrotorIndiControl.hpp`, `TiltrotorIndiParams.hpp`, `*.msg` ·
> MATLAB model ve kontrol tasarımı — `*.m` ·
> SITL deney koşumları ve analiz — `sitl_experiments/` ·
> Mevcut yazılı kayıt — `RUNBOOK.md`, `WLS_LOCKUP_INVESTIGATION_REPORT.md`

---

## 1. Seviye tanımları

Seviyeler **hedef kitleye** göre tanımlıdır, uzunluğa göre değil.
Her seviye bağımsız bir metindir; biri diğerinin özeti veya genişletilmişi değildir.

### L1 — Kolay

| | |
|---|---|
| **Okuyucu** | Alanı bilmeyen ama teknik okuryazarlığı olan kişi |
| **Ön koşul** | Yok |
| **Uzunluk** | 800–1200 kelime |

**Cevaplaması zorunlu sorular**
1. Bu bileşen/yöntem ne işe yarıyor?
2. Olmasaydı ne olurdu? (problemin kendisi)
3. Temel fikri nedir? (bir analoji veya somut örnekle)
4. Sistemin geri kalanıyla nasıl ilişkileniyor?
5. En büyük zorluğu neydi?

**İzin verilen:** analoji, kavramsal şema, somut senaryo, sayısal büyüklük hissi
("saniyede 400 kez", "bir kâğıt yaprağı kalınlığında").

**Yasak:** matematiksel notasyon, kod bloğu, tanımlanmamış kısaltma,
"basitçe", "sadece", "yalnızca" gibi zorluğu küçümseyen kalıplar.

**Kabul testi:** `fresh-reader` agent'ı, yalnızca bu dokümanı okuyarak
5 sorunun tamamını cevaplayabilmeli ve tanımsız terim listesi boş olmalı.

---

### L2 — Orta

| | |
|---|---|
| **Okuyucu** | Projeye yeni katılan, alanı bilen mühendis |
| **Ön koşul** | Dil bilgisi + alanın temel dersleri |
| **Uzunluk** | 2000–3000 kelime |

**Cevaplaması zorunlu sorular**
1. Nasıl çalışıyor? (adım adım mekanizma)
2. Kodun tam olarak neresinde?
3. Girdi/çıktı sözleşmesi nedir?
4. Hangi uç durumlar ele alınmış?
5. Performans/kaynak karakteristiği nedir?

**Zorunlu içerik**
- En az bir sözde kod bloğu veya durum/akış diyagramı
- Karmaşıklık veya kaynak analizi (zaman, bellek, döngü frekansı, bant genişliği — hangisi anlamlıysa)
- En az 3 gerçek kod referansı (bkz. §3)
- En az 2 uç durum, her biri kodda nerede ele alındığıyla
- "Ne zaman bu bileşeni değiştirmeniz gerekir" bölümü

**Yasak:** doğruluk ispatı, türetme, literatür tartışması — bunlar L3'e ait.

---

### L3 — Zor

| | |
|---|---|
| **Okuyucu** | Alan uzmanı; hakem gözüyle okuyor |
| **Ön koşul** | Alanın lisansüstü düzeyi |
| **Uzunluk** | Sınır yok |

**Cevaplaması zorunlu sorular**
1. Yöntemin formal tanımı nedir?
2. Neden doğru/kararlı/yakınsak? (ispat, türetme veya argüman)
3. Hangi alternatifler değerlendirildi ve neden elendi?
4. Hangi varsayımlara dayanıyor?
5. Ne zaman bozulur? (başarısızlık modları, geçerlilik sınırları)
6. Literatürdeki yeri nedir?

**Zorunlu içerik**
- Formal notasyon; tüm semboller ilk kullanımda tanımlı
- Değerlendirilen alternatifler tablosu (yöntem / avantaj / neden elendi)
- Varsayımlar bölümü — açık liste halinde
- Başarısızlık modları bölümü
- `bibliography.md`'den atıflar (bkz. §4)

**Yasak:** doğrulanmamış atıf, "iyi bilinir ki" ile geçiştirilen adım,
kaynaksız sayısal iddia.

---

## 2. Dosya düzeni

> **Bu bölüm sabit değildir.** Düzen, kod tabanı haritalandıktan sonra
> Faz 1.5'te seçilir ve `manifest.yaml` → `meta.layout` alanına yazılır.
> Araçlar o alanı okur; buradaki metin yalnızca seçenekleri tanımlar.

### Sabit olan

```
docs/_meta/                 # her düzende aynı yerde
  DOCS_SPEC.md              # bu dosya
  manifest.yaml             # konu listesi + düzen kararı
  GLOSSARY.md               # terimler, tek tanım
  bibliography.md           # doğrulanmış kaynaklar
  notes/<topic-id>.md       # kavram notları (iç kullanım)
  templates/
```

`_meta` konumu pazarlık konusu değil — tüm scriptler ve agent'lar buraya
sabit yoldan erişir.

### Seçilebilir olan: doküman yerleşimi

| `pattern` | Şema | Ne zaman |
|---|---|---|
| `flat` | `docs/L1-<topic>.md` | ≤ 6 konu |
| `by-topic` | `docs/<topic>/L1-<topic>.md` | 7–20 konu, alt sistem grubu yok |
| `by-level` | `docs/L1/<topic>.md` | Okuyucu seviyeye göre geziyor (onboarding yolu) |
| `grouped` | `docs/<group>/<topic>/L1-<topic>.md` | > 20 konu veya net alt sistemler |
| `mirror` | `docs/<modül-yolu>/L1-<topic>.md` | Dokümanlar kod ağacını izlemeli |

Seçim ölçütleri Faz 1.5'te (SKILL.md) tanımlı.

### Düzen kaydı

`manifest.yaml` içinde:

```yaml
meta:
  layout:
    pattern: by-topic
    dir_template: "docs/{topic}/"
    file_template: "{level}-{topic}.md"
    rationale: "8 konu, net alt sistem grubu yok"
```

`dir_template` ve `file_template` gerçek kaynaktır; `pattern` yalnızca
insan okunabilir etikettir. Değişkenler: `{topic}`, `{level}`, `{group}`,
`{module}`.

`check_frontmatter.py` bu şablonlardan beklenen yolu üretip her dosyanın
doğru yerde ve doğru adla olduğunu doğrular. Düzen değiştirmek isterseniz
şablonu güncelleyip dosyaları taşımak yeterli — script uyar.

### Frontmatter (her doküman için zorunlu)

```yaml
---
topic: attitude-control          # manifest'teki id ile birebir aynı
level: L2                        # L1 | L2 | L3
title: "Uçuş kontrol döngüsü"
status: draft                    # draft | reviewed | verified
code_refs:
  - src/control/attitude.cpp:L120-L188
sources: [ref-3, ref-7]          # yalnızca L3; bibliography.md anahtarları
updated: 2026-08-27
---
```

`status` yalnızca ilgili kapılar geçildikten sonra yükseltilir (§5).

---

## 3. Kod referansı kuralı

Kodun davranışına dair **her** teknik iddia bir referans taşımalı:

```
`src/control/attitude.cpp:L120-L145`
```

- Tek satır: `path:L42`. Aralık: `path:L42-L88`.
- Yol repo kökünden görecelidir.
- Referanslar `scripts/check_doc_refs.py` ile doğrulanır — uydurma yol veya
  var olmayan satır aralığı CI'ı düşürür.
- Kod **yapıştırmayın**. 15 satırdan uzun blok yasak; referans verin.
  İstisna: L2'deki sözde kod (gerçek kod değil).

---

## 4. Atıf kuralı

> **Yalnızca `bibliography.md`'de kayıtlı, gerçekten getirilip okunmuş
> kaynaklar atıflanabilir.** İstisnası yoktur.

- Atıf formatı: `[ref-7]`, `bibliography.md`'deki anahtara karşılık gelir.
- Bir iddia için kaynak bulunamadıysa iki seçenek var: iddiayı kaldırın,
  veya `> **[KAYNAK GEREKLİ]**` bloğuyla işaretleyin.
- Hafızadan atıf üretmek, doküman tamamen yanlış sayılmasına yol açar.
- Bir kaynak yalnızca bibliyografyaya işlenirken doğrulanır; yazım sırasında
  yeniden arama yapılmaz.

---

## 5. Kalite kapıları

Hiçbir doküman kapıları geçmeden `status: verified` olamaz.

| # | Kapı | Nasıl | Kapsam |
|---|---|---|---|
| 1 | Kod referansı | `scripts/check_doc_refs.py` | Tümü |
| 2 | Frontmatter | `scripts/check_frontmatter.py` | Tümü |
| 3 | Taze okuyucu | `fresh-reader` agent | L1, L2 |
| 4 | Seviye tutarlılığı | `level-consistency` agent | Konu başına 3'ü birlikte |
| 5 | Teknik denetim | `technical-auditor` agent | L2, L3 |

**Denetleyici kısıtı:** Kapı 3–5'teki agent'lar yalnızca (a) yanlış olan veya
(b) bu sözleşmenin zorunlu tuttuğu bir şeyin eksik olduğu yerleri işaretler.
Stil tercihi, kapsam önerisi ve "şu da eklenebilirdi" türü bulgular geçersizdir.

---

## 6. Terminoloji

- Her terim `GLOSSARY.md`'de bir kez tanımlanır. Dokümanlar oraya link verir.
- Aynı kavram için tüm dokümanlarda aynı terim kullanılır. Eş anlamlı
  serbestliği yoktur. (`tilt mekanizması` / `döndürme sistemi` / `nacelle
  aktüatörü` üçü aynı şeyse, biri seçilir.)
- Kısaltmalar her dokümanda ilk kullanımda açılır.
- Kod sembolü adları kod tabanındaki haliyle yazılır, çevrilmez.

---

## 7. Seviyeler arası kurallar

- **Türetme yasağı:** L1, L3'ün sadeleştirilmişi olarak yazılamaz. Her seviye
  kavram notundan sıfırdan yazılır. Cümle taşımak ihlaldir.
- **Bağlantı:** L1 sonunda "Daha derine → L2", L2 sonunda "Daha derine → L3".
- **Çelişki yasağı:** Üç seviye aynı gerçeği anlatır. Kesinlik derecesi
  değişebilir ("yaklaşık 400 Hz" ↔ "400 Hz ± 5"), gerçek değişemez.
- **Kısmi seviye:** Bir konu üç seviyeyi hak etmeyebilir. `manifest.yaml`
  içindeki `levels` alanı belirleyicidir; zorlama seviye üretilmez.

---

## 8. Yazım

- Doğrudan cümle. Bir paragraf bir fikir.
- Pasif yapı yerine etken.
- "Basitçe", "sadece", "aşikâr olduğu üzere", "kolayca görülebileceği gibi"
  kullanılmaz — okuyucuya zorluğunu küçümsediğinizi hissettirir.
- Tablo, listelenebilir karşılaştırmalar için; anlatı için düz metin.
- Diyagramlar Mermaid ile inline yazılır (versiyonlanabilir olsun).
