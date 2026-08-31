---
name: docs-pipeline
description: Büyük bir kod tabanı için çok seviyeli (kolay/orta/zor) teknik ve akademik dokümantasyon üretme akışı. Doküman yazma, doküman seti planlama, mevcut dokümantasyonu denetleme veya bir konuyu farklı zorluk seviyelerinde anlatma isteklerinde kullanılır.
---

# Çok seviyeli dokümantasyon akışı

Bu akışın çözdüğü iki problem: (1) proje bağlama sığmıyor, (2) çıktının
doğruluğunu kontrol edecek bir test paketi yok.

**Yönetici kuralı:** Ana oturum yazma ve okuma işi yapmaz. Fazları
koordine eder, subagent'lara devreder, kapıları çalıştırır. Ana bağlam
temiz kalırsa geri kalan her şey daha iyi çalışır.

---

## Faz 0 — Sözleşme

`docs/_meta/DOCS_SPEC.md` §0'ı doldur. Doldurulmadan hiçbir faza geçme.

## Faz 1 — Haritalama

Üst düzey modül başına **bir `docs-explorer` subagent'ı**, paralel.
Bağımsız modüller birbirini beklemez.

Çıktıları `docs/_meta/inventory.md`'de topla, oradan `manifest.yaml` üret.

> **DUR VE SOR.** Manifest'i kullanıcıya sun, onay al. Yanlış manifest ile
> 40 doküman üretmek bu akıştaki en pahalı hata. Özellikle `levels`
> alanlarını sorgula — her konu üç seviyeyi hak etmez.

## Faz 1.5 — Yerleşim kararı

> Kod tabanı haritalandı, manifest onaylandı. Dosya düzenine **ancak
> şimdi** karar verilebilir; konu sayısını ve alt sistem yapısını
> görmeden verilen düzen kararı yanlış olur.

`manifest.yaml` → `meta.layout` alanını doldur. Seçenekler DOCS_SPEC §2'de.

**Karar ölçütleri, sırayla:**

1. **Konu sayısı**
   - ≤ 6 → `flat` (dizin hiyerarşisi bu ölçekte gereksiz gezinme yükü)
   - 7–20 → `by-topic`
   - > 20 → `grouped` (düz dizin bu ölçekte taranamaz hale gelir)

2. **Alt sistem yapısı var mı?**
   Manifest'teki `depends_on` grafiğine bak. Konular birbirine sıkı bağlı
   kümeler oluşturuyor ve kümeler arası bağ zayıfsa → `grouped`, kümeler
   `group` alanı olur. Grafik iç içeyse zorlama gruplama yapma.

3. **Okuyucu nasıl geziyor?**
   Baskın kullanım "yeni kişi tüm L1'leri sırayla okur" ise → `by-level`.
   "Bir konuyu derinleştirmek için gelir" ise → `by-topic`.
   Emin değilsen `by-topic`; daha yaygın olan bu.

4. **Dokümanlar kod ağacını izlemeli mi?**
   Monorepo'da paket başına doküman gerekiyorsa → `mirror`, konulara
   `module` alanı eklenir. Tek başına bir sebep değil; ekip zaten kod
   ağacında geziniyorsa anlamlı.

**Kararı yazdıktan sonra doğrula:**

```bash
python scripts/check_frontmatter.py --show-layout
```

Bu, üretilecek tüm dosya yollarını listeler. Yazıma başlamadan önce
listeye bak — `grouped` veya `mirror` seçtiysen eksik `group`/`module`
alanları burada hata olarak çıkar.

**Kararı kullanıcıya sun.** Yolları ve gerekçeni göster, onay al. Düzen
sonradan değiştirilebilir (şablonu güncelle, dosyaları taşı, script uyar)
ama 40 dosya yazıldıktan sonra taşımak çapraz linkleri de kırar.

## Faz 2 — Kaynaklar

Manifest'teki `methods` listesindeki her yöntem için bir
`source-researcher` subagent'ı. Çıktı `bibliography.md`'ye.

`Getirildi: evet` olmayan hiçbir kayıt atıflanamaz. Bu dosyayı da
kullanıcının gözden geçirmesini iste — uydurulmuş tek bir atıf tüm
dokümantasyonun güvenilirliğini götürür.

## Faz 2.5 — Genel bakış

`/project-overview` çalıştır → `docs/OVERVIEW.md`.

**Faz 3'ten önce.** OVERVIEW konu dokümanlarının özeti değil, tüm sistemi
anlatan bağımsız bir metindir. L1'lerden sonra yazılırsa içindekiler
tablosuna dönüşür; önce yazılırsa konu dokümanları ona link verebilir.

## Faz 3 — Yazım

`depends_on` topolojik sırasında, konu başına:

**3a.** Kavram notu → `docs/_meta/notes/<topic>.md`
(şablon: `templates/concept-note.md`). §11'i özenle doldur — L1'in
kendine ait çıpaları oradan gelir.

**3b.** Manifest'teki her seviye için ayrı doküman. Her biri kavram notunu
ve DOCS_SPEC'i okuyup **sıfırdan** yazar.

> **Türetme yasağı.** L1, L3'ün sadeleştirilmişi değildir. Farklı bir
> okuyucuya aynı gerçeği anlatan bağımsız bir metindir. Cümle taşıma,
> jargonu parantez içi açıklamayla aktarma. Sıfırdan yaz.

**İlk üç konuyu tek tek, elle koş.** Promptun ilk konuda kesinlikle
yanlış olacak. Düzelt, sonra ölçekle.

## Faz 4 — Kapılar

| # | Kapı | Ne zaman |
|---|------|----------|
| 1 | `python scripts/check_doc_refs.py` | Her yazımdan sonra |
| 2 | `python scripts/check_frontmatter.py` | Her yazımdan sonra |
| 3 | `fresh-reader` | L1 ve L2 için |
| 4 | `level-consistency` | Konunun tüm seviyeleri bitince |
| 5 | `technical-auditor` | L2 ve L3 için |

1–2 geçince `status: reviewed`. 3–5 geçince `status: verified`.

**Denetleyici kısıtı:** Boşluk aramakla görevlendirilen agent, doküman
sağlam olsa bile boşluk bulur. Her bulguyu kovalamak aşırı mühendisliğe
götürür. Yalnızca yanlış olan veya DOCS_SPEC'in zorunlu tuttuğu eksikler
işlenir; stil ve kapsam önerileri reddedilir.

## Faz 5 — Ölçekleme

Pipeline 2-3 konuda çalıştıktan sonra: git repo'sunda `/batch`, veya
`claude -p` döngüsü. Konu başına bir worktree ve bir PR.

## Faz 6 — Bakım

Tek seferlik iş değil. Bayat dokümantasyon, dokümantasyon olmamasından
kötüdür.

- `check_doc_refs.py` CI'a bağlanır
- `manifest.yaml`'daki `code` globlarına dokunan PR'lar ilgili
  dokümanların gözden geçirilmesini tetikler
- Kod değişikliğinde `status` `verified`'dan `draft`'a düşürülür

---

## Sık hatalar

| Hata | Belirti | Çözüm |
|------|---------|-------|
| Manifest onaysız ölçekleme | 40 doküman, yanlış konular | Faz 1'de dur ve sor |
| Repoyu görmeden düzen seçme | 40 konu düz dizinde, taranamaz | Faz 1.5'i atlama |
| Zorlama gruplama | Tek konuluk gruplar, anlamsız hiyerarşi | Küme yoksa `by-topic` |
| Türetilmiş L1 | Parantez içi jargon açıklamaları | Kavram notu §11'den sıfırdan yaz |
| Yazarken araştırma | Halüsinasyon atıflar | Faz 2'yi Faz 3'ten ayır |
| Ana bağlamda kod okuma | Yazım kalitesi düşer | `docs-explorer`'a devret |
| Denetleyiciye tam yetki | `fresh-reader` boşluğu kendi kapatır | `tools: Read` ile sınırla |
| Kod yapıştırma | Dokümanlar anında bayatlar | Referans ver, 15 satır sınırı |
