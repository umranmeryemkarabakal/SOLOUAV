# Kavram Notu: <konu başlığı>

> **İç kullanım.** Bu dosya yayınlanmaz. L1/L2/L3 dokümanlarının **tek
> ortak kaynağıdır** — üçü de bundan bağımsız olarak yazılır.
>
> Burada seviye kaygısı yok. Tek ölçüt doğruluk ve eksiksizlik.
> Uzun olması normaldir.

---

## 1. Problem

Bu bileşen olmasaydı ne olurdu? Hangi ihtiyaçtan doğdu?
(Çözümü değil, problemi anlat.)

## 2. Ne yapıyor

Bileşenin sorumluluğu. Sınırları: neyi yapmıyor?

## 3. Nasıl çalışıyor

Adım adım mekanizma. Veri akışı. Durum geçişleri.

## 4. Kodda nerede

| Sorumluluk | Konum |
|------------|-------|
| <ne> | `dosya:Lxx-Lyy` |

Giriş noktaları ve dışa açık arayüzler:
- `sembol` — `dosya:Lxx` — <açıklama>

## 5. Kullanılan yöntemler

Her yöntem için:

### <yöntem adı>
- **Formal tanım:** <notasyonla>
- **Neden bu yöntem:** <gerekçe>
- **Uygulamamızdaki sapma:** <standart halinden farkımız, varsa>
- **Kaynak:** `[ref-N]`

## 6. Değerlendirilen alternatifler

| Alternatif | Avantajı | Neden elendi |
|------------|----------|--------------|

Bu tablo boşsa muhtemelen yeterince düşünmediniz veya konu gerçekten
L3 hak etmiyordur.

## 7. Varsayımlar

Açık liste. Her biri için: bu varsayım bozulursa ne olur?

- <varsayım> → bozulursa: <sonuç>

## 8. Başarısızlık modları

Ne zaman bozulur? Geçerlilik sınırları nerede?

| Mod | Tetikleyici | Belirti | Ele alınışı |
|-----|-------------|---------|-------------|

## 9. Uç durumlar

| Uç durum | Kodda ele alınışı |
|----------|-------------------|

## 10. Performans / kaynak karakteristiği

Zaman, bellek, döngü frekansı, bant genişliği, gecikme — hangisi anlamlıysa.
Ölçüm varsa ölçüm; yoksa analiz ve bunun analiz olduğunun notu.

## 11. L1 için ham malzeme

> Bu bölüm özellikle önemli. L1'in kendine ait somut çıpaları olmalı,
> yoksa kaçınılmaz olarak L3'ün sadeleştirilmişi olarak yazılır.

- **Analoji adayları:** <alandan bağımsız benzetmeler>
- **Somut senaryo:** <gerçek bir uçuş/kullanım anı>
- **Büyüklük hissi:** <"saniyede 400 kez", "insan tepki süresinin 100 katı hızda">
- **En büyük zorluk neydi:** <hikâyeleştirilebilir kısım>

## 12. Açık sorular

`[BELİRSİZ: ...]` işaretleri ve cevaplanmamış sorular.
