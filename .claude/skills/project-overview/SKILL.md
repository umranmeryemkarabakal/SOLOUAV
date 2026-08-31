---
name: project-overview
description: Projenin tamamını hiç bilmeyen birine anlatan tek bir genel bakış dokümanı üretir. Modül listesi değil, sistemden geçen gerçek bir işlemi takip eden anlatı.
disable-model-invocation: true
---

# Proje genel bakışı

`docs/OVERVIEW.md` üret. Argüman verilmişse ($ARGUMENTS) çıktı yolu odur.

**Okuyucu:** Teknik okuryazarlığı olan ama bu projeyi ve alanı bilmeyen biri.
Ön koşul yok.

**Bu doküman konu-başına L1'lerin özeti DEĞİLDİR.** Tüm sistemi anlatan
bağımsız bir metindir. L1'lerden türetirseniz içindekiler tablosuna dönüşür.

---

## Keşif

Üst düzey modül başına bir `docs-explorer` subagent'ı (paralel). Ana
konuşmaya kod okutma. Çıkarılacaklar: sistemin girdi/çıktısı, ana
bileşenler ve veri akışı, tipik bir işlemin tam yolu, aşikâr olmayan
tasarım kararları, kodda görünen kısıtlar.

`manifest.yaml` varsa konu listesini oradan al, yeniden haritalama yapma.

---

## Bölümler — tam bu sırayla

**1. Tek cümle.** Jargonsuz. Okuyucu başkasına aktarabilmeli.

**2. Hangi problem.** Bu sistem olmasaydı ne olurdu? En az iki paragraf.
Çözümü anlatma. Bu bölümü kısaltma isteğine diren — problem anlaşılmadan
çözüm anlaşılmaz.

**3. Neden zor.** Naif yaklaşım ne olurdu, neden yetmez? Bu bölüm olmadan
okuyucu mühendisliği fazla mühendislik sanır.

**4. Nasıl çalışıyor — bir örneği takip et.** *En önemli bölüm.*
Sistemden geçen gerçek bir işlemi baştan sona izle. Bileşen listeleme;
girdiyi takip et, bileşenler yolda tanıtılsın. Her adımda: ne oluyor,
hangi bileşen, neden gerekli.

**5. Ana parçalar.** Bileşenler ancak şimdi adlandırılır — okuyucu §4'te
hepsini iş başında gördü. Her biri: sorumluluğu (1 cümle) + neyi yapmadığı.

**6. Önemli kararlar.** `| Karar | Alternatif | Neden bu |` — en fazla 5
satır, hepsi projeyi gerçekten şekillendirmiş kararlar. Kütüphane seçimi değil.

**7. Kapsam dışı.** Bilinçli olarak yapılmayanlar.

**8. Nereden devam.** Koda başlama noktası (`dosya:satır`), derle/çalıştır,
derinleşmek için hangi doküman.

---

## Kurallar

| Yasak | Neden |
|---|---|
| §5'ten önce bileşen listesi | Doküman içindekiler tablosuna dönüşür |
| Kod bloğu (>15 satır) | Anında bayatlar; `dosya:satır` ver |
| Formül, notasyon | Matematiksiz anlatılamayan kavramı anlatma, işlevini anlat |
| Parantez içi jargon açıklaması | Terimi hiç kullanma |
| "Yüksek performanslı", "sağlam" | Sayı ver: "12 ms içinde", "64 KB RAM'de" |
| Uzunluk için içerik uydurma | Hedef 1500–2500 kelime, sistem basitse daha kısa |

Bilinmeyeni `[BELİRSİZ: soru]` işaretle, sonunda kullanıcıya sor.
Diyagram gerekiyorsa Mermaid.

---

## Bitirince

Öz kontrol: §1 aktarılabilir mi, §4 gerçekten bir işlem mi takip ediyor
yoksa gizli modül listesi mi, §5'ten önce liste var mı, tanımsız terim
kaldı mı.

Sonra `fresh-reader` ile test et:

```
1. Bu sistem ne yapıyor?
2. Hangi problemi çözüyor?
3. Neden bu problem zor?
4. Sistemden geçen bir işlem hangi adımlardan geçiyor?
5. Bu sistem neyi YAPMIYOR?
6. Koda nereden bakmaya başlarım?
```

Kod tabanı henüz yoksa: keşif yerine `AskUserQuestion` ile kullanıcıyı
sorgula (ne yapacak, hangi problem, naif yaklaşım neden yetmiyor, tipik
işlem hangi adımlardan geçecek), sonra yaz.
