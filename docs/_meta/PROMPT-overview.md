# Prompt: Projeyi basitçe anlatan tek doküman

Aşağıdaki bloğu Claude Code'a **temiz bir oturumda** (proje kökünde)
yapıştırın. Alan-bağımsızdır.

Çıktı: `docs/OVERVIEW.md` — projenin tamamını anlatan tek dosya.
Konu-başına L1 dokümanlarından farklıdır; onlar bir konuyu, bu tüm sistemi anlatır.

---

## Prompt

````
docs/OVERVIEW.md yaz: bu projeyi hiç bilmeyen birine tümüyle anlatan tek doküman.

## Okuyucu
Teknik okuryazarlığı olan ama bu projeyi ve muhtemelen bu alanı bilmeyen biri.
Yeni ekip üyesi, başka takımdan mühendis, teknik danışman, yatırımcı.
Ön koşul yok.

## Önce keşif

Kod tabanını oku. Ana konuşmanı doldurmamak için üst düzey modül başına ayrı
bir subagent kullan ve her birinden sadece özet iste.

Şunları çıkar:
- Sistem ne yapıyor, girdisi ve çıktısı ne
- Ana bileşenler ve aralarındaki veri akışı
- Sistemden geçen tipik bir işlemin tam yolu
- Aşikâr olmayan tasarım kararları ve gerekçeleri
- Kodda görünen kısıtlar (gerçek zamanlılık, kaynak, güvenlik, uyumluluk)

Anlayamadığın veya kodun cevaplamadığı şeyleri not et — sonunda soracaksın.

## Yapı

Bölümleri tam olarak bu sırayla yaz:

### 1. Tek cümle
Sistemin ne olduğu, tek cümlede. Jargonsuz.
Test: okuyucu bu cümleyi başkasına aktarabilmeli.

### 2. Hangi problem
Bu sistem olmasaydı ne olurdu? Kim, neyi, neden yapamıyordu?
Çözümü anlatma. Sadece problemi. En az iki paragraf.
Problem anlaşılmadan çözüm anlaşılmaz — bu bölümü kısa tutma isteğine direnç göster.

### 3. Neden zor
Naif yaklaşım ne olurdu ve neden yetmez?
Bu bölüm olmadan okuyucu sistemin neden bu kadar karmaşık olduğunu anlamaz
ve mühendisliği fazla mühendislik sanır.

### 4. Nasıl çalışıyor — bir örneği takip et
> Bu dokümandaki en önemli bölüm.

Sistemden geçen GERÇEK BİR İŞLEMİ baştan sona izle. Bileşenleri
listeleme — girdiyi takip et, bileşenler yolda kendiliğinden tanıtılsın.

Örnekler: bir isteğin gelişinden yanıtına, bir dosyanın yüklenişinden
işlenmesine, bir uçuşun kalkıştan inişe, bir işlemin başlangıcından
mutabakatına.

Her adımda: ne oluyor, hangi bileşen yapıyor, neden gerekli.

### 5. Ana parçalar
Ancak ŞİMDİ bileşenleri adlandır. Okuyucu §4'te hepsini iş başında gördü,
artık isim listesi anlamlı.
Her biri için: sorumluluğu (1 cümle) + neyi yapmadığı.

### 6. Önemli kararlar
| Karar | Alternatif | Neden bu |
En fazla 5 satır. Her satır projeyi gerçekten şekillendirmiş bir karar olmalı;
kütüphane seçimi değil.

### 7. Kapsam dışı
Sistemin bilinçli olarak yapmadığı şeyler. Okuyucunun yanlış beklentiyle
ayrılmasını engeller.

### 8. Nereden devam
- Kodu okumaya nereden başlanır (dosya:satır)
- Derleyip çalıştırma
- Derinleşmek için hangi doküman

## Kurallar

**Modül listesi yazma.** §5'ten önce bileşen listesi çıkarsan doküman
başarısızdır. Anlatı sistemden geçen işlemi takip eder, dizin yapısını değil.

**Kod yapıştırma.** `dosya:satır` referansı ver. En fazla 15 satırlık blok,
o da mecburi kalırsa.

**Formül ve notasyon yok.** Bir kavram matematik olmadan anlatılamıyorsa
o kavramı anlatma, ne işe yaradığını anlat.

**Parantez içi jargon açıklaması yok.** "Kuaterniyon (bir tür dönüş temsili)
kullanıyoruz" yasak — terimi hiç kullanma, ne yaptığını anlat.

**Somut ol.** "Yüksek performanslı", "ölçeklenebilir", "sağlam" gibi
doldurma sıfatlar yerine sayı ver: "saniyede 400 kez", "12 ms içinde",
"64 KB RAM'de".

**Şişirme.** Sistem gerçekten karmaşık değilse doküman kısa olsun.
Hedef 1500–2500 kelime ama uzunluk için içerik uydurma.

**Emin olmadığını uydurma.** Kodun cevaplamadığı şeyi `[BELİRSİZ: soru]`
olarak işaretle.

Diyagram gerekiyorsa Mermaid kullan (versiyonlanabilir olsun).

## Bitirince

1. Kendi dokümanını oku ve şunları kontrol et:
   - §1'deki cümle jargonsuz mu ve aktarılabilir mi
   - §4 gerçekten bir işlemi mi takip ediyor, yoksa gizli bir modül listesi mi
   - §5'ten önce hiç bileşen listesi var mı
   - Tanımsız terim kaldı mı

2. `[BELİRSİZ]` işaretlerini bana sor.

3. Şunu söyle: "fresh-reader agent'ı ile test edilmeye hazır."
````

---

## Doğrulama

`.claude/agents/fresh-reader.md` kuruluysa, dokümanı bitirdikten sonra:

```
fresh-reader agent'ını docs/OVERVIEW.md üzerinde çalıştır. Sorular:
1. Bu sistem ne yapıyor?
2. Hangi problemi çözüyor?
3. Neden bu problem zor?
4. Sistemden geçen bir işlem hangi adımlardan geçiyor?
5. Bu sistem neyi YAPMIYOR?
6. Koda nereden bakmaya başlarım?
```

`fresh-reader`'ın yalnızca `Read` yetkisi vardır — kod tabanına bakamaz,
yani eksik bilgiyi kendi kapatamaz. Cevaplayamadığı soru gerçek bir boşluktur.

---

## Kullanım notları

**Boru hattı olmadan da çalışır.** DOCS_SPEC, manifest veya kavram notları
gerekmez. Tek başına, herhangi bir projede kullanılabilir.

**Boru hattı varsa Faz 3'ten önce çalıştırın.** OVERVIEW, L1'lerin özeti
değildir ve onlardan türetilmemelidir. Önce yazılırsa konu dokümanları
ona link verebilir; sonra yazılırsa kaçınılmaz olarak bir içindekiler
tablosuna dönüşür.

**Kod tabanı yoksa** (proje henüz yazılmadıysa) "Önce keşif" bölümünü
şununla değiştirin:

```
Kod yok. AskUserQuestion aracıyla beni sorgula: sistem ne yapacak,
hangi problemi çözecek, naif yaklaşım neden yetmiyor, tipik bir işlem
hangi adımlardan geçecek. Yeterince anlayana kadar devam et, sonra yaz.
```

**Yenileme.** Mimari değiştiğinde yeniden üretin, elle yamamayın —
yamalanan genel bakış dokümanları hızla tutarsızlaşır.
