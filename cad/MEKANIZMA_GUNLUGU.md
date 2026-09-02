# Mekanizma çalışması — günlük

Bu dosya, CAD modelini *simülasyon geometrisi* olmaktan çıkarıp *imal
edilebilir bir montaja* yaklaştırma çalışmasının kaydıdır. Kontrol
tarafındaki `WLS_LOCKUP_INVESTIGATION_REPORT.md` ile aynı işlevi görür:
ne denendi, ne ölçüldü, ne geri alındı.

**Kural:** buradaki her sayı bir ölçümden gelir. Tahmin edilen değer girmez.
Geri alınan yollar ⛔ ile işaretlidir ve tekrar denenmemelidir.

---

## ► Durum (1 Eylül 2026)

| Alan | Durum |
|---|---|
| İki aşamalı üretim boru hattı | ✅ çalışıyor (`build_tiltrotor_cad.py` → `build_mechanism.py`) |
| Mekanizma kapısı | ✅ **61 geçti, 2 uyarı, 0 HATA** |
| Ayrıştırıcı paritesi | ✅ 11/11 |
| Tilt mekanizması (3 nacelle) | ✅ kuruldu, süpürme sınandı |
| İniş takımı bağlantısı | ✅ 3 bacak da gövdeye bağlı |
| Elevon menteşe hattı | ✅ boşluk 11 mm → **0,17 / 0,56 mm** |
| Kuyruk menteşe hattı | 🔴 **11,0 mm boşluk — AÇIK** |
| Kontrol kodu / `model.sdf` | ✅ **hiç el değmedi** (md5 doğrulandı) |

**Kritik kısıt:** bu çalışmanın tamamı CAD tarafında kaldı. `model.sdf`,
MATLAB dosyaları ve PX4 C++ portu oturum boyunca md5 ile izlendi ve
değişmedi. Mekanizma parçalarının hiçbiri SDF'te karşılığı olan bir sayıyı
tekrarlamaz; eksenler `fusion_02_joints.py`'nin ayrıştırıcısı ödünç alınarak
`model.sdf`'ten okunur.

---

## 1. Yapılanlar ve ölçümleri

### 1.1 Tilt mekanizması
Üç nacelle için: çatal (yatak bosslu), MR105ZZ yatak çifti, motor kelepçesi,
krank, servo, horn, itki çubuğu. Kuyrukta ayrıca taşıyıcı direk — çünkü
`motor_2_tail` **hiçbir şeye değmiyordu**, boom'un 27,5 mm üstünde
boşluktaydı.

Tasarımın dayandığı geometrik kural: tilt ekseni disk düzlemine paralel ve
25 mm altında, bu ilişki dönme altında korunuyor, dolayısıyla disk hiçbir
açıda eksene 25 mm'den yakınlaşamaz. İki güvenli bölge: **eksenin 25 mm
yakını** ve **eksenin arkası+altı** (orada `a·sinθ + b·cosθ = 25`
denkleminin θ∈[0°,90°] için çözümü yok). Bütün sabit parçalar bu iki
bölgeye hapsedildi.

Süpürme sınaması: kanatlar 0→90° yedi açı, kuyruk 0→20° beş açı. Sonuç
temiz. Sınama bir **gerçek hata** yakaladı: motor her açıda pylon'un
içinden geçiyordu (mevcut modelden geliyordu, eklediğim şeyden değil);
pylon'lara süpürme boşluğu açıldı (189 → 182,5 cm³).

### 1.2 İniş takımı
Üç bacak da gövdeden **~133 mm uzakta boşluktaydı** (ilk tahmin 48 mm'ydi;
o rakam kanadın *global* en alt noktasına göreydi, bacakların kendi
istasyonuna göre değil). Ön bacaklara dikme + montaj plakası, kuyruğa
boom'u saran eyer kelepçe eklendi. Sınama sırasında kuyruk dikmesinin
tailplane'in içinden geçtiği yakalandı ve eyer hücum kenarının önüne
kaydırıldı.

### 1.3 Kumanda yüzeyleri
Beş yüzeye menteşe pimi + korna + servo + itki çubuğu.

**Elevon menteşe hattı** — şikâyet: "yapboz gibi oturmuyor". Sebep bendeydi:
elevonun burnu silindirik olmadığı için (ölçüm: yarıçap **2–11,6 mm**
arasında geziyor) ±44,7°'de sıyırmaması ancak yarıçapı o azami değere kadar
büyük bir ceple mümkündü, ince elevon o cebi doldurmuyordu. Çözüm gerçek
kumanda yüzeylerindeki gibi: burun menteşe eksenine göre **dolu yarım
silindir** yapıldı (kesmek yetmez — sivri burun silindirin içinde kalıyor,
o hacmi *eklemek* gerekiyor). Yarıçap iki taraftan küçüğüne göre alınır:
eksen tam orta kalınlıkta değil (elevon z −11,5…0, orta −5,75; eksen −5),
büyük tarafı almak silindiri üst yüzeyden 1,5 mm dışarı taşırıyordu.

Sonuç (BRepExtrema asgari mesafe): **0,169 / 0,563 mm**.

**Kuyruk servoları** — tailplane servo istasyonunda **9,9 mm**, fin
**16,5 mm** kalın; 22 mm'lik servo sığmıyor. Boom'a gömüldüler (40×20 mm
kesit, servo yan yatırılınca sığıyor ve mil ekseni zaten y). Rudder'ın mil
ekseni z olduğu için 22 mm > 20 mm; altına küçük bir servo karinası eklendi.
Barınma: elevatör servoları %90 boom içinde (dışarıda kalan mil + horn,
tasarım gereği), rudder servosu karinayla birlikte %99.

### 1.4 Kuyruk çubuğu
Boom gövdeye yalnızca **~56 mm** giriyordu (kanat merkez hattında x≈−270'te
bitiyor, boom −190'dan başlıyor), kalan ~630 mm desteksiz konsoldu.
Kanadın firar kenarından boom'a V-destek eklendi; konsol ~370 mm'ye indi.

### 1.5 Aerodinamik gövdeler
- **Pervane dalgalanması:** 61 mesh kesiti loft'un tam içinden geçtiği için
  istasyon gürültüsü doğrudan yüzeye yansıyordu. `smooth_stations` ile
  veter dizisindeki yön değişimi **15/59 → 3/59**.
- **Dış pala kesitleri:** `iris_prop_cw.dae` bir *görsel* mesh; dış palada
  kabuk 0,2 mm'ye, uçta 0,01 mm'ye iniyor ve dilimleyince kontur kendi
  üstüne katlanıyor (uç yüzeyi alanı, sınırlayıcı kutusunun **%3-6**'sı;
  profilde ~%65). `rebuild_thin_sections` ölçülen veter/burulma/merkezi
  korur, konturu elipsle ve kalınlığı 1,2 mm imalat alt sınırıyla yeniden
  kurar. Bozuk istasyon **18 → 0**.
- **Pala ucu:** loft son istasyonda bitince OCC ucu düz kapatıyordu.
  `round_blade_tips` çeyrek elips kapak koyar; kapak **dışarı eklenmez**,
  içeri oturtulur (yoksa çap 256,7 → 264,2 mm büyür).
- **Kenar tırtıklanması:** kesit noktası 40 → **120** (yüz sayısı 42 → 122).
  Spline tel denendi, OCC'de çöktü.
- **Winglet taşması:** gövde 207,2'de bitmesi gerekirken 211,2 mm'ye
  uzanıyordu; o uzantı kendi kendini kesen ince bir dilimdi (üst 8 mm'de
  dilim genişlikleri `0,0 111,8 0,0 0,0 0,0 110,4 109,3 0,0`).
  `close_winglet_tip` ile zmax **207,7 mm**, dilimler tek yönlü.

---

## 2. Açık kalanlar

### 2.1 🔴 Kuyruk menteşe hattında 11 mm boşluk
Ölçüm: elevon↔kanat 0,17/0,56 mm (doğru), elevatör↔tailplane ve rudder↔fin
**11,0 mm**. Sebep: cep 6 mm yarıçaplı yani **12 mm çaplı**, tailplane ise
**12 mm** kalın — silindir levhayı boydan boya kesiyor, firar şeridi
kopuyor, kopan parça düşüyor, gövde eksenden 16 mm geriye çekiliyor
(bbox x −780 → −749,2). Fin'de (20 mm) aynı mekanizma.

**Reçete:** iki kısıt AYNI ANDA sağlanmalı — cep yarıçapı ana gövdenin
yarım kalınlığından küçük (kopmasın) VE burun yarıçapı + kaçıklık payından
büyük (çakışma kalmasın). Tailplane'de yarım kalınlık 6 mm olduğundan:
**burun 4,5 mm, cep 5,5 mm, artı açıklıkla sınırlı arka kırpma.** Her
adımdan sonra `dogrula_mekanizma.py`.

### 2.2 Elevon asimetrisi (8 mm)
Sağ elevon açıklıkta 8,0 mm, veterde 0,9 mm kaymış; hacimler 125,28 /
126,08 cm³. Kaynağı `build_tiltrotor_cad.py`'deki `ELEVON` tablosu: iki poz
SDF'ten **bağımsız** alınıyor, aynalanarak üretilmiyor. Kanat ve winglet'ler
aynalama işleminden geçmiş, elevonlar geçmemiş tek çift.

### 2.3 Pervane geçiş basamağı
Yeniden kurulan elips ile ölçülen kamburlu profil arasındaki geçişte alan
30,1 → 23,5 diye bir basamak kalıyor. İnip çıkan V çentiği giderildi (alan
sürekliliğinden kalınlık türetilerek) ama tam pürüzsüz değil. Çözümü elipsi
kamburlu bir profille değiştirmek.

### 2.4 Poligon kesit çizgileri
Yalnızca pervane n=120'ye çıkarıldı. Kanat (n=64), winglet (n=48) ve
elevonlar hâlâ düşük nokta sayısında; yüzeyde boyuna çizgiler görünüyor.

### 2.5 Fusion ↔ STEP eşitsizliği
Fusion dokümanındaki mekanizma parçalarının bir kısmı `fusion_execute` ile
kurulmuş eski sürüm; üreticinin ürettiğiyle ufak farklar var (elevatör
servolarının yeri, kuyruk dikmesi). Boş bir Design'a 61 parçayı baştan
aktarmak gerekiyor. Bu ayrışma ileride yanıltıcı olabilir: baktığın şeyle
depodaki şey aynı değil.

---

## 3. ⛔ Denenip elenenler — tekrar denemeyin

| Ne denendi | Ölçülen sonuç |
|---|---|
| Pala ucuna küçültülmüş sentetik kesit koymak | ovallik 222× → 78×, ama katı **GEÇERSİZ** |
| Dejenere istasyonları kırpmak (eşik 0,15/0,30/0,45/0,60) | yalnız 0,15 geçerli; eşiğe göre gidip gelmesi OCC'nin sınırda çalıştığını gösterir |
| `fallback=False` + istasyon aralığını içeri çekmek | çoğu geçersiz; 5 mm'de hacim **−723174 cm³** |
| Katıyı düzlemle kesip sağlam kesitte bitirmek | geçerli ama uç dolgu oranı 0,06'da kaldı |
| Pervanede spline kesit teli | OCC çöküyor (`BSplCLib::Interpolate`, sonra `BRep_API: command not done`) |
| Elips ↔ ölçülen profil geçişini harmanlamak | alan 45,3 → 32,5 → 24,7 → **21,0** (derin çukur); harmansız hali daha iyi |
| Winglet taşmasını boolean ile kırpmak | `Bnd_Box is void` ile çöküyor |
| Menteşe arkasını **sınırsız** kutuyla kırpmak | kanat yıkılıyor: 29465 → **18030 cm³** |
| Cep yarıçapını yarım kalınlıkla sınırlamak (tek başına) | kopmayı çözüyor ama çakışma kalıyor: elevatör∩tailplane **4,69 cm³**, rudder∩fin **3,32 cm³** — 11 mm boşluktan kötü |
| Burnu "azami yarıçap" silindiriyle tıraşlamak | tanım gereği **işlemsiz** (hacimler hiç değişmedi) |

---

## 4. Araç tuzakları

- **Aşama 2 idempotent değil.** `build_mechanism.py` kendi çıktısının
  üzerine ikinci kez koşulamaz; zaten burnu tamamlanmış yüzeye yeniden burun
  eklemeye çalışıp `Null TopoDS_Shape` ile çöker. `git checkout -- cad/step/`
  ile "temiz duruma dönmek" tam bu tuzağa düşürür: depodaki STEP'ler Aşama
  2'nin **çıktısıdır**, girdisi değil. Bu oturumda kuyruk düzeltmesinin
  birkaç denemesi bu yüzden yanlış yere teşhis edildi.
- **`build_tiltrotor_cad.py` aralıklı segfault veriyor** (bu oturumda iki
  kez). Art arda OCC koşularının bellek baskısı olabilir. Segfault'tan sonra
  gövde parçaları eksik üretilir ve Aşama 2 onların üzerine koşarsa ölçümler
  yanıltıcı çıkar — her koşudan sonra Aşama 1'in "yazıldı" satırını görün.
- **Fusion'ın boolean ve `pointContainment` sorguları `wing` gövdesinde
  güvenilmez.** 60 mm küp %100 içeride derken 80 mm küp %0,2 diyor
  (80'lik 60'lığı kapsadığı için imkânsız); `pointContainment` sınırlayıcı
  kutunun en uç köşesini bile "içeride" diyor. Kanadı içeren her kontrol
  `dogrula.py` / `dogrula_mekanizma.py` ile yapılmalı.
- **`fusion_screenshot` bozuk** (köprünün kendi hatası:
  `Viewport.saveAsImageFileWithOptions` argüman uyuşmazlığı).
- **Ölçüt seçimi dersi.** Bu oturumda birkaç kez "düzeldi" denip yanlış
  çıktı; ortak sebep ölçütün dar seçilmesiydi (tek bir yüz, girdi verisi,
  yalnız ana gövde). Kesin sonuç `BRepExtrema` ile **iki katı arasındaki
  asgari mesafe** ölçülünce geldi. Geometrik "oturuyor mu" sorusunun doğru
  ölçütü budur; hacim ya da yüz alanı değil.
