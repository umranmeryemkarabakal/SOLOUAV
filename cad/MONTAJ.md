# Tilt-rotor CAD montajı — eklemler ve hareket

`step/parts/*.step` içindeki 20 parça **mutlak konumda** üretilmiştir: hepsini
"orijine göre yerleştir" seçeneğiyle içe aktarırsanız araç kendiliğinden doğru
biçimde oturur. Sonrasında yalnızca eklemleri tanımlamanız yeterli.

Koordinat sistemi SDF gövde çerçevesidir: **+x burun, +y sol kanat, +z yukarı**,
başlangıç `base_link`. Birim **mm**. (SDF'teki `<model><pose>` z = 246 mm yer
ofseti uygulanmamıştır; o yalnızca aracı Gazebo zemininin üstüne kaldırır.)

## Eklem tablosu

Tümü döner (revolute) eklem. Eksen vektörleri gövde çerçevesinde birim vektördür.

| Eklem | Gövde | Menteşe noktası [mm] | Eksen | Limit | SDF adı |
|---|---|---|---|---|---|
| Sağ tilt | `motor_0_right` + `rotor_0_right` | (220, −250, 85) | (0, 1, 0) | 0° … 90° | `motor_0_joint` |
| Sol tilt | `motor_1_left` + `rotor_1_left` | (220, 250, 85) | (0, 1, 0) | 0° … 90° | `motor_1_joint` |
| Kuyruk tilt | `motor_2_tail` + `rotor_2_tail` | (−650, 0, 45) | (0, 1, 0) | **0° … 20°** | `motor_2_joint` |
| Sağ pervane | `rotor_0_right` | (220, −250, 110) | (0, 0, 1) | serbest (CCW) | `rotor_0_joint` |
| Sol pervane | `rotor_1_left` | (220, 250, 110) | (0, 0, 1) | serbest (CW) | `rotor_1_joint` |
| Kuyruk pervanesi | `rotor_2_tail` | (−650, 0, 70) | (0, 0, 1) | serbest (CCW) | `rotor_2_joint` |
| Sol elevon | `elevon_left` | (−180, 600, −5) | (−0.2619, 0.9651, 0) | ±44.7° | `left_elevon_joint` |
| Sağ elevon | `elevon_right` | (−180, −600, −5) | (0.2619, 0.9651, 0) | ±44.7° | `right_elevon_joint` |
| Sol elevatör | `elevator_left` | (−765, 150, −40) | (0, 1, 0) | ±29.8° | `left_elevator_joint` |
| Sağ elevatör | `elevator_right` | (−765, −150, −40) | (0, 1, 0) | ±29.8° | `right_elevator_joint` |
| Rudder | `rudder` | (−965, 0, 120) | (0, 0, 1) | ±29.8° | `rudder_joint` |

**Tilt ekseni +y yönündedir**, yani 0° = pervane ekseni yukarı (hover),
90° = pervane ekseni ileri (seyir). Kanat tiltleri 0…**89.95°** (SDF'te
`1.57 rad`).

⚠ **KUYRUK TİLTİ FARKLI: 0…20° (2026-08-30, adım 133).** Bu bir yazılım
tercihi değil, **fiziksel bir kısıt**: 100 mm yarıçaplı disk 90°'de alt ucu
`motor_z − 100 = −55 mm`'ye iner ve kuyruk çubuğunun İÇİNDEN geçer (ölçüldü:
`check_model_clearance.py`). Alternatif — motoru yükseltmek — denendi ve geri
alındı, çünkü `r_z` geri geçişte `τ_y = r_z·Fx − r_x·Fz` üzerinden ısırıyor ve
üç rotor da doydu (BIG_M 0 → 3843). 20°, tam görevde ölçülen en büyük kuyruk
tiltinin (2,5°) **8 katı**; o açıda çubuğa açıklık 24 mm.
Fusion'da bu limiti kurarken 90° yazmayın — montaj çakışır.

**Elevon menteşe ekseni saf y değildir.** SDF'te elevon eklemlerinde
`use_parent_model_frame` yoktur, dolayısıyla `<xyz>0 1 0</xyz>` ekseni eklemin
kendi çerçevesinde okunur ve eklemin `<pose>` künyesindeki ±0.265 rad (15.18°)
sapma açısı eksene uygulanır — menteşe, süpürülmüş firar kenarını izler.
Diğer tüm eklemlerde `use_parent_model_frame` 1'dir, eksenler doğrudan gövde
çerçevesindedir.

## Tilt gövde grupları

Tilt eklemi motoru döndürür, pervane de motorun çocuğudur. Fusion'da bunu
kurmanın en kolay yolu, önce pervaneyi motora bağlayıp sonra motoru gövdeye
bağlamaktır:

```
base_link (wing + winglet + boom + tailplane + fin + pylon + strut
           + 3 inis ayagi, sabit)
└── motor_0_right      ← tilt eklemi, (220, −250, 85), y ekseni, 0…90°
    └── rotor_0_right  ← döner eklem, (220, −250, 110), z ekseni, serbest
```

Sol grup aynıdır. **Kuyruk grubu limit bakımından farklıdır: 0…20°**
(yukarıdaki uyarı).

## Fusion 360

1. `Insert > Insert McMaster / Upload` yerine **File > Open** ile
   `step/tiltrotor_assembly.step` — **23 gövde** tek montaj olarak gelir.
   Alternatif: `step/parts/*.step` dosyalarını tek tek `Insert > Insert Derive`.
2. Gövde eklemleri (kanat, winglet, boom, tailplane, fin, pylon, strut,
   **3 iniş ayağı**) için **Rigid Group** oluşturun. Bunların hiçbiri hareketli
   değildir; yalnızca hareketli 11 parça (3 motor + 3 pervane + 5 yüzey) eklem
   alır.
3. Her hareketli parça için `Assemble > Joint > Revolute`:
   - `Motion > Rotate` eksenini yukarıdaki tabloya göre seçin,
   - eklem konumunu tabloda verilen noktaya taşıyın (Joint origin > Between two
     faces yerine `Offset` alanlarına mm değerlerini yazmak en hızlısı),
   - `Joint Limits` altına min/max açıyı girin.
4. Tilt hareketini görmek için eklemi sağ tıklayıp **Drive Joints**.

## SolidWorks

1. `File > Open > tiltrotor_assembly.step`, "Import as **assembly**" seçin
   (multibody part değil — o hâlde eklem tanımlayamazsınız).
2. Gövde parçalarını **Fix** yapın.
3. Hareketli parçalara `Mate > Mechanical > Hinge` veya eksenel `Concentric` +
   `Coincident` çifti; eksen için tablo değerlerine göre bir **Reference Axis**
   oluşturun (`Features > Reference Geometry > Axis`).
4. Açı sınırları için `Advanced Mates > Angle` mate'ine min/max girin.

## Doğrulama

`onizleme.png` üç görünüşü gösterir. Ana ölçüler:

| Ölçü | CAD | SDF/mesh kaynağı |
|---|---|---|
| Kanat açıklığı (winglet dahil) | 2157 mm | mesh zarfı 2151 mm |
| Kanat açıklığı (winglet hariç) | 2035 mm | — |
| Kök kirişi (y=0 kesiti) | 776 mm | mesh kesiti 776 mm |
| Kanat gövdesi x uzanımı | 872 mm | süpürme dahil |
| Toplam uzunluk | 1387 mm | burun +530, rudder firar −855 |
| Pervane çapı | 259 mm | mesh 258.8 mm — SDF collision da bu değere düzeltildi (adım 142) |
| Tilt rotor istasyonu | x=220, y=±250 | SDF `motor_0/1` pose |
| Elevon açıklığı (her biri) | 485 mm | `x8_elevon_*.dae` |
| **Kuyruk çubuğu uzanımı** | **−990 … −190 mm** | 2026-08-31, fin'i tam destekler |
| **Fin (dikey stab.) x** | **−980 … −820 mm** | fin çubuğun ÜZERİNDE oturur |
| **Rudder x** | **−1015 … −960 mm** | fin ile birlikte geriye alındı |
| **Ön pilon x** | **165 … 270 mm** | öne uzanan kol (adım 143) |
| **Ön motor konumu** | **x=270, y=±350 mm** | nasel hücum kenarının önünde |
| **İniş ayağı ucu z** | **−200 mm** | gövde alt yüzünden 40 mm aşağıda |


## 2026-08-31 geometri güncellemesi

Bu CAD, `tiltrotor_tailplane_model.sdf`'in **31 Ağustos 2026 hâlinden**
üretilmiştir. Önceki sürümden farkları — hepsi ölçülmüş bir çakışmayı çözer
(`check_model_clearance.py`, rapor Adım 132-139):

| parça | önce | sonra | neden |
|---|---|---|---|
| ön pilonlar | 50 mm boy, x=220 | **30 mm, x=205** | tam tiltte (90°, seyir duruşu) disk KENDİ pilonunu süpürüyordu |
| fin | x=−740 | **x=−900** | disk tilt=0'da, yani hover boyunca fininin içinden geçiyordu |
| rudder | x=−830 | **x=−990** | fin ile birlikte (menteşe havada kalmasın) |
| kuyruk çubuğu | 620 mm, 40 mm kalın | **800 mm, 20 mm** | fin desteklensin + disk çubuğu kesmesin |
| kuyruk tilt limiti | 0…90° | **0…20°** | fiziksel kısıt, yukarıdaki uyarı |
| iniş takımı | **yoktu** | **3 ayak** | araç gövde levhasıyla yere oturuyordu |
| **ön motorlar** | x=220, y=±250 | **x=270, y=±350** | disk kanadın gövde podunun içinden geçiyordu (adım 143) |
| **ön pilonlar** | 30 mm kutu, x=205 | **105 mm kol, x=217,5** | motor artık hücum kenarının önünde, pilon öne uzanıyor |

**Çakışma durumu: 5 → 0.** En dar açıklık 24,2 mm (kuyruk çubuğu, tilt 20°).
Ön rotorlar `cad/dogrula.py` katı denetiminde **ilk kez tüm tilt açılarında temiz**
(0°–90°); mesh açıklığı 32,8 mm.

⚠ **PERVANE YARIÇAPI DÜZELTİLDİ (adım 142).** SDF'in rotor collision silindiri
100 mm yarıçapındaydı, oysa gerçek pervane mesh'i (`iris_prop_*.dae`)
**129,4 mm**. Yani disk fiziksel olarak olduğundan küçük görünüyordu ve
`check_model_clearance.py` SDF'ten okuduğu için aynı hatayı tekrarlıyordu —
yapılan her açıklık ölçümü **29 mm iyimserdi**. Bunu `cad/dogrula.py` mesh'ten
ölçüp yakaladı; fin bu yüzden −860'tan −900'e alındı.

⚠ Kuyruk motoru z=45 mm'de KALDI. Bir ara 135 mm'ye yükseltilmişti (9 cm
pilon) ve **geri alındı**: `r_z` geri geçişte kontrol otoritesini bozuyordu.
Eski bir CAD sürümünde o pilonu görürseniz, güncel olan bu değildir.


## Adım 143 — ön naseller hücum kenarının önüne alındı (31 Ağustos 2026)

Ön motorlar **x=220 → 270 mm, y=±250 → ±350 mm** taşındı; pilon 30 mm'lik bir
kutudan **105 mm'lik öne uzanan bir kola** dönüştü. Motor artık kanadın hücum
kenarının (o istasyonda x=+202 mm) **68 mm önünde** duruyor — gerçek
tiltrotorların nasel düzeni. `r_z` (motor z=85 mm) **değişmedi**.

### Neden zorunluydu

Pervane diski, kanadın **gövde podunun** içinden geçiyordu. İki ayrı ölçüm
hatası bunu uzun süre gizledi:

1. **SDF'in collision kutusu kanadı temsil etmiyor.** Kutu `0,55 × 2,144 ×
   0,05` m ve yalnızca `z = −160…−110 mm` aralığında; gerçek mesh ise
   `z = −100…+208 mm`. Pod (yarım genişlik 200 mm, x=+288 mm'ye kadar öne
   uzanıyor) kutuda **hiç yok**. Kutuya bakan her ölçüm 65 mm açıklık
   raporluyordu, gerçek 0 mm'ken.
2. **Mesh'in köşelerine bakmak da yetmiyor.** İlk mesh kontrolü köşe
   noktalarına uzaklık ölçüyordu; bu mesh'te köşe aralığı ~21 mm, disk yüzeye
   köşeler arasından girdiğinde ölçüm "32,8 mm temiz" diyordu. `cad/dogrula.py`
   gerçek katı boolean'ı yaptığı için baştan beri nüfuz raporluyordu ve **haklıydı**.

`check_model_clearance.py` artık üçgenleri alanla orantılı örnekliyor (~1,5 mm
yoğunluk) ve diskin 10,6 mm kalınlığını hesaba katıyor. Eski konumu ⛔ olarak
yakaladığı bir regresyon sınamasıyla doğrulandı.

### Neden başka çözüm yok

90°'de disk **dikey bir düzlem**. Göbek yerel hücum kenarının gerisindeyse
kanadın kirişini keser. Süpürme yüzünden dışa gitmek x'i geriye zorlar, bu
yüzden pilon kanadın üstünde kaldığı sürece **açıklık boyunca hiçbir istasyon
çalışmıyor** (y=350…700 mm'nin tamamında 0,0 mm).

| seçenek | sonuç |
|---|---|
| Pervaneyi küçült | **işe yaramıyor** — 170 mm çapta bile 0,0 mm; sorun diskin yarıçapı değil *düzlemi* |
| Açıklıkta dışa al | **işe yaramıyor** — süpürme x'i geriye zorluyor |
| Tilt'i sınırla | **ölü** — sabit kanat 90° gerektiriyor (55°'de temizleniyor) |
| Pilonu yükselt | çalışır ama **+80 mm** gerekir; `r_z`'yi değiştirir, ki bu Adım 132'de geri geçişi bozan büyüklüğün ta kendisi |
| **Naseli öne al** | **seçildi** — 32,8 mm, `r_z` sabit |

### Kontrol bedeli

`arm_ratio = 2·ROTOR_PX[0]/|ROTOR_PX[2]|` 0,677 → **0,831**. Hover bölüşümü
buna göre kayıyor; `TiltrotorIndiControl.hpp` içinde otomatik türetiliyor,
elle güncellenecek ayrı bir sabit yok. Kuyruk kolu ve `r_z` değişmedi.

## Adım 154 — kuyruk kolu, fin ve strut (31 Ağustos 2026)

CAD, SDF'in Adım 154 hâline eşitlendi. Değişenler:

| parça | önce | sonra | neden |
|---|---|---|---|
| kuyruk motoru/rotoru | x=−650 | **x=−550** | kuyruk trim payı %29 → %33; iniş sıçramasında kuyruk 0 N'e dayanıyordu |
| fin | x=−900, z=120 | **x=−780, z=100** | öne alındı (kullanıcı kararı); z indirildi çünkü **20 mm havada asılıydı** |
| rudder | x=−990, z=120 | **x=−870, z=100** | fin ile birlikte |
| çubuk | 800 mm, merkez −590 | **710 mm, merkez −545** | fin öne gelince arka uzantı gereksizleşti |
| tailplane strut | z merkez −15, boy 60 | **z merkez −22,5, boy 45** | üst yüzü çubuğun 15 mm üstüne taşıyordu; fin inince **4,9 cm³ girişim** oluştu |

**Strut girişimini `cad/dogrula.py` yakaladı**, SDF denetimleri değil. Strut'un
tek işi tailplane'i çubuğa bağlamak; çubuğun üstüne taşmasının işlevi yoktu.
Üst yüzü çubuğun üst yüzüyle (z=0,000) hizalandı.

**Fin neden 20 mm havadaydı:** bütün açıklık denetimleri **çakışma** arıyordu,
hiçbiri **boşluk** aramıyordu. Artık `check_model_clearance.py` içindeki
`connectivity_audit()` köke ulaşılabilirlik arıyor.

Doğrulama: **61 geçti, 0 CAD hatası**, MATLAB 8/8, tam otonom görev SITL'de
10 evrenin 10'u (157 s).
