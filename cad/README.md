# Gazebo modelinden CAD (STEP)

`tiltrotor_indi` Gazebo modelini SolidWorks / Fusion 360'ta açılabilen
parametrik katı gövdelere çevirir.

```
cad/
├── build_tiltrotor_cad.py   üretici betik (cadquery)
├── sdf_mesh.py              COLLADA okuma + düzlem kesiti + loft hazırlığı
├── dogrula.py               bağımsız denetleyici — STEP çıktısını SDF'e karşı sınar
├── dogrulama_raporu.txt     son denetim çıktısı (dogrula.py üretir)
├── render_onizleme.py       üç görünüş önizlemesi üretir
├── onizleme.png             üç görünüş doğrulaması
├── build_mechanism.py       mekanizma üreticisi (tilt grubu, iniş takımı, menteşe/tahrik)
├── dogrula_mekanizma.py     mekanizmanın bağımsız denetleyicisi
├── fusion_01_import.py      Fusion 360'a parçaları içe aktarma betiği (Aşama 1)
├── fusion_02_joints.py      11 revolute eklemi SDF'ten kurma betiği (Aşama 2)
├── dogrula_fusion_parser.py Aşama 2'nin SDF ayrıştırıcısını rapora karşı sınar
├── MONTAJ.md                eklem tablosu, montaj adımları, imalat kısıtları
└── step/
    ├── tiltrotor_assembly.step   23 gövdenin tamamı (tek dosya)
    └── parts/*.step              parça başına ayrı dosya
```

## Neden yeniden inşa, neden doğrudan dönüşüm değil

SDF bir CAD dosyası değildir. Modelde geometri iki biçimde duruyor:

- **üçgen mesh** (`x8_wing.dae`, `x8_elevon_*.dae`, `iris_prop_*.dae`) — Fusion'a
  "mesh body", SolidWorks'e "graphics body" olarak girer; kalınlık profili,
  menteşe hattı, parametre taşımaz;
- **kutu ve silindir** (kuyruk kirişi, tailplane, fin, elevatör, rudder, pylon,
  motor) — atalet ve çarpışma için konmuş kaba hacimler.

Bu yüzden dosya "çevrilmedi", ölçüleri korunarak **yeniden inşa edildi**:

| Parça | Yöntem |
|---|---|
| `wing` | `x8_wing.dae`'den 60 açıklık istasyonunda gerçek kesit alınıp loft |
| `winglet_left/right` | Uç bölgeden 20 düşey (z) kesit, loft |
| `elevon_left/right` | `x8_elevon_*.dae`'den 14 kesit, loft |
| `rotor_*` | `iris_prop_cw.dae`'den 61 kesit, loft — pala burulması korunur |
| `tailplane`, `vertical_stabiliser` | SDF kutu zarfına oturtulmuş NACA simetrik profil |
| `elevator_*`, `rudder` | Yuvarlak hücum kenarlı, sivri firar kenarlı flap kesiti; menteşe ekseni yuvarlak burnun merkezinden geçer |
| `motor_*`, `pylon_*`, `tail_boom`, `tailplane_strut` | SDF'teki silindir/kutu ölçüleri birebir |

Kuyruk yüzeylerinin profili SDF'te **yok** — orada yalnızca kutu var. Kutunun
kiriş/açıklık/azami kalınlık zarfı korunarak içine simetrik profil oturtuldu;
bu bir varsayımdır, sim geometrisinden bir sapma değil ama ondan da fazlasıdır.

## Yeniden üretme — İKİ AŞAMA, sırası önemli

```bash
python3 -m venv env && ./env/bin/pip install cadquery
./env/bin/python cad/build_tiltrotor_cad.py    # 1) aerodinamik gövdeler (mesh ister)
./env/bin/python cad/build_mechanism.py        # 2) mekanizma (mesh İSTEMEZ)
```

**Sıra bağlayıcıdır.** Aşama 2, Aşama 1'in ürettiği STEP gövdelerini okur ve
bazılarını değiştirir (pylon'lara motor süpürme boşluğu, ana yüzeylere menteşe
boşluğu). Aşama 1'i tek başına koşmak bu boşlukları geri alır ve motor kendi
pilonunun içinden geçmeye başlar — `dogrula_mekanizma.py` bunu yakalar.

Mesh dizini varsayılan olarak
`~/PX4-Autopilot/Tools/simulation/gz/models/standard_vtol/meshes`;
`VTOL_MESH_DIR` ortam değişkeniyle değiştirilebilir. Mesh'ler PX4 ağacında
yoksa doğrudan indirilebilir:

```bash
BASE=https://raw.githubusercontent.com/PX4/PX4-gazebo-models/main/models/standard_vtol/meshes
for M in x8_wing x8_elevon_left x8_elevon_right iris_prop_cw iris_prop_ccw; do
  curl -sL -o "$M.dae" "$BASE/$M.dae"
done
```

### Mekanizma neden ayrı bir betikte

`build_mechanism.py`'nin ürettiği hiçbir parça mesh'e ihtiyaç duymaz —
hepsi SDF'ten türeyen eksenler etrafında kutu ve silindirdir, menteşe
boşlukları da mevcut STEP gövdelerinden kesilir. Böylece mesh'i olmayan bir
makinede de mekanizma yeniden üretilebilir ve denetlenebilir.

Menteşe sayıları burada da tekrarlanmaz: betik `fusion_02_joints.py`'nin
ayrıştırıcısını ödünç alır (`dogrula_fusion_parser.py` ile aynı teknik).

Kaynak model: `Tools/simulation/gz/models/tiltrotor_indi/model.sdf`
(depodaki `tiltrotor_tailplane_model.sdf` ile birebir aynı).

## Fusion 360'a aktarma

```
Fusion 360 > Utilities > Scripts and Add-Ins > Scripts > + > cad/fusion_01_import.py
```

Boş bir Design açıp çalıştırın. Betik 23 parçayı ayrı bileşen olarak içe
aktarır ve 12 sabit parçayı `base_link` adlı Rigid Group'a alır. Parçalar
mutlak konumda üretildiği için occurrence dönüşümü birim matris kalır — araç
kendiliğinden doğru oturur, hizalama gerekmez.

**Parça klasörü otomatik bulunur.** Sırayla denenen yerler: `TILTROTOR_CAD_PARTS`
ortam değişkeni, `~/Documents/tiltrotor_cad/step/parts`,
`~/tiltrotor_project_updates/cad/step/parts`, `~/SOLOUAV/cad/step/parts`.
Hiçbiri tutmazsa betik durur ve denediği yolları listeler; o zaman dosyanın
başındaki `BASE` değişkenine tam yolu yazın.

Betik iki çalıştırma yolunu da destekler: Fusion'ın Scripts paneli `run(context)`
çağırır; MCP köprüsünün `fusion_execute`'u ise kodu düz `exec` eder ve `run()`'ı
kimse çağırmaz — dosyanın sonundaki koruma bu ikinci durumu yakalar.

Başlamadan önce üç şeyi kontrol eder ve gerekirse durur: klasör bulundu mu, 23
STEP dosyasının hepsi yerinde mi, bu Design'da aynı adlı bileşenler zaten var mı
(ikinci kez çalıştırma kopya üretmesin diye).

### Aşama 2 — eklemler

```
Scripts > + > cad/fusion_02_joints.py      (Aşama 1'den sonra, aynı Design'da)
```

11 hareketli parça için revolute eklem kurar: 3 tilt (kanat rotorları 0…90°,
kuyruk **0…20°**), 3 serbest pervane, 5 kumanda yüzeyi (elevon ±44,7°,
elevatör ve rudder ±29,8°).

**Menteşe sayıları betiğe yazılmaz — `model.sdf`'ten okunur.** Bu depo aynı
sayının birden çok yerde ayrışmasından zarar gördü; eklem tablosunu dördüncü bir
kopya olarak tutmak aynı hatayı tekrarlamak olurdu. Betik SDF yolunu da otomatik
arar (`TILTROTOR_SDF` env değişkeni, sonra parça klasörüyle aynı adaylar).

Kurmadan önce kontrol eder: SDF bulundu mu, 11 eklem çıktı mı, gerekli
bileşenler var mı, bu Design'da zaten eklem var mı. Parçalar mutlak konumda
olduğu için menteşe noktası her iki bileşende de aynı yere inşa edilir ve eklem
kurulurken hiçbir şey yerinden oynamaz.

**Ayrıştırıcı ayrı bir kapıyla sınanır** — Fusion içinde cadquery
kullanılamadığı için betik SDF'i kendi regex'leriyle okur, yani ayrıştırma
mantığının ikinci bir kopyası vardır:

```bash
python3 cad/dogrula_fusion_parser.py     # bağımlılık yok, sistem python3'ü yeter
```

11 eklemin nokta/eksen/limit değerlerini `dogrulama_raporu.txt` [5] bölümüyle
karşılaştırır. Geliştirilirken iki **gerçek ve sessiz** hatayı yakaladı:
SDF'te link adları çift, eklem adları tek tırnaklı olduğu için elevatör ve
rudder menteşesi 790–870 mm kaymıştı; `<inertial>` içindeki poz kütle merkezi
olduğu halde link yerleşimi sanıldığı için elevon menteşesi tam 300 mm
kaymıştı. Kapının ikisini de yakaladığı, hatalar kasten geri konularak
doğrulandı.

## Bilinen sapmalar

- **Winglet ucunda ~3.5 mm loft taşması** (z 211.2 mm, mesh zarfı 207.7 mm).
  Pürüzsüz loft uçta zarfın dışına çıkıyor; boolean kırpma bu gövdede kararsız
  çalıştığı için bırakıldı. Açıklığın %0.16'sı mertebesinde.
- **Winglet'ler kanatla tek gövdeye kaynatılmadı.** Tabanları kanat ucunun
  içine oturur (z = −5 mm) ama ayrı katı olarak kalır; OCC'de bu birleşme
  ya hacmi şişiriyor ya sıfıra çöküyordu. Montajda ikisini `Rigid Group`
  yapın, ya da CAD içinde birleştirin.
- **Kanat mesh'i 3.5 mm eksantrik** geliyordu (bbox y = −1071.9 / +1078.9);
  açıklık ortasına kaydırılıp sağ yarı sol yarının aynası olarak üretildi.
- **`base_link_collision` kutusu üretilmedi.** O, kanadın 135 mm altında duran
  bir çarpışma vekilidir, araca ait bir parça değildir.
- **Pervane çarpışma diski (r = 100 mm) ile görsel mesh (r = 129 mm) uyuşmuyor.**
  CAD'de görsel mesh esas alındı; SDF'teki çarpışma silindiri daha küçüktür.
- **CCW pervaneler CW mesh'inden aynalanarak üretildi.** `iris_prop_ccw.dae`
  kesitleri pala boyunca kendini kesiyor (61 kesitin 23'ü) ve loft geçersiz
  katı veriyor; `iris_prop_cw.dae` aynı ayarlarla temiz. CCW pervane zaten
  CW'nin aynasıdır; iki mesh arasındaki fark yalnızca pala faz açısıdır
  (bbox'ları ~4 mm kayık), o da dönen bir pervanede anlamsızdır.
- Pervanelerin **eksen etrafındaki başlangıç açısı** mesh'ten geldiği gibidir;
  SDF bir başlangıç açısı tanımlamıyor.
- **Pala ucu kesitleri dejenere** (1 Eylül 2026, AÇIK). `stations(fallback=True)`
  mesh'in kapanmadığı dış istasyonlarda zinciri zorla kapatıyor ve oradaki
  kesitler kendi üstüne katlanıyor. Ölçüm: uç yüzeyinin alanı kendi
  sınırlayıcı kutusunun yalnızca **%3-6**'sı (düzgün bir profilde ~%65).
  Dış 11+14 istasyon atıldığında bile oran değişmiyor, yani sorun yalnızca
  en uçta değil palanın **dış bölgesinin tamamında**. İstasyon seçimiyle
  ya da yumuşatmayla çözülmüyor; o bölgenin veter/kalınlık/burulma
  dağılımından **yeniden inşası** gerekiyor. Denenip elenen dört yol
  `build_tiltrotor_cad.py` içindeki "⛔ PALA UCU DEJENERASYONU" notunda.
- **Elevonlar birbirinin aynası değil** (1 Eylül 2026, AÇIK). İki elevonun
  pozu SDF'ten **bağımsız** alınıyor (`ELEVON` tablosu), aynalanarak
  üretilmiyor. Ölçüm: sağ elevon açıklıkta **8,0 mm**, veterde 0,9 mm kaymış;
  hacimler 125,28 / 126,08 cm³. Kanat için "sağ yarı solun aynası" işlemi
  uygulanmış, winglet'ler de simetrik (417,81 / 417,82 cm³) — elevonlar bu
  işlemden geçmemiş tek çift. Menteşe boşluğu yarıçaplarının asimetrik
  çıkmasının (20,09 / 25,19 mm) sebebi budur.

## Doğrulama

```bash
/path/to/cadquery-venv/bin/python cad/dogrula.py > cad/dogrulama_raporu.txt
/path/to/cadquery-venv/bin/python cad/render_onizleme.py     # onizleme.png
```

`dogrula.py` üretici betiğin tablolarını **kullanmaz**; SDF'i baştan ayrıştırır,
böylece elle aktarma hataları da yakalanır. Sekiz denetim: kapsam, ölçü, mesh
zarfı, katı sağlığı, menteşe, duruşta girişim, tilt süpürmesi, yüzey sapması.

Çıktı dört sınıfa ayrılır ve **yalnızca ilki bir CAD hatasıdır**:

| sınıf | anlamı |
|---|---|
| `HATA` | STEP çıktısı SDF ile uyuşmuyor — üretici betik düzeltilmeli |
| `MODEL` | CAD doğru, **SDF geometrisinin kendisi** çakışıyor |
| `UYARI` | çalışma aralığında kalan, incelenmesi gereken girişim |
| `limit dışı` | eklem `<upper>` sınırının ötesi; araç o açıya gidemez |

İki denetim **eşik seçimiyle** çalışır, ikisi de kasıtlı:

- **Tilt süpürmesi** eklem tavanını SDF'ten okur (`joints[jn]['upper']`). Kuyruk
  rotoru 30°'den sonra çubuğa girer ama tavan 20°'dir, dolayısıyla o açılar
  bulgu değil `limit dışı` sayılır. Satırlar yine basılır: limit ileride
  gevşetilirse bedelinin ne olacağı kayıtlı kalsın diye.
- **Yüzey sapması** mutlak hacimle değil **ortalama film kalınlığıyla** yargılar
  (`FILM_TOL = 0,05 mm`). Elevon kanatla aynı loft yüzeyinden kesildiği için
  ikisi duruşta zaten yüzey olarak çakışık; 0,5 m açıklıkta 0,01 mm'lik bir film
  bile 0,7 cm³ hacim yapar. Mutlak hacim eşiği bu yüzden büyük parçalarda
  ölçtüğü şeyi değil parçanın boyutunu yansıtır.

### Mekanizma kapısı

```bash
/path/to/cadquery-venv/bin/python cad/dogrula_mekanizma.py
```

`dogrula.py`'nin mekanizma karşılığı: üretici betiğin tablolarını kullanmaz,
STEP'leri ve SDF'i baştan okur. Beş denetim — katı sağlığı, tilt süpürmesi
(SDF limitleri boyunca), kumanda yüzeyi hareketi, iniş takımı bağlantısı,
duruşta istenmeyen girişim.

Geliştirilirken üç **gerçek** hata yakaladı: (1) pylon'lara süpürme boşluğu
açmayı unutmuştum, motor üç nacelle'de de kendi pilonundan geçiyordu;
(2) elevatör servoları `tailplane_strut`'ın içindeydi (5,24 cm³);
(3) kuyruk dikmesi tailplane'i kesiyordu. Ayrıca kapının kendisinde bir
boşluk vardı: yüzeyleri yalnızca ana gövdelerine karşı sınıyordu, o yüzden
elevatörün **kuyruk çubuğuna** çarpmasını ıskalıyordu — komşu gövdeler eklendi.

Son durum: **61 geçti, 2 uyarı, 0 HATA.** İki uyarı elevatörün +29,8°'de
kuyruk çubuğuna girmesidir; temiz sınır **~±25°**. Not: `MONTAJ.md`'deki
+26° mekanik durdurucu ölçümle 0,02 cm³ girişime izin veriyor.

---

Son durum (`dogrula.py`): **62 geçti, 2 uyarı, 0 CAD hatası, 0 kaynak model
bulgusu, 5 limit dışı.** 23 STEP dosyasının hepsi `BRepCheck_Analyzer` ile geçerli ve tek
kapalı katı. Kalan 2 uyarı elevatör–kuyruk çubuğu girişimidir; bilerek açık
bırakıldı ve montajda çözülecek (bkz. `MONTAJ.md`, imalat kısıtı bölümü).

Ana ölçülerin SDF/mesh karşılaştırması `MONTAJ.md` sonundadır.
