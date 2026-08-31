# Gazebo modelinden CAD (STEP)

`tiltrotor_indi` Gazebo modelini SolidWorks / Fusion 360'ta açılabilen
parametrik katı gövdelere çevirir.

```
cad/
├── build_tiltrotor_cad.py   üretici betik (cadquery)
├── sdf_mesh.py              COLLADA okuma + düzlem kesiti + loft hazırlığı
├── MONTAJ.md                eklem tablosu, Fusion/SolidWorks montaj adımları
├── onizleme.png             üç görünüş doğrulaması
└── step/
    ├── tiltrotor_assembly.step   20 gövdenin tamamı (tek dosya)
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

## Yeniden üretme

```bash
python3 -m venv env && ./env/bin/pip install cadquery
./env/bin/python cad/build_tiltrotor_cad.py
```

Mesh dizini varsayılan olarak
`~/PX4-Autopilot/Tools/simulation/gz/models/standard_vtol/meshes`;
`VTOL_MESH_DIR` ortam değişkeniyle değiştirilebilir.

Kaynak model: `Tools/simulation/gz/models/tiltrotor_indi/model.sdf`
(depodaki `tiltrotor_tailplane_model.sdf` ile birebir aynı).

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

## Doğrulama

Tüm 20 STEP dosyası geri okundu: her biri `BRepCheck_Analyzer` ile geçerli ve
tek kapalı katı. Ana ölçülerin SDF/mesh karşılaştırması `MONTAJ.md` sonundadır.
