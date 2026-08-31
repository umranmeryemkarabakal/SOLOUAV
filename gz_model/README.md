# Gazebo (gz Harmonic) model ve dünya dosyaları

Bu klasör, SITL koşumlarının kullandığı **Gazebo tarafını** deponun içinde
tutar. Uçuş yazılımı depo kökünde düz duruyor; burası simülasyon ortamı.

## İçerik

| Yol | Ne | PX4'teki karşılığı |
|---|---|---|
| `tiltrotor_tailplane/model.sdf` | Araç modeli | `Tools/simulation/gz/models/tiltrotor_tailplane/model.sdf` |
| `tiltrotor_tailplane/model.config` | Gazebo model künyesi | aynı klasör |
| `worlds/windy_tiltrotor.sdf` | 6 m/s yanal rüzgârlı dünya (projeye özgü) | `Tools/simulation/gz/worlds/` |

## `model.sdf` bir KOPYA DEĞİL, hardlink

`tiltrotor_tailplane/model.sdf` ile depo kökündeki
`tiltrotor_tailplane_model.sdf` **aynı inode**'dur (`ln`, kopya değil).

```bash
stat -c '%i %h %n' tiltrotor_tailplane_model.sdf gz_model/tiltrotor_tailplane/model.sdf
# iki satırda da aynı inode, bağ sayısı 2
```

**Neden böyle:** bu depo üç uygulamanın (MATLAB referansı, Simulink codegen,
PX4 C++) ayrışmasından defalarca zarar gördü; aynı dosyanın iki kopyasını
tutmak aynı hatayı geometri tarafında tekrarlamak olurdu. Hardlink ayrışmayı
**imkânsız** kılar: birini düzenlemek diğerini de günceller.

Bir düzenleyici hardlink'i bozarsa (bazı editörler yeni dosya yazıp
`rename` yapar) şu komut geri kurar:

```bash
cd ~/tiltrotor_project_updates
ln -f tiltrotor_tailplane_model.sdf gz_model/tiltrotor_tailplane/model.sdf
```

## PX4 ağacıyla senkron

PX4 kopyası **ayrı bir inode**'dur, elle senkronlanır:

```bash
cat tiltrotor_tailplane_model.sdf > \
  ~/PX4-Autopilot/Tools/simulation/gz/models/tiltrotor_tailplane/model.sdf
```

`cat >` kullanın, `cp` değil: PX4 tarafında `tiltrotor_indi`,
`tiltrotor_tailplane`'e **sembolik bağdır** (ayrı klasör değil), ve `cat >`
hedef dosyayı yerinde yazarak bu düzeni bozmaz.

```bash
ls -la ~/PX4-Autopilot/Tools/simulation/gz/models/tiltrotor_indi
# -> tiltrotor_tailplane
```

Yani airframe `4023_gz_tiltrotor_indi`, `PX4_SIM_MODEL=gz_tiltrotor_indi` ile
bu modeli sembolik bağ üzerinden yükler.

## Dünya dosyaları

`windy_tiltrotor.sdf` **projeye özgüdür** (PX4 stok değil): `default.sdf`'in
aynısı artı `<wind><linear_velocity>0 6 0</linear_velocity></wind>`.
Rüzgâr **opt-in**'dir — bu depodaki bütün eşikler sıfır rüzgârda kalibre
edildi, o yüzden varsayılan `default`:

```bash
INDI_WORLD=windy_tiltrotor python3 sitl/run_mission_test.py
```

**PX4'ün stok `default.sdf`'inde yapılan tek değişiklik:** `<grid>false</grid>`
→ `<grid>true</grid>`. Zemin gri 100×100 m bir düzlem ve arka plan da gri
olduğu için 40 m irtifada görsel referans kalmıyordu; grid salt bir sahne
render ayarıdır, dinamiğe/rüzgâra/zemin temasına dokunmaz. Aynı değişiklik
`windy_tiltrotor.sdf`'te de var (bu klasördeki kopyada dâhil).

## Doğrulama

Geometri değiştirdikten sonra:

```bash
python3 check_model_clearance.py     # açıklık + collision + mesh + XML + bağlantı
cd cad && ~/cq-venv/bin/python dogrula.py   # bağımsız katı denetimi
```

İkisi çelişirse **katı denetimi esastır** — bkz. `WLS_LOCKUP_INVESTIGATION_REPORT.md`
Adım 143.
