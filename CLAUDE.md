# Tilt-Rotor INDI Uçuş Kontrolü

PX4 tabanlı tilt-rotor VTOL uçuş kontrol yazılımı: INDI tutum kontrolü,
WLS kontrol tahsisi, MATLAB model/tasarım tarafı ve SITL deneyleri.

## Depo düzeni

Kod **depo kökünde düz duruyor** — `src/` yok. Bunu her seferinde yeniden
keşfetmeye gerek yok:

| Grup | Dosyalar |
|---|---|
| PX4 uçuş yazılımı | `MulticopterIndiTiltrotor.{hpp,cpp}`, `TiltrotorIndiControl.hpp`, `TiltrotorIndiParams.hpp`, `TiltrotorIndi{Setpoint,Status}.msg` |
| MATLAB model ve kontrol tasarımı | `indi_attitude_controller.m`, `sf_wls_alloc.m`, `tiltrotor_params.m`, `tiltrotor_plant_deriv.m`, `altitude_loop.m`, `init_ctrl_state.m` |
| SITL deneyleri ve analiz | `sitl_experiments/` (`run_*.m` koşumları + `*.py` analiz) |
| Araç modeli | `tiltrotor_tailplane_model.sdf` (PX4'teki `models/tiltrotor_indi/model.sdf` ile birebir aynı) |
| CAD | `cad/` — SDF'ten üretilmiş STEP katıları; `README.md`, `MONTAJ.md`, **`MEKANIZMA_GUNLUGU.md`**, üreticiler `build_tiltrotor_cad.py` + `build_mechanism.py`, kapılar `dogrula*.py` |
| PX4 airframe | `px4_airframes/` — SITL (4023), gerçek kart (14002), **HITL (14003)** + `README.md`; PX4 ağacındaki kopyaların yedeği |
| Gazebo tarafı | `gz_model/` — model.sdf (depo kökündekine **hardlink**) + model.config + `worlds/windy_tiltrotor.sdf`; açıklama `gz_model/README.md` |
| Mevcut yazılı kayıt | `RUNBOOK.md` (78 KB), `WLS_LOCKUP_INVESTIGATION_REPORT.md` (534 KB) |

Son iki dosya, kodun kendisinden çıkarılamayacak "neden böyle yapıldı"
bilgisinin ana kaynağı. Tasarım gerekçesi aranıyorsa önce oraya bakın.

## CAD mekanizma aşaması (1 Eylül 2026) — ÖNCE BUNU OKUYUN

CAD artık iki aşamalı üretiliyor ve **sıra bağlayıcıdır**:

```bash
$CQ/python cad/build_tiltrotor_cad.py    # 1) aerodinamik gövdeler — MESH İSTER
$CQ/python cad/build_mechanism.py        # 2) mekanizma + montaj — mesh istemez
$CQ/python cad/dogrula_mekanizma.py      # kapı: 61 geçti / 2 uyarı / 0 hata olmalı
```

| Nerede ne var |
|---|
| `cad/MEKANIZMA_GUNLUGU.md` — **çalışma günlüğü**: yapılanların ölçümleri, 5 açık madde, ⛔ denenip elenen 11 yol, araç tuzakları |
| `cad/MONTAJ.md` sonu — **montajda karar verilecek 8 madde** (K1…K8) |
| `cad/README.md` "Başka bir makinede devam etme" — kurulum adımları |
| `cad/step/tiltrotor_assembly.step` — 74 parça, mekanizma dahil tek dosya |

### Bu makineye özgü, depoda OLMAYAN bağımlılıklar

Depo tek başına yetmez; üçü de bilerek dışarıda (kurulum: `cad/README.md`):
cadquery venv, `.dae` mesh dizini (`VTOL_MESH_DIR`), Fusion MCP köprüsü.
`.mcp.json` **git'te izlenmez** — mutlak yol içerir, her makinede yeniden
yazılmalı ve Claude Code yeniden başlatılmalıdır.

### Sessizce yanlış sonuç üreten dört tuzak

1. **Aşama 2 idempotent değil.** `build_mechanism.py` kendi çıktısının
   üzerine ikinci kez koşulamaz (`Null TopoDS_Shape` ile çöker). Depodaki
   STEP'ler Aşama 2'nin **çıktısıdır**; `git checkout -- cad/step/` yapıp
   üstüne koşmak tam bu hatayı yaptırır.
2. **`build_tiltrotor_cad.py` aralıklı segfault veriyor.** Sonrasında
   gövdeler eksik üretilir ve Aşama 2 onların üzerine koşarsa ölçümler
   yanıltıcı çıkar. Her koşuda "yazıldı" satırını görün.
3. **Fusion'ın boolean ve `pointContainment` sorguları `wing` gövdesinde
   güvenilmez** (60 mm küp %100 içeride derken 80 mm küp %0,2 diyor).
   Kanadı içeren her kontrol `dogrula.py` / `dogrula_mekanizma.py` ile.
4. **Mesh doğruluğu:** yeniden üretimde `winglet_left` 417,3 cm³ ve
   `elevon_left` 125,3 cm³ çıkmalı. Tutmuyorsa mesh sürümü farklıdır.

### En büyük açık madde

Kuyruk menteşe hattında **11 mm boşluk** (elevon tarafı 0,2 mm ile doğru).
Kök neden ve çözüm reçetesi günlükte yazılı — sıfırdan teşhis etmeyin.
Geometrik "oturuyor mu" sorusunun doğru ölçütü `BRepExtrema` ile **iki katı
arasındaki asgari mesafedir**; hacim ya da yüz alanı yanıltır.

## Dokümantasyon boru hattı

`docs-pipeline` iskeleti bu depoya kurulu (28 Ağustos 2026). Kod tabanını
üç zorluk seviyesinde dokümante eden, çıktısını beş kapıyla sınayan bir akış.

- Akışın tamamı: `.claude/skills/docs-pipeline/SKILL.md`
- Bağlayıcı sözleşme (seviye tanımları, kurallar, kapılar): `docs/_meta/DOCS_SPEC.md`
- Konu listesi (tek gerçek kaynak): `docs/_meta/manifest.yaml`
- İskeletin kendi genel bakış dokümanı: `/home/umran/Claude-skill-agent-prompt/docs-pipeline/docs/OVERVIEW.md`

Doğrulama kapıları — bağımlılık gerektirmez, her yazımdan sonra koşulur:

```bash
python3 scripts/check_doc_refs.py                    # kod referansı canlılığı
python3 scripts/check_frontmatter.py                 # künye + yerleşim + kapsam
python3 scripts/check_frontmatter.py --show-layout   # üretilecek yolları önden gör
```

### Bilinmesi gerekenler

- **Agent tanımları yalnızca oturum açılışında yüklenir.** `.claude/agents/`
  değiştiyse veya yeni kurulduysa Claude Code yeniden başlatılmalı; aksi
  halde `docs-explorer`, `fresh-reader` gibi agent'lar çağrılamaz.
- **Faz 1'den sonra durup manifest onayı alınır.** Yanlış manifest ile
  40 doküman üretmek bu akıştaki en pahalı hata.
- **Dosya düzeni Faz 1.5'te seçilir**, `manifest.yaml` → `meta.layout`
  şablonlarına yazılır. Şu an TODO.
- **Denetleyicilerin kısıtları kasıtlı.** `fresh-reader`'a yalnızca `Read`
  verilmiş; arama yetkisi eklemek testi anlamsızlaştırır.

## Çalışma tercihleri

- Dokümanların ve konuşmanın dili Türkçe.
- CAD üretimi cadquery gerektirir; ayrı bir venv'de kurulur (depo dışında).
- Depo git ile izleniyor (`origin` = github.com:umranmeryemkarabakal/SOLOUAV, dal `main`).
  Kullanıcı **her değişiklikten sonra commit + push** istiyor (2026-08-31).
  `sitl/px4_mission.log` her koşuda baştan yazılan bir çıktıdır; anlamlı
  değişikliklerle aynı commit'e katmayın.
- Oturum başında bu dosya + proje hafızası okunur. Burada yazan bir şeyi
  kodu tarayarak yeniden türetmeyin.
