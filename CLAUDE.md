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
| CAD | `cad/` — SDF'ten üretilmiş STEP katıları, `cad/README.md` + `cad/MONTAJ.md` |
| Gazebo tarafı | `gz_model/` — model.sdf (depo kökündekine **hardlink**) + model.config + `worlds/windy_tiltrotor.sdf`; açıklama `gz_model/README.md` |
| Mevcut yazılı kayıt | `RUNBOOK.md` (78 KB), `WLS_LOCKUP_INVESTIGATION_REPORT.md` (534 KB) |

Son iki dosya, kodun kendisinden çıkarılamayacak "neden böyle yapıldı"
bilgisinin ana kaynağı. Tasarım gerekçesi aranıyorsa önce oraya bakın.

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
