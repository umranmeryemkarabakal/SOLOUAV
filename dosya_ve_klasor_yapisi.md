# Dosya ve Klasör Yapısı

Bu belge klasördeki her dosyanın ne işe yaradığını, hangi dosyayı çağırdığını
ve neyi bağımlı olduğunu listeler. Genel mimari için bkz. `README.md`.

```
tiltrotor_Matlab files/
├── README.md                       # Proje genel bakışı
├── dosya_ve_klasor_yapisi.md       # Bu dosya
│
├── ── Parametre & yardımcı ──
│   ├── tiltrotor_params.m          # Tüm araç/kontrol parametreleri (tek kaynak)
│   ├── hover_trim.m                # Hover denge (trim) aktüatör durumu
│   ├── init_ctrl_state.m           # Kontrolcü başlangıç durumu (struct)
│   ├── rk4_step.m                  # 4. derece Runge-Kutta tek adım
│   ├── quat_from_euler.m           # ZYX Euler -> kuaterniyon
│   ├── quat_to_euler.m             # Kuaterniyon -> ZYX Euler
│   ├── quat_to_dcm.m               # Kuaterniyon -> DCM (body -> earth)
│   └── quat_deriv.m                # Kuaterniyon kinematiği (qdot)
│
├── ── Kontrolcü (saf MATLAB) ──
│   ├── indi_attitude_controller.m  # Ana kontrolcü: P + INDI + LESO + WLS
│   ├── gain_schedule.m             # Tilt açısına göre kazanç/ağırlık interpolasyonu
│   ├── effectiveness_matrix.m      # Anlık etkinlik Jacobian'ı G(u) + nu0
│   ├── wls_allocate.m              # Active-set ağırlıklı en küçük kareler tahsisi
│   ├── leso_axis_update.m          # Tek eksen LESO (2. derece gözlemci) adımı
│   ├── leso_bandwidth_gains.m      # wo -> [beta1, beta2]
│   └── altitude_loop.m             # İrtifa dış döngüsü (P + PI, anti-windup)
│
├── ── Plant (simülasyon modeli) ──
│   ├── tiltrotor_plant_deriv.m     # 19 durumlu tam nonlineer 6-DOF türev fonksiyonu
│   └── tiltrotor_plant_sfcn.m      # Aynı plant'in Level-2 S-Function sarmalayıcısı
│
├── ── Simulink (MATLAB Function blok içerikleri) ──
│   ├── sf_indi_rate_law.m          # P + INDI + LESO, saf fonksiyon (13x1 durum)
│   ├── sf_wls_alloc.m              # G(u) + WLS, sabit boyutlu büyük-M sürümü
│   ├── sf_altitude_loop.m          # İrtifa döngüsü, saf fonksiyon (1x1 durum)
│   └── sf_quat_to_euler.m          # Kuaterniyon -> Euler, codegen uyumlu
│
├── ── Test betikleri & çıktılar ──
│   ├── run_hover_gust_test.m       # Hover bozucu reddi testi (LESO açık/kapalı)
│   ├── run_transition_test.m       # Hover -> cruise tilt geçiş testi
│   ├── run_yaw_step_test.m         # Yaw adım yanıtı (+-30 deg), aşım/yerleşme/salınım
│   ├── run_yaw_ablation.m          # SITL kusurlarını enjekte eden yaw ablasyonu (tanı)
│   ├── run_backtrans_sm_test.m     # Geri geçiş durum makinesinin saf mantık testi (madde R + S)
│   ├── hover_gust_test.png         # Yukarıdaki testin çıktı grafiği
│   └── transition_test.png         # Yukarıdaki testin çıktı grafiği
│
├── ── Simulink modeli ──
│   ├── tiltrotor_indi_build.m      # Modeli programatik kuran betik
│   ├── tiltrotor_indi.slx          # Üretilen Simulink modeli
│   ├── tiltrotor_indi.slxc         # Simulink derleme önbelleği (üretilmiş)
│   └── slprj/                      # Simulink derleme çıktıları (üretilmiş)
```

---

## Parametre ve yardımcı fonksiyonlar

### `tiltrotor_params.m`
`p = tiltrotor_params()` — tüm sabitlerin tek kaynağı. Kütle/atalet, rotor
geometrisi (`p.rotor.pos` 3×3, sütunlar rotor konumları), tork/itki oranı
`km`, itki katsayısı ve limitler, tilt servo limitleri/zaman sabiti, aero
katsayıları ve döngü periyotları (`Ts_ctrl` 400 Hz, `Ts_leso` 200 Hz,
`Ts_att` 200 Hz, `Ts_pos` 50 Hz). Değerler PX4/Gazebo `tiltrotor_tailplane`
airframe'inden alınmıştır.

**Not:** `sf_*.m` dosyaları kod üretimi için bu sabitleri **kopyalar**.
Bir parametreyi değiştirirken ilgili `sf_*.m` dosyasını da güncellemek gerekir.

### `hover_trim.m`
`u_trim = hover_trim(p)` — Fz ve pitch dengesini δ = 0 için kapalı formda
çözer (`2·T_ön·0.22 = T_arka·0.65`). Bu konfigürasyonda km işaretleri
(2× CCW + 1× CW) net bir yaw momenti bırakır; bu, sol kanat rotorunun tilt'i
ile tek eksenli olarak sıfırlanır. Kalan küçük artık, kapalı çevrimde INDI/WLS
tarafından düzeltilir. → `effectiveness_matrix.m` çağırır.

### `init_ctrl_state.m`
LESO durumları (`z1`, `z2`), son yayınlanan `d_hat`, ESO girişi
(`prev_u_leso`) ve desimasyon birikicisi (`leso_accum`) içeren başlangıç
struct'ını döndürür.

### `rk4_step.m`
`x_next = rk4_step(@(x) ..., x, dt)` — kontrol/bozucu girişleri kapatma içinde
sabit tutularak sabit adımlı RK4 entegrasyonu.

### Kuaterniyon yardımcıları
`quat_from_euler.m`, `quat_to_euler.m`, `quat_to_dcm.m`, `quat_deriv.m` —
Hamilton konvansiyonu, `[q0;q1;q2;q3]`, body→earth. `quat_to_euler`'da
`asin` argümanı `[-1,1]` aralığına sıkıştırılmıştır.

---

## Kontrolcü (saf MATLAB yolu)

### `indi_attitude_controller.m`
Tek adımlık ana kontrolcü. Sırasıyla:

1. `gain_schedule(delta_bar, p)` — ortalama tilt açısına göre kazançlar
2. Dış tutum P döngüsü → `omega_sp` (yaw hatası sarmalanır, ±3 rad/s doygunluk)
3. İç hız döngüsü → `omega_dot_des`
4. LESO güncellemesi (desimasyonlu, sadece `leso_axes_enable` eksenlerinde)
5. INDI artımlı yasa: `dtau = I·(omega_dot_des_adj − omega_dot_meas)`
6. `effectiveness_matrix` + `wls_allocate` ile tahsis, kutu kısıtları
   (mutlak limit ∩ slew limiti) ile

**Girişler:** `att_sp, att, omega, omega_dot_meas, F_sp, u_actual, ctrl_state, p, leso_axes_enable`
**Çıkışlar:** `T_cmd, delta_cmd, ctrl_state, diagn`

`diagn` struct'ı loglama için `sched`, `nu_des`, `nu0`, `du`, `sat_flag`,
`n_iter`, `d_hat`, `omega_sp` taşır.

### `gain_schedule.m`
`s = δ̄/(π/2)`, `w = 3s² − 2s³` (smoothstep) ile hover↔cruise interpolasyonu.
Döndürdükleri: `Kp_att` (roll/pitch/yaw), `Kp_rate`, `wu_thrust`, `wu_tilt`,
`smooth`. Yaw kazancı roll/pitch'ten kasıtlı olarak düşüktür.

### `effectiveness_matrix.m`
`[G, nu0] = effectiveness_matrix(u, p)` — u = [T0;T1;T2;δ0;δ1;δ2] noktasında
5×6 Jacobian ve o noktadaki gerçek (tam nonlineer, geometrik) sanal kontrol
değeri. `ν = [τx; τy; τz; Fx; Fz]`. Her rotor için itki yönü `[sinδ; 0; −cosδ]`
ve türevi `[cosδ; 0; sinδ]` kullanılır. INDI'nin lineerleştirme adımı budur;
her kontrol adımında yeniden hesaplanır.

### `wls_allocate.m`
Bodson (2002) sequential least squares / PX4 `control_allocator` tarzı
active-set çözücü. Kutu kısıtını ihlal eden aktüatörler "sabit" kümeye alınır,
hedef onların katkısı kadar düzeltilir, kalanlar için problem yeniden çözülür.
Kötü koşullanmaya karşı `1e-9·I` regularizasyonu vardır (bir rotor itkisi ≈ 0
iken o rotorun tilt hassasiyeti de ≈ 0 olur → sütun yetersizliği).
Varsayılan `max_iter = 2n + 2`.

### `leso_axis_update.m` / `leso_bandwidth_gains.m`
Tek eksen, göreli derece 1 modeli `ω̇ = u + d(t)` için 2. derece gözlemci:
```
z1 += Ts·(z2 + u_applied + β1·e)
z2 += Ts·(β2·e),      e = y_ölçülen − z1
```
`β1 = 2wo`, `β2 = wo²` (kritik sönümlü kutuplar). Fonksiyon **saftır** —
durum dışarıda (`ctrl_state`) tutulur, böylece aynı kod birden fazla
eksende/simülasyonda kullanılabilir.

### `altitude_loop.m`
`[Fz_sp, state_out] = altitude_loop(z_sp, z, vz, state_in, p)`.
P (pozisyon, `Kp_z = 0.6`, ±2 m/s hız limiti) + PI (hız, `Kp_vz = 4.0`,
`Ki_vz = 1.5`, integral ±3 m/s² clamp). Durum tek skaler (`integral_vz`).
Desimasyonu **çağıran taraf** yönetir (`run_*.m` içinde `Ts_pos` birikicisi).

---

## Plant

### `tiltrotor_plant_deriv.m`
19 durumlu türev fonksiyonu:

| İndis | Durum | Birim |
|---|---|---|
| 1:3   | `pos_ned` [x;y;z] | m (NED) |
| 4:6   | `vel_body` [u;v;w] | m/s (gövde) |
| 7:10  | `quat` [q0..q3] | – (body→earth) |
| 11:13 | `omega_body` [p;q;r] | rad/s |
| 14:16 | `T_actual` [T0;T1;T2] | N (1. derece gecikmeli) |
| 17:19 | `delta_actual` [δ0;δ1;δ2] | rad (1. derece gecikmeli + slew limitli) |

Ek girişler: `wind_ned` (3×1, rüzgar) ve `ext_moment` (3×1, gövde ekseninde
dış moment) — bozucu testleri için. İtki artışı/azalışı farklı zaman
sabitleri kullanır. Aero bölümü yalnızca `Va > 0.5 m/s` iken devrededir.

**Önemli:** Aero modeli kontrolcüden gizlidir; INDI ölçülen `omega_dot`
sayesinde onu bilmeden çalışır, kalan yavaş/kümülatif kısmı LESO tahmin eder.

### `tiltrotor_plant_sfcn.m`
`tiltrotor_plant_deriv`'i çağıran ince Level-2 S-Function sarmalayıcısı.
Level-2 MATLAB S-Function normal simülasyonda **yorumlanarak** çalıştığı için
struct gibi genel MATLAB yapılarını serbestçe kullanabilir.

- Girişler: `u_cmd` (6), `wind_ned` (3), `ext_moment` (3)
- Çıkış: **tek port, 38 eleman** = `[x(19); xdot(19)]`. INDI'nin ölçülen açısal
  ivmesi `y(30:32)`'dir. Çok portlu boyut çözümlemesi build script'inde
  kararsızlığa yol açtığı için tek portta birleştirilmiştir.
- Diyalog parametreleri: `p` (struct), `x0` (19×1)

---

## Simulink MATLAB Function blok içerikleri (`sf_*.m`)

Bu dosyalar `#codegen` işaretlidir ve şu kısıtlara uyar: değişken boyutlu dizi
yok, `find` yok, `persistent` yok, struct girişi/çıkışı yok. Durum, blok
dışındaki **Unit Delay** blokları üzerinden geri beslenir.

| Dosya | Karşılığı | Durum vektörü |
|---|---|---|
| `sf_indi_rate_law.m` | `indi_attitude_controller` (1–4. adımlar) + `gain_schedule` | 13×1: `[z1(3); z2(3); prev_u_leso(3); leso_accum; omega_dot_filt(3)]` |
| `sf_wls_alloc.m` | `effectiveness_matrix` + `wls_allocate` (5. adım) | yok |
| `sf_altitude_loop.m` | `altitude_loop` | 1×1: `integral_vz` |
| `sf_quat_to_euler.m` | `quat_to_euler` | yok |

**`sf_indi_rate_law.m`** ayrıca `omega_dot_raw` üzerine tek kutuplu filtreyi
(α = 0.3) kendi içinde uygular — saf MATLAB yolunda bu, `run_*.m` betiğinde
yapılır.

**`sf_wls_alloc.m`** active-set yerine **büyük-M ceza** yöntemi kullanır:
kutu kısıtını ihlal eden aktüatör kümeden çıkarılmak yerine ağırlığı `1e6`'ya
çıkarılarak o sınıra "yapıştırılır". En fazla 6 iterasyon.

**`sf_altitude_loop.m`** Simulink'te 400 Hz'de çağrıldığı için `Ts_pos`
0.0025 s'ye küçültülmüştür (integral örnekleme hızından bağımsız olduğu için
matematiksel olarak eşdeğer).

---

## Test betikleri

### `run_hover_gust_test.m`
50 m irtifada hover. t = 4 s'den itibaren 1 s'lik rampa ile:
- `wind_ned = [-5; 0; -0.8]` m/s (aero model üzerinden pitch bozucusu)
- `ext_m(1) = 0.4 + 0.15·sin(2π·0.3·t)` Nm (yavaş değişen roll bozucusu)

İki konfigürasyon koşulur — LESO açık (roll+pitch) ve LESO kapalı — ve
`t ≥ 4` penceresinde RMS `p` / RMS `q` karşılaştırılır. 3 alt grafik:
roll rate, pitch rate, ve `d_hat` vs gerçek bozucu ivmesi.
Süre 12 s, çıktı `hover_gust_test.png`.

### `run_transition_test.m`
80 m irtifada başlar, `Fx_sp` 12 s'de 0 → 10 N rampalanır (bozucu yok).
Konsola son ortalama tilt açısı, ileri hız, irtifa değişimi ve max |ω|
yazar. 6 alt grafik: tilt açıları, itkiler, ileri hız, tutum, `Kp_rate` vs
tilt, `Wu_tilt` vs tilt. Süre 14 s, çıktı `transition_test.png`.

### `run_yaw_step_test.m`
50 m irtifada hover, bozucu yok. t = 4 s'de `yaw_sp = ±30°` adımı, 25 s.
İki yön de koşulur, çünkü yanıt **yön-asimetriktir** (hover trim'de δ1 tek
yönlü tilt aralığının tabanına — `p.tilt.min = 0` — çakılı olduğundan +yaw
sınıra vurur, −yaw serbesttir).

Bu test yaw ekseninin **en kötü koşulunu** ölçer: `tiltrotor_plant_deriv.m`
aero modeli tamamen boylamsal olduğu için (`F_aero(2) = 0`,
`r_cp = [-0.05;0;0.05]`) `M_aero(3) ≡ 0` — yani plant **hiçbir hızda**
aerodinamik yaw momenti üretmez. SITL'de yaw'ı asıl sönümleyen şeyin ileri
hızdaki rüzgâr gülü etkisi olduğu ölçüldüğünden (bkz.
`sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md` Adım 16/17), MATLAB yaw açısından
yapısal olarak "gerçek hover"dır.

Konsol metrikleri: aşım %, ±2° yerleşme süresi, kalıcı hata `e_ss`,
`max|r|`, son 5 s yaw hızı RMS ve buradan türetilen **`salınım?`** verdisi
(kalıcı ofseti kalıcı salınımdan ayırt etmek için — ikisi karıştırılabilir),
ayrıca bağlam değişkeni olarak yatay hız. 4 alt grafik: ψ, `r` vs `r_sp`
(`rate_sp_limit` çizgileriyle), δ0/δ1 (0° alt sınırıyla), ve WLS'in talep
ettiği vs tahsisatın ürettiği Δτ_z. Çıktı `yaw_step_test.png`.

PX4 portuyla aynı tilt slew limitinde koşmak için (MATLAB 3.0, PX4 2.0
kullanır — Adım 9'da düşürüldü, MATLAB'a kasıtlı taşınmadı):
```matlab
setenv('YAW_TEST_TILT_RATE_MAX','2.0'); run_yaw_step_test   % -> yaw_step_test_rate2.0.png
```

### `run_yaw_ablation.m`
**Tanı aracı** (Adım 20) — test değil. SITL'de var olup MATLAB'da olmayan
kusurları tek tek ve birlikte MATLAB'a enjekte edip hangisinin yaw
sönümlemesini bozduğunu arar:

| ablasyon | ne yapar |
|---|---|
| `shadow` | Kontrolcü, plant'in gerçek aktüatör durumu yerine PX4'teki gibi **açık çevrim** bir gölge model görür (`MulticopterIndiTiltrotor.cpp:414-421` kopyası). MATLAB normalde `u_actual = x(14:19)`, yani gerçek durumu verir — SITL'in sahip olmadığı bir avantaj. |
| `odot_delay` | `omega_dot_meas`'e ek taşıma gecikmesi (ms) |
| `odot_noise` | `omega_dot_meas`'e beyaz gürültü (rad/s² RMS) |
| `dt_jitter` | Kontrol adımına ± jitter (PX4 gerçek-zamanlı zamanlama) |

**Hiçbir kontrol sabitini değiştirmez**; `rng(12345)` ile tekrarlanabilir.
Çıktı `yaw_ablation.png`. Şu ana kadarki sonuç: dördü de ve birleşimi
yerleşme süresini değiştirmedi (3.64-3.70 s) — aday listesi hâlâ eksik,
bkz. `sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md` Adım 20a.

Dört betik de: kontrol 400 Hz (`dt_ctrl`), fizik `n_sub = 5` alt adımla
RK4 (2 kHz).

### `run_backtrans_sm_test.m`
**Plant yok** — diğer beş betikten farkı bu. `backtrans_loop.m` durum
makinesini sentetik `v_h` izleriyle sürer ve yalnızca **mantığı** denetler
(madde (R), Adım 38).

Gerekçe: `backtrans_loop.m`'in kendi başlığı MATLAB'ın bu **manevrayı**
üretemediğini doğru şekilde söylüyor (tek boylamsal yüzey, 12 m/s'de ~25 N
taşıma). Ama madde (R)'nin sorusu manevra değil, *"çıkış koşulu sağlanmazsa
durum makinesi ne yapar?"* — bu saf mantık, ve mantık için plant gereksiz.
Sentetik `v_h` gerçek aerodinamikten **daha iyidir**, çünkü arızalı rejim
(terminal hız eşiğin üstünde) istenildiği gibi kurulabilir; SITL'de o rejim
rastlantısaldı (8 uçuşun 3'ünde çıktı). Adım 21d'nin kuralı korunuyor: ortam,
hedeflenen mekanizma orada aktif olduğu için geçerli.

**Adım 39'da (2026-08-03) madde (S) için genişletildi: 9 grup / 13 kontrol.**
Yeni testler: (7) yanal sürüklenme büyüklüğü eşiğin üstünde tutarken de
HANDOFF'a geçiliyor mu — iz ölçülen arızadan kuruldu (`v_fwd` 13.5 → −0.51,
yanal sabit 3.04, yani `|v_h|` hiç 3.04'ün altına inemiyor); (8) **eski
BÜYÜKLÜK mantığı aynı izde 300 s BRAKE'te kalıyor** (regresyonun varlığı
gösterilir, varsayılmaz — test 4 ile aynı disiplin); (9) frenleme marjı geri
giderken tam olarak sıfırlanıyor mu (7.39° → 3.39°, yani net itme ~0 N).
`simulate` artık opsiyonel bir `v_fwd` izi alıyor; verilmezse `v_fwd = v_h`,
bu yüzden Adım 38'in testleri değişmeden geçiyor.

Altı kontrol: normal dizinin hız terimiyle çıkması, ölçülen 8.97 m/s
takılmasının artık çıkması, terminal hız **yeni** eşiğin de üstündeyken
emniyetin tam `floor_dwell` sonra tetiklenmesi, emniyetsiz mantığın aynı izde
takılı kalması (regresyonun gerçekten var olduğunun kanıtı), `enable=0`'da
tavanın bırakılıp sayacın sıfırlanması, ve tavan **inerken** emniyetin
tetiklenmemesi. Ayrıca BRAKE giriş hızının eşiğe payını raporlar.

Sayıların (`release_v = 10.0`, `floor_dwell = 20 s`) doğrulanması burada
**değil**, SITL'de yapılır.

---

## Simulink modeli

### `tiltrotor_indi_build.m`
`tiltrotor_indi.slx`'i sıfırdan kurar. Adımlar: eski modeli sil → çözücüyü
ayarla (`ode4`, sabit adım 0.0025, StopTime 12) → `p` ve `x0_tilt`'i base
workspace'e koy (S-Function parametreleri derleme sırasında **base**
workspace'te aranır; `InitFcn` yalnızca sonraki `sim()` çağrılarında çalışır)
→ blokları ekle → MATLAB Function bloklarının içeriğini `sf_*.m`'den yükle
(`set_mfcn` yardımcısı) → hatları bağla → kaydet.

Eklenen bloklar:
- **Kaynaklar:** `att_sp`, `Fx_sp`, `z_sp`, `leso_enable` ([1;1;0]),
  `wind_ned`, `ext_moment` (hepsi Constant — senaryo değiştirmek için bunları
  düzenleyin)
- **Unit Delay:** `u_cmd_delay` (cebirsel döngüyü kırar, X0 = trim),
  `leso_state_delay` (13×1), `alt_state_delay` (skaler)
- **Plant:** Level-2 MATLAB S-Function → `tiltrotor_plant_sfcn`
- **Selector'lar:** 38 elemanlı plant çıkışından `quat`, `omega`, `u_actual`,
  `omega_dot`, `z`, `vz` ayıklanır
- **MATLAB Function:** `sf_quat_to_euler`, `sf_altitude_loop`,
  `sf_indi_rate_law`, `sf_wls_alloc`
- **Scope'lar:** `scope_att`, `scope_omega`, `scope_dhat`, `scope_satflag`,
  `scope_act`

Kullanım: dosyayı diğer `.m` dosyalarıyla aynı klasöre koy, o klasörü Current
Folder yap, `tiltrotor_indi_build` çalıştır, sonra `sim('tiltrotor_indi')`.

### Üretilen dosyalar (elle düzenlenmez)
- **`tiltrotor_indi.slx`** — build script'in çıktısı. `tiltrotor_indi_build`
  her çalıştırıldığında **silinip yeniden oluşturulur**; modelde GUI'den
  yapılan değişiklikler kaybolur.
- **`tiltrotor_indi.slxc`** — Simulink derleme önbelleği.
- **`slprj/`** — Simulink derleme çıktı klasörü: `_sfprj/` (Stateflow/MATLAB
  Function derlemeleri), `_jitprj/` (JIT derleme önbelleği), `sim/varcache/`
  (değişken önbelleği). Silinmesi güvenlidir; sonraki derlemede yeniden üretilir.
- **`*.png`** — test betiklerinin çıktı grafikleri.

---

## Gazebo tarafı (`gz_model/`)

SITL koşumlarının kullandığı simülasyon ortamı. Ayrıntı: `gz_model/README.md`.

### `gz_model/tiltrotor_tailplane/model.sdf`

Araç modeli. **Kopya değil, depo kökündeki `tiltrotor_tailplane_model.sdf`'e
hardlink** (aynı inode) — bu depo üç uygulamanın ayrışmasından defalarca zarar
gördü, aynı hatayı geometri tarafında tekrarlamamak için. Birini düzenlemek
diğerini de günceller.

### `gz_model/tiltrotor_tailplane/model.config`

Gazebo model künyesi. Depoda başka kopyası yoktu.

### `gz_model/worlds/windy_tiltrotor.sdf`

Projeye özgü dünya: `default.sdf` + `<wind>0 6 0</wind>`. Rüzgâr **opt-in**
(`INDI_WORLD=windy_tiltrotor`), çünkü bu depodaki bütün eşikler sıfır rüzgârda
kalibre edildi.

### PX4 ağacıyla ilişki

PX4 kopyası ayrı inode'dur, elle senkronlanır (`cat >` ile, `cp` ile değil).
PX4 tarafında `tiltrotor_indi`, `tiltrotor_tailplane`'e **sembolik bağdır** —
ayrı bir klasör değildir.

## Bağımlılık zinciri (özet)

```
run_hover_gust_test.m ─┬─► tiltrotor_params.m
run_transition_test.m ─┤
                       ├─► hover_trim.m ──► effectiveness_matrix.m
                       ├─► quat_from_euler.m, quat_to_euler.m
                       ├─► init_ctrl_state.m
                       ├─► altitude_loop.m
                       ├─► indi_attitude_controller.m ─┬─► gain_schedule.m
                       │                               ├─► leso_bandwidth_gains.m
                       │                               ├─► leso_axis_update.m
                       │                               ├─► effectiveness_matrix.m
                       │                               └─► wls_allocate.m
                       └─► rk4_step.m ──► tiltrotor_plant_deriv.m ──► quat_to_dcm.m
                                                                  └─► quat_deriv.m

tiltrotor_indi_build.m ─┬─► tiltrotor_params.m, hover_trim.m, quat_from_euler.m
                        ├─► tiltrotor_plant_sfcn.m ──► tiltrotor_plant_deriv.m
                        └─► sf_quat_to_euler.m, sf_altitude_loop.m,
                            sf_indi_rate_law.m, sf_wls_alloc.m   (blok içeriği olarak)
```
