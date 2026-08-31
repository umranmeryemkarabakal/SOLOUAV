# Tilt-Rotor VTOL — INDI + LESO + WLS Kontrolcü (MATLAB/Simulink)

3 tilt-rotorlu bir VTOL aracı (2 kanat rotoru + 1 kuyruk rotoru) için
**INDI (Incremental Nonlinear Dynamic Inversion)** tabanlı tutum kontrolcüsü,
**LESO** (Lineer Genişletilmiş Durum Gözlemcisi) ile bozucu telafisi,
**WLS** (ağırlıklı en küçük kareler) kontrol tahsisi ve tilt açısına bağlı
basit **gain-scheduling** içeren tam bir simülasyon paketi.

Depo iki paralel uygulama barındırır:

| | Saf MATLAB | Simulink |
|---|---|---|
| Kontrolcü | `indi_attitude_controller.m` | `sf_indi_rate_law.m` + `sf_wls_alloc.m` |
| Plant | `tiltrotor_plant_deriv.m` + `rk4_step.m` | `tiltrotor_plant_sfcn.m` (Level-2 S-Function) |
| Koşturma | `run_hover_gust_test.m`, `run_transition_test.m` | `tiltrotor_indi.slx` (`tiltrotor_indi_build.m` ile üretilir) |

İkisi de **aynı matematiği** uygular; Simulink tarafındaki `sf_*.m` dosyaları
MATLAB Function bloğunun kod üretimi kısıtlarına (değişken boyut yok,
`persistent` yok) göre yeniden yazılmış sürümlerdir.

---

## Araç ve model

Parametreler `tiltrotor_params.m` içindedir; kaynak olarak PX4/Gazebo
`tiltrotor_tailplane` airframe'i (SDF + `4022_gz_tiltrotor_tailplane`)
kullanılmıştır.

- Kütle 5 kg, `I = diag([0.2, 0.25, 0.25])` kg·m²
- 3 rotor, hepsi tilt edebiliyor (0 rad = hover / dikey, π/2 rad = cruise / yatay)
  - Rotor 0: sağ kanat, Rotor 1: sol kanat, Rotor 2: kuyruk (cruise'da pusher)
- Gövde ekseni **FRD** (X ileri, Y sağ, Z aşağı), konum NED
- Aktüatör dinamiği: rotor itkisi 1. derece gecikme (τ_up = 12.5 ms, τ_down = 25 ms),
  tilt servosu 1. derece gecikme (τ = 0.15 s) + 3 rad/s slew limiti

Plant modeli (`tiltrotor_plant_deriv.m`) 19 durumlu tam nonlineer 6-DOF'tur ve
basit bir boylamsal aerodinamik model içerir. **Kontrolcü bu aero modelini
bilmez** — bu bilinçli bir tasarım: aero etkiler INDI/LESO'nun telafi etmesi
gereken model belirsizliğini temsil eder.

---

## Görev profili (CONOPS) ve her evrenin bugünkü durumu

**HEDEF: TAM OTONOM GÖREV.** (2026-08-03'te gereksinim olarak netleştirildi.)
Pilot bu profilin sürücüsü değildir; pilot yolu yalnızca bir **müdahale/emniyet**
yoludur. Profilin kendisi baştan sona otomatik çalışmalı:

```
  kalkış: MULTİKOPTER
      ↓  (tilt ile ileri geçiş — OTOMATİK)
  seyir: SABİT KANAT / tilt motor
      ↓  (tilt ile geri geçiş — OTOMATİK)
  iniş:  MULTİKOPTER
```

Evre evre üç ayrı soru — ve bu üçünün karıştırılması engelleyici B1'in ta
kendisidir:

| evre | kontrol yasası var mı | SITL'de uçtu mu | **OTONOM tetikleniyor mu** |
|---|---|---|---|
| kalkış / hover | ✅ INDI + irtifa + `pos_hold` | ✅ | ⚠️ irtifa hedefi dışarıdan (`z_sp`) |
| **hover → seyir (ileri geçiş)** | ✅ **üç durumlu makine** (`forwardtrans_loop.m` / `forwardTransition()`, Adım 42). **İPTAL ETMEZ** (Adım 48, gereksinim) | ✅ **otomatik** | ✅ `ft_enable` |
| seyir | 🟠 **MATLAB'da kararlı, PX4'te hâlâ KISMİ.** SITL'de (PX4) kanat ağırlığın yalnızca %41-50'sini taşıyor. MATLAB'da hız döngüsü (Adım 47) + pitch trimi (Adım 49) + **ileri besleme (Adım 53)** ile **16-24 m/s zarfı kararlı**, kanat yük payı %75.6→%86.3, `Fz_sp` her hızda hedefte. Ama **hiçbiri PX4'e gönderilmedi**, ve tam kanat-taşımalı uçuş için Fx kanalına yüzey gerek — bkz. aşağısı | ✅ 14-20 m/s, tilt 32-72° | ⚠️ |
| **seyir → hover (geri geçiş)** | ✅ dört durumlu makine (B5, Adım 31-39) | ✅ otomatik | ✅ `bt_enable` |
| iniş / hover alçalma | ✅ | ✅ kademeli iniş | ❌ iniş dizisi test betiğinde, modülde değil |

**2026-08-03 (Adım 42): profilin tamamı ilk kez OTONOM uçtu** — iki bağımsız
uçuş, elle hiçbir komut yok, yalnızca iki bayrak (`ft_enable`, `bt_enable`):
kalkış → ileri geçiş (14.4 s rampa) → **tilt 44.1°, 15.2 m/s seyir** → geri
geçiş → hover → iniş. İrtifa sapması 0.58-0.80 m, itki doyumu %0.00, BIG_M 0,
NaN 0. Test: `sitl/run_mission_test.py`.

**Aynı fizik Adım 28'den beri uçuyordu; değişen, rampayı test betiğinin değil
ARACIN yürütmesi.** Bir yeteneğin var olması ile ona otonom erişilebilmesi ayrı
şeylerdir — engelleyici B1'in tamamı bu ayrımdır.

**🔴 VE SEYİR HENÜZ "SABİT KANAT" DEĞİL (Adım 43'te ölçüldü).** 14.4 m/s'de
kanat tilti 32°, kuyruk tilti 0.6°, ve **ağırlığın %59'u hâlâ rotorlarda.**
Zarf süpürüldüğünde (fx 12→24 N) tiltin 72°'ye yattığı ve 20.6 m/s'de her iki
**kanat rotorunun tamamen kapandığı** görüldü — ama kuyruk rotoru tek başına
21.8 N taşımaya devam ediyor ve tahsisat %20 doygun; bir kademe daha zarfı
kırıyor (%99.6 doygunluk, irtifa +21 m).

**Kök neden mimari:** `MulticopterIndiTiltrotor.cpp:1200` — kontrol yüzeyleri
(elevator/aileron/rudder, servo 0-4) **hiç oynatılmıyor**, sabit 0'da tutuluyor
("out of scope for the ported controller"). Dolayısıyla seyirde bütün duruş
otoritesi rotorlardan gelmek zorunda, rotorlar kapatılamıyor ve kuyruk rotoru
dik kalmak zorunda. **Tam kanat-taşımalı uçuş için kontrol yüzeyleri WLS
tahsisatına girmeli** (etkinlik matrisi 5×6 → 5×(6+N), yüzey etkinliği dinamik
basınçla ölçeklenir). Bu bir ayar değil, mimari bir adım.

**🆕 Adım 46 (2026-08-03) — 3. deneme MATLAB'da çalışıyor, ama PX4'e GİTMEDİ.**
Yüzeyler artık beş bağımsız aktüatör değil, **üç sanal aktüatör** (aileron =
antisimetrik elevon, elevator = simetrik servo_2/3, rudder); simetrik elevon
(flap) bilerek aktüatör değil. fx = 10 N'de 70 s, yüzeyler kapalı → açık:
ortalama tilt **52.1° → 70.6°**, kanat yük payı **%98.9 → %106.4**, toplam rotor
itkisi **24.0 → 16.1 N**, ve hız aktivitesi **iyileşti** (max |ω| 0.021 → 0.013).
Gereken sapmalar minik: −0.1° / −0.6° / +1.1°.

**Ama iki engel ölçüldü ve bu yüzden SITL'e gönderilmedi:**
(1) aileron, elevator olmadan açıkken ıraksıyor (iki ayrı fx'te tekrarlandı);
(2) fx ≥ 12 N'de rejim **yüzeyler kapalıyken bile** marjinal — kanat tilt'i 90°
mekanik durakta çakılı ve aracı yavaşlatan şey o durak. Yüzeyler bu kazara
korumayı kaldırıyor. Yani sıradaki iş yüzey değil, **aşağıdaki 1. madde**.
Detay: `sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md` Adım 46.

**Ayrıca aynı adımda MATLAB plant'inde iki gerçek kusur bulundu ve düzeltildi:**
kanadın taşıma **işareti tersti** (15 m/s'de burun yukarı 43-67 N *aşağı*) ve
sürükleme ~7 kat fazlaydı (gz'de `cd = |cda·alpha|`, sabit değil). Plant artık
gz LiftDrag'in birebir portu (`aero_panels.m`, 5 panel, stall dahil) ve SITL'de
ölçülen kanat yük payını (%41-50) bağımsız olarak yeniden üretiyor.

**🆕 Adım 49 (2026-08-04) — TECS'in pitch yarısı yazıldı; 16 m/s'de büyük
kazanç, duvar duruyor, ve ASIL KISIT bulundu.** `cruise_pitch_loop.m`
(`θ̇ = −Ki·(Fz_sp − hedef)`, hız kapılı, ±6°). 16 m/s'de aynı kararlılıkla
tilt **37.7° → 60.7°**, kanat yük payı **%89.4 → %106.1**, kanat rotoru yükü
**6.71 → 2.46 N**. Ama 17 m/s üstünde pitch açık da kapalı da ıraksıyor —
ve **90 saniyelik çok yavaş bir rampa da kurtarmadı**, yani geçici rejim
değil.

**İlk deneme hedefi 0 aldı ("Fz_sp → 0 = tam kanat-taşımalı uçuş"). Yasa
çalıştı ve tam bu yüzden kırdı:** irtifa/hız sabit, tilt 41→65°, kanat
%93→%106, gereken pitch sadece 0.8° — sonra rotor itkisi tam **0.00 N**'e
değdi ve araç 2 s'de ıraksadı. Sebep yapısal: etkinlik matrisinde
`dτ/dδ ∝ T`, yani **itkisi sıfır olan tilt rotorunun otoritesi de sıfır**, ve
**Fx kanalının hiçbir yüzeyi yok**. Hedef ölçüme dayanarak −12 N'e alındı.

**Adım 50 — itki tabanı DENENDİ, GERİ ALINDI.** Kanat rotorlarına seyirde
`Tmin_cruise = 4 N` konuldu ve **daha önce kararlı olan noktayı bozdu**:
taban, rotorları **90° tiltte** itki üretmeye zorluyor; orada itki hiçbir
taşıma üretmez, sadece ileri kuvvet. Araç hızlanıyor → kanat fazla taşıyor →
irtifa döngüsü tek dikey aktüatörü (kuyruk rotoru) sıfıra kesiyor → `Fz_sp`
+96 N'e patlıyor. Geri alındı, geri alma birebir doğrulandı.

**Adım 51 — duvarın sebebi arandı: dört aday çürütüldü, sınır AÇIK.**
Çürüyenler: `wu_ele` (80/160/320/640), hız döngüsü kazancı (Kp 1/4/8), kanat
rotoru itki tabanı (daha kötü, geri alındı), seyir tilt tavanı (70/65/60°).
**Duruş kontrolü ıraksamaya kadar kusursuz** (θ setpoint'i birebir izliyor,
pitch talebi ~0.05, elevatör 0.11°, Fz_sp tam hedefte) — yani ne pitch
otoritesi ne tahsisat doygunluğu. Ölçülen mekanizma: tilt 56.7→64.8° kayarken
`T0` 5.13→0.38 N'e ölüyor, çünkü yüksek tiltte kanat rotorlarının dikey
etkinliği düşüyor ve tahsisat Fz'yi kuyruk rotoruna devrediyor.

Kanat oturma açısı `a0` **gerçek ama kısmi** bir katkı: yarıya indirmek
(0.0598→0.030) sınırı "16 kararlı / 17 ıraksıyor"dan "**17 kararlı** / 20
ıraksıyor"a taşıdı — ama orantılı değil (V_wb 1.41× arttı, sınır artmadı).

**🆕 Adım 52 — kuyruk aşağı yükü: sınırı ilk kez maddi olarak oynatan
müdahale, ve KALICI.** SDF kuyruk panellerini `a0 = −0.2` ile kuruyor; bu,
V² ile büyüyen kalıcı bir AŞAĞI kuvvet (25 m/s'de 34 N = ağırlığın %70'i) ve
kontrolcü onu **göremediği** için yükü sessizce **rotorlar** taşıyor. Tek bir
sabit elevatör ofseti tüm hızlarda iptal ediyor (iki terim de q̄ ile ölçekli):
`cla·a0 + rad_to_cl·δ = 0` → **δ = −4.54°**. Bu, madde (P)'nin `fx_trim`'inin
yüzey tarafındaki ikizi.

Ölçüm (v_sp = 17, trimsiz 29.9 s'de ıraksıyor): oran 0.4→32.3 s, 0.6→34.9,
0.8→75.6, **1.0→82.9 (2.8×)**, 1.3→73.7, 1.6→64.3 — **tepe tam olarak
analitik değerde**, yani sayı ayarlanmadı. 16 m/s çalışma noktasında da
kazanç: toplam rotor itkisi **16.13 → 14.14 N (−%12)**.

**Sınır o turda kalkmadı** (2.8× gecikiyor, kayboluyor değil). Detay: rapor
Adım 49-52.

**🆕 Adım 53 (2026-08-04) — DUVAR KALKTI. Eksik olan bir kazanç değil, bir
İLERİ BESLEME TERİMİYDİ.** Duvar ilk kez kanal bazında ölçüldü (her tick'te
WLS'in *istenen* ve *başarılan* sanal kontrolü ayrı kaydedildi) ve sebep tek
bir cümleye indi: **gereken trim zaten küçüktü ve sınırın çok içindeydi; yasa
oraya zamanında varamıyordu.** 19 m/s'de ıraksamadan hemen önce gereken
`theta` = **−0.71°** (sınır 6°, otoritenin %12'si), ulaşılan = **−0.59°**.
Ne otorite ne doygunluk sorunu vardı — **yarış kaybedilmişti**: yasa saf
integraldi (`tau` ≈ 19-24 s) ve hız 16 → 19 m/s'ye çıkarken gereken trim de
hareket ediyordu.

Doğru cevap **kendi test dosyamızın içinde** yazılıydı — `D2` denetimi Adım
49'dan beri gereken dengeyi elde türetiyordu:

```
theta_ff(V) = (W + Fz_hedef) / (qbar·S·cla) − a0
```

Bu ifade artık yasanın **içinde**; integral yalnızca **model hatasını**
kapatıyor. **`pitch_ki` DEĞİŞMEDİ** (5e-5) — kazanç ×4 de duvarı kaldırıyordu
(ölçüldü) ama irtifa döngüsünden 10× yavaş olma garantisini 3.5×'e düşürürdü;
ileri besleme aynı sonucu bedava veriyor ve `E1` denetimi `tau`yu hâlâ 17.5 s
ölçüyor.

Ölçüm (kapalı çevrim, hız + pitch + yüzeyler, 120 s):

| `v_sp` | ileri besleme YOK | **ileri besleme VAR** |
|---|---|---|
| 16 / 17 / 18 | kararlı | kararlı |
| **19** | **ıraksıyor, 39.8 s** | **kararlı** |
| **20** | **ıraksıyor, 39.4 s** | **kararlı** |
| 22 / 24 / 26 | — | **kararlı** |

Tilt 51.3° → 77.1°, `Fz_sp` her hızda tam hedefte (−11.8…−12.0 N), `T0` ≈
7.3 N, irtifa sapması 1.2 m. **26 m/s komut edilince araç ıraksamıyor,
24.9 m/s'de dengeye oturuyor** — çünkü orada hızı sınırlayan şey bir
kararsızlık değil `p.tecs.fx_max = 13 N`, yani kasıtlı bir emniyet tavanı.
Sınır bir arıza olmaktan çıkıp bir **tercih** oldu.

**İki dürüstlük notu:** (1) Adım 52'nin "17 m/s'de 82.9 s'de ıraksıyor"
ölçümü bu turun probe'unda yeniden üretilmedi — 17 ve 18 m/s 150 s boyunca
kararlı, ıraksama 19'da başlıyor; iki probe birebir aynı değil ve buradaki
sayılar **bu turun probe'una** aittir (mekanizma aynı). (2) Kuyruk trimi bu
kapalı çevrim rejiminde aşağı yükü **kaldırmıyor** (tahsisat sanal elevatörü
park edip trimi geri alıyor), ama yine de kritik: kapatıldığında 16 m/s'de
tilt 90° mekanik durağa yapışıyor ve 20 m/s 49.7 s'de ıraksıyor. Faydası
"yükü kaldırması" değil, **kontrolcünün modeli ile plant'in aynı şeyi
söylemesi.**

**HÂLÂ AÇIK: bu tam kanat-taşımalı uçuş DEĞİL.** `Fz_sp` hedefi −12 N olduğu
için rotorlar tasarım gereği ağırlığın ~%24'ünü taşıyor (kanat yük payı
%75.6 → %86.3). Bu bir kusur değil, Adım 49'un ölçülmüş kararı: sıfır itki
bir uçurumdur (`dtau/ddelta ∝ T`). Rotorların gerçekten kapanabilmesi için
**Fx kanalına bir yüzey** gerekir — bugün hiç yok. Detay: rapor Adım 53.

**Doğrulanmış çalışma zarfı: `v_sp` = 16-24 m/s** (16'da tilt 51.3°, 24'te
77.1°). Tümü MATLAB; **PX4'e gitmedi** (seyir katmanının PX4 karşılığı yok).

**🆕 Adım 48 (2026-08-04) — ileri geçiş artık İPTAL OLMUYOR.** Gereksinim:
profil tek parçadır ve ileri geçiş kendini kesmez (`FT_ALLOW_ABORT = false`).
İki emniyet dedektörü **silinmedi** — çalışmaya devam ediyor ve `warn_code` ile
loglanıyor; yalnızca eylemleri kalktı. Bedeli açık: Adım 29'un kaçış tırmanışı
artık otomatik tepki üretmez, sadece rapor edilir. Kazancı ölçülmüş: iptalin
garantili bir kaçış yolu **zaten yoktu** — iptal, geri geçişi *istemek* demekti
(Adım 30: `fx_sp = 0` aracı yavaşlatmıyor) ve sabitler sıfır marj bırakıyordu:
`FT_MIN_ALT(20) − FT_ALT_BAND(5) = 15 = BT_MIN_ALT`, yani **alçalarak** iptal
eden bir geçiş tam olarak kaçışının reddedildiği irtifada iptal ediyordu.
İptal geri getirilecekse önce o marj kapatılmalı. Detay: rapor Adım 48.

**🆕 Adım 47 (2026-08-03) — hız döngüsü kuruldu, ve ASIL DUVAR bulundu.**
`cruise_speed_loop.m` (PI, bumpless devralma, anti-windup) `fx`'i açık döngü
olmaktan çıkardı: v_sp = 16 m/s'de araç hedefi tutuyor (15.99) ve **tahsisat
doygunluğu %15.7 → %2.5**'e düşüyor. Ama v_sp ≥ 17'de hâlâ ıraksıyor, ve sebebi
ölçüldü — yüzeyler ya da ağırlıklar değil (`wu_ele` 8 kat değiştirildi, hiçbir
şey değişmedi):

```
theta = 0'da kanadın tam ağırlığı taşıdığı hız = sqrt(W/(0.5·rho·S·cla·a0))
                                               = 16.925 m/s
```

Yüzeysiz araç, fx ne olursa olsun (8/10/12/14 N) hep **16.85-16.90 m/s**'de
duruyordu — bu bir tilt durağı değil, o hızın kendisi. **Pitch setpoint'i her
yerde sabit sıfır**, oysa 16.9 m/s üstünde seviye uçuş negatif hücum açısı
ister. Yani TECS'in eksik yarısı **pitch (enerji dağılımı)**; itki yarısı
kuruldu. Tam kanat-taşımalı uçuş (rotorların kapanması) o yarım olmadan bu
hızın üstüne çıkamaz. Detay: rapor Adım 47.

**Otonomi için kalan diğer iki eksik:**

1. **Seyirde enerji yönetimi — İKİ YARI DA KURULDU (Adım 47 itki, Adım 49+53
   pitch), ama yalnızca MATLAB'da.** 16-24 m/s zarfı kapalı çevrim kararlı.
   Adım 29'un uyarısı (burun yukarı = tırmanma komutu) korunuyor: pitch yasası
   13 m/s'nin altında **tam sıfır otoriteli**. Kalan: **PX4'e port** ve
   ardından `sitl-lockup-check`.
2. **Evreleri zincirleyen bir görev dizicisi yok.** Evreleri hâlâ dışarıdan
   gelen iki bayrak zincirliyor. PX4'ün `navigator` / `flight_mode_manager`'ı bu
   airframe'de kasıtlı olarak durdurulmuş (bkz. `.post` betiği), yani diziyi
   modülün kendisi yürütmeli.

**İki önemli kavramsal not:**

1. **Buradaki "sabit kanat" PX4'ün fixed-wing modu DEĞİLDİR.** Bu airframe
   bilinçle `rc.mc_defaults` ile açılıyor (`rc.vtol_defaults` ile değil) ve
   kontrolcünün PX4'ün VTOL durum makinesinden haberi yok: seyir, *aynı INDI
   kontrolcüsünün* rotorları ileri eğip kanadın taşımaya başladığı hâlidir.
   Tilt açısını bir mod değişimi değil, WLS tahsisatı seçer. Gerekçe:
   `vtol_att_control` tilti açık çevrim komut ederdi ve bu, WLS'in kendi tilt
   kararıyla doğrudan çakışırdı.
2. **Tilti izlemek isteyen, geri geçiş testini izlemeli.** Pilot testinde
   (`run_pilot_input_test.py`) tilt havada yalnızca **3.7-15.1°** oynar — çünkü
   pilot dalı `fx_sp = 0` sabitler (madde (V)) ve araç saf multikopter uçar.
   Tavanın **45° → 9° → 20°** rampası yalnızca `run_backtrans_test.py`'de
   görülür; kamera için `INDI_GZ_CAM=side` en okunaklısıdır.

Açık maddeler ve donanım durumu için → `HARDWARE_READINESS_CHECKLIST.md`.

---

## Kontrol mimarisi

```
 att_sp ─┐
         ├─► [P tutum döngüsü]──► omega_sp ─┐
   att ──┘        (200 Hz)                  ├─► [P hız döngüsü] ──► omega_dot_des
                                     omega ─┘        (400 Hz)             │
                                                                          ▼
   omega ────────► [LESO 200 Hz] ──► d_hat ────────────────────────►  (−) çıkarma
                                                                          │
   omega_dot_ölçülen ──────────────────────────────────────────────►  (−) çıkarma
                                                                          ▼
                                                                    dtau = I·Δω̇
                                                                          │
   z_sp, z, vz ──► [irtifa P+PI, 50 Hz] ──► Fz_sp ──┐                     │
   Fx_sp ────────────────────────────────────────────┴─► F_sp ──┐         │
                                                                ▼         ▼
                                          u_actual ──► [G(u) Jacobian + WLS tahsisi]
                                                                ▼
                                                    u_cmd = [T0 T1 T2; δ0 δ1 δ2]
```

### 1. Kademeli tutum döngüsü
Dış P döngüsü tutum hatasından `omega_sp` üretir (yaw hatası sarmalanır,
±3 rad/s doygunluk), iç döngü `omega_dot_des` üretir. Kazançlar
`gain_schedule.m` ile ortalama tilt açısına göre smoothstep interpolasyonuyla
hover↔cruise arasında geçiş yapar — ayrı bir durum makinesi yoktur.

### 2. INDI artımlı kontrol yasası
```
Δω̇  = ω̇_des_telafili − ω̇_ölçülen
Δτ   = I · Δω̇
```
Model tersine çevirme yerine **ölçülen açısal ivme geri beslemesi** kullanılır;
dayanıklılığın kaynağı budur.

### 3. LESO bozucu telafisi
Eksen başına 2. dereceden gözlemci (`leso_axis_update.m`), bant genişliği
wo = 15 rad/s, kazançlar `β1 = 2wo`, `β2 = wo²` (`leso_bandwidth_gains.m`).
200 Hz'de desimasyonlu çalışır ve yalnızca etkin eksenlerde (varsayılan:
roll + pitch) devrededir.

> **Kritik ayrıntı:** Gözlemcinin "girişi" olarak telafi **sonrası**
> (`omega_dot_des_adj`) değer beslenir. Telafi öncesi değer beslenirse ESO
> kendi telafisini görmemiş gibi davranır ve `z2` sınırsızca sürüklenir.

### 4. WLS kontrol tahsisi
`effectiveness_matrix.m` her adımda **anlık** 5×6 Jacobian `G = ∂ν/∂u`
hesaplar (ν = [τx; τy; τz; Fx; Fz], u = [T0 T1 T2 δ0 δ1 δ2]). `wls_allocate.m`
kutu kısıtlı ağırlıklı en küçük kareler problemini active-set ile çözer:

```
min ‖Ws·(G·Δu − ν_des)‖² + ‖Wu·(Δu − Δu_pref)‖²   s.t.  Δu_min ≤ Δu ≤ Δu_max
```

**Ağırlıklar `Ws = diag([200 200 3 0.05 20])`** — bu değerler deneysel olarak
ayarlanmıştır ve bu airframe'e özgü iki tekillikle başa çıkar:

- **Yaw önceliği düşük (3):** Yaw otoritesi (diferansiyel tilt) roll ile aynı
  aktüatörü paylaşır; eşit ağırlık WLS'i roll↔yaw arasında sönümsüz bir limit
  cycle'a sokar.
- **Fx önceliği çok düşük (0.05):** Yaw'ı düzelten diferansiyel tilt aynı
  zamanda kayda değer bir Fx üretir. Fx = 0 hedefi roll/pitch mertebesinde
  ağırlıklandırılırsa δ1 rate limitinde salınır ve ψ referansa asla yakınsamaz.
  ~0.1'in altında bu çekişme tamamen kalkar.
- **Fz önceliği yüksek (20)** tutulabilir, çünkü irtifa hatası
  `altitude_loop.m` integrali tarafından ayrıca kapatılır.

`Wu` ağırlıkları tilt açısına göre planlanır: hover'da tilt sapması
`cos(δ)` üzerinden dikey itkiyi hızla düşürdüğü için tilt pahalıdır
(wu = 3.0), cruise'a doğru ucuzlar (1.5). Kuyruk rotoru merkez hatta olduğu
ve tilt yoluyla roll/yaw üretemediği için ×3 ek ceza alır.

> **2026-07-26 kısmi iyileştirme (kök neden HÂLÂ ÇÖZÜLMEDİ):**
> `wu_tilt_hover` eskiden 8.0'dı; hover trim itkisinde tilt roll üretiminde
> thrust'tan ~3.665× daha etkili olduğundan (`|dtau_ddelta0| ≈ 0.916` vs
> `|dtau_dT0| = 0.25`), 8.0 bu eşiğin üstünde kalıp WLS'i roll hatasını
> tilt yerine bir kanat rotorünü sıfıra kadar tüketerek kapatmaya
> itiyordu. `wu_tilt_hover` 3.0'a düşürüldü (eşik ~3.665) ve tilt
> kullanımı gerçekten arttı — **ama SITL doğrulamasında kilitlenme
> tamamen ortadan kalkmadı, sadece hangi rotorün kilitlendiği değişti**
> (T0 yerine T1) ve sistem birkaç saniye sonra sönümsüz salınıma girdi.
> Kök neden, ağırlık oranından önce `nu_des`'in neden kalıcı/büyüyen
> tek-işaretli kaldığında (LESO `d_hat` drift'i mi, `hoverTrim()`
> uyumsuzluğu mu) yatıyor. Ayrıca gerçek bir **mekanik model uyuşmazlığı**
> bulunup düzeltildi (`ROTOR_KM` 0.05→0.06, gerçek SDF `momentConstant`
> ile eşleşecek şekilde) — bu da tek başına yetmedi, ama farklı/daha
> uzun vadeli bir arıza modu (yaw ekseninde sınırsız savrulma, ~20s
> sonra) ortaya çıkardı. Sorun hâlâ AÇIK. Ayrıntı ve sonraki adım için
> `sitl/RUNBOOK.md` §4 "Aday çözüm 2" ve "Aday çözüm 3" bölümlerine bakın.

> **2026-07-27 GÜNCELLEME — kök neden bulundu ve düzeltildi (Adım 11):**
> Yukarıdaki paragraf tarihsel kayıttır; aradığı kök neden WLS
> ağırlıklandırmasında DEĞİLDİ. Gerçek sebep PX4 çıktı katmanındaydı:
> modül itki komutunu `motors.control[i] = u_cmd(i)/ROTOR_TMAX` ile
> **doğrusal** gönderiyordu, oysa Gazebo zinciri karesel
> (`w = 10 + control·1490` rad/s, sonra `T = 2e-5·w²`) — iki model
> yalnızca 45 N uç noktasında uyuşuyordu. Bu hem gerçek toplam itkiyi
> ağırlığın (49 N) altında bırakıyor hem de WLS'in etkinlik matrisini
> itkiye bağlı biçimde bozarak (gerçek `d(T)/du`: 5 N'da 0.23, 45 N'da
> 1.99; kontrolcü hep 1.0 sanıyordu) düşük itkili rotoru tabana iten bir
> **pozitif geri besleme** yaratıyordu. `thrustToNormalized()` ile
> karekök tersleme eklendi. Ayrıca `test_sp` setpoint'inin kontrolcüye
> hiç ulaşmadığı ikinci bir hata bulundu — yani önceki tüm SITL koşuları
> yanlış senaryoyu test etmiş. Düzeltmelerden sonra iki bağımsız koşuda
> (25s ve 40s) 6 m tırmanış ~0.15 m hatayla takip ediliyor, **hiçbir
> aktüatör kilitlenmiyor**, roll/pitch ≤0.5°. **Kalan tek açık sorun:**
> yaw. Tam anlatım → `sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md` Adım 11.
>
> **2026-07-27 GÜNCELLEME #2 — ikinci kök neden (Adım 12):**
> `p.rotor.km` / `ROTOR_KM` işaretleri **FRD çerçevesinde tersti** (üç
> rotorda da). gz gövdeye `tau_z(FLU) = -turningDirection·T·km` uygular;
> FLU→FRD z çevrimiyle bu `+km·T` olur, model ise `m_i = km_i·T_i·dir_i`
> (`dir=(0,0,-1)`) ile `-km·T` diyordu. Sonucu: `hover_trim`'in yaw
> sıfırlayıcı diferansiyel tilt'i, rotor reaksiyon-torku dengesizliğini
> gidermek yerine **aynı yönde ekliyordu**. Adım 11 ile aynı sınıftan bir
> hata: **saf MATLAB bunu yapısal olarak göremez**, çünkü plant ve
> kontrolcü aynı `p.rotor.km`'yi paylaşır (ikisi birlikte yanlıştır).
> Düzeltme `[-0.06, +0.06, -0.06]` (MATLAB + Simulink + PX4) + `hover_trim`
> artık uygulanabilir (pozitif) tilt veren kanat rotorünü seçiyor.
> Etkisi: MATLAB regresyonu **iyileşti** (RMS p/q 0.0065/0.0015 →
> 0.0013/0.0004; transition max|omega| 0.0264 → 0.0126), SITL'de arm
> anındaki tepe yaw ivmesi **+6.5 → +0.47 rad/s² (14×)**.
> **Yaw hâlâ açık:** araç uçuş boyunca ort. +1.44 rad/s ile dönmeye devam
> ediyor. (Adım 11'in "±60° gezinme" ifadesi bir örnekleme artefaktıydı —
> yaw'ı açıdan değil **hızdan** ölçün.) Ölçülen mekanizma: tahsisat
> talep edilen yaw torkunun %6.8'ini üretebiliyor (tilt slew limiti) ve
> dış döngü dönerken yaw hız setpoint'ini sürekli ters çeviriyor.
> Ayrıntı → `sitl/WLS_LOCKUP_INVESTIGATION_REPORT.md` Adım 12.
>
> **2026-07-27 GÜNCELLEME #3 — YAW ÇÖZÜLDÜ (Adım 13):** Çözüm yaw'a
> otorite eklemek değildi. Dış attitude döngüsünün hız setpoint doygunu
> tek skalerdi (3.0 rad/s, üç eksen için); araç dönerken yaw hatası
> ±180°'de sarmalandığı için `omega_sp(3)` sürekli işaret değiştiriyor ve
> **zamanın yarısında iç döngü dönüşü sönümlemek yerine hızlandırıyordu**.
> Limit eksen bazlı yapıldı: `p.ctrl.rate_sp_limit = [3.0; 3.0; 0.5]`
> (MATLAB + Simulink + PX4). Sonuç: yaw hızı RMS 1.91 → **0.058 rad/s**,
> 25 s'de integre dönüş 1879° → **44°**, yerleşmiş yaw hatası **≤1.6°**;
> ayrıca roll/pitch hız RMS'i 30-65× ve irtifa hata RMS'i 0.456 →
> 0.211 m iyileşti. `yaw_sp = +30°` komutu 8 s'de, salınımsız izleniyor.
> **`sitl-lockup-check`'in üç kriteri de ilk kez aynı koşuda geçti.**
> **AMA — 2026-07-27 GÜNCELLEME #4 (Adım 16), bu sonucu daraltıyor:**
> yaw'ı asıl sönümleyen şey kontrolcü değil, ileri hızdaki aerodinamik
> rüzgâr gülü etkisiymiş. Tek uçuşta kontrollü ölçüm: 11.6 m/s'de +30°
> yaw adımı monoton ve temiz (~%13 aşım), **2.45 m/s'de ±25°'lik
> sönümsüz salınım**. Bu kontrolcüde yatay pozisyon döngüsü olmadığı ve
> araç yapısal ileri kuvvetle sürekli hızlandığı için (aşağıdaki madde)
> **bugüne kadarki tüm "hover" testleri aslında ~10 m/s seyir uçuşuydu**;
> gerçek anlamda yerinde duran hover — bir pozisyon kontrolcüsünün
> komut edeceği asıl durum — yaw için en kötü koşul ve orada kriter
> sağlanmıyor. Donanım durumu bu nedenle 🔴 NO-GO'ya geri alındı.
> Genel ders: zayıf otoriteli bir eksende dış döngü hız limiti, o eksenin
> gerçekten ulaşabileceği hızın ALTINDA tutulmalıdır — yoksa dış döngü iç
> döngünün sönümlemesini engeller.
> Yeni açık maddeler: hover'da yaw trim'i kaçınılmaz ileri kuvvet üretiyor
> (tilt aralığı [0°,90°] tek yönlü) → pozisyon döngüsü olmadığı için araç
> 25 s'de 235 m sürükleniyor; ve agresif adım-alçalmada rotorler ~68°'ye
> eğilip bir itki kanalı sıkışıyor. Ayrıntı → rapor Adım 13 ve §4 (L/N/O).
>
> **2026-07-28 GÜNCELLEME #5 (Adım 17) — Adım 16'nın MEKANİZMASI çürütüldü,
> ÖLÇÜMÜ duruyor.** Yeni `run_yaw_step_test.m` ile yaw adım yanıtı ilk kez
> saf MATLAB'da ölçüldü. Kritik nokta: bu plant'in aero modeli tamamen
> boylamsal olduğundan `M_aero(3) ≡ 0` — yani MATLAB **hiçbir hızda**
> aerodinamik yaw sönümlemesi üretmez, yaw açısından yapısal olarak "gerçek
> hover"dır. Buna rağmen ±30° adımı 3.1-3.7 s'de, **kalıcı salınım olmadan**
> oturuyor (son 5 s yaw hızı RMS ≤ 0.0001 rad/s; SITL'in aynı senaryosunda
> ±0.5 rad/s). Yani "düşük hızda rüzgâr gülü sönümlemesi kayboluyor" tek
> başına yaw sorununu açıklamıyor — SITL'e özgü ek bir kararsızlaştırıcı
> var ve aero sönümleme onu yalnızca maskeliyor. **En güçlü aday:** PX4
> portundaki `_u_actual` **açık çevrim gölge aktüatör modeli** (Gazebo'dan
> hiç geri besleme okumuyor), oysa MATLAB'da kontrolcü gerçek aktüatör
> durumunu görüyor — Adım 11 ve 12 ile aynı sınıftan bir kontrolcü/plant
> arayüz uyuşmazlığı. Destekleyen kanıt: iki ortam `TILT_RATE_MAX` 2.0↔3.0
> konusunda **ters işaretli** sonuç veriyor. Ayrıca **yaw adımının yön
> asimetrisi MATLAB'da yeniden üretildi ve sebebi kanıtlandı**: hover
> trim'de δ1 tek yönlü tilt aralığının tabanına (0°) çakılı olduğu için
> −yaw serbest, +yaw sürekli sınıra vuruyor (+30° aşımı `rate_max`=2.0'da
> %59.2, −30° %8.0). Ayrıntı → rapor Adım 17 ve §4 (Q)/(P).
>
> **2026-07-28 GÜNCELLEME #6 (Adım 18) — ölçüldü: gölge model şüphesi elendi,
> (P) ile (Q) tek köke bağlandı.** SITL'de düşük hızda (1.5-2.3 m/s) gerçek
> Gazebo tilt eklem açıları 250 Hz'de kaydedilip PX4'ün iç `_u_actual` gölge
> modeliyle karşılaştırıldı. Sapma **p99'da ≤ 0.55°** — aynı pencerede yaw
> **73.7°** bantta salınırken; yani gölge model gerçek bir katkı payı ama
> **baskın neden değil.** Buna karşılık koşu iki şey gösterdi: (1) **(Q) bir
> adım yanıtı kusuru değil, denge kararsızlığı** — sürekli salınım
> `yaw_sp = 0` iken oluşuyor (periyot 5.18 s, yaw hızı RMS 0.417 rad/s);
> (2) salınımın en güçlü korelatı **δ1'in tilt alt sınırından (0°) kalkıp
> geri çakılması** (salınım boyunca 12 olay, salınım durunca sıfır) — yani
> **(P) ve (Q) muhtemelen aynı kök nedenin iki görünümü: tek yönlü tilt
> aralığı.** Yan bulgu: gölge δ1/δ2 örneklerin %75/%97'sinde tam `0.000°`
> okurken gerçek eklemler 0.53°'de duruyor, dolayısıyla WLS bu eksenlerde
> 0.53°'lik gerçek hareket alanını göremiyor. Ayrıntı → rapor Adım 18.
>
> **2026-07-28 GÜNCELLEME #7 (Adım 19) — hız hipotezi de çürütüldü, (Q)
> nihayet doğru tarif edildi.** `yaw_sp` tüm uçuş boyunca 0'da sabit
> tutulup **yalnızca ileri hız** değiştirilen tek değişkenli bir A/B
> koşuldu. Salınım, yatay hız **2.00-2.09 m/s'de sabitken** söndü (yaw hızı
> RMS 0.439 → 0.006); hızlanma bundan *sonra* başladı. Yani **ileri hız,
> salınımı bitiren şey değil** — Adım 16'nın aerodinamik rüzgâr gülü
> mekanizması, Adım 17'nin MATLAB kanıtının yanına gelen ikinci bağımsız
> kanıtla çürütüldü. Dahası salınım **sönümsüz değil**: genlik düzenli
> biçimde sıfıra iniyor, **yerleşme süresi ~30-35 s**, iki bağımsız uçuşta
> tekrarlandı. Yani (Q) "kararsız yaw" değil, **çok zayıf sönümlü bir yaw
> modu** — ama bir pozisyon kontrolcüsü bu modu sürekli yeniden uyaracağı
> için hâlâ kabul edilemez. Adım 18'in önerdiği "tilt trim'ini ön-yükle"
> fikri de MATLAB'da çürütüldü: WLS'in amaç fonksiyonunda `du_pref = 0`
> olduğundan `Fx→0` hedefi δ1'i her tohumdan (bias 0/5/10°) aynı dengeye
> (δ0=9.00°, δ1=0.00°) geri itiyor. **Kalan tek nicel boşluk:** MATLAB aynı
> ±30° adımını 3.1-3.7 s'de oturtuyor, SITL ~8 s (adım) / ~33 s (arm
> geçicisi) alıyor — bu 2-10×'lik sönümleme farkı hâlâ açıklanmadı.
> Ayrıntı → rapor Adım 19 ve §4 (Q).
>
> **2026-07-28 GÜNCELLEME #8 (Adım 20) — yukarıdaki #7'nin iki çıkarımı GERİ
> ALINDI, ve (Q) nihayet üç uçuşla doğru tarif edildi.** Üçüncü bir uçuşta
> salınım 1.46-1.87 m/s'de **112 saniye boyunca hiç sönmedi** (Adım 19'un
> söndüğü uçuşta hız platosu 2.00-2.10 m/s idi). Yani sistem **kararlılık
> sınırında**: **<~2 m/s sürekli salınım, ~2 m/s marjinal, üstünde kararlı.**
> #7'deki "ileri hız sebep değil" ve "~30-35 s'de oturan zayıf sönümlü mod"
> ifadeleri tek uçuşa dayanan aşırı genellemelerdi; ikisi de geri alındı.
> Ayrıca yeni `run_yaw_ablation.m` ile SITL'in dört kusuru (PX4'ün açık
> çevrim gölge aktüatör modeli, `omega_dot` +8 ms gecikme, `omega_dot`
> gürültüsü, %25 `dt` jitter) MATLAB'a tek tek ve birlikte enjekte edildi:
> **hiçbiri yerleşme süresini değiştirmedi** (3.64-3.70 s). Yüksek hızın
> neden stabilize ettiği SDF'den türetildi: tek yanal yüzey olan dikey
> kuyruğun (alan yalnızca 0.032 m²) sönümleme türevi 2 m/s'de 0.27, 11.6
> m/s'de 9.1 Nm/rad — 34× — ve ±25-40°'lik salınımda yan kayma kuyruğun
> 19.4°'lik stall açısını aşıyor. **Ama aero, düşük hızdaki kararsızlığın
> kendisini açıklamıyor:** MATLAB'da hiç aero yaw momenti yokken sistem
> ζ≈0.4 ile rahatça kararlı. **SITL'de MATLAB'da bulunmayan, henüz
> bulunamamış bir kararsızlaştırıcı var** — araştırmanın açık merkezi bu.
> Ayrıntı → rapor Adım 20 ve §4 (Q).
>
> **2026-07-28 GÜNCELLEME #9 (Adım 21) — PX4 portunda GERÇEK BİR KOD HATASI
> bulundu.** Önce itki kanalı da elendi: gerçek Gazebo itkisi rotor eklem
> hızından türetilip (`T = 2e-5·(ω·20)²`; doğrulama: toplam 49.62 N ≈ ağırlık
> 49.05 N) gölge modelle karşılaştırıldı — sapma ~%0.1, yaw reaksiyon torkuna
> etkisi otoritenin %0.4'ü. Asıl bulgu tahsisat verimi ölçülürken çıktı:
> `MulticopterIndiTiltrotor.cpp:315-316, 324-325` **WLS'in slew kutusunu sabit
> `TS_CTRL = 1/400` ile boyutluyor**, oysa modül `vehicle_angular_velocity`
> callback'iyle **250 Hz'de** dönüyor (aynı fonksiyon `dt`'yi 171. satırda
> doğru hesaplayıp gölge model, LESO ve irtifa için kullanıyor). Ölçümle
> doğrulandı: `|ddelta|` p99 tam **0.00500 rad**, tick **4.00 ms**, tilt
> kanalları zamanın **%99.4-99.9**'unda doyumda, tahsisat yaw verimi **%20.6**.
> Sonuç: **efektif tilt slew tavanı 1.25 rad/s — hedeflenen 2.0'ın %62'si**, ve
> kanat tilt'i yaw'ın tek gerçek aktüatörü. Bu, Adım 11 (itki eşlemesi) ve
> Adım 12 (km işareti) ile **aynı sınıftan** bir hata: kontrolcünün varsayımı
> ile ortamın gerçeği uyuşmuyor, ve **saf MATLAB bunu yapısal olarak göremez**
> çünkü orada döngü gerçekten `p.Ts_ctrl` periyodunda koşar. Ayrıca Adım 14'ü
> geriye dönük açıklıyor (nominal 2.0 → efektif 1.25 çalıştı; nominal 3.0 →
> efektif 1.875 ıraksadı) ve **naif düzeltmenin zararlı olacağını öngörüyor**:
> kutuyu gerçek `dt` ile boyutlamak nominal 2.0'ı 2.0 efektif yapar, yani
> ıraksatan 1.875'in de üstüne. Ayrıntı → rapor Adım 21 ve §4 (Q).
>
> **2026-07-28 GÜNCELLEME #10 (Adım 22) — kutu ayrıştırıldı, davranış-nötr
> düzeltme SITL'de doğrulandı.** Önce bir düzeltme: #9'daki "kod hatası"
> nitelemesi fazla sertti — sabit periyot kullanmak jitter'a karşı **kasıtlı**
> bir korumaydı (kod yorumunda yazılı); gözden kaçan, nominal periyodun
> döngünün *gerçek* periyoduyla eşleşmesi gerektiğiydi. Yapılan: PX4'te
> `TILT_RATE_MAX` (2.0 rad/s) artık **yalnızca gölge modelin fiziksel servo
> limiti**; tahsisat kutusu ayrı bir sabit çiftine taşındı —
> **`TILT_SLEW_BOX_RATE = 1.25 rad/s` × `TS_BOX = 1/250`**. Bu ikisi aynı
> şey değildi: biri servonun *fiziksel olarak yapabildiği*, diğeri tahsisatın
> *tek tick'te isteyebileceği*. Sonuç tilt kutusunda 1 ULP farkla aynı
> (0.005 vs 0.0050000004), yani **davranış-nötr** — SITL'de doğrulandı:
> `|ddelta|` p99 hâlâ tam 0.00500 rad, itki doyumu %0.0, **aktüatör
> kilitlenmesi yok** (itki 8.26-19.10 N), `|vz|` ≤ 0.78 m/s, irtifa hata RMS
> 0.234 m. Yaw kriteri hâlâ kalıyor (37.09°) — bu değişiklikten değil, düşük
> hızdaki (Q) salınımından. **Kazanç davranışta değil, ölçülebilirlikte:**
> sabit artık gerçek rad/s anlamına geliyor ve taranabilir (1.25 çalışıyor,
> 1.875 ıraksıyordu — ilgi aralığı arası). MATLAB tarafına dokunulmadı
> (orada bu hata yapısal olarak yok). Ayrıntı → rapor Adım 22.
>
> **2026-07-28 GÜNCELLEME #11 (Adım 23) — ⭐ (Q)'NUN MEKANİZMASI BULUNDU.**
> Kutu hızını **uçuş içinde** değiştirebilen bir test kancası (`slewbox`)
> eklenip iki koşuda, **ters sıralarda**, her değerde aynı +30° yaw adımı
> uyarımıyla tarandı. Adımın son 5 s'sindeki yaw hızı RMS:
>
> | kutu (rad/s) | koşu A | koşu B | test edilen hız | sonuç |
> |---|---|---|---|---|
> | **1.25** (eski) | 0.583 | 0.466 | 2.20 / 0.86 m/s | **salınıyor** |
> | 1.50 | 0.391 | 0.005 | 2.02 / 2.00 m/s | marjinal |
> | **1.75** | 0.0037 | 0.0051 | 1.35 / 2.76 m/s | **sakin** |
> | **2.00** | 0.0056 | 0.0055 | 0.81 / 3.14 m/s | **sakin** |
>
> **Hız kesin olarak elendi:** 1.25 hem 0.86 hem 2.20 m/s'de salınıyor,
> 1.75/2.00 ise 0.81-3.14 m/s aralığının tamamında sakin. Yani düşük hızdaki
> yaw salınımı **kontrol yasasında bir sönümleme eksikliği değil, tahsisatın
> tilt slew'undan aç bırakılmasıymış.** MATLAB çapraz kontrolü de tutuyor:
> onun efektif kutusu `3.0·(1/400)` @ gerçek 400 Hz = **3.0 rad/s**, yani
> denenen her değerin üstünde, ve orada aşım %24.1 / yerleşme 3.69 s — trend
> düzgün ekstrapole oluyor. Adım 14 çelişmiyor, **açıklanıyor**: o, kutuyu ve
> gölge modelin fiziksel limitini *birlikte* oynatmıştı; #10'daki ayrıştırma
> kutunun tek başına yükseltilmesini mümkün kılan şey. Varsayılan
> **1.25 → 1.75** yapıldı (fiziksel limite 0.25 rad/s pay). Doğrulama:
> kilitlenme ✅, dikey hız ✅ (0.816 m/s), roll/pitch ✅ (±0.06°); **yaw
> kriteri hâlâ ❌** (tepe 35.80° vs 37.09°) — kalan aşım arm geçicisinin tek
> seferlik salınımı, asıl kazanç **kalıcı salınımın yok olması (RMS ~0.5 →
> ~0.005, 100×)**. Ayrıntı → rapor Adım 23.
>
> **2026-07-28 GÜNCELLEME #12 (Adım 24) — "gölge modeli gerçek servoya sadık
> kıl" DENENDİ, GERİ ALINDI.** SDF'den gerçek servo: `JointPositionController`
> p_gain=100, cmd_max=2 N·m, eklem sürtünmesi 1.0 N·m, etkin atalet
> J=0.0168 kg·m². İki türetilmiş sayı: **Coulomb ölü bandı =
> friction/p_gain = 0.573°** ve max ivme 59.4 rad/s². Ölü bant, #9'da ölçülen
> kalıcı 0.52-0.53°'lik ofseti niceliksel açıklıyor. Çevrimdışı doğrulamada
> (kayıtlı komut dizisi → gerçek eklem açısı) gölge-gerçek hata RMS'i:
> 1. derece 0.287/0.408/0.554° → **tam 2. derece 0.414/0.462/0.553° (DAHA
> KÖTÜ)** → **1. derece + ölü bant 0.082/0.051/0.0040° (3.5×/8.0×/139× daha
> iyi)**. Yani sadakat açığının tamamı sürtünme ölü bandıymış; atalet birkaç
> ms'de oturduğu için 2. derece dinamik 250 Hz'de alâkasız. **Ama
> uygulandığında kapalı çevrimde KİLİTLENDİ:** `u_cmd = u_actual + du` komutu
> gölgeye bağlıyor, `du` ise slew kutusuyla 0.40° ile sınırlı — ölü banttan
> küçük — dolayısıyla hiçbir tick sürtünmeyi kıramıyor, gölge ve komut birlikte
> donuyor (SITL: üç tilt de tüm uçuş donuk, yaw bandı 238°, araç dönüyor).
> Geri alındı, baseline doğrulandı. **İki ders:** (1) *açık çevrim replay,
> kapalı çevrim geri besleme tuzağını gösteremez* — çevrimdışı doğrulama kendi
> içinde doğruydu ve yine de ölümcül sorunu kaçırdı; (2) *artımlı allocator
> mimarisi gölgenin komuta sürünmesini zorunlu kılıyor*, stiction bununla
> temelden uyumsuz. Ayrıntı → rapor Adım 24.
>
> **2026-07-28 GÜNCELLEME #13 (Adım 25) — "arm geçicisini kaynağında küçült"
> yolu da kapandı; premis iki kez çürüdü, kod değiştirilmedi.**
> (1) *Tilt ön-konumlandırma gereksiz:* disarm'da servolar NaN alıyor ve 0°'de
> duruyor, arm'da gölge δ0 ≈ 9.39°'de tohumlanıyor — gerçek bir 9.39°'lik
> başlangıç uyuşmazlığı var, ama **gerçek δ0 trim'in %90'ına 72 ms'de
> ulaşıyor** ve +0.20 s'de gölgeyle 0.16° içinde örtüşüyor; oysa yaw hızı
> *saniyeler* boyunca birikiyor. Zaman ölçekleri uyuşmuyor.
> (2) *Trim itkiden bağımsız geçerli:* makul görünen "diferansiyel tilt hover
> itkisine göre boyutlanmış, tırmanışta reaksiyon torku büyüyünce bozulur"
> hipotezi yanlış — **`τ_tilt = −Σ py·T·sin δ` de içinde `T` taşıyor**, tıpkı
> `τ_react = −Σ km·T·cos δ` gibi; ikisi birlikte ölçekleniyor ve oran
> korunuyor (itki 49.7 → 68.9 N çıkarken NET tork −0.021 → −0.011 N·m).
> **Sonuç: ayrı bir "arm geçicisi" mekanizması yok** — arm sonrası savrulma
> madde (Q)'nun kendisi, zayıf yaw ekseninin 0.03-0.15 N·m'lik artık torklara
> yanıtı (tahsisat yaw otoritesi adım başına ~0.033 N·m). **Genel ders: bir
> dengesizliğin "ölçekle bozulduğunu" iddia etmeden önce, onu dengeleyen
> terimin aynı ölçekle büyüyüp büyümediğine bakın.** Ayrıntı → rapor Adım 25.
>
> **2026-07-30 GÜNCELLEME #14 (Adım 37) — geri geçişin FREN YASASI düzeltildi;
> Adım 31'in "doğrulandı" kaydı marjinalmiş.** Mevcut tilt geçişleri Gazebo
> arayüzü açık şekilde yeniden koşuldu (yeni `INDI_SITL_GUI=1` anahtarı) ve
> manevra **iki bağımsız uçuşta devir hızının üstünde takıldı**: v_h 3.2-3.5
> m/s'de 90+ s kararlı denge, `BT_HANDOFF_V = 3.0`'a hiç inemedi, `pos_hold`
> hiç istenmedi. Ölçülen sebep: `pitch = BT_PITCH_MAX·v_h/BT_BRAKE_V_FULL`
> hızla **sönüyor**, ama yenmesi gereken ileri kuvvet **sönmüyor** — δ1/δ2
> `TILT_MIN`'de çakılı olduğu için (madde (P)) yaw trimi δ0'ı 10-15°'de tutuyor
> ve kalıcı 3.1-4.1 N üretiyor; yalnızca *durmak* için gereken açı
> `asin(fx_trim/(m·g)) = 3.39°`, `BT_PITCH_MAX = 4°`'nin hemen altında.
> İlk düzeltme (`BT_BRAKE_V_FULL` → `BT_HANDOFF_V`) **yetmedi** — ikinci uçuş
> bu kez 4.9 m/s'de takıldı, çünkü otorite (3.42 N) bozucunun dağılımının
> (3.1-4.1 N) tam içinde. **Asıl düzeltme yasayı AYRIŞTIRMAK:**
> `pitch = asin(FX_TRIM/(m·g))` (sönmez, duruş trimi) `+ BT_PITCH_MAX·fade`
> (sönen frenleme payı). İki doğrulama uçuşu: 15.38/14.66 m/s → 0.13/0.07 m/s,
> BRAKE evresi 92-99 s'lik takılmadan **5.7-6.1 s**'ye indi, yaw −3.4/+1.1°,
> irtifa bandı 1.11-1.14 m, itki doyumu %0.00, 0 BIG_M; MATLAB regresyonu tam
> nötr, zorunlu `sitl-lockup-check` geçti. **Genel ders: bir kontrol yasasını
> "bozucu" ve "otoritem" diye ayrıştırın — burada bozucu sabitti, otorite
> sönüyordu ve tek terimli yazıldığı için bu uyumsuzluk görünmüyordu.**
> Ayrıntı → rapor Adım 37.
>
> **2026-08-03 GÜNCELLEME #15 (Adım 39) — madde (S) kapatıldı: bir eşik,
> yasasının KONTROL ETTİĞİ ekseni okumalı.** `BRAKE → HANDOFF` koşulu
> `v_h < BT_HANDOFF_V` idi ve `v_h` bir BÜYÜKLÜK; fren yasası ise yalnızca gövde
> ileri eksenini kontrol ediyor ve yanal eksende HANDOFF'a kadar hiçbir kontrol
> yok. Ölçülen arıza (Adım 38, log 14_13_11): `min|v_h| = 3.08` iken `u = −0.51`,
> yanal `v = +3.04` — araç ileri yönde durmuştu, eşiği tutan şey manevranın
> kaldıramadığı bileşendi; handoff hiç istenmedi, sönmeyen fren pitch'i araç
> **geri yönde 12.8 m/s**'ye kaçana kadar itti. Düzeltme iki terimli:
> (1) çıkış **işaretli** `v_fwd < BT_HANDOFF_V` okur (`v_fwd < 0` da geçerli
> çıkıştır), (2) frenleme marjı `max(0, v_fwd)` ile söner, yani geri giderken
> marj SIFIRDIR — **kaçışı fiilen durduran terim budur.** RETRACT bilerek
> büyüklükte kaldı: oradaki soru aerodinamiktir ("kanat hâlâ taşıyor mu").
> İki normal + bir kancalı uçuş, plantsız durum makinesi testi (13/13, ve eski
> mantığın aynı izde 300 s takıldığı gösterildi), nötr MATLAB regresyonu,
> `sitl-lockup-check` geçti. **Ama kancalı uçuş rejimi HİÇ KURMADI** (yanal
> yalnızca 1.28 m/s), o yüzden asıl kanıt yeni bir probe'dan geldi
> (`probe_lateral_handoff.py`, frenlerken heading +90°): yanal 4.76 m/s,
> HANDOFF `|v_h| = 5.04`'te oldu — **eski koşul o anda sağlanmıyordu.**
> **Ve probe benim kendi ölçütümde aynı hatanın dördüncüsünü buldu:** madde
> (S)'yi yakalasın diye eklediğim "işaretli ileri hız ≥ −2.0" ölçütü KALDI
> (−3.95) ama araç kaçmamıştı — heading 186.7° dönmüştü ve `v_fwd` DÖNEN bir
> çerçeveye izdüşüm. Ölçüt çerçeveden bağımsız hâle getirildi (fren
> penceresinde yeniden-hızlanma ≤ 1.0 m/s) ve **gerçek arıza log'una karşı**
> doğrulandı (10.31 m/s → KALDI). **Genel ders: bir ölçüt de bir kontrol yasası
> kadar sinyal seçer ve aynı hatayı yapabilir; ölçütü geçmesi gereken uçuşlarda
> değil, KALMASI gereken uçuşta sınayın.** Ayrıntı → rapor Adım 39.

### 5. İrtifa dış döngüsü
`altitude_loop.m`: P (pozisyon) + PI (dikey hız, anti-windup clamp), 50 Hz.
Sabit `Fz_sp = −m·g` kullanıldığında uzun hover'da tilt kaynaklı dikey itki
kaybı telafi edilemiyor ve irtifa düşüyordu; integral terim bunu kapatır.

---

## Çalıştırma

### Saf MATLAB (Simulink gerekmez)

```matlab
cd('tiltrotor_Matlab files')
run_hover_gust_test      % hover'da bozucu reddi, LESO açık vs kapalı
run_transition_test      % hover -> cruise tilt geçişi
run_yaw_step_test        % yaw adım yanıtı (+-30 derece), aşım/yerleşme/salınım
run_yaw_ablation         % SITL kusurlarını enjekte eden ablasyon (tanı aracı)
```

Hepsi konsola metrik yazar ve klasöre PNG kaydeder (`hover_gust_test.png`,
`transition_test.png`, `yaw_step_test.png`, `yaw_ablation.png`).

- **`run_hover_gust_test`** — t = 4 s'de roll ekseninde yavaş değişen sentetik
  bir bozucu moment ve bir rüzgar esintisi (kontrolcünün bilmediği aero model
  üzerinden pitch bozucusu) uygulanır. LESO açık/kapalı iki koşu karşılaştırılıp
  bozucu penceresindeki RMS p/q hataları raporlanır.
- **`run_transition_test`** — `Fx_sp` 12 s'de 0 → 10 N rampalanır. WLS'in
  rotorları kendiliğinden öne yatırması ve gain-scheduling'in tilt açısıyla
  birlikte kayması gösterilir.
- **`run_yaw_step_test`** — t = 4 s'de `yaw_sp = ±30°` adımı, 25 s. Yaw
  ekseninin en kötü koşulunu ölçer: bu plant `M_aero(3) ≡ 0` olduğu için
  **hiçbir hızda aerodinamik yaw sönümlemesi üretmez** (SITL'de yaw'ı asıl
  sönümleyen şey ileri hızdaki rüzgâr gülü etkisi — bkz. yukarıdaki Adım
  16/17 notları). Metrikler: aşım %, ±2° yerleşme süresi, kalıcı hata ve
  **kalıcı salınım göstergesi** (son 5 s yaw hızı RMS). Her iki yönü de
  koşar, çünkü yanıt yön-asimetriktir. PX4 portuyla aynı tilt slew
  limitinde koşmak için:
  `setenv('YAW_TEST_TILT_RATE_MAX','2.0'); run_yaw_step_test`
- **`run_yaw_ablation`** — tanı aracı (Adım 20). SITL'de olup MATLAB'da
  olmayan kusurları tek tek ve birlikte enjekte eder: PX4'ün **açık çevrim
  gölge aktüatör modeli**, `omega_dot` taşıma gecikmesi, `omega_dot`
  gürültüsü, kontrol adımı jitter'ı. Amaç, SITL'in düşük hızdaki yaw
  kararsızlığını MATLAB'da yeniden üretebilen kusuru bulmak. **Hiçbir
  kontrol sabitini değiştirmez.** Şu ana kadarki sonuç: dördü de etkisiz —
  yani aday listesi hâlâ eksik.

Her iki betik de kontrolü 400 Hz'de, fiziği RK4 ile 5 alt-adımda (2 kHz)
entegre eder.

### Simulink

```matlab
tiltrotor_indi_build     % tiltrotor_indi.slx'i sıfırdan programatik kurar
sim('tiltrotor_indi')    % veya Simulink'te Run
```

`tiltrotor_indi_build.m` blokları ekler, MATLAB Function bloklarının içeriğini
`sf_*.m` dosyalarından yükler, hatları bağlar ve modeli kaydeder. Çözücü:
sabit adım `ode4`, `FixedStep = 0.0025` (400 Hz).

Modeldeki iki tasarım kararı:
- **Unit Delay bloğu (`u_cmd_delay`)** — plant'in doğrudan-besleme çıkışı WLS
  üzerinden girişine döndüğü için oluşan cebirsel döngüyü kırar; aynı zamanda
  gerçek donanımdaki bir adımlık hesaplama gecikmesini doğru temsil eder.
- **Durum, Unit Delay ile dışarıdan geri beslenir** — MATLAB Function blokları
  içinde `persistent` kullanmak sürekli örnekleme zamanıyla çelişir. Bu yüzden
  `sf_indi_rate_law` LESO/filtre durumunu 13×1 vektör olarak, `sf_altitude_loop`
  integralini skaler olarak dışarı verir.

---

## Bilinen sınırlamalar

- **MATLAB referansında pozisyon/hız dış döngüsü yok.** Orada sadece irtifa
  kapalı çevrimdir; `Fx_sp` elle verilir ve `run_transition_test` tam bir
  hover→cruise geçiş kontrolcüsü değil, iç döngünün değişen tilt açısına
  tepkisinin gösterimidir. **PX4 tarafı bunu geçti:** Adım 28'de yatay pozisyon
  döngüsü (`positionLoop`, hover-only, 3 m/s'de kapılı) ve Adım 31-39'da
  dört durumlu geri geçiş makinesi eklendi. **Ama hover→cruise yönü hâlâ bir
  KONTROLCÜ değil, bir `fx_sp` girdisidir** — ve pilot onu komut edemiyor
  (madde (V), yukarıdaki CONOPS tablosu).
- **Yaw otoritesi zayıf.** Airframe'in kendisinden gelen bir kısıt (diferansiyel
  tilt roll ile aynı aktüatörü kullanıyor); yaw kasıtlı olarak yavaş ve düşük
  öncelikli tutulmuştur.
- **`omega_dot` doğrudan modelden okunuyor** (tek kutuplu α = 0.3 filtresiyle).
  Gerçek uçuşta bu bir IMU türevidir; gürültü/gecikme modellenmemiştir.
- **Aero modeli basit** — yalnızca boylamsal (X-Z), sabit `Cd`, yanal
  aerodinamik yok.
- **`sf_wls_alloc.m` active-set yerine büyük-M ceza yöntemi** kullanır (kod
  üretimi için sabit boyut gerekliliği). Aynı sonuca yakınsar ama birebir aynı
  algoritma değildir.
- Simulink'te irtifa döngüsü 50 Hz yerine 400 Hz'de çalışır (blok örnekleme
  zamanını programatik ayarlama sorunu); integral örnekleme hızından bağımsız
  olduğu için matematiksel olarak eşdeğerdir.

---

## Dosya yapısı

Dosya ve klasörlerin tek tek açıklaması için bkz. **`dosya_ve_klasor_yapisi.md`**.
