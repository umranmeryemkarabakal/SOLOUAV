# WLS Aktüatör Kilitlenmesi / Yaw Savrulması — Araştırma Raporu

> Bu, `RUNBOOK.md` §4'teki canlı araştırma logunun (ham veri, komut
> dizileri, tick-bazlı teşhis) özet/rapor biçimidir. Tam ayrıntı ve
> yeniden-üretim komutları için `RUNBOOK.md` §4'e bakın.

> ## ► BURADAN BAŞLAYIN (son güncelleme: 2026-07-28, Adım 26 — YARIM KALDI)
>
> ### ⏸ OTURUM BURADA DURDURULDU — devam etmek için tek iş:
> `TiltrotorIndiParams.hpp`'de **`TILT_SLEW_BOX_RATE` 1.75f → 3.00f** yap,
> `make px4_sitl_default`, sonra **`sitl-lockup-check` kriter koşusu**.
> Kanıt hazır (Adım 26c: iki koşu, ters sıra, ort. aşım %24.8 = MATLAB
> paritesi), ama kriter koşusu **yapılmadı** — tahsisatın tick başına
> otoritesini artırmak aktüatör kilitlenmesi arıza modunu yeniden
> açabilecek türden bir değişiklik, atlanmamalı. Ağaç şu an **doğrulanmış
> 1.75'te ve kaynak/ikili uyumlu.**
>
> **Asıl arıza çözüldü, ama yaw ekseni gerçek hover'da hâlâ sönümsüz.**
>
> | | Durum |
> |---|---|
> | Aktüatör kilitlenmesi (asıl bug) | ✅ **ÇÖZÜLDÜ** — Adım 11 (itki eşlemesi) + Adım 12 (`ROTOR_KM` işareti). 845 örneklik koşuda sıfır BIG_M. |
> | Dikey hız kontrolsüzlüğü | ✅ **ÇÖZÜLDÜ** — `\|vz\| ≤ 0.17 m/s` |
> | Yaw'ın sınırsız dönmesi | ✅ **ÇÖZÜLDÜ** — Adım 13 (eksen-bazlı hız limiti), iki bağımsız koşu |
> | **(Q) Yaw, düşük hızda kararsız** | ✅ **ÇÖZÜLDÜ (Adım 23 mekanizma + Adım 27 dağıtım/doğrulama).** Sebep: tahsisatın tilt slew kutusundan aç bırakılması. Kutu 1.25 → 1.75 ile kalıcı salınım yok oldu, **1.75 → 3.00 (Adım 27)** ile MATLAB paritesine ulaşıldı. **Kriterin dördü de iki bağımsız tutuş uçuşunda geçti** (yaw hata bandı −7.3…+5.9° ve −10.3…+10.3°; yaw hızı RMS 0.0014-0.0028; \|vz\| ≤1.74; irtifa RMS 0.052-0.072 m; sıfır BIG_M). ⚠️ Ayrıca Adım 27 şunu buldu: Adım 22-24'te "yaw ❌ 35-37°" diye raporlanan şey **bir kusur değil, ölçüm artefaktıydı** — araç +90° heading'de doğuyor, `yaw_sp=0` ise 90°'lik dönüş komutlamak demek. Aşağıdaki eski satırlar tarihsel: |
> | *(eski değerlendirme)* | ❌ **AÇIK — araştırmanın merkezi.** Üç uçuşluk ölçülmüş hâli: **<~2 m/s sürekli salınım, ~2 m/s marjinal, üstünde kararlı** (yaw hızı RMS ~0.4 rad/s, bant 40-80°). MATLAB aynı adımı 3.69 s'de, ζ≈0.4 ile oturtuyor — üstelik **hiç aero yaw momenti olmadan**. Yani SITL'de MATLAB'da olmayan bir kararsızlaştırıcı var. Ölçümle elenenler: gölge aktüatör modelinin tilt kanalı, `omega_dot` gecikme/gürültüsü, `dt` jitter'ı, trim ön-yükleme. Ölçülmemiş tek boşluk: **itki kanalları**. |
> | **(P) Yaw adımı yön-asimetrik** | ✅ **ÇÖZÜLDÜ (Adım 28).** Sebep doğrulandı ve nicelleştirildi: tahsisattan `Fx = 0` istemek, aracın tek yönlü tilt aralığı yüzünden **yapısal olarak iptal edemeyeceği** ~2.9 N'lik ileri kuvveti iptal etmesini istemekti; kısıtsız çözüm üç tilt için de negatif çıkıp δ1/δ2'yi `TILT_MIN` tabanına çakıyordu. `p.ctrl.fx_trim = 2.9 N` (pozisyon döngüsünün ürettiği) bunu kaldırdı: asimetri **7.4× (Adım 17) → 1.33× (Adım 27) → 1.02× MATLAB / 1.05× SITL**, δ1 taban süresi %100 → %0.3. |
> | **(N) Yatay sürüklenme / pozisyon döngüsü yok** | ✅ **ÇÖZÜLDÜ (Adım 28).** `position_loop.m` + `positionLoop()`: P(konum)→PI(hız)→attitude setpoint. Sürüklenme MATLAB'da **137.15 m → 0.36 m**, SITL'de **25 s'de 235 m → ortalama 0.06 m / max 0.17 m**. Bunun asıl kazancı: Adım 16'nın "gerçek duruş hiç test edilmedi" boşluğu kapandı — duruşta (v_h≈0.01 m/s) yaw adımı temiz oturuyor. |
> | Donanım uçuşu | 🔴 **NO-GO** — SITL'de tüm kriterler geçiyor, ama pozisyon döngüsü gerçek donanımda hiç uçmadı ve Adım 11'in itki eşlemesi düzeltmesi SITL'e özgü. |
>
> **ADIM 18 (ölçüm yapıldı) — gölge model şüphesi ELENDİ, (Q) yeniden
> üretildi ve yeniden çerçevelendi:**
> - Gölge `_u_actual` ile gerçek Gazebo eklem açısı arasındaki fark
>   **p99'da ≤ 0.55°**; aynı pencerede yaw **73.7°** bantta salınıyor. İki
>   mertebe fark → gölge sapması katkı payı, **sürücü değil.** (Adım 17'nin
>   "en güçlü aday" değerlendirmesi bu ölçümle düzeltildi.)
> - **(Q) bir adım yanıtı kusuru değil, DENGE KARARSIZLIĞI:** sürekli salınım
>   `yaw_sp = 0` iken oluyor (bant 73.7°, periyot 5.18 s, r RMS 0.417 rad/s,
>   1.62 m/s'de).
> - **Salınımın en güçlü korelatı: δ1'in `TILT_MIN = 0` sınırından kalkıp geri
>   çakılması** (salınım boyunca 12 olay, salınım durunca sıfır). Yani
>   **(P) ile (Q) muhtemelen aynı kök neden: tek yönlü tilt aralığı.**
> - Yan bulgu: gölge δ1/δ2 örneklerin %75/%97'sinde **tam 0.000°** okuyor,
>   gerçek eklemler 0.53°'de → WLS "aşağı inemez" sanıyor, yaw'ın tek gerçek
>   aktüatöründe 0.53°'lik otorite allocator'dan gizleniyor.
>
> **ADIM 19-20 — (Q)'nun DOĞRU tarifi (üç uçuş), ve iki öz-düzeltme:**
> - **Doğru tarif:** düşük hızda (**<~2 m/s**) yaw salınımı **SÜREKLİ**;
>   **~2 m/s civarında MARJİNAL**; üstünde kararlı. Üç uçuş: 1.6 m/s →
>   sürüyor; plato 2.00-2.10 m/s → ~35 s'de söndü; plato 1.46-1.87 m/s →
>   **112 s sönmedi**. Aynı konfigürasyonda zıt sonuçlar = kararlılık sınırı.
> - ⛔ **Adım 19'un "ileri hız sebep değil" çıkarımı GERİ ALINDI** (çıkarım
>   34): sınıra çok yakın bir sistem, hız sabit kalsa da yavaşça söner —
>   tek uçuşluk gözlemden nedensellik çıkarılamazdı.
> - ⛔ **Adım 19'un "~30-35 s'de oturan zayıf sönümlü mod" ifadesi de GERİ
>   ALINDI** (çıkarım 35): tek uçuşa dayanan aşırı genellemeydi.
> - **Ablasyon (Adım 20a) dört adayı ELEDİ:** PX4'ün açık çevrim gölge
>   aktüatör modeli, +8 ms `omega_dot` gecikmesi, 0.02 rad/s² `omega_dot`
>   gürültüsü, %25 `dt` jitter'ı — ve dördü birlikte — MATLAB'ın 3.69 s'lik
>   yerleşmesini **hiç değiştirmedi**.
> - **Yüksek hız neden stabilize ediyor (SDF'den türetildi, ölçülmedi):**
>   tek yanal yüzey olan dikey kuyruğun sönümleme türevi 2 m/s'de
>   **0.27 Nm/rad**, 11.6 m/s'de **9.1 Nm/rad** (34×); ayrıca ±25-40°'lik
>   salınımda yan kayma **19.4°'lik stall açısını** aşıyor ve `cla_stall`
>   negatif → kuyruk çevrimin çoğunda sönümlemiyor.
> - **Ama aero, düşük hızdaki KARARSIZLIĞI açıklamıyor:** MATLAB'ın
>   plant'inde hiç aero yaw momenti yok (`M_aero(3) ≡ 0`) ve orada sistem
>   rahatça kararlı (ζ≈0.4). **SITL'de MATLAB'da olmayan bir
>   kararsızlaştırıcı var; aero onu yalnızca maskeliyor.** Araştırmanın
>   açık merkezi bu.
>
> **ADIM 21 — SOMUT BİR KOD HATASI BULUNDU (ilk kez bir düzeltme adayı var):**
> - İtki kanalı da elendi (sapma ~%0.1, yaw'a etkisi otoritenin %0.4'ü).
> - **`MulticopterIndiTiltrotor.cpp:315-316, 324-325` — WLS slew kutusu sabit
>   `TS_CTRL = 1/400` ile boyutlanıyor, ama modül `vehicle_angular_velocity`
>   callback'iyle 250 Hz'de dönüyor** (aynı fonksiyon `dt`'yi 171. satırda
>   doğru hesaplayıp gölge model/LESO/irtifa için kullanıyor). Ölçüldü:
>   |ddelta| p99 tam **0.00500 rad**, tick **4.00 ms**, tilt `sat_flag`
>   **%99.4-99.9**, tahsisat yaw verimi **%20.6**. → **efektif tilt slew
>   tavanı 1.25 rad/s, hedeflenen 2.0'ın %62'si.** Kanat tilt'i yaw'ın tek
>   gerçek aktüatörü olduğundan doğrudan yaw otoritesini kısıyor.
>   **Adım 11/12 ile aynı sınıf; saf MATLAB yapısal olarak göremez.**
> - **Adım 14'ü geriye dönük açıklıyor:** nominal 2.0 → efektif 1.25 (çalıştı),
>   nominal 3.0 → efektif **1.875** (ıraksadı). **Bu yüzden hatayı naif
>   düzeltmek zararlı olur:** `dt` ile boyutlamak nominal 2.0'ı **2.0 efektif**
>   yapar, yani ıraksatan 1.875'in de üstüne çıkarır.
>
> **ADIM 22 — kutu AYRIŞTIRILDI ve dürüst hale getirildi (KALICI, davranış-nötr):**
> - `TILT_RATE_MAX = 2.0` artık **yalnızca gölge modelin fiziksel servo limiti**;
>   tahsisat kutusu ayrı: **`TILT_SLEW_BOX_RATE = 1.25 rad/s` × `TS_BOX = 1/250`**.
>   Tilt kutusu 1 ULP farkla aynı (0.005 vs 0.0050000004) → **davranış-nötr**.
> - SITL'de doğrulandı: `|ddelta|` p99 hâlâ **tam 0.00500 rad**, itki `sat_flag`
>   %0.0, **kilitlenme yok** (itki 8.26-19.10 N), \|vz\| ≤ 0.78 m/s, irtifa hata
>   RMS 0.234 m. Yaw hâlâ kalıyor (37.09°) — bu **(Q)**, değişiklikten değil.
> - ⚠️ *Adım 21'in "kod hatası" nitelemesi fazla sertti:* sabit periyot kullanmak
>   jitter'a karşı **kasıtlı** bir korumaydı (kod yorumunda yazılı); gözden kaçan,
>   nominal periyodun döngünün gerçek periyoduyla eşleşmesi gerektiğiydi.
> - ⚠️ İtki kutusu **nötr değil** (üst sınır 22.5 → 36 N/tick); ilk gerekçemde
>   yalnızca alt sınıra bakmıştım. Ölçülen itki doyumu önce ve sonra %0.0,
>   yani aktif olmayan bir kısıt gevşetildi.
>
> **ADIM 23 — ⭐ (Q)'NUN MEKANİZMASI BULUNDU ve büyük ölçüde giderildi:**
> - **Düşük hızdaki yaw salınımı bir sönümleme eksikliği DEĞİL — tahsisat
>   tilt slew'undan aç bırakılıyormuş.** Uçuş-içi tarama (`slewbox` test
>   kancası), **iki koşu, ters sıra**, her değerde aynı +30° adım uyarımı.
>   Adımın son 5 s'sindeki yaw hızı RMS:
>
>   | kutu (rad/s) | koşu A | koşu B | test edilen hız | sonuç |
>   |---|---|---|---|---|
>   | **1.25** (eski) | 0.583 | 0.466 | 2.20 / 0.86 m/s | **SALINIYOR** |
>   | 1.50 | 0.391 | 0.005 | 2.02 / 2.00 m/s | marjinal |
>   | **1.75** | 0.0037 | 0.0051 | 1.35 / 2.76 m/s | **sakin** |
>   | **2.00** | 0.0056 | 0.0055 | 0.81 / 3.14 m/s | **sakin** |
>
> - **Hız kesin olarak elendi:** 1.25 hem 0.86 hem 2.20 m/s'de salınıyor;
>   1.75/2.00 ise 0.81-3.14 m/s aralığının tamamında sakin.
> - **MATLAB çapraz kontrolü tutuyor:** onun efektif kutusu **3.0 rad/s**
>   (`3.0·(1/400)` @ gerçek 400 Hz) — denenen her değerin üstünde, ve orada
>   aşım %24.1 / yerleşme 3.69 s. Trend düzgün ekstrapole oluyor.
> - **Adım 14 çelişmiyor, açıklanıyor:** o, kutuyu ve gölge modelin fiziksel
>   limitini *birlikte* oynatmıştı. **Adım 22'nin ayrıştırması kutunun tek
>   başına yükseltilmesini mümkün kılan şey.**
> - **Varsayılan 1.25 → 1.75 yapıldı (KALICI).** `sitl-lockup-check`:
>   kilitlenme ✅ (itki 12.83-19.11 N, sat %0.0), dikey hız ✅ (0.816 m/s,
>   irtifa hata RMS 0.279 m), roll/pitch ✅ (±0.06°). **Yaw hâlâ ❌**
>   (tepe 35.80° vs önceki 37.09°) — kalan aşım arm geçicisinin tek seferlik
>   salınımı; asıl kazanç **kalıcı salınımın yok olması (RMS ~0.5 → ~0.005)**.
>
> **ADIM 24 — "gölge modeli gerçek servoya sadık kıl" DENENDİ, GERİ ALINDI:**
> - Sadakat açığının **tamamı Coulomb sürtünme ölü bandı** (SDF'den:
>   friction/p_gain = 1.0/100 = **0.573°**), 2. derece dinamik değil. Eklem
>   ataleti J=0.0168 kg·m² ile max ivme 59.4 rad/s² — birkaç ms, 4 ms tick'in
>   çok içinde. Çevrimdışı doğrulama (kayıtlı `u_cmd` → gerçek eklem açısı):
>   1. derece 0.287/0.408/0.554° → **tam 2. derece 0.414/0.462/0.553°
>   (DAHA KÖTÜ)** → **1. derece + ölü bant 0.082/0.051/0.0040°
>   (3.5×/8.0×/139× daha iyi)**.
> - **Ama uygulandığında KİLİTLENDİ.** `u_cmd = _u_actual + du` komutu gölgeye
>   bağlıyor ve `du` slew kutusuyla **0.40°** ile sınırlı — 0.573°'lik ölü
>   banttan küçük. Hiçbir tick sürtünmeyi kıramıyor → gölge donuyor → komut
>   donuyor. SITL: üç tilt de tüm uçuş donuk (δ0 sabit 9.31°, sıfır varyans),
>   yaw bandı **238°**, araç dönüyor. Geri alındı ve baseline doğrulandı.
> - **Ders 1:** açık çevrim replay, kapalı çevrim geri besleme tuzağını
>   gösteremez — komut yolunun içindeki model değişiklikleri kapalı çevrimde
>   doğrulanmalı.
> - **Ders 2 (mimari):** artımlı allocator, gölgenin komuta sürünmesini
>   zorunlu kılıyor; stiction bununla temelden uyumsuz. Çıkarım 30/45'in
>   "gölgeyi sadık kıl" önerisi bu yoldan **kapandı**.
>
> **ADIM 25 — yol (a) "arm geçicisini kaynağında küçült" DE KAPANDI; premis
> iki kez çürüdü, kod değiştirilmedi:**
> - **Tilt ön-konumlandırma gereksiz:** disarm'da servolar NaN alıyor (0°'de
>   duruyorlar) ve arm'da gölge δ0 ≈ 9.39°'de tohumlanıyor, ama **gerçek δ0
>   trim'in %90'ına 72 ms'de ulaşıyor**, +0.20 s'de gölgeyle 0.16° içinde.
>   Yaw hızı ise saniyeler boyunca birikiyor — **zaman ölçekleri uyuşmuyor.**
> - **Trim itkiden bağımsız geçerli:** `τ_tilt = −Σ py·T·sin δ` **de** itkiyle
>   ölçekleniyor, tıpkı `τ_react = −Σ km·T·cos δ` gibi. Tırmanışta itki
>   49.7 → 68.9 N çıkarken NET tork −0.021 → −0.011 N·m kalıyor.
> - **Sonuç: ayrı bir "arm geçicisi" mekanizması yok** — arm sonrası savrulma
>   madde (Q)'nun kendisi: zayıf yaw ekseninin 0.03-0.15 N·m'lik artık
>   torklara yanıtı (tahsisat yaw otoritesi adım başına ~0.033 N·m).
>
> **Sıradaki adım:** §4 (Q) **yol (b)** — sabitler Adım 22'de ayrıştırıldığı
> için artık **yalnızca `TILT_RATE_MAX`'ı** yükseltmek mümkün (kutu 1.75'te
> sabit). Bu hiç denenmedi; Adım 14 ikisini birden oynatmıştı. Rebuild gerekir,
> ve Adım 20'nin dersi gereği her değer en az iki koşuda doğrulanmalı.
>
> **Okuma sırası:** bu blok → §1a "★ EN GÜNCEL DURUM ★" → §4 "★ GÜNCEL
> ÖNCELİK SIRASI ★" → §3 çıkarım 38-42 (ve geri alınan 34-35) → gerekirse
> Adım 11-20 gövdesi.
> §2'nin Adım 1-10 kısmı **tarihsel kayıttır**, birçok ara hipotezi
> sonradan çürütüldü.

**Durum: BÜYÜK ÖLÇÜDE ÇÖZÜLDÜ (2026-07-27, Adım 11) — kök neden
bulundu ve düzeltildi; geriye tek açık sorun kaldı.**
*(Aşağıdaki üç paragraf Adım 11 dönemine ait tarihsel özettir; güncel
durum için yukarıdaki bloğa bakın.)*

Asıl arıza (aktüatör kilitlenmesi + dikey hız kontrolsüzlüğü) iki
bağımsız SITL koşusunda **artık tekrar üretilemiyor**. Kök neden,
kontrol matematiğinde değil çıktı katmanındaydı: PX4 modülü itki
komutunu Gazebo'nun **karesel** motor modeline **doğrusal** bir
eşlemeyle gönderiyordu (`u_cmd/ROTOR_TMAX`). Bu hem gerçek itkiyi
ağırlığın altında bırakıyor hem de WLS'in etkinlik matrisini itkiye
bağlı biçimde bozarak düşük itkili rotoru tabana iten bir pozitif geri
besleme yaratıyordu. Ayrıca ikinci bir hata bulundu: `test_sp`
setpoint'i kontrolcüye **hiç ulaşmıyordu**, yani bu araştırmadaki tüm
önceki koşular aslında yanlış senaryoyu (tırmanış yerine irtifa-koruma)
test etmişti.

**Kalan açık sorun (Adım 11 dönemi ifadesi):** yaw ekseni ±60°'lik bir
bantta geziniyor (kriter ±30°). **DÜZELTME:** bu ifade yanlıştı — Adım
12b, bunun 5 saniyelik açı örneklemesinden doğan bir aliasing artefaktı
olduğunu, aracın aslında sürekli döndüğünü gösterdi.

2026-07-24'te GUI'de canlı gözlemle keşfedildi; on altı adım denendi
(dördü geri alındı). Bu bölüm her adımı (denenen ve denenmeyen, başarılı
ve başarısız) sırayla belgeler.

---

## 1. Sorunun tanımı

`mc_indi_tiltrotor` (MATLAB referans kontrolcünün PX4 C++ portu) SITL'de
arm edilip bir irtifa/hover setpoint'i verildiğinde:

- Kısa sürede (birkaç saniye) bir veya iki aktüatör (kanat rotorleri T0/T1,
  bazen kuyruk T2) doygunluğa kilitleniyor.
- Uzun vadede (~15-25 s) sistem **yaw ekseninde kontrolsüzce savruluyor**
  (gözlenen: -161.9°), dikey hız hedefin çok üstüne çıkıyor (-11 m/s), ve
  araç flip/çarpışma sınırına geliyor.
- Aynı senaryo saf MATLAB referansında (`indi_attitude_controller.m` +
  `tiltrotor_plant_deriv.m`, sabit `Ts_ctrl`, idealize fizik) **kusursuz**
  çalışıyor — bu, sorunun kontrol algoritmasının matematiğinde değil,
  SITL'e özgü bir etkileşimde olduğunu gösteriyor.

**Neden önemli:** Bu, gerçek bir uçuşta kontrolsüz sürüklenme/çarpışma
anlamına gelir. Düzeltilmeden bu airframe SITL dışında (donanımda)
denenmemeli.

---

## 1a. Uçuş kritiklik değerlendirmesi (2026-07-26)

Kontrol-teorisi tarafında 7 adım (§2, Adım 1-7) tükendikten sonra, soruna
kontrol matematiği yerine **uçuş güvenliği/operasyonel etki** açısından
bakıldı.

**Ne zaman, ne kadar sürede tetikleniyor:**
- İlk **~13-17 saniye her koşuda makul/kararlı** görünüyor (roll/pitch
  küçük, irtifa hedefe yaklaşıyor) — bu, **yanıltıcı bir güven** yaratıyor:
  kısa bir duman testi ("5 saniye arm edip bak") sorunu YAKALAMAZ.
- ~15-25 saniye civarı (koşudan koşuya değişken, gerçek-zamanlı zamanlama
  jitter'ı nedeniyle deterministik değil) sistem **her denemede** bir
  biçimde bozuluyor — bu ana kadar test edilen HER senaryoda (M4/M5/M6
  repro'ları, bu oturumun 7 farklı konfigürasyonu) tekrarlandı. Yani bu
  bir kenar durum değil, **temel/varsayılan senaryonun kendisi**.

**Gözlenen arıza şiddetleri (farklı koşularda, farklı biçimlerde):**
- Aktüatör doygunluğu: bir kanat rotorü (T0 ya da T1) 0 N'a, kuyruk
  rotorü (T2) 45 N tavanına kilitleniyor.
- Yaw savrulması: 23 saniyede **-161.9°** (neredeyse tam tur, ilk 40s'lik
  MATLAB doğrulama testinin < 1°'lik hatasıyla tam tezat).
- Dikey hız kontrolsüzlüğü: **vz=-11.01 m/s** ölçüldü — komut edilen
  `ALT_VZ_MAX=2.0 m/s`'nin **5.5 katı**, serbest düşüşe yakın.
- Daha önceki bir GUI oturumunda (RUNBOOK §4) **tam ters dönme (roll
  ≈180°)** gözlenmiş — bu bir simülasyon istatistiği değil, gerçek
  donanımda **çarpışma/kayıp** demektir.

**Neden operasyonel olarak kritik:**
1. **Her gerçekçi görev bu pencereyi aşar.** Bu kontrolcünün pozisyon/hız
   dış döngüsü yok (bilinen sınırlama) — yani en basit görev bile
   (kalkış+hover+iniş) rahatlıkla >20-30 saniye sürer. Sorunun tetiklendiği
   pencere, "nadir uzun uçuşlarda" değil, **her normal görevde** aşılıyor.
2. **Kısa testler yanlış "ÇALIŞIYOR" sinyali veriyor.** İlk ~15s'nin
   kararlı görünmesi, aceleyle yapılan bir doğrulamanın ("arm et, 5-10s
   bekle, iyi görünüyor") sorunu KAÇIRMASINA yol açar — bu, bu oturumun
   Adım 3'te bizzat yaşadığı bir tuzak (kısa test "düzeldi" izlenimi
   verdi, 25s'lik test göstermedi).
3. **SITL'e özgü değil, muhtemelen donanımda DAHA KÖTÜ.** Kök
   nedenlerin bir kısmı (km büyüklüğü, gerçek-zamanlı jitter) SITL'e
   özgü olsa da, altta yatan zayıflık (yaw ekseninin P/rate/WLS
   önceliğinin gerçek bozucuları karşılayamaması, LESO'nun yaw'da kapalı
   olması) gerçek donanımda muhtemelen DAHA BELİRGİN olur — gerçek
   motorlar/pervaneler simülasyondan daha asimetrik, gerçek IMU daha
   gürültülü/gecikmeli, gerçek aero etkiler basit SITL modelinden
   karmaşık.
4. **Zarar geri döndürülemez.** Bu bir performans/konfor sorunu değil —
   kontrol kaybı + kontrolsüz iniş/dönme demek, yani donanımda **araç
   kaybı ve/veya bölgesel güvenlik riski**.

**Sonuç — GO/NO-GO değerlendirmesi:**

| Ortam | Durum |
|---|---|
| **Gerçek donanım uçuşu** | 🔴 **NO-GO.** Sorun çözülmeden hiçbir gerçek uçuş denemesi yapılmamalı. |
| **SITL geliştirme/test** | 🟡 **KOŞULLU GO.** Yalnızca test süresi kısıtlanırsa (öneri: ≤10s, kesinlikle §"Adım 4" bulgusu gereği sonuçların "düzeldi" olarak yanlış yorumlanmaması için ≥20-25s'lik AYRI bir doğrulama olmadan hiçbir düzeltme "çözüldü" ilan edilmemeli) ve otomatik disarm/timeout güvenceleri varsa. |
| **Saf MATLAB geliştirme** | 🟢 **GO.** Sorun burada hiç görülmüyor, referans kontrolcü üzerinde çalışmak güvenli. |

Bu değerlendirmeyi her oturumda elle tekrarlamak yerine sistemleştirmek
için iki proje skill'i yazıldı (`.claude/skills/`), bkz. §6.

### GÜNCELLEME 2026-07-27 (Adım 11 sonrası)

Yukarıdaki tablo Adım 1-10 dönemine aittir ve **tarihsel kayıt olarak
bırakılmıştır**. Adım 11'den sonraki güncel değerlendirme:

| Ortam | Durum |
|---|---|
| **Gerçek donanım uçuşu** | 🔴 **HÂLÂ NO-GO.** Kök neden düzeltildi ama (a) yaw hâlâ ±30° kriterini geçmiyor, (b) düzeltmeler yalnızca 2 SITL koşusunda doğrulandı, (c) düzeltmelerden biri (itki eşlemesi) **Gazebo'nun motor modeline özgü** — gerçek bir ESC/pervane için kalibrasyonu yeniden türetilmelidir, doğrudan taşınamaz. |
| **SITL geliştirme/test** | 🟢 **GO.** Kilitlenme ve vz kontrolsüzlüğü iki bağımsız koşuda (25s ve 40s) tekrar üretilemedi; süre kısıtlaması artık gerekli değil. ≥25s doğrulama disiplini yine de korunmalı. |
| **Saf MATLAB geliştirme** | 🟢 **GO.** Değişmedi — Adım 11'in iki düzeltmesi de PX4/SITL'e özgü, MATLAB tarafına hiç dokunulmadı. |

### GÜNCELLEME 2026-07-27/28 (Adım 16-17 sonrası) — ★ EN GÜNCEL DURUM ★

> **Adım 17 (2026-07-28) tabloyu değiştirmiyor** — GO/NO-GO durumları aynı
> kaldı (hiçbir kontrol sabiti değişmedi, yalnızca bir MATLAB test sürücüsü
> eklendi). Değişen şey **(Q)'nun teşhisi**: Adım 16'nın "aero rüzgâr gülü
> sönümlemesi kayboluyor" mekanizması yetersiz çıktı; en güçlü aday artık
> `_u_actual` açık çevrim gölge aktüatör modeli. Ayrıntı → Adım 17, §4 (Q).

| Ortam | Durum |
|---|---|
| **Gerçek donanım uçuşu** | 🔴 **NO-GO.** (Adım 13 sonrası "🟡, teknik engel yok" denmişti — **Adım 16 bunu geri aldı.**) `sitl-lockup-check`'in üç kriteri iki bağımsız koşuda geçti, ama o koşuların hepsi ~10 m/s ileri hızdaydı; **gerçek, yerinde duran hover'da yaw ekseni sönümsüz ve ±25° salınıyor** (§4 (Q)). Yaw sönümlemesi şu an aerodinamikten geliyor, kontrolcüden değil. Ayrıca: (a) rüzgâr/bozucu ve transition senaryolarında hiç denenmedi; (b) Adım 11'in itki eşlemesi hâlâ Gazebo sabitlerine kalibre — gerçek motor/ESC eğrisinden yeniden türetilmeli; (c) yatay sürüklenme sınırlanmıyor (pozisyon döngüsü yok, §4 (N)); (d) agresif alçalmada aktüatör sıkışması gözlendi (§4 (O)). |
| **SITL geliştirme/test** | 🟢 **GO.** Yaw ±1.6°, roll/pitch hız RMS ≤0.002, irtifa hata RMS 0.21 m, kilitlenme yok. |
| **Saf MATLAB geliştirme** | 🟢 **GO.** Adım 13 MATLAB'da tam nötr; Adım 12'nin kazanımları duruyor. |

---

### GÜNCELLEME 2026-07-27 (Adım 12 sonrası)

| Ortam | Durum |
|---|---|
| **Gerçek donanım uçuşu** | 🔴 **HÂLÂ NO-GO.** Yaw ekseni SITL'de sürekli dönüyor (ort. +1.44 rad/s) — ±30° kriterinden çok uzak. Adım 12'nin km düzeltmesi donanıma **taşınabilir** (gerçek rotor dönüş yönlerinden türetilir, Gazebo'ya özgü değil), ama Adım 11'in itki eşlemesi hâlâ Gazebo'ya özgüdür. |
| **SITL geliştirme/test** | 🟢 **GO.** Kilitlenme yok (0 BIG_M), irtifa/roll/pitch temiz ve Adım 12'de bir miktar daha iyi. |
| **Saf MATLAB geliştirme** | 🟢 **GO.** Adım 12 MATLAB'ı da değiştirdi ve regresyon testleri **iyileşti** (RMS p/q 2-5×, transition max\|omega\| 2×). |

**Uyarı — ölçüm yöntemi:** Yaw durumunu `px4-listener` ile birkaç
saniyede bir açı okuyarak değerlendirmeyin; bu, Adım 11'de sürekli
dönüşün "±60° gezinme" sanılmasına yol açtı (Adım 12b). Yaw hızını
(`vehicle_angular_velocity.xyz[2]`) ve tercihen ulog'u kullanın.

**Donanım için kritik uyarı:** Adım 11d'nin düzeltmesi
`ROTOR_KF=2e-5`, `ROTOR_WMAX=1500`, `ROTOR_WMIN=10` (SIM_GZ_EC_MIN/MAX)
değerlerine dayanıyor; bunlar **Gazebo SDF/airframe sabitleri**.
Gerçek donanımda karşılıkları ölçülmüş motor/ESC eğrisinden
türetilmelidir — aynı sayıları taşımak yeni bir itki eşleme hatası
yaratır.

---

## 2. Zaman çizelgesi — denenen yöntemler

### Aday çözüm 1 — Sabit `TS_CTRL` (2026-07-25)

**Hipotez:** `Run()`'daki ölçülen `dt` titreşiyor (ortalama ~4ms, nominal
2.5ms'in %60 üstünde, düzensiz: min 0µs, max 8000µs). Bu jitter, WLS'in
hız-limiti kutusuna (`rate_lo/hi ∝ dt`) doğrudan besleniyor; küçük geldiği
tick'lerde kutu daralıp bir aktüatörü o tick'te dondurabiliyor.

**Uygulama:** WLS hız-limiti kutusu hesabında gerçek `dt` yerine sabit
`TS_CTRL=1/400s` kullanıldı (yalnızca kutu hesabında; entegrasyon/decimasyon
`dt` referanslarına dokunulmadı).

**Sonuç: ÇÖZMEDİ.** Davranış düzeltmeden önceki ile bit bit aynıydı
(`u_actual[0]` 10s boyunca kesintisiz 0, `sat_flag[0]` sürekli `True`).
Kod incelemesi: `wlsAllocate()`'in büyük-M ağırlığı her `Run()` çağrısında
sıfırdan başlıyor — tick'ler arası kalıcı "mandal" durumu yok. Hipotez
kısmen çürütüldü; kutu genişliği sorunun kaynağı değildi.

**Teşhis logu ile kök neden bulundu (2026-07-25):** Roll ekseninde
(`nu_des(0)`) kalıcı, büyüyen, tek-işaretli bir talep oluşuyor. WLS'in
"birim tork başına ceza" karşılaştırması (`Wu_i/|G_i|`) roll ağırlığının
(200) diğer eksenlere (Fx=0.05, Fz=20) göre ezici üstünlüğü yüzünden, en
ucuz çözüm T0'ı (roll+Fz'ye katkısı olan rotor) sıfıra kadar azaltmak
oluyor — T0'ın Fz katkısını kaybetmesine rağmen.

---

### Aday çözüm 2 — `Wu_tilt` yeniden dengeleme (2026-07-26)

**Hipotez:** Tilt farkı (δ), roll/pitch düzeltmesi için thrust farkından
fiziksel olarak daha etkili olmasına rağmen, `WU_TILT_HOVER=8.0` onu
ölçüsüz pahalı gösteriyor — WLS bu yüzden tilt yerine thrust'ı (T0)
tüketmeyi tercih ediyor.

**Analitik türetme:** Hover trim itkisinde (`Tw≈18.32N`) roll ekseni için
`|dtau_ddelta0|≈0.916`, `|dtau_dT0|=0.25` — tilt ~3.665× daha etkili.
Eşik: `wu_tilt < wu_thrust·(0.916/0.25) ≈ 3.665` olmalı ki tilt tercih
edilsin. Eski değer (8.0) bu eşiğin üstündeydi.

**Uygulama:** `WU_TILT_HOVER` 8.0→**3.0** (`gain_schedule.m`,
`sf_wls_alloc.m`, `TiltrotorIndiParams.hpp`, `sitl/run_transition_test.py`
tutarlı biçimde).

**Doğrulama:**
- Saf MATLAB: regresyon yok, tilt kullanımı gerçekten arttı.
- SITL repro (5s): **kilitlenme kalkmadı, yer değiştirdi** — T0 yerine
  T1 sıfıra kilitlendi, aynı imzayla (`d_hat` büyüyor, ilgili rotor
  tavana tırmanıyor). ~10s'de sistem işaret değiştirip sönümsüz salınıma
  girdi.

**Sonuç: ÇÖZMEDİ.** WLS'in *hangi* aktüatörü tükettiğini değiştirdi ama
`nu_des`'in kalıcı/büyüyen tek-işaretli kalma eğilimini (asıl motor)
değiştirmedi. Yararsız değil (tilt kullanımını gerçekten artırdı) ama
tek başına yeterli değil.

---

### Aday çözüm 3 — `ROTOR_KM` mekanik düzeltmesi (2026-07-26)

**Bulgu (gerçek mekanik/model uyuşmazlığı):** Kontrolcünün kullandığı
`ROTOR_KM=[0.05,-0.05,0.05]` Nm/N (airframe parametresi `CA_ROTORi_KM`'den
miras), gerçek Gazebo fizik motorunun kullandığı
`Tools/simulation/gz/models/tiltrotor_tailplane/model.sdf`
`<momentConstant>0.06</momentConstant>` değeriyle **%20 farklı**. gz-sim
kaynak kodu (`MulticopterMotorModel.cc`) doğrulandı:
`dragTorque_z = -turningDirection·thrust·momentConstant` — yani
`momentConstant`, `km` ile birebir aynı tanım/birim (Nm/N), doğrudan
karşılaştırılabilir. İşaret deseni tutarlıydı (yalnızca büyüklük yanlıştı).

**Etkisi:** `hoverTrim()`'in analitik yaw-nulling düzeltmesi (`d1_trim`),
gerçek reaksiyon-tork dengesizliğini ~%17 eksik telafi ediyordu — kalıcı,
gerçek bir uncorrected yaw torku bırakıyordu.

**Uygulama:** `ROTOR_KM` 0.05→**0.06** (`TiltrotorIndiParams.hpp`,
`tiltrotor_params.m`, `sf_wls_alloc.m`). Saf MATLAB'da plant+kontrolcü
zaten aynı değeri paylaştığından (self-consistent), bu yalnızca
PX4-gerçek fizik arasındaki gerçek uyuşmazlığı kapattı.

**Doğrulama (SITL, ~23s izlendi — önceki testlerden daha uzun):**
- **t+5..13s: gerçek bir iyileşme.** `d_hat(0)` başta büyüdü (-1.34→-2.05)
  ama sonra **kendiliğinden 0'a yakınsadı** (-2.05→-0.50) — önceki iki
  denemede hiç görülmeyen bir davranış. T0 kilitli kalsa da T1/T2 düzgün
  telafi etti.
- **t+14..23s: farklı ve daha ciddi bir moda geçiş.** T0 kısa süre
  toparlandı, ama T1 çökmeye başladı; t+23s'de **hem T0 hem T1 ≈ 0**,
  kuyruk rotoru (T2) **tavana kilitlendi** (45N) ve tilt'i sürekli arttı
  (25°→84°, neredeyse tam cruise — hover'dayken!), `d_hat` her iki
  eksende patladı (`[4.24, 8.48]`). **Yaw -161.9°'ye savruldu**,
  `vz=-11.01 m/s`. Test disarm edilip durduruldu (flip sınırı).

**Sonuç: ÇÖZMEDİ**, ama önemli yeni sinyal: erken evrede gerçek bir
kendiliğinden-yakınsama + geç evrede ayrı, daha ciddi bir yaw-savrulma
modu. Kuyruk rotorünün Fz katkısını kaybetmesiyle (cos(δ2) küçülürken)
zamansal olarak örtüşüyor — kanat rotorlerine binen ek Fz yükünün onları
da tüketmesi ihtimali var: çok-eksenli, kendini besleyen bir döngü.

---

### Aday çözüm 4 — `ROTOR_PY` işaret düzeltmesi (2026-07-26, GERİ ALINDI)

**Teşhis genişletildi:** `TiltrotorIndiControl.hpp`'ye kuyruk rotoru/yaw
ekseni için ikinci bir tanı logu (`T2dbg`) eklendi. 25s'lik bir SITL
koşusunda: **`nu_des(2)` (yaw) arm anından itibaren hemen -1.19 ve test
boyunca neredeyse hiç azalmadan kaldı** — büyüyen bir drift değil, kalıcı
bir başlangıç hatası. LESO yaw ekseninde kapalı olduğundan bu LESO
kaynaklı olamazdı.

**Gerçek mekanik uyuşmazlık bulundu:** `model.sdf`, `motor_0`'ı açıkça
`<!-- Right wing rotor -->` yorumuyla **Y=-0.25**'te, `motor_1`'i
`<!-- Left wing rotor -->` yorumuyla **Y=+0.25**'te tanımlıyor —
kontrolcünün varsaydığının (`ROTOR_PY=[+0.25,-0.25,0]`) **tam tersi**.
Fiziksel türetme (`dtau_dT`'nin roll bileşeni `= -ry`) bunun roll↔thrust
etkinlik işaretini tersine çevirebileceğini gösteriyordu — ve tilt'in
roll etkinliğinin yalnızca `km`'ye bağlı olması (ry'den bağımsız), aday
çözüm 2'nin (tilt'i ucuzlatma) neden kısmen işe yaradığını tutarlı
biçimde açıklıyordu.

**Uygulanan düzeltme (DENENDİ, GERİ ALINDI):** `ROTOR_PY`
`[-0.25,+0.25,0]`'e çevrildi (üç dosyada tutarlı, `km` değiştirilmedi).
`turningDirection→sayısal` eşlemesi ayrıca gz-sim kaynağından doğrulandı
(`ccw=+1, cw=-1`) — aday çözüm 3'teki işaret varsayımını teyit etti.

**Sonuç: CİDDİ REGRESYON.** Saf MATLAB referans testinde (plant+kontrolcü
bu değişiklikten SONRA da birbiriyle tutarlıydı — tamamen kontrollü bir
karşılaştırma) RMS p/q **~0.0065/0.0015'ten 0.34/1.29 rad/s'e fırladı**
(~50-800×). Basit işaret değişimi yanlış (veya eksik) bir düzeltme —
**geri alındı**, PX4 tarafına hiç deploy edilmedi (regresyon MATLAB'da
yakalandığı için SITL'e hiç gitmedi). SDF'deki Y-etiket uyuşmazlığı
gerçek ve açıklanmamış duruyor, ama doğru düzeltme belirsiz — gz-sim'in
tam itki formülünün (yalnızca `momentConstant`/tork değil, `motorConstant`/
itki hesabına da `turningDirection`'ın girdiği görüldü) ham kaynaktan
satır satır incelenmesi ve/veya SDF poz referans çerçevesinin `gz model`
ile ampirik doğrulanması gerekiyor. **Bu konuya tahminle tekrar
girilmemeli** — MATLAB referansını bile bozabildi.

---

### Adım 5 — gz-sim ham kaynağı satır satır incelendi (2026-07-26)

**Yöntem:** `MulticopterMotorModel.cc`'nin tamamı `curl` ile indirilip
(WebFetch'in küçük-model özetine değil, ham koda) doğrudan okundu.

**Bulgu 1 — itki formülü çözüldü:**
```cpp
double thrust = turningDirection * realMotorVelocitySign *
                realMotorVelocity * realMotorVelocity * motorConstant;
...
Vector3 dragTorque(0, 0, -turningDirection * thrust * momentConstant);
```
`realMotorVelocitySign`, joint'in GERÇEK ölçülen hız işaretidir; joint
hız SETPOINT'i de ayrı bir yerde `turningDirection * refMotorRotVel`
olarak komut ediliyor (satır ~680) — yani CW rotorlar FİZİKSEL OLARAK
negatif işaretli joint hızıyla döndürülüyor. Sonuç: `turningDirection *
realMotorVelocitySign` çarpımı CW/CCW farketmeksizin **+1'e sabitleniyor**
(ikisi de aynı işareti taşıyor, çarpım hep pozitif) — yani net itki
`turningDirection`'dan **bağımsız**, her zaman pozitif. Bu, Aday çözüm
3'teki km işaret-eşleşme analizimi (yalnızca `momentConstant` üzerinden)
DOLAYLI olarak doğruluyor — orada bir hata yoktu.

**Bulgu 2 — pose/frame zinciri statik dosya okumasıyla çözülemedi:**
İtki, rotor linkinin KENDİ yerel çerçevesinde `(0,0,thrust)` olarak
uygulanıp `worldPose->Rot()` ile döndürülüyor; drag-tork da benzer şekilde
`poseDifference.Rot()` (child→parent dönüşümü) üzerinden aktarılıyor.
`motor_0`'ın SDF'deki pose'u rotasyonsuz (`rpy=0 0 0`) — yani yerel
çerçevesi, referans aldığı üst çerçeveyle (muhtemelen `base_link` ya da
model kökü) hizalı. Bu üst çerçevenin FRD mi yoksa başka bir kural mı
kullandığını (ve Y ekseninin gerçekten "sağ" mı "sol" mu temsil ettiğini)
statik SDF okumasıyla **kesin olarak çözemedim** — canlı `gz model`/`gz
topic` sorgusu ya da Gazebo'nun kendi pose-resolve mantığını çalıştırmak
gerekir, ki bu oturumda yapılmadı.

**Karar — Y-işareti hattı TERK EDİLDİ (şimdilik):** Ek olarak, sistemin
davranışı da bu teoriyle çelişiyor: roll etkinliği GERÇEKTEN tam ters
olsaydı (WLS'in inandığı işaretin fiziksel gerçeğin negatifi olması),
kapalı çevrim ilk tick'ten itibaren pozitif geri besleme ile ANINDA
kararsız olurdu — ama gözlenen davranış ilk ~13-15 saniye makul/kararlı,
sonra bozuluyor. Bu, "tam ters işaret" hipoteziyle uyuşmuyor. Bu yüzden
Y-işareti hattını bırakıp **farklı bir hipotezle** devam edildi (Adım 6).

---

### Adım 6 — Hipotez: LESO yaw ekseninde kapalı, gerçek aero bozucusu hiç telafi edilmiyor

**Gözlem (ham kaynaktan, Adım 5'in yan ürünü):** `MulticopterMotorModel.cc`
her rotor için, kontrolcünün effectiveness modelinin hiç bilmediği iki
ek fiziksel etki uyguluyor: `airDrag` (gövde hızına bağlı, rotor
konumunda uygulanan bir KUVVET — offset'ten dolayı örtük bir tork da
üretir) ve `rollingMoment` (`rollingMomentCoefficient=1e-6`,
`rotorDragCoefficient≈8e-5`, `model.sdf`'de üç rotor için de tanımlı).
`effectiveness_matrix.m`/`effectivenessMatrix()` yalnızca itki+reaksiyon-
tork geometrisini modelliyor — bu terimlerden HİÇ haberi yok. LESO tam
olarak "kontrolcünün bilmediği model belirsizliğini" telafi etmek için
var (bkz. README "Kontrol mimarisi" §3), ama `leso_enable=[true,true,false]`
ile **yaw ekseninde LESO baştan kapalı** — `run_hover_gust_test.m`'nin
doğrulanmış konfigürasyonundan miras.

**Zayıf nokta:** Bu katsayılar (`1e-6`, `8e-5`) çok küçük; hover'da (gövde
hızı ~0) bu terimlerin `nu_des(2)=-1.19` büyüklüğünde bir kalıcı torku tek
başına açıklaması olası değil — büyüklük hesabı tutarsız. Yine de LESO'nun
yaw'da kapalı olması, kaynağı ne olursa olsun (bu terimler, kalıntı km/
geometri hatası, ya da başka bir etki) **hiçbir düzeltme mekanizmasının
olmadığı** anlamına geliyor — yalnızca zayıf `Ws_yaw=3` P/rate-loop yolu
var. Bu, ampirik olarak ucuz bir deney: kod değişikliği/rebuild
GEREKTİRMEDEN, `test_sp` komutunun son argümanını (`leso_enable_yaw`)
`0`'dan `1`'e çevirerek doğrudan SITL'de test edilebilir.

**Deney planı:** Aynı repro (§4/RUNBOOK), ama
`px4-mc_indi_tiltrotor test_sp 0 0 0 0 $z_sp 1 1 1` (son parametre 0→1),
en az 20-25s izlenecek. Beklenti: yaw savrulması ya tamamen kalkar ya da
belirgin biçimde yavaşlar; kalkmazsa (ör. hâlâ sabit ~-1.2 civarında
kalırsa) bu, sorunun disturbance-tahmin edilebilir bir şey olmadığını,
gerçek bir **model/geometri hatası** olduğunu daha da güçlü kanıtlar
(LESO sabit bir kalıcı hatayı da normalde absorbe edip nüfuz eder,
absorbe edemiyorsa kaynak farklı bir şeydir — ör. LESO'nun kendisi de
`wo=15` bant genişliğiyle yalnızca belirli bir dinamikte iyi çalışır).

**Sonuç (26s SITL koşusu, rebuild GEREKMEDİ — yalnızca `test_sp` argümanı
değişti): KARIŞIK — kısmen olumlu, çözmedi.** `T0dbg1`/`T2dbg2`
loglarından çıkarılan tam `nu_des` zaman serisi (52 örnek, 500ms
aralıklarla):

| Aralık (örnek #, ~t) | roll | pitch | **yaw** | Fx | Fz |
|---|---|---|---|---|---|
| 1 (t≈0s) | -0.00 | 0.03 | **-1.19** | -2.96 | -0.24 |
| 16 (t≈8s) | 0.14 | -0.03 | **-1.21** | -0.00 | 2.44 |
| 34 (t≈17s) | -0.13 | -0.21 | **-1.20** | -7.79 | 0.33 |
| 35 (t≈17.5s) | **-0.80** | 0.28 | -0.84 | -8.91 | 2.08 |
| 37-41 (t≈18.5-20.5s) | -0.15..-0.31 | dalgalı | 0.18→2.26 (aşım) | -7 .. -11 | 1.6..6.5 |
| 46-52 (t≈24-26s) | ~0.02-0.05 | ~0.00 | **+1.49..+1.50** | ~-6.0..-6.5 | ~0.1 |

**Olumlu:** Roll/pitch, ~t=17.5s'deki geçiş olayı dışında **testin tamamı
boyunca küçük kaldı** (< 0.3, çoğunlukla < 0.15) — bu, önceki DÖRT
denemenin HİÇBİRİNDE görülmemiş bir kararlılık düzeyi (hepsinde roll ya
da pitch er ya da geç büyüyüp bir aktüatörü tüketiyordu). LESO'yu yaw'da
açmak roll/pitch eksenlerini gözle görülür biçimde iyileştirdi.

**Olumsuz:** Yaw kendisi **sıfıra yakınsamadı** — ilk ~17s boyunca eski
davranışla aynı (~-1.2'de sabit, LESO'nun `wo=15 rad/s` bant genişliğiyle
beklenen hızlı yakınsamanın ÇOK altında bir tepki), sonra ~t=17.5s'de ani
bir geçiş yaşayıp (roll spike'ı eşliğinde) **yeni, farklı bir sabit
değere** (+1.49) yerleşti — sıfır değil, yalnızca işaret ve büyüklüğü
değişmiş başka bir kalıcı hata. `Fx` (nu_des(3)) de sürekli negatife
büyüyüp (-2.96→-11'e kadar) ancak ~-6'da yarı-kararlı bir düzeye oturdu —
muhtemelen yaw düzeltmesi için kullanılan diferansiyel tilt'in yan etkisi
(bkz. proje notları, "Fx'i düşük öncelikli tutma" gerekçesi).

**Yorum:** Bu, `nu_des(2)`'nin kaynağının **LESO'nun kolayca telafi
edebileceği yumuşak/yavaş bir bozucu olmadığını** güçlendiriyor — ya
`wo=15`'in yakalayamayacağı kadar "sert" (adım benzeri, hızlı) bir şey,
ya da LESO'nun kendisi bu spesifik senaryoda (INDI'nin zaten zayıf yaw
önceliğiyle etkileşerek) yakınsamıyor/salınıyor. **Roll/pitch'teki
iyileşme gerçek ve tutulmaya değer** (aşağıya bkz.), ama yaw sorunu
BAŞKA bir mekanizma gerektiriyor — muhtemelen `Ws_yaw=3`'ü artırmak
(rebuild gerektirir, bu oturumda denenmedi) ya da yaw'ın kendi P/rate
kazançlarını (`Kp_att[2]`, `Kp_rate[2]`) SITL'in gerçek dinamiğine göre
yeniden ayarlamak.

**Durum:** `leso_enable_yaw=1` DEĞİŞİKLİĞİ KALICI HALE GETİRİLMEDİ (yalnızca
`test_sp` çağrısında elle denendi, `Run()`'daki varsayılan
`{true,true,false}` ve `custom_command`'daki varsayılan `false`
DOKUNULMADAN bırakıldı) — çünkü yaw kendisi hâlâ yakınsamıyor ve kalıcı
yapmadan önce Ws_yaw ile birlikte ayrıca test edilmesi gerekiyor.

---

### Adım 7 — `Ws_yaw` artırma denemesi (3→6), `leso_enable_yaw=1` ile birlikte (2026-07-26, GERİ ALINDI — DAHA KÖTÜ)

**Gerekçe:** Adım 6'nın en somut ipucu buydu — LESO açıkken bile yaw
yakınsamıyordu; düşük `Ws_yaw=3`'ün WLS'e yeterli yaw-düzeltme yetkisi
vermediği düşünüldü.

**Disiplin (Y-işareti fiyaskosundan sonra):** Önce saf MATLAB'da
(`indi_attitude_controller.m` + `sf_wls_alloc.m`'de `Ws_yaw` 3→6)
`run_hover_gust_test`/`run_transition_test` çalıştırıldı — **regresyon
yok** (RMS p/q ~aynı, 0.0064/0.0016). Ancak bu, MATLAB'ın zaten
`nu_des(2)`'nin SITL'deki gibi kalıcı bir hata görmediği anlamına
geliyordu (aday çözüm 3'te not edildiği gibi sorun saf MATLAB'da hiç
yeniden üretilmiyor) — yani bu "güvenli" kontrol, SITL'deki asıl senaryoyu
test etmiyordu, sadece bariz bir regresyon olmadığını doğruluyordu.
`TiltrotorIndiParams.hpp`'de `WS_YAW=6.0f` yapılıp rebuild edildi.

**SITL sonucu (26s, `leso_enable_yaw=1` ile birlikte):**

| Aralık (~t) | roll | pitch | **yaw** | Fx | Fz |
|---|---|---|---|---|---|
| t≈0s | 0.01 | 0.02 | -1.18 | -2.96 | -0.24 |
| t≈14s (örnek 28) | 0.66 | -0.14 | -1.16 | -7.84 | 12.87 |
| t≈17s (örnek 34) | 0.73 | -0.18 | -2.29 | -12.31 | 15.05 |
| t≈21.5-24.5s (örnek 42-47) | ~0.32-0.35 | ~0.0-0.08 | **-0.07..-0.09** (sıfıra en yakın an) | -11..-16 | 6.8-7.4 |
| t≈26s (örnek 52, son) | 0.35 | 0.09 | -2.54 (tekrar bozuluyor) | **-27.99** | 7.66 |

Test sonunda `T0dbg2`: `Wu0=1000000.0, dmin0=-0.0000` — **T0 yine sıfıra
kilitlenmiş** (aynı eski imza).

**Sonuç: DAHA KÖTÜ, GERİ ALINDI.** `Ws_yaw`'ı artırmak yaw'ı **hiç
hızlandırmadı** (ilk ~14s boyunca -1.16..-1.18'de, `Ws_yaw=3` ile
BİREBİR AYNI büyüklükte kaldı — bu ağırlığın sınırlayıcı faktör
OLMADIĞINI gösteriyor). Roll bu sefer daha da büyüdü (0.7'ye kadar,
Adım 6'daki <0.3'ün üstünde) ve **Fx talebi -28N'e kadar patladı**
(Adım 6'daki -6'nın çok üstünde) — beklendiği gibi, yaw'a daha fazla
öncelik vermek diferansiyel tilt kullanımını artırıp Fx yan etkisini
büyüttü (`Ws_Fx=0.05` hâlâ çok düşük, bunu durdurmuyor). t≈21-24s'deki
kısa "yaw sıfıra yakın" penceresi kalıcı bir yakınsama değildi — test
sonunda yaw tekrar bozuldu ve T0 yine sıfıra kilitlendi. **`Ws_yaw`,
üç dosyada da 3.0'a geri alındı**, MATLAB testleriyle doğrulandı, `slx`
ve PX4 modülü yeniden build edildi.

**Çıkarım:** Yaw'ın yakınsamama sorunu bir "öncelik/ağırlık" sorunu
değil — WLS ne kadar öncelik verirse versin aynı yavaşlıkta tepki
veriyor, bu da sorunun **WLS ağırlıklandırma katmanının dışında**
(muhtemelen `Kp_att[2]`/`Kp_rate[2]` kazançlarında, ya da hâlâ
doğrulanamamış geometri/frame sorununda) olduğunu düşündürüyor.

---

### Adım 8 — Soru: "Gazebo ile ilgili bir sorun olabilir mi?" (2026-07-26)

**Yeni açı:** Kontrolcü katmanında (Wu, Ws, Kp) aramanın tükendiği bir
noktada, soru fizik simülatörünün KENDİSİNE çevrildi.

**Kritik yapısal bulgu — kontrolcü "kör" çalışıyor:**
`MulticopterIndiTiltrotor.cpp` içindeki `_u_actual` (WLS'in effectiveness
matrisini VE kutu kısıtlarını hesapladığı "gerçek" aktüatör durumu),
Gazebo'dan HİÇ geri besleme okumuyor — kendi varsaydığı basit birinci
derece gecikme modeliyle (`ROTOR_TAU_UP/DOWN`, `TILT_TAU=0.15s`,
`TILT_RATE_MAX=3rad/s`) open-loop entegre edilen bir **"gölge" model**
(kod yorumu: "No servo/ESC position feedback exists in PX4"). Saf
MATLAB'da bu sorun yapısal olarak imkânsız — plant (`tiltrotor_plant_deriv.m`)
ve kontrolcü AYNI basit birinci-derece modeli paylaşıyor, ayrışma
olamaz. SITL'de ise gölge model, GERÇEK Gazebo fiziğinden bağımsız
ilerliyor — ikisi arasında sapma birikebilir.

**Gazebo'nun gerçek aktüatör dinamiği kontrol edildi (`model.sdf`):**
- **İtki (motor) zaman sabitleri EŞLEŞİYOR:** SDF `timeConstantUp=0.0125`,
  `timeConstantDown=0.025` — `ROTOR_TAU_UP`/`ROTOR_TAU_DOWN` ile BİREBİR
  aynı. Bu kanalda mekanik uyuşmazlık YOK.
- **Tilt servoları TAMAMEN FARKLI bir dinamikle çalışıyor:**
  `gz-sim-joint-position-controller-system` ile **P=100, I=0, D=0,
  `cmd_max=2`, `err_max=0.2`** bir PID pozisyon kontrolcüsü — yani
  çıkış torku `100·hata` yerine pratikte **±2 (N·m) ile sınırlı**
  (`100×0.2=20 > cmd_max=2`, yani ~1.1°'den büyük her hata için
  doygunlaşıyor), üstelik eklem üzerinde `friction=1.0` var ve pervane
  reaksiyon/jiroskopik yüklerine karşı bu sınırlı torkla çalışmak
  zorunda. Kontrolcünün varsaydığı model (basit, yüksüz, `TILT_TAU`
  ile üstel gecikme) bu gerçek tork-sınırlı/sürtünmeli PID davranışını
  YAKALAMIYOR.

**Ampirik doğrulama (canlı `gz model -m tiltrotor_indi_0 -l motor_N -p`
ile gölge model karşılaştırması, aynı repro senaryosu):**

| t | δ0 gölge/gerçek (fark) | δ1 gölge/gerçek (fark) | δ2 gölge/gerçek (fark) |
|---|---|---|---|
| 5s | 0.0959 / 0.0924 (0.0035) | 0.000 / 0.0089 (0.0089) | — |
| 15s | 0.0055 / 0.0091 (0.0036) | 0.3535 / 0.3492 (0.0043) | 1.2361 / 1.3009 (**0.0648**) |
| 23s | 0.0091 / 0.0035 (0.0056) | 0.000 / 0.0051 (0.0051) | 1.2856 / 1.2405 (**0.0451**) |

**Yorum:** Gerçek bir sapma VAR ve doğrulandı — ama iki farklı
karakterde. δ0/δ1 (kanat rotorleri) için sapma küçük ve sabit kalıyor
(~0.003-0.009 rad, <0.5°) — tek başına bir çöküşü açıklamaya yetecek
büyüklükte değil. **δ2 (kuyruk) için sapma çok daha büyük (~0.045-0.065
rad, ~2.6-3.7°) VE işaret değiştiriyor** (15s'te gölge geride, 23s'te
gölge önde) — yani monotonik bir "drift" değil, gerçek bir **dinamik
izleme hatası/salınım**, tam da tork-sınırlı PID + sürtünme + değişken
yük (T2 aynı pencerede 18.9N'dan 45N'a çıkıyor) kombinasyonundan
beklenecek türden. Bu, T2'nin neden tekrar tekrar tavana kilitlenip
cruise-tilt'e sürüklendiğiyle (bkz. Adım 3, 6, 7) zamansal/nitel olarak
tutarlı.

**Sonuç: DOĞRULANMIŞ, GERÇEK bir Gazebo-kaynaklı katkı — ama muhtemelen
TEK BAŞINA yeterli değil.** Ölçülen sapma büyüklüğü (birkaç derece)
gözlenen tam çöküşü (T0=0, T1/T2=tavan, yaw ±160°) tek başına
açıklayacak kadar büyük görünmüyor; muhtemelen bir **çarpan/tetikleyici**
— küçük bir gerçek/gölge sapması, effectiveness matrisini hafifçe
yanlış hesaplatıp WLS'in zaten kırılgan olan (Adım 1-7'de belgelenen)
roll/yaw/Fx dengesini bozmaya yetecek kadar itiyor olabilir, tam kök
neden değil ama **gerçek, ölçülmüş bir katkı payı**.

**Henüz denenmedi (somut sonraki adım):** Gölge modelin tilt dinamiğini
gerçek servo'ya yakınsatmak — ör. `TILT_TAU`'yu büyütmek ve/veya
`TILT_RATE_MAX`'ı gerçek tork-sınırlı davranışı yansıtacak şekilde
yüke bağlı küçültmek (SITL'e özgü bir düzeltme olur, MATLAB'ı
etkilemez çünkü MATLAB'ın kendi plant'i zaten aynı basit modeli
kullanıyor — `safe-control-change` skill'inin öngördüğü gibi önce
MATLAB'da kontrol edilip sonra SITL'de en az 25s doğrulanmalı).

---

### Adım 9 — `TILT_RATE_MAX` düşürme (3.0→2.0 rad/s), Adım 8'in devamı (2026-07-26)

**Gerekçe:** Adım 8'de ölçülen δ2 sapması işaret-değiştiren/salınımlı
olduğundan (tek yönlü bir "gecikme" değil), `TILT_TAU`'yu ayarlamak
yerine daha savunulabilir bir müdahale seçildi: WLS'in kutu kısıtının
(`TILT_RATE_MAX`), gerçek tork-sınırlı servonun (P=100, `cmd_max=2 Nm`,
`friction=1.0`, artı yükle büyüyen jiroskopik reaksiyon) tek tick'te
GÜVENİLİR biçimde teslim edemeyeceği kadar büyük tilt komutlarına izin
vermesini engellemek. `TILT_RATE_MAX` %33 düşürüldü (3.0→2.0 rad/s).
Yalnızca `TiltrotorIndiParams.hpp`'de (PX4'e özgü) — `tiltrotor_params.m`
BİLEREK dokunulmadı (MATLAB'da plant+gölge model tanım gereği hep aynı,
test edilecek bir "gerçek vs gölge" ayrışması yok; `safe-control-change`
skill'inin genel kuralının bilinçli bir istisnası, gerekçesi koddaki
yorumda belgelendi).

**SITL sonucu — İKİ ayrı 25s koşu:**

**Koşu 1 (yalnızca T0dbg/T2dbg log analizi):**

| Aralık (~t) | roll | yaw | Fx |
|---|---|---|---|
| t≈0s | -0.02 | -1.18 | -2.96 |
| t≈6s (örnek 12, tepe) | **-0.57** | 0.62 | -6.37 |
| t≈17-25s (örnek 34-52) | **-0.19..-0.33** (SINIRLI, büyümüyor) | **+1.00..+1.06** (SABİT PLATO) | -4.4..-14.3 |

Test sonunda `T0dbg2`: **`Wu0=1.0`** (BIG_M DEĞİL) — **bu oturumdaki 9
denemenin İLK KEZ** T0'ın kilitlenmediği/serbest aktüatör olarak
kaldığı sonuç.

**Koşu 2 (canlı `tiltrotor_indi_status`/`vehicle_attitude` ile tam durum,
aynı senaryo, 25s):**

```
u_actual:  [37.35, 0.00, 12.85, 0.000, 0.231, 0.000]   (T1 bu sefer 0'a kilitli)
sat_flag:  [False, True, False, True, False, True]
nu_des:    [-1.40, -0.40, 1.05, -0.00, 26.13]           (Fz talebi cok yuksek)
Roll: 0.1°  Pitch: 0.0°   Yaw: -80.3°
z: -1.67 (hedef -7.34'e karsi -- tirmanis DURMUS)
vz: -0.125 m/s  (ONCEKI -11 m/s FELAKETININ AKSINE, neredeyse SIFIR)
```

**Değerlendirme — GERÇEK, ÖLÇÜLEBİLİR İYİLEŞME (ama YENİ bir sorunla):**

- ✅ **Roll/pitch her iki koşuda da sınırlı/küçük kaldı** — koşu 2'de
  neredeyse mükemmel (0.1°/0.0°), koşu 1'de sınırlı (<0.6 rad, önceki
  denemelerin çoğunda >0.7 rad'a büyüyordu). Bu, dokuz denemedeki EN
  İYİ attitude sonucu.
- ✅ **Dikey hız kontrolsüzlüğü YOK** — `vz=-0.125 m/s`, Adım 3/8'deki
  `vz=-11 m/s` felaketinin aksine (o zamanki komut edilenin 5.5 katıydı,
  şimdi neredeyse sıfır). Sönümsüz salınım/flip riski bu koşularda
  gözlenmedi.
- ✅ **Yaw hâlâ sıfıra yakınsamıyor ama artık SINIRSIZ SAVRULMUYOR** —
  koşu 1'de +1.0-1.06 rad'lık sabit bir platoya oturup orada kaldı
  (~t=17s'den t=25s'e kadar, 8 saniye boyunca sabit); koşu 2'de -80.3°'ye
  ulaştı ama bu Adım 3/8'deki -161.9°'nin YARISI kadar ve `nu_des(2)=1.05`
  (koşu 1'in platosuyla aynı büyüklükte) — savrulmanın YAVAŞLADIĞINA/
  düzleştiğine işaret ediyor, tamamen durmasa da.
- ❌ **YENİ sorun: bir kanat rotorü (T1) yine 0'a kilitleniyor VE bu
  sefer toplam itki yetersiz kalıp tırmanış tamamen DURUYOR** (`z=-1.67`,
  hedef `-7.34`) — `nu_des(4)=26.13` çok yüksek bir Fz açığına işaret
  ediyor. Yani "flip/kontrolsüz düşüş" riski azaldı ama yerine "yetersiz
  itki, görevi tamamlayamama" sorunu geldi — bu daha az TEHLİKELİ (ani
  çarpışma riski düşük) ama HÂLÂ bir görev-başarısızlığı.

**Sonuç: KISMİ BAŞARI — İLK KEZ net bir iyileşme yönü bulundu, ama
tam çözüm değil.** `TILT_RATE_MAX` düşürmek gerçekten roll/pitch/vz
kararlılığını iyileştirdi (Adım 8'in hipotezini destekliyor: gölge/gerçek
tilt sapması gerçekten katkıda bulunuyormuş), ama T0/T1'in periyodik
olarak sıfıra kilitlenme eğilimini TAMAMEN ortadan kaldırmadı — yalnızca
sonucun şiddetini (flip riski → itki yetersizliği) değiştirdi. **Bu
değişiklik KALICI YAPILDI** (geri alınmadı) çünkü net bir iyileşme —
ama tek başına yeterli değil, ek çalışma gerekiyor.

**Sonraki adım için ipucu:** Artık iki bağımsız katkı payı doğrulandı
(Adım 8/9: tilt-servo gerçekçiliği; Adım 1-7: WLS/yaw ağırlıklandırma
dinamiği) — ikisi birden kısmi iyileşme sağladı ama hiçbiri tek başına
yeterli değil. Bir sonraki mantıklı deney, TILT_RATE_MAX=2.0 KALICI
İKEN (D) yaw P/rate kazançlarını (`Kp_att[2]`/`Kp_rate[2]`) ayrıca
denemek olabilir — iki düzeltme birlikte T1'in kilitlenmesini de
çözebilir.

---

### Adım 10 — Yaw `Kp_att[2]`/`Kp_rate[2]` artırma (2026-07-26, GERİ ALINDI — MATLAB'da yakalandı)

**Gerekçe:** Adım 9'un bıraktığı yerden — (D)'nin denenmesi.
`gain_schedule.m`'deki yorum, yaw kazancının roll/pitch'e göre
KASITLI düşük tutulduğunu ve aksi halde "WLS'in roll/yaw arasında
çekişmesi sonucu sönümsüz bir limit cycle" oluşacağını açıkça
uyarıyordu — ama bu hiç ampirik olarak test edilmemişti.

**Uygulama:** `Kp_att_hover[2]` 1.5→2.5, `Kp_rate_hover[2]` 2.0→3.5
(roll/pitch'in tam eşiti değil — 3.0/4.0'a çıkarmak yerine kademeli,
orta bir artış seçildi; cruise değerleri orantılı güncellendi).
`safe-control-change` disiplinine göre önce yalnızca MATLAB'da
(`gain_schedule.m`) test edildi.

**Sonuç: GERÇEK REGRESYON, MATLAB'DA YAKALANDI, PX4'E HİÇ GİTMEDİ.**
`run_hover_gust_test`: RMS p/q **0.0065/0.0015 → 0.046/0.155**
(~10-100× kötüleşme) — kod yorumunun uyardığı roll↔yaw limit cycle
BİREBİR tetiklendi. `run_transition_test` etkilenmedi (bu test yaw
bozucusu içermiyor, dolayısıyla çekişme orada görünmedi — bu da
regresyonun spesifik olarak roll+yaw etkileşiminden geldiğini
doğruluyor). Değişiklik `gain_schedule.m`'de geri alındı, MATLAB
testleriyle baseline'a dönüş doğrulandı. **PX4 tarafına hiç
dokunulmadı** (rebuild/SITL testi gerekmedi — tam da
`safe-control-change` skill'inin amaçladığı, ucuz/erken yakalama).

**Çıkarım — kalıcı, önemli:** Bu, "yaw'a daha fazla otorite ver"
ailesindeki **üçüncü** başarısız deneme (Adım 6: LESO açma — kısmi/
karışık, Adım 7: `Ws_yaw` artırma — başarısız, Adım 10: `Kp` artırma —
başarısız, MATLAB'da bile). Roll↔yaw paylaşımlı-aktüatör kısıtlaması
(kod yorumunda önceden tarif edilen) bu airframe'de **gerçekten sert
bir tasarım duvarı** — yaw'ın kendisini "daha güçlü" yapmaya çalışan
hiçbir müdahale güvenli görünmüyor. Buna karşılık, Adım 9'un YÖNÜ
(aktüatör-dinamiği gerçekçiliği, yaw'ı GÜÇLENDİRMEDEN) tek başarılı
yön olarak öne çıkıyor — sonraki çalışma muhtemelen bu hatta devam
etmeli (bkz. §4 (G): TILT_RATE_MAX=2.0 ile hâlâ süren kanat-rotoru
kilitlenmesini izole etmek), yaw kazancı/önceliği hattı değil.

---

### Adım 11 — §4 (G) izole edildi VE KÖK NEDEN BULUNDU: itki komut eşlemesi karesel yerine doğrusal (2026-07-27)

**Başlangıç noktası:** §4'ün (G) maddesi — "TILT_RATE_MAX=2.0 kalıcıyken
neden hâlâ periyodik olarak bir kanat rotorü 0'a kilitleniyor". Bunu
izole etmek için önce **eksen-bazlı atıf (attribution) tanı logu**
eklendi.

**11a — Yeni tanı aracı (`WRdbg`):** WLS'in kısıtsız çözümü
`du = H⁻¹·(Gᵀ·Ws²·nu_des + Wu²·du_pref)` olduğundan `nu_des`'te
DOĞRUSALDIR; dolayısıyla her `nu_des` ekseninin `du(i)`'ye katkısı
BİREBİR ayrıştırılabilir:
`a[k] = (H⁻¹·(G(k,:)ᵀ·Ws²(k)·nu_des(k)))(i)`, artı BIG_M sabitleme
terimi `pin(i)`. Yapı gereği `Σₖa[k] + pin == du(i)` (son kutu
kırpması öncesi) — yani satır kendi kendini doğruluyor. **Kritik
boşluk:** mevcut `T0dbg`/`T2dbg` yalnızca rotor 0 ve 2'yi logluyordu;
Adım 9 koşu 2'de asıl kilitlenen **T1 hiç enstrümante değildi**. Yeni
log her iki kanat rotorünü de kapsıyor.

**11b — (G) CEVAPLANDI: kilitlenme roll+Fz üst üste binmesi, YAW DEĞİL.**
Baseline koşusundan (26s, 51 örnek) T1'in sıfıra sürüklendiği pencere
(u₁: 15.5→11.7→4.6→0 N, ~1.5s içinde):

| örnek | a1[roll] | a1[pitch] | a1[yaw] | a1[Fx] | a1[Fz] | du1 | u₁ (N) |
|---|---|---|---|---|---|---|---|
| 4 | **-0.094** | +0.072 | 0.000 | 0.000 | -0.048 | -0.108 | 15.50 |
| 5 | **-0.147** | -0.146 | 0.000 | 0.000 | -0.086 | -0.418 | 11.66 |
| 6 | **-0.196** | -0.086 | 0.000 | 0.000 | -0.072 | -0.386 | 4.61 |
| 7-18 | ~0 | ~0 | ~0 | ~0 | ~0 | 0 | **0 (BIG_M kilitli, ~5.5s)** |

Aynı anda T0 için roll (+0.4…+0.9) ve Fz (-0.38…-0.94) **birbirini
neredeyse tam götürüyordu** (du0 ≈ 0.02-0.10 gibi küçük bir artık).

**Mekanizma (yeni, net):** Roll talebi iki kanat rotorünü ZIT yönde
iter (diferansiyel itki), Fz talebi ise ikisini AYNI yönde. Hangi
rotorde bu iki katkı aynı işaretli denk gelirse orada **toplanıp**
rotoru tabana sürüyor; diğerinde götürüyor. Kilitlenmenin koşudan
koşuya T0↔T1 arasında yer değiştirmesi (Aday çözüm 2'de gözlenen) bunun
doğrudan sonucu.

**Yan ürün — yaw hattının neden tükendiğinin YAPISAL kanıtı:** `a[yaw]`
kanat rotorü itki kanallarında **her örnekte tam 0.000** (51 örnekte
tek istisna: bir örnekte -0.133). Yani yaw talebinin kanat rotoru
itkisi üzerinde pratikte hiç kaldıracı yok. Bu, Adım 6/7/10'un
(LESO/Ws_yaw/Kp ile "yaw'a otorite ver") neden üçünün de başarısız
olduğunu ampirik olarak açıklıyor — ampirik değil yapısal bir duvar.

**11c — Beklenmedik gözlem tetikleyici oldu:** Baseline koşusunda araç
**hiç kalkmadı** (z: 0.46→0.54, hedef -5.54), oysa gölge model 60 N
itki ürettiğine inanıyordu (ağırlık 5 kg = 49 N). Bu tutarsızlık
çıktı katmanına bakılmasına yol açtı.

**11d — KÖK NEDEN: Newton→normalize komut eşlemesi karesel modeli
doğrusal sanıyordu.** `MulticopterIndiTiltrotor.cpp` şunu yapıyordu:
```cpp
motors.control[i] = u_cmd(i) / ROTOR_TMAX;   // DOĞRUSAL
```
Gerçek zincir ise:
- `GZMixingInterfaceESC` mikser çıkışını doğrudan Actuators
  `velocity` alanına yazıyor; `MixingOutput` normalize komutu
  `SIM_GZ_EC_MIN1..3=10` → `SIM_GZ_EC_MAX1..3=1500` aralığına
  **doğrusal** ölçekliyor: `w = 10 + control·1490`
- `MulticopterMotorModel` (Adım 5'te ham kaynaktan zaten
  doğrulanmıştı): `T = motorConstant·w² = 2e-5·w²`

yani **itki, komutun KARESİYLE** değişiyor. İki model yalnızca uç
noktada uyuşuyor (`control=1 → 2e-5·1500² = 45 N = ROTOR_TMAX`) — hatanın
bu kadar uzun süre gözden kaçmasının sebebi bu. Arada doğrusal eşleme
sistematik olarak eksik itki veriyor:

| u_cmd (N) | gerçek itki (N) | d(gerçek)/du (kontrolcü 1.0 sanıyor) |
|---|---|---|
| 5 | 0.62 | **0.23** |
| 10 | 2.33 | 0.45 |
| 22.5 | 11.40 | 1.00 |
| 37.8 | 31.83 | 1.67 |
| 45 | 45.00 | **1.99** |

**İki ayrı zarar:**
1. **Toplam itki açığı** — baseline koşusunun dört örneğinde de gerçek
   toplam itki ağırlığın altında kaldı (43.7 / 28.8 / 30.0 / 44.6 N vs
   49.05 N), yani araç **fiziksel olarak kalkamazdı**. Adım 9'un
   "yetersiz itki, tırmanış duruyor" bulgusunun gerçek açıklaması bu.
2. **Etkinlik matrisi (G) itkiye bağlı biçimde YANLIŞ** — G, gölge
   `_u_actual`'dan Newton cinsinden kuruluyor, ama kontrolcünün kendi
   komut değişkenine göre gerçek etkinlik `2u/45`. Düşük itkili bir
   rotorde gerçek etkinlik çöküyor (u=5 N'da %23) ama kontrolcü hâlâ
   1.0 sanıyor: azaltma komutu bekleneni vermiyor, WLS daha da
   azaltıyor → **tabana doğru pozitif geri besleme**. 11b'deki
   roll+Fz üst üste binmesini kilitlenmeye çeviren çarpan tam olarak bu.

**Bu, "saf MATLAB'da neden hiç görülmüyor" sorusunun da kesin cevabı:**
MATLAB plant'i (`tiltrotor_plant_deriv.m`) doğrudan Newton ile
sürülüyor, normalize komut katmanı YOK — dolayısıyla karesel uyuşmazlık
orada yapısal olarak imkânsız.

**Düzeltme** (`thrustToNormalized()`, `TiltrotorIndiControl.hpp`):
```
w = ROTOR_WMAX·sqrt(u/ROTOR_TMAX)
control = (w - ROTOR_WMIN)/(ROTOR_WMAX - ROTOR_WMIN)
```
`ROTOR_WMIN=10` (SIM_GZ_EC_MIN) yeni sabit olarak eklendi. SITL/PX4'e
özgü çıktı eşlemesi — MATLAB karşılığı olmadığı için
`safe-control-change`'in MATLAB-önce kuralına Adım 9 gibi bilinçli,
gerekçesi kodda belgelenmiş bir istisna.

**11e — İKİNCİ HATA: test setpoint'i kontrolcüye hiç ulaşmıyormuş.**
Düzeltme sonrası araç hâlâ tırmanmayınca `nu_des(4)≈+0.05` incelendi;
bu ancak `z_sp ≈ mevcut irtifa` ise mümkün — yani `Run()`'daki
`_setpoint_valid=false` yedeği (`z_sp = lpos.z`, "mevcut irtifayı
koru"). Sebep: `custom_command()`'daki `test_sp`, fonksiyon-yerel bir
`uORB::Publication` kullanıyordu; fonksiyon dönerken yıkıcı
`orb_unadvertise()` çağırıyor, node "advertised değil" görünüyor,
`Subscription::advertised()` sürekli başarısız oluyor ve
`_setpoint_sub.updated()` **hiç tetiklenmiyor**.

**Ampirik doğrulama:** 60 ardışık `test_sp` çağrısından sonra bile
`listener tiltrotor_indi_setpoint` → **"never published"**, `nu_des(4)`
~0.24'te sabit. Düzeltme: publication `static` yapıldı (px4 süreci
boyunca advertise canlı kalıyor) + publish dönüş değeri kontrol
ediliyor.

**Bunun geriye dönük anlamı — ÖNEMLİ:** Bu düzeltmeden önceki **tüm**
SITL koşuları (Aday çözüm 1'den Adım 10'a kadar) aslında
"6 m tırmanış" değil, **"mevcut irtifayı koru + roll/pitch/yaw_sp=0"**
senaryosuydu. Gözlenen yaw savrulması gerçekti (yaw_sp zaten 0'dı),
ama Adım 9'un "tırmanış durdu / itki yetersiz" yorumu yanlış
atfedilmişti — ortada hiç tırmanış komutu yoktu.

**11f — Doğrulama: iki bağımsız koşu, ikisi de temiz.**

*Koşu 1 (25s, z₀=-0.78, z_sp=-6.78):* setpoint topic'te doğrulandı;
araç 6 m tırmandı (z=-6.69 @5s, -7.07 @25s), itki `sat_flag`'leri 25s
boyunca **tamamen False**, roll/pitch ≤0.5°, yaw **yakınsadı**:
100.2° → 31.3° → 12.1° → 3.4° → 3.6°.

*Koşu 2 (40s, z₀=-0.49, z_sp=-6.49):*

| t | z | vz | roll/pitch | yaw | u_actual (itki) | itki sat_flag |
|---|---|---|---|---|---|---|
| 5s | -6.17 | -0.20 | 0.5/-0.3 | 73.0° | 18.4 / 18.8 / 13.3 | F F F |
| 10s | **-6.49** | 0.03 | -0.0/0.1 | -10.4° | 18.8 / 19.2 / 13.5 | F F F |
| 15s | -6.65 | 0.04 | -0.1/-0.0 | -1.6° | 18.1 / 18.0 / 13.3 | F F F |
| 20s | -6.55 | 0.03 | 0.0/-0.0 | -56.4° | 18.5 / 18.6 / 13.3 | F F F |
| 25s | -6.65 | 0.10 | 0.0/0.0 | -50.8° | 18.2 / 18.8 / 13.2 | F F F |
| 30s | -6.53 | -0.03 | 0.2/-0.2 | -22.0° | 17.4 / 18.2 / 13.1 | F F F |
| 35s | -6.50 | -0.10 | -0.0/-0.2 | -5.2° | 15.9 / 16.6 / 13.8 | F F F |
| 40s | **-6.50** | -0.05 | 0.1/0.1 | 7.4° | 14.5 / 15.3 / 14.0 | F F F |

Log analizi (82 `WRdbg` örneği, 40s): **hiçbir örnekte BIG_M
sabitlemesi yok** (`Wu0`/`Wu1` = 1e6 sayısı: 0/0), kanat rotoru itkileri
14.50–22.05 N bandında — ne 0'a ne 45'e yaklaştı. Eksen katkıları artık
|a| ≤ 0.03 (baseline'daki roll +0.9 / Fz -0.94 çekişmesiyle kıyaslayın).

**SONUÇ — `sitl-lockup-check` geçti/kaldı kriterine göre:**

| Kriter | Sonuç |
|---|---|
| Hiçbir aktüatör 0/45 N'a kalıcı kilitlenmiyor | ✅ **GEÇTİ** (40s, 82 örnek, sıfır kilitlenme) |
| `vz`, `ALT_VZ_MAX=2.0 m/s`'yi belirgin aşmıyor | ✅ **GEÇTİ** (\|vz\| ≤ 0.20 m/s) |
| Yaw ±30°'yi aşmıyor | ❌ **KALDI** (-56.4°'ye ulaştı) |

**Dürüst değerlendirme: aktüatör kilitlenmesi ve dikey hız
kontrolsüzlüğü ÇÖZÜLDÜ; yaw ekseni büyük ölçüde iyileşti ama hâlâ
kriteri geçmiyor.** Yaw artık sınırsız savrulmuyor (-161.9° ve
büyüyordu) — ±60°'lik bantta kalan, kendini düzelten bir salınım
(koşu 1'de 3.6°'ye yakınsadı, koşu 2'de 7.4°'de bitti). Adım 6/7/10'un
belgelediği roll↔yaw paylaşımlı-aktüatör zayıflığı hâlâ görünür durumda,
ama artık kararsızlık değil, sönümlü gezinme biçiminde.

**Not — Adım 9 (TILT_RATE_MAX=2.0) gerekçesi zayıfladı:** O adımın
sağladığı kısmi iyileşme muhtemelen asıl nedeni (itki eşlemesi)
dolaylı olarak hafifletmesinden geliyordu. Şimdi kök neden
giderildiğine göre 3.0'a geri döndürmek yeniden değerlendirilmeli —
ama kendi başına, ayrı bir doğrulama koşusuyla (bkz. §4).

---

### Adım 12 — §4 (H) denendi; yaw savrulmasının İKİ ayrı mekanizması ölçülerek ayrıştırıldı, biri (km işareti) düzeltildi (2026-07-27)

**Başlangıç noktası:** §4 (H) — Adım 6/7/10'un yaw denemeleri YANLIŞ bir G
matrisi altında yapıldığı için şüpheliydi; en ucuzu (rebuild gerektirmeyen)
`leso_enable_yaw=1` düzeltilmiş kod üzerinde tekrar denenecekti.

**12a — LESO yaw açık: KESİN OLARAK REDDEDİLDİ (düzeltilmiş G altında).**
`test_sp 0 0 0 0 z_sp 1 1 1` ile koşu, 5 saniyede felaketle sonuçlandı:
`d_hat[2]` 377'ye fırladı, `nu_des[yaw]` ilk saniyede -45'e gitti, araç
**ters döndü** (roll ≈ -178°), her iki kanat rotoru 45 N'a kilitlendi
(160 BIG_M sabitlemesi). Aynı ortamda hemen ardından yapılan kontrol
koşusu (`1 1 0`) temiz uçtu — yani bu, ortamdan değil doğrudan LESO yaw
ekseninden kaynaklanıyor. **Adım 6'nın "kısmen olumlu" sonucu artık
geçersiz; yaw ekseninde LESO açılmamalı.**

**12b — ÖLÇÜM HATASI DÜZELTMESİ: "±60° sınırlı gezinme" aslında sürekli
dönüştü (aliasing).** Adım 11'in sonucu 5 saniyelik `px4-listener`
örneklemesine dayanıyordu. `vehicle_angular_velocity.xyz[2]` (yaw hızı)
ilk kez ölçüldüğünde: **1.55, 0.17, 1.63, 2.92, 1.65, 0.07, 0.92, 3.48
rad/s** — araç 200°/s'ye varan hızlarla sürekli dönüyor. 5 s'lik
örnekleme, ~1.4 rad/s'lik bir dönüşü (5 s'de ~400°) rastgele yaw
açılarına dönüştürüyor. Tam çözünürlüklü ulog analizi (40 s):
**ortalama +0.79 rad/s, toplam integre yaw 1818° (5 tam tur)**, roll/pitch
hız RMS'i ise sadece 0.004 rad/s. **Adım 11f'in "sönümlü/kendini
düzelten gezinme" yorumu yanlıştı — sınırlı olan yaw AÇISI değil, sadece
sarmalanan bir gösterimdi.** Ders: yaw eksenini açıdan değil **hızdan**
ölçün; `px4-listener` ile 5 s aralıklı örnekleme bu eksende yanıltıcıdır.

**12c — Test artefaktı hipotezi elendi.** `yaw_sp=0` iken araç 91°'de
doğduğu için dış döngü 2.4 rad/s'lik bir slew istiyordu; bu, dönüşün
sebebi olabilirdi. `yaw_sp = mevcut heading` (yani sıfır başlangıç hatası)
ile tekrarlandı: **araç yine arm'dan sonraki 4 s içinde +5.95 rad/s'ye
fırladı.** Dolayısıyla kaynak setpoint değil, arm anında var olan gerçek
ve dengelenmemiş bir yaw torkudur.

**12d — KÖK NEDEN #2 BULUNDU: `ROTOR_KM` işaretleri FRD çerçevesinde
tersti.** Zincir:

- gz-sim gövdeye `dragTorque = (0, 0, -turningDirection·T·km)` uygular;
  bu, rotor link'inin kendi çerçevesindedir (SDF'de `rpy = 0 0 0`, yani
  `base_link` ile hizalı; gz **FLU** kullanır: x-ön, y-**SOL**, z-**YUKARI**).
- Rotor 0 `ccw` (turningDirection=+1, T>0) → `tau_z(FLU) = -km·T < 0`.
  FLU→FRD'de z işaret değiştirir → **`tau_z(FRD) = +km·T > 0`** (burun sağa).
  Bu, yukarıdan bakınca CCW dönen bir rotorün fiziksel reaksiyonuyla da uyumlu.
- Kontrolcü/plant modeli ise `m_i = km_i·T_i·dir_i`, `dir=(0,0,-1)` →
  **`m_z = -km·T < 0`** (burun sola). **Üç rotor için de ters.**

Adım 3'ün notu gz formülünü doğru aktarmıştı ama yalnızca işaret
**DESENİNİ** (+,-,+) `ccw/cw/ccw` ile eşledi; desenin FRD'deki **TOPLAM**
işareti hiç karşılaştırılmadı. Zararı: `hoverTrim()`'in yaw sıfırlayıcı
diferansiyel tilt'i, dengesizliği gidermek yerine **aynı yönde ekliyordu**.

**Nicel doğrulama (teşhisi kanıtlayan ölçüm):** eski işaretlerle açık
çevrim öngörüsü, arm anında net **+1.54 N·m** (0.76 sürükleme + 0.78
yanlış yönlü trim tilt'i) → `α = +6.2 rad/s²`. Eski modelin kendi
öngörüsü ise "trim tam sıfırlar, α ≈ 0". **İki bağımsız koşuda ölçülen
tepe yaw ivmesi: +6.45 ve +6.56 rad/s².** Düzeltilmiş model %5 içinde
tutuyor, eski model tamamen çürüyor.

**Adım 11 ile AYNI SINIFTAN bir hata:** plant (`tiltrotor_plant_deriv.m`)
ve kontrolcü (`effectiveness_matrix.m`) aynı `p.rotor.km`'yi paylaştığı
için saf MATLAB bu hatayı **yapısal olarak göremez** — ikisi birlikte
yanlıştır, testler kendi içinde tutarlı kalır.

**12e — Düzeltme ve MATLAB regresyonu (`safe-control-change` prosedürü).**
`km = [-0.06, +0.06, -0.06]` (MATLAB + Simulink + PX4). Ayrıca
`hover_trim.m`/`hoverTrim()` düzeltildi: işaret değişince rotor 1 ile
sıfırlama **negatif tilt** isterdi (tilt fiziksel olarak [0, π/2]) —
karşı kanat rotorünün (zıt PY) **pozitif** tilt'i birebir eşdeğer
düzeltmeyi verdiğinden trim artık uygulanabilir olanı seçiyor.

| Ölçüt | Öncesi | Sonrası |
|---|---|---|
| `run_hover_gust_test` RMS p / q (LESO açık) | 0.0065 / 0.0015 | **0.0013 / 0.0004** |
| RMS p / q (LESO kapalı) | 0.0129 / 0.0031 | **0.0045 / 0.0009** |
| `run_transition_test` max\|omega\| | 0.0264 rad/s | **0.0126 rad/s** |
| transition tilt / hız / irtifa | 9.4° / 6.70 / -0.08 | 9.5° / 6.70 / -0.08 |
| `hover_trim` | δ1 = +0.1625 | **δ0 = +0.1625**, artık `tauz ≈ -0.011 N·m` |

MATLAB'da regresyon yok — aksine 2-5× **iyileşme**. (Beklenti nötrdü:
plant ve kontrolcü birlikte değiştiği için yaw dinamiği aynadan
yansıması olmalıydı. Fark, trim tilt'inin artık rotor 1 yerine rotor 0'da
olmasından ve bunun rüzgâr bozucusuyla kuplajının değişmesinden
geliyor olmalı — iyileşme yönünde olduğu için durdurucu değil, ama
mekanizması tam olarak izlenmedi.) Ayrıca trim tilt'inin rotor 0'a
geçmesi, uçuşta WLS'in zaten ampirik olarak δ0'ı ~0.2 rad'da tutuyor
olmasıyla **bağımsız biçimde tutarlı**.

**12f — SITL doğrulama: arm geçici rejimi ÇÖZÜLDÜ, sürekli dönüş DEVAM
EDİYOR.** 28 s koşu (`leso 1 1 0`):

| Ölçüt | Düzeltme öncesi | Düzeltme sonrası |
|---|---|---|
| arm+0-2s tepe yaw ivmesi | +6.56 / +6.45 rad/s² | **+0.47 rad/s² (14× azaldı)** |
| arm+2s'de yaw hızı | +3.37 rad/s | **-0.81 rad/s** |
| 5-28s ortalama yaw hızı | +0.49 … +1.43 rad/s | +1.44 rad/s (**değişmedi**) |
| roll/pitch hız RMS | 0.0036 / 0.0038 | 0.0026 / 0.0034 |
| aktüatör kilitlenmesi | yok | **yok (0 BIG_M)** |
| irtifa takibi | ✅ | ✅ (z=-6.55 hedef, ±0.1 m) |

**12g — Kalan dönüşün mekanizması ÖLÇÜLDÜ (yeni `YWdbg` tanı logu).**
`YWdbg`, talep edilen yaw torkunu (`nu2`), tahsisatın gerçekten ürettiğini
(`ach2 = (G·du)(2)`), yaw hızını (`r`) ve dış döngünün hız setpoint'ini
(`wsp2`) birlikte kaydediyor. 30 örnek:

- **Tahsisat, talep edilen yaw torkunun yalnızca %6.8'ini üretiyor**
  (ortalama `ach2/nu2`); örn. `nu2=-3.34 → ach2=-0.053`. `ach2` talepten
  bağımsız olarak **±0.05 N·m'ye çakılı**.
- Sebep doğrudan görünüyor: `ddelta` **her örnekte tam ±0.0050** — yani
  `TILT_RATE_MAX·dt = 2.0 × 0.0025` tilt slew limitinde sürekli doygun.
  Kanat tilt'i yaw'ın tek gerçek aktüatörü (`dtau_z/dδ ≈ 4.6 N·m/rad`),
  adım başına 0.005 rad ise ≈ 0.023 N·m demek — ölçülen tavanla birebir.
- **İkinci mekanizma:** `wsp2` (yaw hız setpoint'i) örnekler arasında
  sürekli işaret değiştiriyor ve ±3.0'a (`RATE_SP_LIMIT`) çarpıyor:
  -1.01, +1.01, +2.07, -1.00, -3.00, +1.10, -2.22, -3.00, +3.00 …
  Araç dönerken heading ±180°'de sarmalandığı için dış attitude döngüsü
  **kendini bozan bir komut üretiyor** — talep, tahsisatın yetişemeyeceği
  hızda yön değiştiriyor.

Yani kalan dönüş "yaw otoritesi az" ile "dış döngü dönerken anlamsız
komut veriyor" mekanizmalarının **bileşimi**; ikisi birbirini besliyor.

**SONUÇ — `sitl-lockup-check` kriterine göre:**

| Kriter | Sonuç |
|---|---|
| Hiçbir aktüatör 0/45 N'a kilitlenmiyor | ✅ **GEÇTİ** (0 BIG_M) |
| `vz`, `ALT_VZ_MAX`'ı aşmıyor | ✅ **GEÇTİ** |
| Yaw ±30°'yi aşmıyor | ❌ **KALDI** (sürekli dönüş, ort. +1.44 rad/s) |

---

### Adım 13 — §4 (K) uygulandı: eksen-bazlı yaw hız limiti — **YAW KRİTERİ İLK KEZ GEÇTİ** (2026-07-27)

**Hipotez (Adım 12g'nin ölçümünden doğrudan):** Sorun yaw otoritesinin
azlığı DEĞİL, dış attitude döngüsünün ürettiği hız setpoint'inin sürekli
işaret değiştirmesi. `RATE_SP_LIMIT` tek skalerdi (3.0 rad/s, üç eksen
için). Araç dönerken yaw hatası ±180°'de sarmalandığı için `omega_sp(2)`
±3 rad/s arasında salınıyor; `omega_dot_des = Kp_rate·(omega_sp - omega)`
olduğundan **zamanın yarısında iç döngü dönüşü sönümlemek yerine
hızlandırıyordu**. Limit gerçek dönüş hızlarının (1-3.5 rad/s) altında
tutulursa hata işaretini ölçülen hız belirler ve iç döngü her zaman
sönümleme yönünde çalışır.

**Değişiklik** (`safe-control-change` prosedürü, üç uygulama senkron):
`rate_sp_limit` skaler `3.0` → eksen bazlı **`[3.0, 3.0, 0.5]`** rad/s
(`tiltrotor_params.m` `p.ctrl.rate_sp_limit`, `sf_indi_rate_law.m`
literali, PX4 `RATE_SP_LIMIT[3]`). Yaw için 0.5 rad/s ≈ 29°/s.

**MATLAB regresyonu: tam nötr** (RMS p/q 0.0013/0.0004 ve 0.0045/0.0009,
transition max|omega| 0.0126 — Adım 12 sonrası değerlerle birebir aynı).
Beklenen sonuç: MATLAB'da araç zaten limiti zorlayacak hızda dönmüyor.

**SITL sonucu (ulog, t_arm+10…35 s):**

| Ölçüt | Adım 12 (limit 3.0) | **Adım 13 (limit 0.5)** |
|---|---|---|
| Ortalama yaw hızı | +1.52 rad/s | **-0.031 rad/s** (50×) |
| Yaw hızı RMS | 1.91 rad/s | **0.058 rad/s** (33×) |
| \|yaw hızı\| max | 3.68 rad/s | **0.305 rad/s** (12×) |
| 25 s'de integre dönüş | 1879° (5.2 tur) | **44°** |
| Yaw açısı | sürekli ±180° sarmalıyor | **-0.3…+41.9°, +1.2°'de oturuyor** |
| Roll/pitch hız RMS | 0.054 / 0.070 | **0.0008 / 0.0021** (65× / 33×) |
| İrtifa hata RMS | 0.456 m | **0.211 m** |
| \|vz\| max | 8.25 m/s | **0.17 m/s** |

Roll/pitch'in de 30-65× iyileşmesi, dönüşün o eksenlere sızdığını
doğruluyor — yaw'ı düzeltmek diğer iki ekseni de rahatlattı.

**Yaw takip testi (yeni):** hover'da `yaw_sp = +30°` komut edildi →
4.8° → 11.4° → 20.1° → **30.2°** (8 s), ~34°'de küçük aşım, oturma.
Ardından `yaw_sp = -30°` → düzgün, tek yönlü geri dönüş (29.1 → 5.9°,
~1.7°/s). **Salınım yok.** Yani yaw ekseni artık yalnızca "dönmüyor"
değil, gerçekten **takip ediyor**.

**SONUÇ — `sitl-lockup-check` kriteri:**

| Kriter | Sonuç |
|---|---|
| Hiçbir aktüatör 0/45 N'a kalıcı kilitlenmiyor | ✅ **GEÇTİ** (uçuş fazında 0; bkz. aşağıdaki iniş notu) |
| `vz`, `ALT_VZ_MAX=2.0`'ı aşmıyor | ✅ **GEÇTİ** (\|vz\| ≤ 0.17 m/s) |
| Yaw ±30°'yi aşmıyor | ✅ **GEÇTİ** — yerleşmiş durumda \|yaw\| ≤ 1.6° |

**Üç kriter de ilk kez aynı koşuda geçti.**

**13a-doğrulama — İKİNCİ BAĞIMSIZ KOŞU, sonuç tekrar etti.** Simülasyon
sıfırdan başlatılıp aynı senaryo tekrarlandı. Yerleşmiş pencerede
(t_arm+20…38 s):

| Ölçüt | Koşu 1 | Koşu 2 (doğrulama) |
|---|---|---|
| Yaw açısı bandı | -0.31…+6.89° | **-0.13…+1.88°** |
| Yaw hızı RMS | 0.0156 rad/s | **0.0045 rad/s** |
| Roll/pitch hız RMS | 0.0009 / 0.0023 | 0.0008 / 0.0025 |
| İrtifa hata RMS | 0.249 m | **0.113 m** |
| \|vz\| max | 0.17 m/s | 0.10 m/s |

İkisi de üç kriteri geçiyor; ikinci koşu daha da temiz. **Sonuç artık tek
koşuya dayanmıyor.**

**13b — Yan gözlemler (yeni, açık maddelere eklendi):**

1. **Yatay sürüklenme 53 m → 235 m'ye çıktı** (25 s'de, ~9 m/s). Bu bir
   regresyon değil, doğrudan sonuç: araç artık dönmediği için trim
   tilt'inin ürettiği ileri kuvvet (`Fx = T·sin δ0 ≈ 18.5·0.16 ≈ 3 N`,
   5 kg'da 0.6 m/s²) dönüşle ortalamada sıfırlanmıyor. **Yapısal neden:
   tilt aralığı [0°, 90°] tek yönlü olduğu için hover'da yaw torku ile
   ileri kuvvet AYRILAMAZ** — negatif tilt mümkün olsaydı iki kanat
   rotorü zıt yönde eğilip yaw üretirken Fx'i sıfırlayabilirdi. Bu
   kontrolcüde yatay pozisyon döngüsü olmadığından sürüklenme
   sınırlanmıyor (bkz. README "Bilinen sınırlamalar"). GUI'li koşularda
   kamera kilidi (`sitl/gz_follow.sh`) bu yüzden zorunlu.
2. **İniş fazında 15/266 örnekte BIG_M sabitlemesi** (log satırı
   1549-1591 / 1784). Uçuş fazında sıfır. 6 m'lik alçalmayı tek adımda
   (`z_sp` -6.35 → -0.50) komut ettiğim için WLS alçalmayı itki azaltmak
   yerine **rotorleri ~68° eğerek** yapıyor (`G0` Fx etkinliği 0.933) ve
   bu sırada bir itki kanalı alt sınıra çakılıyor.
   **ÇÖZÜLDÜ — sebep komut profiliydi, kontrolcü değil:** doğrulama
   koşusunda iniş kademeli yapıldı (`z_sp` 4.5 → 3.0 → 1.5 → 0.6 m,
   6 s aralıklarla) ve tüm koşu boyunca (171 örnek) **sıfır BIG_M**
   ölçüldü. Yani tek adımlık büyük alçalma komutu vermeyin; iniş
   profili rampalanmalı. (Aynı ders tırmanış için geçerli değil —
   6 m'lik tek adımlık tırmanış sorunsuz.)
3. **Yaw slew hızı artık limitin değil, OTORİTENİN sınırında:** adım
   testinde ölçülen dönüş hızı 0.03-0.09 rad/s, yeni limit ise 0.5 —
   yani limit artık bağlayıcı değil. Sıradaki kazanç §4 (L)'de
   (`TILT_RATE_MAX` 2.0 → 3.0).

---

### Adım 14 — §4 (L) `TILT_RATE_MAX` 2.0 → 3.0: **DENENDİ, GERİ ALINDI** (2026-07-27)

**Gerekçe:** Adım 12g, tilt slew limitinin yaw torku artışını doğrudan
sınırladığını ölçmüştü (`ddelta` her örnekte ±0.005 rad'da doygun).
Diferansiyel kanat tilt'i yaw'ın tek gerçek aktüatörü olduğuna göre 2.0,
yeni düzeltilen ekseni gereksiz yere kısıtlıyor görünüyordu. Ayrıca
MATLAB (`p.tilt.rate_max`) ve `sf_wls_alloc.m` **zaten 3.0'daydı** —
Adım 9 yalnızca PX4'ü düşürmüştü — yani bu değişiklik senkronu geri
kuracaktı ve bugünkü MATLAB regresyonları zaten 3.0 ile koşmuştu (yeni
MATLAB testi gerekmedi).

**İlk sonuç olumluydu:** yaw ~21 s yerine **~6 s'de** oturdu (t=6s'de
yaw 0.2°). Hover kriterleri geçmeye devam ediyordu.

**Ama yaw ADIM yanıtı bozuldu — iki kez tekrarlandı:**

| `yaw_sp = +30°` | 1. deneme | 2. deneme |
|---|---|---|
| t=3.0 s | 20.8° | 22.3° |
| t=4.5 s | 39.8° | 65.7° |
| t=6.0 s | **167.8°** | **-157.2°** (tam tur) |
| tepe yaw hızı | 2.14 rad/s | 1.69 rad/s |

Tepe hız, dış döngünün hız setpoint limitinin (0.5 rad/s) **4 katı**.
Aynı koşuda `yaw_sp = -30°` ise **kusursuz**: -3.2 → -7.3 → … → -28.5°,
hız sabit -0.035 rad/s, hiç aşım yok. `TILT_RATE_MAX=2.0`'da aynı +30°
adımı 8 s'de, ~%13 aşımla izleniyordu.

**Geri alındı** (PX4 2.0f'e döndürüldü; MATLAB/Simulink 3.0'da bırakıldı,
yani Adım 9'un belgelenmiş kasıtlı ayrışması korundu). **Geri alma
doğrulandı:** aynı +30° adımı yeniden temiz — -0.6 → 1.0 → 3.2 → 5.9 →
9.5 → 14.0 → 19.3 → 26.4°, yaw hızı ≤0.072 rad/s, aşım yok; öncesinde
25 s hover'da yaw -1.0°.

**Anlamı — Adım 9'un gerekçesi ÇÜRÜMEDİ, ölçümle DOĞRULANDI.** Adım 11
sonrası "gerekçesi zayıfladı" denmişti (§4 (I)); Adım 14 bunun yanlış
olduğunu gösterdi. Yüksek varsayılan slew hızıyla WLS, gerçek tork-sınırlı
Gazebo servosunun (p_gain=100, `cmd_max=2` N·m, friction=1.0) tek tick'te
veremeyeceği tilt düzeltmelerine bel bağlıyor; gölge model gerçeklikten
ayrışıyor ve düzeltme geç ve fazla geliyor — Adım 8'in ölçtüğü
gölge/gerçek tilt sapmasının tam olarak beklenen sonucu.

---

### Adım 15-16 — Gözlem altyapısı, yapısal ileri sürüklenme ve **yaw'ın ileri hıza bağımlılığı** (2026-07-27)

Kullanıcı GUI'de aracı göremediğini bildirdi. Ölçüldü: `gz`'nin follow
modu **aktif ama yetişemiyor** — 30 s'de kamera araçtan ~170 m geride.

1. **`follow_pgain` eklendi** (`Tools/simulation/gz/worlds/default.sdf`,
   `CameraTracking` eklentisi): varsayılan 0.01 kamerayı hedefe çok yavaş
   yaklaştırıyor. 1.0'a çekilince geride kalma ~170 m'den **~62 m'de
   platoya** indi — iyileşme var ama tek başına yetmiyor.
2. **Asıl sebep kamera değil, aracın 10 m/s ile uçup gitmesi** ve bu
   **yapısal**: tüm tiltler `[0, π/2]` aralığında olduğu için toplam Fx
   hover'da **her zaman ≥ 0** — araç geri kuvvet üretemez. Yaw dengesi
   için gereken diferansiyel tilt kaçınılmaz olarak ~3 N ileri itki
   doğuruyor ve `WS_FX = 0.05` bunu neredeyse hiç cezalandırmıyor.
   Kuyruk rotoru tilt'i de yardımcı olmuyor (PY=0, yalnızca +Fx üretir).
   Bu, §4 (N)'in nicel doğrulaması.
3. **YENİ BULGU (Adım 15c/16) — yaw ADIM yanıtı ileri hıza bağlı ve
   GERÇEK HOVER'DA BOZULUYOR. Adım 13'ün yaw sonucu ciddi biçimde
   daraltılmalı.**

   İlk gözlem: uzun gözlem koşusunda (~2.5 m/s) `yaw_sp=+30°` adımı
   26.9 → **89.6** → 32.8 → 31.7° şeklinde ~60° aştı; oysa daha önceki
   koşularda aynı adım pürüzsüz izlenmişti. İlk hipotezim "hızlı uçarken
   yan kayma momenti bozuyor" idi — **bu YANLIŞ çıktı, ilişki tam
   tersiymiş.** Eski koşuların ulog'ları kontrol edildiğinde temiz adımın
   alındığı anda aracın **11.1 m/s**'de olduğu, aşımlı adımın ise
   **2.1 m/s**'de alındığı görüldü.

   **Kontrollü doğrulama (Adım 16, TEK uçuşta, tek değişken hız):**

   | `yaw_sp = +30°` | v = 11.6 m/s | v = 2.45 m/s |
   |---|---|---|
   | yanıt | 4.6 → 8.7 → 14.2 → 21.3 → 29.8 → 34.1 → **34.9°** | -26 → -10 → 38.7 → 24.2 → 33.7 → 43.1 → 11.7 → 55.1 → 56.7 → **14.9°** |
   | karakter | monoton rampa, ~%13 aşım, oturuyor | **sönümsüz salınım, ±25°, 15 s'de oturmuyor** |
   | yaw hızı | ≤0.1 rad/s | ±0.5 rad/s |

   Pitch trim değişkeni ayrıca elendi: aynı uçuşta `pitch_sp=0` ve
   `pitch_sp=0.061` ile yapılan A/B'de ikisi de aşıyor — fark pitch'ten
   değil, hızdan.

   **Mekanizma (tutarlı açıklama):** ileri hızda dikey kuyruk/gövde
   yüzeyleri (SDF'de 5 `lift-drag` eklentisi) **rüzgâr gülü (weathervane)
   sönümlemesi** sağlıyor — yani yaw eksenini asıl sönümleyen şey
   kontrolcü değil, aerodinamik. Hız sıfıra yaklaşınca bu sönümleme
   kayboluyor ve kendi otoritesi ~0.05 N·m/adım olan yaw ekseni
   sönümsüz kalıyor.

   **KRİTİK SONUÇ:** Bu kontrolcüde yatay pozisyon döngüsü olmadığı için
   **bugüne kadarki tüm "hover" doğrulamaları aslında ~10 m/s'lik seyir
   uçuşuydu** (araç yapısal ileri kuvvetle sürekli hızlanıyor, madde
   (N)). Gerçek anlamda yerinde duran bir hover — ki bir pozisyon
   kontrolcüsünün komut edeceği asıl durum budur — yaw için **en kötü
   koşul** ve orada kriter sağlanmıyor. Adım 13'ün kazanımı geçerli
   (sınırsız dönüş durdu, kilitlenme yok, ±30° hover kriteri geçildi)
   ama **düşük hızda yaw adım takibi hâlâ çözülmemiş.**
4. **Uzun koşuda aktüatör kilitlenmesi YOK:** 756 `WRdbg` örneği (iki
   +30° yaw adımı, pitch trim, kademeli iniş dahil) — **0 BIG_M**.
5. **Gözlem için pratik çözüm — pitch trim:** `pitch_sp = +0.061 rad`
   (≈3.5° = `atan(3/49)`) komut edilince itki vektörü geriye yatıp bu
   kuvveti dengeliyor. Ölçüm: sürüklenme **9.4 m/s → ~1.7 m/s** (5-6×)
   ve araç yavaşlamaya devam ediyor; pitch tam 3.5°'de, yaw 2.1°'de
   duruyor, irtifa korunuyor. Uzun gözlem koşularında bunu kullanın.

---

### Adım 17 — (Q) ve (P) saf MATLAB'da arandı: **(P) yeniden üretildi, (Q) ÜRETİLEMEDİ** (2026-07-28)

**Neden MATLAB?** Adım 16, yaw'ı sönümleyenin ileri hızdaki aerodinamik
rüzgâr gülü etkisi olduğunu öne sürmüştü. Bu doğruysa, **aerodinamik yaw
sönümlemesi hiç olmayan bir ortam** (Q)'nun en kötü koşulunu ücretsiz
sağlar. `tiltrotor_plant_deriv.m`'in aero modeli tam olarak böyle:

```
F_aero = Ry*[-D; 0; -L]          -> F_aero(2) = 0  (yanal kuvvet yok)
M_aero = cross(r_cp, F_aero),  r_cp = [-0.05; 0; 0.05]
      -> M_aero(3) = r_cp(1)*F_aero(2) - r_cp(2)*F_aero(1) = 0   HER ZAMAN
```

Yani **MATLAB plant'i hiçbir hızda yaw ekseni üzerinde aerodinamik moment
üretmez** — yaw açısından yapısal olarak "gerçek hover"dır. Adım 11/12'deki
"MATLAB yapısal olarak göremez" tuzağının TERSİ bir durum: burada MATLAB,
SITL'den daha kötümser bir koşulu temsil ediyor. Ayrıca bugüne kadar hiçbir
MATLAB testi yaw adımı komut etmemişti (`run_hover_gust_test.m`
`att_sp=[0;0;0]`) — yani yaw adım yanıtı referans tarafta hiç ölçülmemişti.

**Yeni test:** `run_yaw_step_test.m` — hover trim, 50 m, bozucu yok, t=4 s'de
`yaw_sp = ±30°` adımı, 25 s, LESO roll+pitch (yaw'da asla, bkz. çıkarım 19).
Loglar: ψ, r, `omega_sp`, δ0/δ1, WLS'in talep ettiği vs tahsisatın ürettiği
Δτ_z, yatay hız. `YAW_TEST_TILT_RATE_MAX` ortam değişkeniyle tilt slew limiti
geçersiz kılınabilir (MATLAB 3.0 kullanıyor, PX4 portu 2.0 — Adım 9'da
düşürüldü ve MATLAB'a **kasıtlı olarak taşınmadı**).

| `p.tilt.rate_max` | adım | aşım% | ts(±2°) | e_ss | max\|r\| | RMS r (son 5 s) | salınım? |
|---|---|---|---|---|---|---|---|
| **3.0** (MATLAB) | +30° | 24.0 | 3.68 s | 0.99° | 0.460 | 0.0000 | hayır |
| **3.0** | −30° | 7.7 | 3.08 s | 0.99° | 0.393 | 0.0000 | hayır |
| **2.0** (PX4) | +30° | **59.2** | banda girmedi | 2.50° | 0.484 | 0.0001 | hayır |
| **2.0** | −30° | 8.0 | banda girmedi | 2.50° | 0.382 | 0.0001 | hayır |

*(2.0 satırlarında "banda girmedi", kalıcı bir **2.5° ofset** olduğu içindir —
salınım değil; yaw hızı RMS'i 0.0001 rad/s. İkisini ayırt etmek için testin
metrik çıktısına ayrı bir `salınım?` sütunu kondu.)*

**Bulgu 1 — (P) YENİDEN ÜRETİLDİ ve mekanizması görsel olarak kanıtlandı.**
Yön asimetrisi MATLAB'da da var ve PX4'ün slew limitinde **keskinleşiyor**:
+30° aşımı 24.0% → 59.2% çıkarken −30° 7.7% → 8.0%'de kalıyor (7.4× asimetri).
Sebep `yaw_step_test*.png` 3. panelinde doğrudan görülüyor: **trim'de
δ1 ≡ 0**, yani tek yönlü tilt aralığının (`p.tilt.min = 0`) tam tabanında
oturuyor; δ0 ise ~8-9°. Geometriden
`τ_z = -0.25·T0·sin δ0 + 0.25·T1·sin δ1` olduğundan:
- **−yaw** için δ0'ı **artırmak** yeterli → kısıt yok, serbest.
- **+yaw** için δ1'i 0 tabanından **kaldırmak** ya da δ0'ı azaltmak gerekiyor →
  grafikte δ1 iki kez ~2°'ye çıkıp tekrar 0'a çakılıyor, yani sürekli sınıra
  vuruyor.

Bu, §4 (P)'nin hipotez olarak yazdığı sebebi **ölçümle doğruluyor** ve (P)'yi
MATLAB'da (yani ucuz, güvenli, `safe-control-change` uyumlu bir ortamda)
üzerinde çalışılabilir hale getiriyor.

**Bulgu 2 — (Q) YENİDEN ÜRETİLEMEDİ. Adım 16'nın mekanizma açıklaması
yetersiz.** Dört koşunun hiçbirinde kalıcı salınım yok: son 5 s yaw hızı RMS'i
**≤ 0.0001 rad/s**, SITL'in (Q) koşusundaki **±0.5 rad/s**'ye karşı — dört
mertebe fark. Aerodinamik yaw sönümlemesi **sıfır** olduğu halde kontrolcü
±30° adımını 3.1-3.7 s'de, salınımsız oturtuyor. Dolayısıyla:

> **"Düşük hızda rüzgâr gülü sönümlemesinin kaybolması" tek başına (Q)'yu
> AÇIKLAMIYOR.** Adım 16'nın *ölçümü* (hıza bağlılık, tek uçuşta kontrollü
> A/B) geçerli; ama önerdiği *nedensellik* eksik — kontrolcü, hiç aero
> sönümleme olmadan da bu adımı sönümleyebiliyor. SITL'de fazladan,
> **SITL'e özgü bir kararsızlaştırıcı mekanizma** olmalı; ileri hızdaki
> aero sönümleme onu yalnızca **maskeliyor**.

**Bulgu 3 — MATLAB ile SITL `TILT_RATE_MAX` konusunda İŞARET olarak
çelişiyor; bu çelişki teşhis edici.**
- MATLAB (bu adım): 3.0, 2.0'dan **açıkça daha iyi** (+30° aşımı 59.2 → 24.0,
  e_ss 2.50 → 0.99°).
- SITL (Adım 14): 3.0, +30° adımında aracı **tam tura soktu** (tepe hız
  2.14 rad/s), iki kez tekrarlandı, geri alındı.

Aynı sabit, iki ortamda ters yönde davranıyor. MATLAB'da tilt servosu komut
edilen slew limitine **birebir uyduğu** için limiti yükseltmek her zaman
yardım eder; SITL'de etmiyor. Fark, MATLAB'ın sahip **olmadığı** şeyde.

**Kalan SITL-özgü aday (en güçlü): `_u_actual` gölge modeli.** Adım 8'in
ölçtüğü olgu kodda tekrar doğrulandı — `MulticopterIndiTiltrotor.cpp:414-421`
gölge aktüatör durumunu **tamamen açık çevrim** entegre ediyor, Gazebo'dan
hiçbir geri besleme okumadan:

```cpp
float ddelta = (u_cmd(3 + i) - _u_actual(3 + i)) / TILT_TAU;   // + slew clamp
_u_actual(3 + i) += dt * ddelta;
```

MATLAB'da ise kontrolcüye **gerçek** aktüatör durumu veriliyor
(`u_actual = x(14:19)`, doğrudan plant durumu). INDI'nin tüm lineerleştirme
noktası `u_actual`'dır ve WLS'in G matrisi ondan türetilir; gölge model
gerçeklikten ayrıştığında hem G hem de `u_cmd = u_actual + du` toplamı yanlış
olur. Gerçek Gazebo servosu basit bir gecikme değil, **tork-sınırlı
(`cmd_max=2`) P=100/D=0 PID + `friction=1.0`** (Adım 8; δ2'de 2.6-3.7°'lik,
işaret değiştiren sapma ölçülmüştü). Bu, Adım 11 (itki eşlemesi) ve Adım 12
(km işareti) ile **aynı sınıftan** bir kontrolcü/plant arayüz uyuşmazlığıdır
— ve saf MATLAB üçünü de yapısal olarak göremez.

**Eleme için diğer MATLAB↔SITL farkları** (bir sonraki adımda sırayla
dışlanmalı): PX4'ün gerçek-zamanlı zamanlama jitter'ı (MATLAB sabit adım);
Gazebo'nun 5 `lift-drag` yüzeyinin v→0'da bile sıfır olmayan katkısı;
`thrustToNormalized()` terslemesinin `ROTOR_KF`/`WMAX` sabitlerine tam
bağımlılığı.

**Bu adımda değişen kontrol sabiti YOK** — yalnızca yeni bir MATLAB test
sürücüsü eklendi; `YAW_TEST_TILT_RATE_MAX` opsiyoneldir ve varsayılan davranışı
değiştirmez. `safe-control-change` gerektiren bir işlem yapılmadı.

---

### Adım 18 — §4 (Q) 1. önceliği yapıldı: gölge model sapması ÖLÇÜLDÜ. **Baskın neden DEĞİL** — ama (Q) yeniden üretildi ve (P) ile aynı köke bağlandı (2026-07-28)

Adım 17 gölge aktüatör modelini "en güçlü aday" ilan etmişti. Ölçüldü.
**Sonuç bu şüpheyi büyük ölçüde eliyor** — ama koşu, aranan şeyden daha
değerli bir şey gösterdi.

**Kurulan ölçüm altyapısı (hiçbiri rebuild gerektirmez, hiçbiri fiziği/
kontrolü etkilemez):**
1. `rootfs/etc/logging/logger_topics.txt` — PX4 logger'a `tiltrotor_indi_status`
   (gölge `u_actual`) dahil 10 topic'i tam hızda logatır. **Tuzak: bu dosya
   varsa logger VARSAYILAN profili tamamen değiştirir** (`logged_topics.cpp:560`
   bir `if/else`), o yüzden ölçümden sonra kaldırıldı; yeniden kullanılabilir
   kopya `sitl/logger_topics_shadow.txt`.
2. `model.sdf`'e `JointStatePublisher` eklentisi (yalnızca `motor_{0,1,2}_joint`)
   → `/world/default/model/tiltrotor_indi_0/joint_state`, 250 Hz.
3. `sitl/gz_joint_csv.sh` — o topic'i gerçek zamanlı CSV'ye indirger.
4. `sitl/shadow_vs_real.py` — ulog + CSV'yi hizalayıp istatistik ve grafik üretir.

**Zaman hizalaması doğrulandı:** ulog'daki yaw adımı t=125.932 s, gz sim
işareti 125.892 s → **ulog hrt saati = gz sim saati** (lockstep), fark 40 ms.

**Uçuş:** arm → 6 m tırmanış, **arm'dan itibaren pitch trim** (`pitch_sp=0.061`,
Adım 15) → 25 s frenleme → `yaw_sp=+30°` → 22 s gözlem → kademeli iniş → disarm.
Yatay hız **1.5-2.3 m/s** — yani Adım 16'nın salınan koşusuyla (2.45 m/s) aynı
rejim, (Q)'nun koşulu. Setpoint'in kontrolcüye ulaştığı doğrulandı.

**ÖNCE BİR ÖLÇÜM TUZAĞI (bu projede üçüncü kez):** `tiltrotor_indi_status`
ulog'unda örneklerin **%1.5'inde zaman damgası yineleniyor** (`dt = 0`).
Elenmezse interpolasyon **10-12°'lik sahte sapma sivrileri** üretiyor — ilk
analizim "δ0'da 10.3° sapma" diyordu, tamamen artefakt. `shadow_vs_real.py`
artık bunları eliyor ve `max` yanında **p99** basıyor. **p99'a bakın.**

#### Sonuç 1 — gölge/gerçek sapması GERÇEK ama KÜÇÜK (derece)

| pencere | | ort. sapma | RMS | p99 | gölge ort | gerçek ort |
|---|---|---|---|---|---|---|
| adım öncesi (salınım sürerken) | δ0 | −0.017° | 0.529° | 0.335° | 10.527° | 10.544° |
| | δ1 | −0.310° | 0.436° | 0.547° | 0.582° | 0.892° |
| | δ2 | −0.530° | 0.530° | 0.531° | 0.000° | 0.530° |
| adım sonrası (oturmuş) | δ1 | −0.521° | 0.521° | 0.521° | 0.000° | 0.521° |

Sapma **p99'da ≤ 0.55°**. Aynı pencerede yaw ekseni **73.7°'lik** bir bantta
salınıyor. **İki mertebe fark var: gölge model sapması (Q)'nun baskın nedeni
olamaz.** Adım 17'nin "en güçlü aday" değerlendirmesi bu ölçümle düzeltilmiştir.
(Adım 8'in δ2'de ölçtüğü 2.6-3.7°'lik sapma da burada 0.53°'ye inmiş —
muhtemelen o ölçüm Adım 11/12 öncesi yanlış G matrisi ve itki eşlemesi
altındaydı.)

#### Sonuç 2 — ama sınırda YAPISAL bir tahsisat hatası doğrulandı

Gölge δ1 örneklerin **%75'inde tam `0.000°`**, gölge δ2 **%97'sinde**; oysa
aynı anlarda gerçek eklemler **0.53°**'de duruyor. Sebep: tork-sınırlı P
kontrolcüsünün (`p_gain=100`, `cmd_max=2`, `err_max=0.2`, I=0) kalıcı bir
konum ofseti var — 1. derece gölge model bunu **yapısal olarak** üretemez,
komutu tam takip edip 0'a oturuyor. Etkisi doğrudan WLS'te:

```cpp
const float abs_lo = TILT_MIN - _u_actual(3 + i);   // = 0 - 0 = 0
```

yani allocator "δ1 ve δ2 **aşağı hiç inemez**" sanıyor, gerçekte **0.53°**
hareket alanı var. Bu, tam da yaw'ın tek gerçek aktüatörü olan eksende
allocator'dan gizlenen otorite. Küçük ama gerçek ve **sistematik**.

#### Sonuç 3 — (Q) YENİDEN ÜRETİLDİ, ve yeniden çerçevelenmeli

| pencere | yaw bandı | periyot | r RMS | max\|r\| | yatay hız |
|---|---|---|---|---|---|
| **adım öncesi 20 s (`yaw_sp = 0`)** | **−23.6…+50.1° (73.7°)** | **5.18 s** | **0.4174 rad/s** | 0.726 | 1.62 m/s |
| adım + 10…22 s (`yaw_sp = +30°`) | 31.9…32.8° (**1.0°**) | — | **0.0077 rad/s** | 0.021 | 2.34 m/s |

Adım 16'nın ±25°/±0.5 rad/s ölçümüyle uyumlu — hatta daha şiddetli.
**Kritik yeniden çerçeveleme: salınım `yaw_sp = 0` iken oluyor**, yani bu bir
*adım yanıtı* kusuru değil, **denge noktası kararsızlığı**. "Düşük hızda yaw
adım takibi kötü" değil, "**düşük hızda yaw ekseninin kararlı bir dengesi
yok**" demeliyiz. Adım 16'nın adım-tabanlı A/B'si olguyu doğru yakalamış ama
dar tarif etmiş.

#### Sonuç 4 — salınımın en güçlü korelatı: δ1'in `TILT_MIN = 0` sınırında zıplaması

`sitl/shadow_vs_real_tilt.png` 3. ve 4. panel: salınım boyunca δ1 sınırdan
kalkıp geri çakılıyor — **12 "kalkış" olayı**, hepsi salınım penceresinde;
salınım durduktan sonra **sıfır**. Eşzamanlı olarak δ0, ~5 s periyotlu (yaw
periyoduyla aynı) 8°↔13° testere dişi yapıyor. δ1 sınırda kalır kalmaz salınım
duruyor.

Bu tam olarak **madde (P)'nin tek yönlü tilt aralığı** mekanizması. Yani
**(P) ve (Q) muhtemelen aynı kök nedenin iki görünümü**: `p.tilt.min = 0`
nedeniyle δ1 trim'de sınırda oturuyor; +yaw düzeltmesi onu sınırdan kaldırmayı
gerektiriyor, −yaw düzeltmesi ise sınırın altına itmeyi — ki mümkün değil.
Asimetrik doyum → limit cycle.

#### Sonuç 5 — adım sonrası neden oturdu? ÇÖZÜLMEDİ

`yaw_sp=+30°` sonrası salınım tamamen durdu (r RMS 54× daha iyi). İki
açıklama da veriyle uyumlu, bu koşu ayırt etmiyor:
- **(a) yan kayma kaynaklı aero sönümleme:** araç hâlâ eski yönünde
  sürükleniyorken burnu 32° çevrildiği için yan kayma doğuyor; 5 `lift-drag`
  yüzeyi bu durumda gerçek bir yaw sönümlemesi üretir (Adım 16'nın mekanizması,
  burada *hıza* değil *yan kaymaya* bağlı olarak).
- **(b) sınır dinamiği:** yeni dengede δ1 sürekli sınırda kaldığı için
  zıplama çevrimi kesiliyor.

**Ayırt edici deney:** aynı uçuşta `yaw_sp`'yi 0'a geri döndürmek. (a) doğruysa
yan kayma kaybolunca salınım geri gelir; (b) doğruysa δ1 sınırda kaldığı sürece
gelmez.

#### Sonuç 6 — adım aşımı merdiveni

| ortam | +30° adım aşımı |
|---|---|
| MATLAB, `rate_max=3.0` | 24.0 % |
| MATLAB, `rate_max=2.0` (PX4 değeri) | 59.2 % |
| **SITL, 2.3 m/s** | **95.1 %** (tepe 58.5°) |
| SITL, 11.6 m/s (Adım 16) | ~13 % |

Slew limiti 24→59'u, SITL'e özgü etkiler 59→95'i, ileri hız 95→13'ü açıklıyor.

**Bu adımda kontrol sabiti DEĞİŞTİRİLMEDİ.**

---

### Adım 19 — Trim ön-yükleme fikri ÇÜRÜTÜLDÜ; ve tek değişkenli A/B: **(Q)'yu bitiren şey İLERİ HIZ DEĞİL** (2026-07-28)

#### 19a — Önerilen düzeltmenin premisi MATLAB'da çürütüldü (SITL'e hiç gitmeden)

Adım 18'in önerdiği "tilt trim'ini ön-yükleyip δ1'i sınırdan uzaklaştır"
fikri, uygulanmadan önce premisi test edildi: trim aynı diferansiyeli
koruyarak farklı ortalamalarla tohumlandı ve kapalı çevrimin nereye
yakınsadığına bakıldı.

| tohum bias | başlangıç δ0/δ1 | **son 2 s ort. δ0/δ1** |
|---|---|---|
| 0° | 9.31° / 0.00° | 9.00° / **0.00°** |
| 5° | 9.65° / 0.35° | 9.00° / **0.00°** |
| 10° | 14.65° / 5.35° | 9.00° / **0.00°** |

**Denge tohumdan tamamen bağımsız.** Sebep: WLS'in amaç fonksiyonunda
`du_pref = 0`, yani **mutlak** bir tilt tercihi yok; `Fx → 0` hedefi
(`Ws_Fx = 0.05`) ortalama tilt'i δ1 tabana değene kadar aşağı itiyor.
Trim yalnızca başlangıç koşulu olduğu için ön-yükleme **yıkanıyor**.
Kalıcı olması için amaç fonksiyonunun (`du_pref`) ya da kısıtın
değiştirilmesi gerekirdi. **Fikir uygulanmadı** — `safe-control-change`
adım 1'i bile gerekmedi, çünkü hiçbir sabit değişmedi.

#### 19b — Tek değişkenli A/B: yaw_sp = 0 sabit, yalnızca ileri hız değişiyor

Adım 18'de bir konfüzyon vardı: salınım 1.62 m/s'de sürüyor, oturma
2.34 m/s'de oluyordu — ama **aynı anda `yaw_sp` de 0'dan +30°'ye
değişmişti**. Hangisinin sebep olduğu ayrışmıyordu. Bu koşuda `yaw_sp`
**tüm uçuş boyunca 0'da sabitlendi**, tek değişken ileri hız yapıldı:
FAZ1 pitch trim ON (~2 m/s), FAZ2 pitch trim OFF (araç ~6.6 m/s'ye
hızlanıyor).

| t−arm (s) | yatay hız | yaw hızı RMS | yaw bandı | δ1 sınırda |
|---|---|---|---|---|
| 2 | 0.38 | 0.404 | 79.3° | 48% |
| 12 | 1.31 | 0.455 | 66.8° | 28% |
| 22 | **2.01** | **0.439** | 51.3° | 38% |
| 27 | **2.09** | **0.207** | 23.9° | 62% |
| 32 | **2.06** | **0.055** | 6.7° | 100% |
| 37 | **2.00** | **0.006** | 1.1° | 100% |
| 42 (FAZ2) | 3.66 | 0.009 | 1.9° | 100% |
| 62 | 6.27 | 0.003 | 0.2° | 100% |

**Salınım t = 22…37 s arasında sönüyor ve bu aralıkta yatay hız
2.00-2.09 m/s'de SABİT.** Hızlanma daha sonra (t = 40 s'de) başlıyor ve
o noktada yaw zaten sessiz.

> **SONUÇ: İleri hız, düşük hızdaki yaw salınımını bitiren şey DEĞİL.**
> Adım 16'nın "yaw'ı sönümleyen şey aerodinamik rüzgâr gülü etkisi"
> mekanizması bu tek değişkenli A/B ile **doğrudan çürütülmüştür.** Bu,
> Adım 17'nin bağımsız kanıtıyla (MATLAB'da `M_aero(3) ≡ 0` olduğu hâlde
> adım oturuyor) aynı yönde — artık iki bağımsız kanıt var.

#### 19c — (Q) yeniden çerçeveleniyor (Adım 18'in ifadesi de düzeltiliyor)

Grafik (`yaw_airspeed_ab.png`) salınımın **düzenli biçimde söndüğünü**
gösteriyor: genlik ~90° tepe-tepeden sıfıra iniyor. Yani bu:
- **sürekli bir limit cycle DEĞİL** (Adım 16'nın "sönümsüz" ifadesi),
- **denge kararsızlığı da DEĞİL** (Adım 18'de benim yazdığım ifade —
  **düzeltiliyor**),
- **çok zayıf sönümlü, yerleşme süresi ~30-35 s olan bir yaw modu.**

İki bağımsız uçuşta tekrarlandı: Adım 18'de arm'dan ~33 s sonra, Adım
19'da ~35 s sonra oturuyor. Adım 16'nın "15 s'de oturmuyor" gözlemi
doğru ama **yeterince uzun gözlenmemiş**.

**Neden hâlâ kabul edilemez:** gerçek bir uçuşta yatay pozisyon
kontrolcüsü sürekli küçük yaw düzeltmeleri komut eder; yerleşme süresi
30 s olan bir eksen bu uyarımla pratikte hiç oturmaz. Ayrıca Adım 18'de
+30° adımı aynı modu yeniden uyardı (~8 s'de söndü).

**Açık kalan asıl nicel boşluk:** MATLAB aynı ±30° adımını **3.1-3.7 s**'de
oturtuyor (Adım 17), SITL ise adım için ~8 s, arm geçicisi için ~33 s
alıyor. Bu **2-10×'lik sönümleme farkı** hâlâ SITL'e özgü ve açıklanmadı.
Gölge model sapması (Adım 18: p99 ≤ 0.55°) bu farkı açıklayacak
büyüklükte değil.

**Bu adımda kontrol sabiti DEĞİŞTİRİLMEDİ.**

---

### Adım 20 — Ablasyon (tümü negatif), ve **Adım 19'un çıkarımı ÇÜRÜDÜ**: salınım uçuştan uçuşa değişken (2026-07-28)

#### 20a — MATLAB ablasyonu: dört SITL kusuru da sönümlemeyi bozmuyor

Önce SITL'in `omega_dot`'u ölçüldü (Adım 19'un ulog'undan, `xyz_derivative`'i
`xyz`'nin **merkezi fark** türeviyle karşılaştırarak): **gecikme 4-8 ms**
(MATLAB'ın α=0.3 filtresi ~7.0 ms), yaw HF gürültü RMS **0.019 rad/s²**
(sinyalin %4'ü). `IMU_GYRO_CUTOFF=40`, `IMU_DGYRO_CUTOFF=30` Hz.

Sonra `run_yaw_ablation.m` (yeni) ile SITL'in MATLAB'da **bulunmayan**
kusurları tek tek enjekte edildi:

| ablasyon | aşım% | ts(±2°) | son 5 s r RMS | salınım çevrimi |
|---|---|---|---|---|
| baz (MATLAB gibi) | 24.1 | 3.69 s | 0.0000 | 0 |
| **gölge aktüatör modeli** (PX4'ün açık çevrim kopyası) | 23.6 | **3.69 s** | 0.0000 | 0 |
| `omega_dot` +8 ms gecikme | 23.3 | 3.70 s | 0.0001 | 0 |
| `omega_dot` gürültü 0.02 rad/s² | 24.6 | 3.65 s | 0.0008 | 0 |
| `dt` jitter %25 | 24.1 | 3.69 s | 0.0001 | 0 |
| **hepsi birlikte** | 23.0 | **3.64 s** | 0.0011 | 0 |

**Dördü de tamamen etkisiz.** Gölge aktüatör modeli baz ile birebir aynı
(3.69 s) — Adım 18'in "sapma çok küçük" ölçümüyle tutarlı ve onu kesin
olarak eliyor. `omega_dot` hipotezi (Adım 19'un önerdiği sıradaki aday) da
**elendi**. Aday listesi eksik.

#### 20b — Temiz adım yanıtı denemesi: salınım bu koşuda HİÇ SÖNMEDİ

Adım 18'in adım yanıtı, yaw zaten salınırken verildiği için kontamineydi.
Bu koşuda arm geçicisinin tam sönmesi için **55 s** beklenip sonra adım
verilmesi planlandı (Adım 19'da ~35 s'de sönüyordu). Ama:

| t−arm | yatay hız | yaw hızı RMS | yaw bandı |
|---|---|---|---|
| 2-7 | 0.35 | 0.398 | 79.1° |
| 22-27 | 1.67 | 0.406 | 53.5° |
| 47-52 | 1.47 | 0.356 | 50.3° |
| 72-77 | 1.87 | 0.372 | 44.5° |
| 107-112 | 2.00 | 0.540 | 82.1° |

**Salınım 112 saniye boyunca hiç sönmedi**, genlik sabit kaldı.
`yaw_sp = +30°` ve geri `0` adımları salınımın üzerine bindi; +30° adımı
25 s'de ±2° bandına hiç girmedi (kalan hata −10.1°).

#### 20c — DÜZELTME: Adım 19'un "hız sebep değil" çıkarımı geçersiz

Aynı konfigürasyonla iki uçuş, **zıt sonuç** (`yaw_flight_variability.png`):

| | Adım 19 uçuşu | Adım 20 uçuşu |
|---|---|---|
| yatay hız platosu | **2.00-2.10 m/s** | **1.46-1.87 m/s** |
| sonuç | ~35 s'de **söndü** | 112 s **sönmedi** |

Adım 19'da salınımın söndüğü pencerede hız *sabitti* (2.01→2.09) — bu
gözlem doğru. Ama ondan **"hız sebep değil" sonucunu çıkarmak yanlıştı**:
sistem kararlılık sınırına çok yakınsa, sınırı yeni geçmiş bir sistem hız
sabit kalsa da yavaşça söner. İki uçuşun karşılaştırması, kararlılık
sınırının **~2 m/s civarında** olduğunu ve Adım 19'un tam o noktada
ölçüldüğünü gösteriyor. **Çıkarım 34 bu ölçümle geri alınmıştır.**

Doğru ifade: **düşük hızda (<~2 m/s) yaw salınımı sürekli; ~2 m/s
civarında marjinal; üstünde kararlı.** Ve Adım 19'un "sönümsüz değil, ~30-35
s'de oturur" ifadesi de (çıkarım 35) **tek bir uçuşa dayanan aşırı genelleme**
idi — geri alınmıştır.

#### 20d — Neden yüksek hız stabilize ediyor: SDF'den nicel türetme

Modelde **tek yanal aero yüzeyi** var (`model.sdf`, 5 `lift-drag`
eklentisinden biri): dikey kuyruk, `cp = (-0.74, 0, 0.12)` (yani CG'nin
**0.74 m arkasında** → yaw'da kararlılaştırıcı), **alan yalnızca 0.032 m²**,
`cla = 4.753`, **`alpha_stall = 0.339 rad = 19.4°`**, `cla_stall = -3.85`
(stall sonrası eğim **ters işaretli**). Yaw sönümleme türevi
`q·A·cla·kol = 0.5·1.2041·v²·0.032·4.753·0.74`:

| ileri hız | dτ_z/dβ |
|---|---|
| 2 m/s | **0.27 Nm/rad** |
| 11.6 m/s | **9.1 Nm/rad** |

**34× fark** — hızın neden bu kadar belirleyici olduğunu açıklıyor. Ayrıca
salınım genliği ±25-40° iken **yan kayma stall açısını (19.4°) aşıyor**,
yani kuyruk çevrimin büyük bölümünde stall'da ve sönümlemesi çöküyor
(hatta `cla_stall < 0` ile ters dönüyor). Bu, salınımın neden
**genliğe bağlı** olduğunu ve bir kez büyüdükten sonra kendini
sürdürebildiğini açıklayan tutarlı bir mekanizma.

> **DİKKAT — bu bölüm SDF'den TÜRETİLMİŞTİR, ÖLÇÜLMEMİŞTİR.** Doğrulamak
> için yan kayma açısı ile kuyruğun ürettiği yaw momenti uçuşta
> loglanmalıdır.

#### 20e — Geriye kalan asıl soru değişmedi

Aero sönümleme yüksek hızda stabilize ediyor olabilir, **ama düşük hızda
sistemin neden kararsız olduğunu açıklamıyor**: MATLAB'ın plant'inde
**hiç** aerodinamik yaw momenti yok (`M_aero(3) ≡ 0`, Adım 17) ve orada
sistem rahatça kararlı (ζ≈0.4, 3.69 s'de oturuyor). Yani **SITL'de,
MATLAB'da bulunmayan bir kararsızlaştırıcı var ve aero yalnızca onu
maskeliyor.** Adım 20a dört adayı eledi.

**Ölçülmemiş kalan boşluk (Adım 18'in eksiği):** gölge model karşılaştırması
yalnızca **tilt** kanallarında yapıldı; **itki kanalları (T0-T2) hiç
ölçülmedi.** Yaw torkunun bir bileşeni rotor reaksiyon torkudur
(`km·T`), dolayısıyla gerçek/gölge itki farkı doğrudan yaw'a girer. Gazebo'nun
gerçek itkisi rotor eklem hızından hesaplanabilir (`T = 2e-5·ω²`) — bunun için
`JointStatePublisher`'a `rotor_{0,1,2}_joint` eklemek yeterli, rebuild
gerekmez. **Sıradaki iş bu.**

**Bu adımda kontrol sabiti DEĞİŞTİRİLMEDİ.**

---

### Adım 21 — İtki kanalı da elendi; ve **PX4'te gerçek bir kod hatası bulundu: WLS slew kutusu yanlış dt ile boyutlanıyor** (2026-07-28)

#### 21a — Gerçek rotor itkisi ölçüldü (Adım 18'in eksiği kapandı)

`model.sdf`'teki `JointStatePublisher`'a `rotor_{0,1,2}_joint` eklendi;
gerçek itki eklem hızından türetildi:
`T = motorConstant·(w_joint·rotorVelocitySlowdownSim)² = 2e-5·(w·20)²`.
**Doğrulama geçti:** hover'da toplam gerçek itki **49.62 N**, ağırlık 49.05 N.

Gazebo motor parametreleri PX4'ün gölge modeliyle birebir aynı
(`timeConstantUp=0.0125`, `Down=0.025`, `motorConstant=2e-5`,
`maxRotVelocity=1500`). **Yapısal bir şüphe vardı:** gz'nin zaman sabiti
filtresi **rotor hızına (ω)**, PX4'ün gölge modeli ise **itkiye (T)**
uygulanıyor; `T = kf·ω²` olduğundan büyük geçişlerde ayrışmaları gerekir.
Ölçüm:

| pencere | | ort. sapma | RMS | p99 | gölge | gerçek |
|---|---|---|---|---|---|---|
| arm geçicisi | T0 | 0.052 N | 0.851 N | 1.935 N | 19.43 | 19.38 |
| **salınım rejimi** | T0 | **0.000 N** | **0.016 N** | **0.026 N** | 18.080 | 18.080 |
| | T1 | 0.000 N | 0.012 N | 0.024 N | 18.584 | 18.583 |
| | T2 | 0.001 N | 0.015 N | 0.042 N | 12.980 | 12.978 |

Yaw reaksiyon torkuna (`km·T`) etkisi: hata ort. **+0.0001 Nm**, RMS
**0.0020 Nm** — yaw otoritesinin (~0.45 Nm) **%0.4'ü**. **İtki kanalı da
eleniyor.** Şüphe pratikte önemsiz çıktı: salınımdaki itki değişimi küçük
olduğu için `T = kf·ω²` lineerleşiyor ve iki filtre örtüşüyor. (Arm
geçicisinde sapma büyüyor — RMS 0.85 N — çünkü orada geçiş büyük.)

#### 21b — **KOD HATASI: WLS slew kutusu sabit `TS_CTRL` ile boyutlanıyor**

Tahsisatın yaw verimi ölçülürken bulundu. Ölçüm (salınım rejimi, 12124 örnek):

| | değer |
|---|---|
| yaw torku talebi \|nu_des(3)\| ort. | 0.1604 Nm (p99 0.4419) |
| tahsisatın ürettiği \|(G·du)(3)\| ort. | 0.0331 Nm (p99 0.0567) |
| **verim** | **%20.6** (işaret uyumu %93.3) |
| **tilt kanalları `sat_flag`** | **δ0 %99.6, δ1 %99.4, δ2 %99.9** |
| \|ddelta\| p99 (δ0, δ1) | **tam 0.00500 rad** |
| ölçülen kontrol döngüsü periyodu | **4.00 ms (250 Hz)** |

0.00500 = `TILT_RATE_MAX · (1/400)`. Ama tick 4.00 ms sürüyor. Kaynakta
doğrulandı — `MulticopterIndiTiltrotor.cpp`:

```cpp
171:  const float dt = math::constrain((now - _last_run) * 1e-6f, ...);  // GERCEK dt
247:  _alt_accum += dt;                    // dogru
274:  _leso_accum += dt;                   // dogru
415:  _u_actual(i) += dt * (...);          // dogru
324:  const float rate_lo = -TILT_RATE_MAX * TS_CTRL;   // <-- SABIT 1/400 !
325:  const float rate_hi =  TILT_RATE_MAX * TS_CTRL;   // <-- SABIT 1/400 !
315:  const float rate_lo = -ROTOR_TMAX / ROTOR_TAU_UP * TS_CTRL * 5.0f;  // ayni hata
```

Aynı fonksiyon `dt`'yi doğru hesaplayıp gölge model, LESO ve irtifa için
kullanıyor; **yalnızca WLS kutusunda sabit `TS_CTRL = 1/400` kullanıyor.**
Modül ise `vehicle_angular_velocity` callback'iyle **250 Hz'de** dönüyor.

**Etki:** tilt artışı tick başına 0.005 rad ile sınırlı ama tick 4 ms
sürüyor → **efektif tilt slew tavanı 1.25 rad/s**, hedeflenen 2.0'ın
**%62'si**. Kanat tilt'i yaw'ın *tek* gerçek aktüatörü olduğundan
(çıkarım 13) bu doğrudan yaw otoritesini kısıyor — ve tilt kanalları
zamanın %99.4'ünde bu kutuda. (İtki kutusu 45 N/tick olduğu için hiç
bağlamıyor; yalnızca tilt etkileniyor.)

**Adım 14'ü geriye dönük açıklıyor:** `TILT_RATE_MAX` için denenen
nominal değerlerin efektif karşılıkları

| nominal | kutu (rad/tick) | **efektif (4 ms tick)** | Adım 14 sonucu |
|---|---|---|---|
| 2.0 | 0.00500 | **1.25 rad/s** | çalışıyor (mevcut baseline) |
| 3.0 | 0.00750 | **1.875 rad/s** | +30° adımda ıraksadı, geri alındı |

**Kritik uyarı — naif düzeltme muhtemelen ZARARLI olur.** Kutuyu gerçek
`dt` ile boyutlamak, nominal 2.0'da efektif hızı 1.25 → **2.0 rad/s**'ye
çıkarır; bu, Adım 14'te ıraksamaya yol açan **1.875'ten daha yüksektir**.
Yani "hatayı düzelt" hamlesi tek başına Adım 14'ün fiyaskosunu tekrarlar.
Doğru yol: kutuyu `dt` ile boyutla **ve** sabiti, doğrulanmış efektif hızı
(1.25 rad/s) koruyacak şekilde yeniden ayarla — böylece sabit gerçekten
rad/s anlamına gelir ve döngü hızından bağımsız hale gelir.

**Ek tasarım notu:** `TILT_RATE_MAX` şu an **iki farklı işi** yapıyor —
gölge modelin fiziksel servo limiti (satır 420) ve tahsisat kutusu (324-325).
Bunlar kavramsal olarak farklı; düzeltmede ayrılmaları gerekir.

#### 21c — Ablasyon bu adayı test EDEMİYOR (null sonuç bilgilendirici değil)

Kutu/döngü uyumsuzluğu `run_yaw_ablation.m`'e eklendi (250 Hz döngü,
`p.Ts_ctrl = 1/400` ile boyutlanan kutu) — **yerleşme süresini
değiştirmedi (3.69 s)**. Ama bu **eleme sayılmaz**: MATLAB'da tilt slew
kutusu hiç bağlamıyor (δ0 salınım sırasında ~2.5°/s hareket ediyor, kutu
114°/s'ye izin veriyor), oysa SITL'de tilt kanalları %99.4 oranında
kutuda. **İki ortam farklı rejimlerde çalıştığı için ablasyon bu
mekanizmayı sınayamaz.** Tek geçerli sınama SITL'de, düzeltmeyi derleyip
koşmaktır.

**Bu adımda kontrol sabiti DEĞİŞTİRİLMEDİ.**

---

### Adım 22 — Slew kutusu ayrıştırıldı ve dürüst hale getirildi (davranış-nötr), SITL'de doğrulandı (2026-07-28)

**ÖNEMLİ DÜZELTME — Adım 21'in "kod hatası" ifadesi fazla sertti.**
`TS_CTRL`'in yanındaki yorum okununca bunun **kasıtlı bir tercih** olduğu
görüldü:

> *"Run()'s measured dt jitters with scheduling load (observed ~4 ms avg /
> up to 8 ms, vs. this 2.5 ms nominal) — feeding that jittered dt into the
> box instead of this constant starves a rotor's per-tick allowance and can
> latch it into WLS saturation."*

Yani sabit periyot kullanmak **jitter'a karşı bilinçli bir koruma**, gözden
kaçmış bir hata değil. Gözden kaçan şey **yan etkisi**: nominal periyodun
döngünün *gerçek* periyoduyla eşleşmesi gerekir, yoksa kutu aktüatörü
sessizce de-rate eder (ort. dt 4 ms iken 2.5 ms'lik pay → efektif tavan
%62). Bu ayrım rapora ve koda işlendi.

**Yapılan değişiklik** (`safe-control-change` uyarınca; MATLAB'a
DOKUNULMADI — orada hata yok, döngü gerçekten `p.Ts_ctrl` periyodunda koşar):

| | önce | sonra |
|---|---|---|
| tahsisat kutusu periyodu | `TS_CTRL = 1/400` | **`TS_BOX = 1/250`** (ölçülen döngü hızı) |
| tahsisat kutusu hızı | `TILT_RATE_MAX = 2.0` | **`TILT_SLEW_BOX_RATE = 1.25`** rad/s |
| tilt kutusu (rad/tick) | 0.005 | **0.0050000004** (1 ULP fark) |
| `TILT_RATE_MAX` rolü | hem fiziksel limit hem kutu | **yalnızca fiziksel servo limiti** (gölge model, satır 435) |

Sabit ikiye ayrıldı çünkü iki farklı şeydi: servonun **fiziksel olarak
yapabildiği** ile tahsisatın **tek tick'te isteyebileceği**. Ayrılmadan
ikisi de anlamlı biçimde ayarlanamıyordu.

**SITL doğrulaması (`sitl-lockup-check` kriter koşusu, `pitch_sp=0`,
6 m tırmanış, arm+3…arm+32 s):**

| kriter | sonuç |
|---|---|
| aktüatör kilitlenmesi | ✅ **GEÇTİ** — itki 8.26-19.10 N, `sat_flag` %0.0, 0/45 N'a yapışma %0.00 |
| dikey hız | ✅ **GEÇTİ** — \|vz\| max 0.784 m/s (limit 2.0), irtifa hata RMS 0.234 m |
| yaw | ❌ **KALDI** — bant −2.67…+37.09° (kriter ±30), r RMS 0.133 rad/s |
| roll/pitch | bant ±0.08°, p/q RMS 0.0010/0.0038 rad/s |

**Nötrlük doğrulandı:** değişiklik sonrası `|ddelta|` p99 hâlâ **tam
0.00500 rad**, itki `sat_flag` hâlâ %0.0. Yaw'ın kalması **bu
değişiklikten değil** — arm sonrası düşük hız penceresindeki bilinen (Q)
salınımı (araç koşu sonunda 8.09 m/s'ye çıkıyor, o noktada yaw zaten
sessiz).

**İki nokta düzeltildi (ilk yazdığım gerekçeler yanlıştı):**
1. "Bit-aynı" değil, **1 ULP farklı** (0.005 vs 0.0050000004 = 4.7e-10 rad
   ≈ 2.7e-8 derece). Pratikte aynı ama ifade düzeltildi.
2. **İtki kutusu nötr DEĞİL.** İlk yorumumda yalnızca alt sınıra bakıp
   "her iki durumda da bağlamıyor" demiştim; oysa üst sınır
   `ROTOR_TMAX/ROTOR_TAU_DOWN·TS·5` = **22.5 N/tick** (eski) ve hover'da
   `abs_hi = 45−18 = 27 N` olduğundan **canlı bir kısıttı** — aynı kök
   nedenle (kısa nominal periyot) küçük boyutlanmış. Yeni değerle 36 N.
   Ölçümde itki doyumu değişiklik öncesi ve sonrası **%0.0** olduğu için
   pratikte etkisiz: aktif olmayan bir kısıt gevşetildi.

**Bu değişiklik tek başına (Q)'yu çözmez ve çözmesi de beklenmiyordu** —
amacı sabiti dürüst kılmaktı. Kazanç şu: `TILT_SLEW_BOX_RATE` artık gerçek
rad/s cinsinden taranabilir. Bilinen iki nokta: **1.25 çalışıyor**,
**1.875** (eski nominal 3.0) **ıraksıyordu** — ilgi aralığı arası.

---

### Adım 23 — `TILT_SLEW_BOX_RATE` tarandı: **(Q)'nun mekanizması bulundu** — tahsisat tilt slew'undan aç bırakılıyormuş (2026-07-28)

#### 23a — Uçuş-içi tarama kancası

Sabit `constexpr` olduğu için her değer bir rebuild demekti (4 rebuild + 8
uçuş). Bunun yerine `slewbox <rad/s>` custom command'ı eklendi
(`MulticopterIndiTiltrotor.cpp`, `px4::atomic<int32_t>` millirad/s olarak —
`px4::atomic<float>` GCC'nin `__atomic_load_n`'i ile derlenmiyor; geri okuma
`x/1000.0f` varsayılanda tam: 1250/1000 = 1.25f).

**Neden uçuş-içi:** Adım 20'nin dersi — bu airframe düşük hızda **marjinal
kararlı**, aynı konfigürasyon bir uçuşta 35 s'de sönüp diğerinde 112 s
sönmedi. Ayrı uçuşları karşılaştırmak taramayı uçuştan-uçuşa varyansla
karıştırır. Değeri uçuş ortasında değiştirmek diğer her şeyi sabit tutar.
Kanca ayrıca **bağımsız doğrulanabilir**: kutu tam olarak `rate × TS_BOX`
olduğundan aktif değer log'dan `|du(3)| p99.5 / TS_BOX` ile geri okunur —
dört fazda da nominalle birebir eşleşti.

#### 23b — Tarama: her değerde AYNI uyarım (+30° yaw adımı), iki koşu, ters sıra

Arm geçicisinin tam sönmesi için 60 s beklendi, sonra her kutu değerinde
+30° adım (18 s) → 0° (18 s). Koşu A sırası 1.25→2.00, koşu B **ters**.
Ayırt edici metrik: adımın son 5 s'sindeki **yaw hızı RMS** (yerleşme
metriği araç sürüklendiği için gürültülü; hız RMS sağlam).

| kutu (rad/s) | koşu A | koşu B | test edilen hız (A/B) | sonuç |
|---|---|---|---|---|
| **1.25** (o günkü varsayılan) | **0.583** | **0.466** | 2.20 / 0.86 m/s | **SALINIYOR** |
| 1.50 | 0.391 | 0.005 | 2.02 / 2.00 m/s | marjinal |
| **1.75** | **0.0037** | **0.0051** | 1.35 / 2.76 m/s | **sakin** |
| **2.00** | **0.0056** | **0.0055** | 0.81 / 3.14 m/s | **sakin** |

**Hız kesin olarak eleniyor:** 1.25 hem 0.86 hem 2.20 m/s'de salınıyor;
1.75/2.00 ise **0.81'den 3.14 m/s'ye kadar** sakin. Yani ayırt edici hız
değil, **kutu hızı**.

Aşım da monotonik iyileşiyor (koşu A): 219% → 149% → 123% → 75%.

**MATLAB çapraz kontrolü tutuyor:** MATLAB'ın kutusu
`p.tilt.rate_max·Ts_ctrl = 3.0·(1/400)` ve döngüsü gerçekten 400 Hz →
**efektif 3.0 rad/s**, yani burada denenen her değerin üstünde; orada aşım
%24.1 ve yerleşme 3.69 s. **Trend MATLAB'a doğru düzgün ekstrapole oluyor.**

> **(Q)'NUN MEKANİZMASI:** düşük hızdaki yaw salınımı kontrol yasasında bir
> sönümleme eksikliği değil, **tahsisatın tilt slew'undan aç bırakılması.**
> Adım 21'in ölçtüğü de buydu (tilt `sat_flag` %99.4, tahsisat yaw verimi
> %20.6) — ama o zaman nedensellik kurulamamıştı, şimdi kuruldu.

**Adım 14 ile çelişmiyor, onu açıklıyor:** Adım 14 `TILT_RATE_MAX`'ı 3.0
yapınca **hem kutuyu hem gölge modelin fiziksel limitini** birlikte
oynatmıştı; gölge model gerçek tork-sınırlı Gazebo servosunun veremeyeceği
bir slew'a inanınca ıraksadı. **Adım 22'nin ayrıştırması, kutunun tek
başına yükseltilebilmesini sağlayan şeydir.**

#### 23c — Varsayılan 1.25 → **1.75** yapıldı ve doğrulandı

2.00 yerine 1.75 seçildi: fiziksel servo limitine (`TILT_RATE_MAX = 2.0`)
0.25 rad/s pay bırakır. `sitl-lockup-check` kriter koşusu (`pitch_sp=0`,
6 m tırmanış, arm+3…arm+32 s; geri-okunan kutu 1.750 rad/s ✔):

| kriter | 1.25 (önce) | **1.75 (sonra)** |
|---|---|---|
| aktüatör kilitlenmesi | ✅ itki 8.26-19.10 N | ✅ **itki 12.83-19.11 N**, sat %0.0 |
| dikey hız | ✅ 0.784 m/s | ✅ **0.816 m/s**, irtifa hata RMS 0.279 m |
| **yaw** | ❌ tepe 37.09° | ❌ **tepe 35.80°** |
| roll/pitch | ±0.08° | ±0.06°, p/q RMS 0.0012/0.0021 |

**Yaw kriteri hâlâ geçmiyor.** Bu koşuda iyileşme mütevazı (37.09 → 35.80°)
çünkü `pitch_sp=0` senaryosu aracı hızla hızlandırıyor (sonda 7.81 m/s) ve
problemli düşük-hız penceresinden zaten çıkıyor; kalan aşım **arm
geçicisinin tek seferlik salınımı**. Taramanın gösterdiği asıl kazanç
**kalıcı salınımın ortadan kalkması** (yaw hızı RMS ~0.5 → ~0.005, 100×).

**Kalan iş:** arm geçicisinin tepe genliği. Kutu 2.00'de bile aşım %75
(MATLAB %24), yani hâlâ bir boşluk var ve fiziksel limit 2.0 tavan. Daha
ileri gitmek `TILT_RATE_MAX`'ı yükseltmeyi gerektirir — ki Adım 14 bunun
gölge model ayrışmasına yol açtığını ölçtü. Doğru yol, gölge modeli gerçek
tork-sınırlı servoya sadık kılmak (çıkarım 30/45).

---

### Adım 24 — Gölge modeli gerçek servoya sadık kılma denemesi: **DENENDİ, GERİ ALINDI** (2026-07-28)

#### 24a — Gerçek servo parametreleri (SDF'den, tam)

`model.sdf`: `JointPositionController` p_gain=100, i=d=0, **cmd_max=2 N·m**,
err_max=0.2 rad; eklem `<friction>1.0</friction>`, damping yok, limit
[0, 1.57]. Tilt ekseni y → etkin atalet `motor_N` iyy 0.0166704 + `rotor_N`
iyy 0.000167 ≈ **J = 0.0168 kg·m²**. Türetilen iki sayı:

- **ölü bant** = friction / p_gain = 1.0/100 = 0.01 rad = **0.573°**
- **max ivme** = (cmd_max − friction) / J = **59.4 rad/s²**

**0.573°'lik ölü bant, Adım 18/21'de ölçülen kalıcı 0.52-0.53°'lik ofseti
niceliksel olarak açıklıyor** — gerçek eklem sürtünme bandının içinde bir
yere park ediyor, gölge model ise komuta kadar sürünüyor.

#### 24b — Çevrimdışı doğrulama: sadelik kazandı, 2. derece model kaybetti

Kayıtlı `u_cmd` (= `u_actual + du`, log'dan) ile üç model aynı komut
dizisiyle sürülüp gerçek eklem açısıyla karşılaştırıldı
(`sitl/servo_model.py`). Gölge-gerçek hata RMS'i:

| eksen | mevcut (1. derece) | **tam 2. derece** (tork limitli + sürtünme + atalet) | **1. derece + ölü bant** |
|---|---|---|---|
| δ0 | 0.287° | 0.414° | **0.082°** (3.5×) |
| δ1 | 0.408° | 0.462° | **0.051°** (8.0×) |
| δ2 | 0.554° | 0.553° | **0.0040°** (139×) |

Kalıcı ofsetler de sıfırlanıyor (δ2: −0.545° → +0.003°).
**Tam 2. derece model DAHA KÖTÜ:** atalet birkaç ms'de oturuyor (max ivme
59.4 rad/s²), yani 4 ms'lik tick'in çok içinde — 250 Hz'de 2. derece dinamik
alâkasız. **Sadakat açığının tamamı sürtünme ölü bandıydı.**

#### 24c — Uygulandı → **KİLİTLENDİ**. Geri alındı.

İki satırlık ölü bant eklenip SITL'de koşuldu. Sonuç:

| ölçüm | değer |
|---|---|
| δ0 (tüm uçuş) | **sabit 9.308°, sıfır varyans** — hiç kımıldamadı |
| δ1, δ2 | örneklerin **%100'ünde** tam 0.000° |
| yaw bandı | **237.8°** — araç dönüyor |
| yatay hız | 0.25 m/s |

**Sebep — geri besleme tuzağı:** `u_cmd = _u_actual + du`, yani komut gölge
duruma **bağlı**; `du` ise tahsisat slew kutusuyla
`TILT_SLEW_BOX_RATE·TS_BOX = 1.75·0.004 = 0.007 rad = 0.40°` ile sınırlı —
bu **0.573°'lik ölü banttan KÜÇÜK**. Dolayısıyla hiçbir tick sürtünmeyi
kıramıyor → gölge donuyor → komut da gölgeden türediği için donuyor.
Kalıcı kilitlenme.

*(Not: o koşuda ölçülen "gölge hatası RMS 0.101°/0.000°/0.000°" geçerli bir
sadakat sonucu DEĞİL — her şey donmuş olduğu için hata da sıfırdı.)*

**Geri alındı ve doğrulandı:** tilt yeniden hareket ediyor (δ0 7.80-15.23°,
std 1.68), kriterler Adım 23 referansıyla aynı — kilitlenme ✅ (itki
12.63-19.21 N, yapışma %0.00), yaw ❌ (tepe 35.88° vs 35.80°), dikey hız ✅
(0.860 m/s, irtifa hata RMS 0.367 m). `TILT_STICTION_BAND = 0.01f` sabiti
**yeniden denenmek istenirse diye bırakıldı**; kullanılmıyor.

#### 24d — İki ders

1. **Açık çevrim replay, kapalı çevrim geri besleme tuzağını GÖSTEREMEZ.**
   Çevrimdışı doğrulama kendi içinde doğruydu (3.5×-139× iyileşme) ve yine
   de ölümcül bir sorunu kaçırdı: modeli, gölgenin *hareket ettiği* bir
   koşudan gelen kayıtlı komut dizisiyle sürdüğü için komut donmuş durumdan
   uzaklaşıyordu. **Komut yolunun İÇİNDE duran model değişiklikleri kapalı
   çevrimde doğrulanmalı.**
2. **Bu artımlı mimari, gölgenin komuta doğru sürünmesini ZORUNLU kılıyor.**
   Bir stiction/ölü bant terimi bununla temelden uyumsuz — ancak mutlak
   komut gölgeden bağımsız ayrı bir durum olarak tutulursa mümkün olurdu, ki
   o da INDI'nin lineerleştirme noktasını bozar (WLS artışı tanım gereği
   aktüatörün *mevcut* durumuna görelidir). **Yani "gölge modeli gerçek
   servoya sadık kıl" isteği, bu mimaride bu yoldan karşılanamaz.**

---

### Adım 25 — §4 (Q) yolu (a): "arm geçicisini kaynağında küçült" — **premis iki kez çürüdü, kod DEĞİŞTİRİLMEDİ** (2026-07-28)

`safe-control-change` gereği kod değiştirilmeden önce premis ölçüldü. İki
ayrı hipotez sınandı, ikisi de veriyle düştü.

#### 25a — Hipotez 1: "tilt arm'da ön-konumlandırılmamış" → ÇÜRÜDÜ

Kodda doğrulandı: disarm'dayken `publishDisarmed()` tüm servo kanallarına
**NaN** yazıyor, yani tilt servoları hiç komut almıyor ve 0°'de duruyor;
arm anında ise gölge `hoverTrim()` ile δ0 ≈ 9.39°'de tohumlanıyor. Yani
arm anında gerçek ile gölge arasında **9.39°'lik** bir başlangıç
uyuşmazlığı var. Ama ölçüm gösteriyor ki bu **çok hızlı kapanıyor**:

| t−arm | gölge δ0 | **gerçek δ0** | fark |
|---|---|---|---|
| −0.10 s | 9.385° | 0.000° | 9.385° |
| +0.05 s | 9.411° | 4.643° | 4.769° |
| +0.10 s | 9.385° | 11.972° | −2.587° |
| +0.20 s | 9.454° | 9.614° | **−0.161°** |

**Gerçek δ0, trim'in %90'ına arm'dan 72 ms sonra ulaşıyor** ve +0.20 s'de
gölgeyle 0.16° içinde örtüşüyor. Oysa yaw hızı arm'da **sıfır** ve ancak
kademeli olarak büyüyor (−0.135 @1 s → −0.453 @2 s → −0.501 @3 s).
**Zaman ölçekleri uyuşmuyor:** 200 ms'lik bir başlangıç uyuşmazlığı,
saniyeler boyunca biriken bir geçiciyi açıklayamaz. Ön-konumlandırma
(disarm'dayken trim tilt'i komut etmek) **gereksiz.**

#### 25b — Hipotez 2: "trim yalnızca hover itkisinde geçerli, tırmanışta bozuluyor" → ÇÜRÜDÜ

Mantık makuldü: reaksiyon torku `τ_react = −Σ km_i·T_i·cos δ_i` itkiyle
ölçekleniyor, `hover_trim`'in diferansiyel tilt'i ise hover itkisine göre
sabitlenmiş — tırmanışta itki ~%40 arttığında dengesizlik açığa çıkmalı.
Ölçüm:

| t−arm | toplam itki | τ_react | τ_tilt | **NET** |
|---|---|---|---|---|
| +0.0 | 49.7 N | 0.733 | −0.755 | **−0.021** |
| +0.3 | **68.9 N** | **1.011** | **−1.022** | **−0.011** |
| +1.5 | 52.6 N | 0.773 | −0.927 | −0.154 |
| +3.0 | 47.5 N | 0.696 | −0.704 | **−0.008** |

**Hata şuydu: tilt'in ürettiği tork da itkiyle ölçekleniyor**
(`τ_tilt = −Σ py_i·T_i·sin δ_i` — içinde `T_i` var, tıpkı `τ_react` gibi).
İki terim **birlikte** ölçeklendiği için oran korunuyor ve trim, itki
seviyesinden **bağımsız olarak geçerli kalıyor**. İtki %40 arttığında her
iki terim de ~%38 artıyor, net ~0 kalıyor. Tırmanış boyunca NET tork
yalnızca **0.01-0.15 N·m**.

#### 25c — Sonuç: ayrı bir "arm geçicisi" mekanizması YOK

Arm sonrası yaw savrulması **bağımsız bir başlangıç-koşulu artefaktı
değil**; madde (Q)'nun ta kendisi — zayıf ve yavaş yaw ekseninin, küçük
artık torklara (0.03-0.15 N·m) verdiği yanıt. Karşılaştırma: Adım 21'de
ölçülen tahsisat yaw otoritesi adım başına ~0.033 N·m. Yani araç 3 s'de
0.5 rad/s biriktirebiliyor çünkü eksen bu mertebedeki bir torku bile
zamanında karşılayamıyor — Adım 23'ün slew kutusu bulgusunun aynısı.

**Dolayısıyla §4 (Q) yolu (a) kapanmıştır: kaynağında düzeltilecek ayrı
bir mekanizma yok.** Kalan yol (b): sabitler artık ayrıştırıldığına göre
**yalnızca** `TILT_RATE_MAX`'ı yükseltmek (Adım 14 ikisini birden
oynatmıştı; bu hiç denenmedi).

**Bu adımda hiçbir kod/sabit DEĞİŞTİRİLMEDİ.**

---

### Adım 26 — Yol (b) NO-OP çıktı; asıl yönetici parametre bulundu: **etkin slew = kutu·TS_BOX/TILT_TAU** (2026-07-28, YARIM KALDI)

#### 26a — `TILT_RATE_MAX`'ı yükseltmek hiçbir şey yapmaz (ölçüldü)

Adım 22'nin ayrıştırmasından sonra `TILT_RATE_MAX` yalnızca gölge modelin
`ddelta` clamp'inde kalmıştı. Ama `ddelta = du/TILT_TAU` ve `du` zaten kutuyla
sınırlı:

| ölçüm (kutu 1.75, kriter koşusu) | değer |
|---|---|
| `\|du(3)\|` p99 | 0.007000 rad (= kutu, tam) |
| ölçülen `\|dδ0/dt\|` p99 **ve** max | **0.0467 rad/s** |
| `TILT_RATE_MAX` clamp | 2.00 rad/s |
| **clamp'e ulaşan örnek oranı** | **%0.000** (43× başlık) |

**Clamp hiç bağlamıyor → yol (b) bir no-op.** 1.75 seçilirken kullandığım
"fiziksel limite 0.25 pay bırak" gerekçesi de bu yüzden anlamsızdı: iki sabit
aynı büyüklüğe etki etmiyor.

#### 26b — Asıl yönetici parametre

```
etkin tilt slew = TILT_SLEW_BOX_RATE · TS_BOX / TILT_TAU
```

Gölge her tick'te `du`'nun yalnızca `dt/TILT_TAU = %2.7`'sini ilerletiyor
(çünkü `u_cmd = _u_actual + du`, yani komut gölgeye göre tanımlı), dolayısıyla
nominal kutu hızı **38× de-rate** oluyor: 1.75 → **0.0467 rad/s**.

**Ve MATLAB'ın karşılığı `3.0·(1/400)/0.15 = 0.0500 rad/s`.** Yani Adım 23'ün
taraması aslında SITL'i 0.0333'ten 0.0467'ye, MATLAB'ın etkin hızının hemen
altına getirmiş — salınımın tam orada kesilmesinin sebebi bu. İki ortam
arasındaki "sönümleme boşluğu" baştan beri bu tek büyüklükmüş.

#### 26c — Genişletilmiş tarama (iki koşu, ters sıra)

`slewbox` aralık kontrolü `TILT_RATE_MAX` yerine test aralığı [0.1, 4.0]
yapıldı (gerekçe 26a). Her değerde aynı +30° yaw adımı:

| kutu | etkin (rad/s) | koşu C | koşu D | **ort. aşım** |
|---|---|---|---|---|
| 1.75 | 0.047 | %83.5 / 10.90 s | %81.4 / 5.28 s | **%82.5** |
| 2.50 | 0.067 | %37.2 / 4.71 s | %38.7 / 3.50 s | **%38.0** |
| **3.00** | **0.080** | %20.7 / 3.30 s | %28.9 / 4.37 s | **%24.8** |
| 3.50 | 0.093 | %18.2 / 3.04 s | %17.2 / 3.07 s | **%17.7** |
| *MATLAB* | *0.050* | — | — | *%24.1 / 3.69 s* |

Monotonik, tekrarlanabilir ve **3.00'da MATLAB'a oturuyor**. Test edilen hızlar
0.77-2.65 m/s ve **en iyi sonuçlar en DÜŞÜK hızlarda** — yani hız artefaktı değil.

#### 26d — Durum: **3.00 önerilir ama DEPLOY EDİLMEDİ**

Oturum, 3.00 için zorunlu `sitl-lockup-check` kriter koşusundan **önce**
durduruldu. `TILT_SLEW_BOX_RATE` doğrulanmış **1.75**'te bırakıldı (Adım 23c:
kilitlenme ✅, |vz| 0.816 m/s, irtifa hata RMS 0.279 m, roll/pitch ±0.06°);
kaynak ile derlenmiş ikili **uyumlu**. `slewbox` aralık genişletmesi (4.0)
kalıcı ve derlenmiş durumda.

**Devam etmek için:** `TILT_SLEW_BOX_RATE`'i 3.00f yap →
`make px4_sitl_default` → **`sitl-lockup-check` kriter koşusu.** Bu adım
atlanmamalı: tahsisatın tick başına otoritesini artırmak, tam olarak aktüatör
kilitlenmesi arıza modunu yeniden açabilecek türden bir değişiklik.

---

### Adım 27 — `TILT_SLEW_BOX_RATE` = 3.00 **DAĞITILDI ve doğrulandı**; ve **yaw kriterinin ölçüm yöntemi hatalıymış** (2026-07-29)

Adım 26'nın yarım kalan işi tamamlandı: sabit 1.75f → **3.00f** yapıldı,
`make px4_sitl_default` ile derlendi, zorunlu kriter koşusu yapıldı.
MATLAB'a DOKUNULMADI — `TILT_SLEW_BOX_RATE` Adım 22'nin ayrımıyla doğan,
yalnızca PX4'te var olan bir sabit; MATLAB'da `p.tilt.rate_max = 3.0` hem
fiziksel servo limiti hem kutu olarak çift görev yapıyor ve `p.Ts_ctrl = 1/400`
gerçek döngü periyoduna eşit, yani ayrıştırılacak bir defekt orada yok.

**Değerin gerçekten aktif olduğu bağımsız doğrulandı:** ulog'dan
`|du(tilt)| p99.5 / TS_BOX` = **3.000 rad/s** (her iki kriter uçuşunda), konsol
logunda `ddelta=[-0.0120 -0.0120 0.0120]` = 3.00·(1/250).

#### (a) ⚠️ Önce bir ÖLÇÜM HATASI bulundu: "yaw ±30° kriteri" hiç hover tutuşu ölçmüyormuş

Kriter koşularında `yaw_sp = 0` veriliyor, **ama airframe Gazebo'da +90°
heading'de doğuyor** (`vehicle_attitude_groundtruth`, t=0'da tam +90.00°; EKF
buna ~20 s'de yakınsıyor). Yani her "hover kriter koşusu" aslında **90°'lik bir
heading manevrası komutluyor**, ve rapordaki metrik — `[arm+3, arm+32]`
penceresindeki mutlak yaw bandı — o manevranın 3. saniyede nerede olduğunu
ölçüyor. Metrik eski loglardan birebir doğrulandı (08_33_04 → −2.67…+37.09;
09_55_05 → −5.16…+35.80 — rapordaki sayılarla aynı).

**Sonuç: Adım 22/23/24'te "yaw ❌ KALDI (35.80° / 37.09° / 35.88°)" diye
raporlanan şey bir kontrol kusuru değil, komut edilen 90°'lik dönüşün
kuyruğudur.** Metrik ayrıca koşular arasında karşılaştırılabilir değil, çünkü
arm→setpoint gecikmesine ve EKF'in yakınsadığı heading'e duyarlı.

Kanıt — **setpoint'in ulaştığı ana hizalanmış**, aynı manevra:

| | kutu 1.75 (Adım 23c) | kutu 1.75 (Adım 24) | **kutu 3.00** |
|---|---|---|---|
| yakalama açıklığı | +90.0° | +90.3° | +96.6° |
| %90'a varış | 4.06 s | 3.98 s | **3.59 s** |
| ortalama slew | 19.9 °/s | 20.4 °/s | **24.2 °/s** |
| hedefi aşma | −5.16° | −4.27° | **−2.70°** |
| çınlama (c+8'de) | +14.0° | +6.7° | **+0.8°** |

Yani 3.00, *daha büyük* bir dönüşü daha hızlı, daha az aşımla ve pratikte
çınlamasız yakalıyor. Eski metrikle 3.00'ün daha kötü görünmesi
(+54.6° vs +35.8°) tamamen pencere hizalama artefaktı.

#### (b) Doğru kriter koşusu: `yaw_sp` = arm anındaki gerçek heading (tutuş testi)

Kriterin niyeti ("yaw ±30°'yi aşmıyor") bir **tutuş/kararlılık** ölçütü, o yüzden
`yaw_sp` araca ölçülen heading'i verilerek koşuldu. Adım 20'nin dersi gereği
**iki uçuş** (marjinal kararlı bir eksende tek uçuştan sonuç çıkarılmaz):

| kriter (pencere arm+3…arm+32) | uçuş 3 | uçuş 4 | sonuç |
|---|---|---|---|
| yaw hata bandı | −7.29…+5.85° | −10.29…+10.32° | ✅ **GEÇTİ** (±30) |
| yaw hızı RMS (son 8 s) | 0.0028 | 0.0014 rad/s | ✅ sürekli salınım yok |
| roll/pitch hızı RMS | 0.0010/0.0023 | 0.0014/0.0019 rad/s | ✅ |
| max \|vz\| | 1.743 | 1.637 m/s | ✅ (limit 2.0) |
| irtifa tutuş RMS | 0.072 m | 0.052 m | ✅ |
| itki `sat_flag` / BIG_M | %0.0 / 0 | %0.0 / 0 | ✅ kilitlenme yok |
| gölge itki aralığı | 11.94-14.59 / 14.49-19.60 N | 12.15-19.91 N | ✅ 0/45 N'a yapışma yok |
| yatay hız (bağlam) | 1.70-7.08 m/s | 1.91-7.03 m/s | — |

**Dört kriterin dördü de, iki uçuşta da geçti.** İrtifa tutuşu bu projede
kaydedilen en iyi değer (0.052-0.072 m; kutu 1.75'te 0.279 m).

#### (c) İniş fazı: (Q) değil, (O)'nun sınırı — kademe büyüklüğü belirleyici

Uçuş 2'de (kaba iniş, 1.5 m'lik kademeler) **13 adet `Wu1=1000000` (BIG_M)**
sayıldı; hepsi iniş/yere oturma fazında, kriter penceresinde sıfır. Uçuş 4'te
aynı iniş **1.0 m'lik kademelerle** yapıldı → **0 BIG_M**, motor komutları
0.43-0.60 aralığında, yaw tutuluyor. Uçuş 1'de (hiç iniş yok, irtifada disarm)
da 0 BIG_M. Yani bu, kutu 3.00'ün getirdiği bir regresyon değil; **madde (O)'nun
bilinen iniş-fazı yapışması ve eşiği kademe büyüklüğünde** — 1.0 m güvenli,
1.5 m değil. (O) "kademeli iniş ile kapandı" ifadesi bu yüzden daraltılmalı.

#### (d) Yan bulgu: EKF yaw'ının gerçek heading'e göre kalıcı sapması

`vehicle_attitude` ile `vehicle_attitude_groundtruth` karşılaştırıldığında
uçuş boyunca **~5-10°** kalıcı yaw tahmin hatası var (ör. uçuş 4: EKF bandı
81.2…101.8°, gerçek 92.0…108.4°). Kontrolcü *tahmin edilen* heading'i tuttuğu
için fiziksel heading bu kadar kayıyor. Kriter açısından zararsız (tutuş hâlâ
±30 içinde) ama **yaw sayılarını gerçek fiziksel yönelim sanmayın.**

#### (e) ⚠️ Adım 21(d) ÇÜRÜDÜ: MATLAB'da tilt kutusu **bağlıyor** — ve ayrım MATLAB'a taşındı

İlk değerlendirmem "bu sabit PX4'e özgü, MATLAB'da karşılığı yok, regresyon
no-op" idi. **Yanlıştı ve ölçümle düzeltildi** (yeni araç:
`run_box_bind_check.m`). İki ayrı hata vardı:

1. **Karşılığı var.** MATLAB'da `p.tilt.rate_max` — tıpkı PX4'ün Adım 22
   öncesi hâli gibi — **iki işi birden** yapıyordu: plant'in fiziksel servo
   clamp'i (`tiltrotor_plant_deriv.m:42`) *ve* tahsisat kutusu
   (`indi_attitude_controller.m:94-95`, `sf_wls_alloc.m`). Yani ayrım
   MATLAB'a hiç taşınmamıştı; referans uygulama bu sabitte PX4 portunun
   gerisindeydi.
2. **Kutu MATLAB'da bağlıyor.** Adım 21(d) "MATLAB'da tilt slew kutusu hiç
   bağlamaz (~2.5°/s hareket vs 114°/s izin)" demişti. Ölçüm: yaw adımı
   sonrası tick'lerin **%25-28'inde kutu aktif**. Adım 21(d)'nin
   ölçülmemiş varsayımıydı ve yanlıştı. (Doğru olan kısım: **fiziksel**
   clamp gerçekten hiç bağlamıyor — 3.0 → 2.0 yapmak sonucu bit düzeyinde
   değiştirmiyor.)

Ayrım MATLAB'a taşındı: `p.tilt.rate_max` = yalnızca plant clamp'i,
yeni `p.tilt.slew_box_rate` = yalnızca tahsisat kutusu (`sf_wls_alloc.m`'de
codegen-safe literal `slew_box_rate_tilt` olarak da senkronlandı).
**Değer seçimi:** eşleşmesi gereken NOMINAL hız değil **tick başına kutu** —
çünkü etkin slew = (tick başına kutu)/`p.tilt.tau` (Adım 26). PX4:
3.00·(1/250) = 0.0120 rad/tick; MATLAB 400 Hz'de aynı kutu için
**4.8 rad/s**. Nominal sayıların farklı olması (4.8 vs 3.00) doğrudur ve
yalnızca döngü hızı farkını yansıtır; **etkin slew ikisinde de 0.080 rad/s**.

**MATLAB regresyonu (`safe-control-change` zorunlu adımı):**

| test | önce | sonra | sonuç |
|---|---|---|---|
| `run_hover_gust_test` LESO açık | RMS p/q 0.0013/0.0004 | 0.0014/0.0003 | ✅ nötr |
| `run_hover_gust_test` LESO kapalı | 0.0045/0.0009 | 0.0046/0.0008 | ✅ nötr |
| `run_transition_test` | tilt 9.5°, u 6.70 m/s, Δirtifa −0.08 m, max\|ω\| 0.0126 | **birebir aynı** | ✅ |
| `run_yaw_step_test` +30° | %24.0 aşım, 3.68 s | **%14.5, 3.01 s** | ✅ iyileşti |
| `run_yaw_step_test` −30° | %7.7 aşım, 3.08 s | %11.0, 3.03 s | 🟡 hafif kötü |
| madde (P) yön asimetrisi | 24.0/7.7 = **3.1×** | 14.5/11.0 = **1.3×** | ✅ daraldı |

Yani PX4'te ölçülen kazanç MATLAB'da **bağımsız olarak doğrulandı**, ve
madde (P)'nin yön asimetrisi de büyük ölçüde bunun bir sonucuymuş.
`tiltrotor_indi.slx` yeniden derlendi (`tiltrotor_indi_build`).

**Ders (Adım 21(d)'nin ve kendi ilk yargımın ortak hatası):** *"Bu ortam bu
kısıtı görmez" demeden ÖNCE bağlama oranını ölçün.* Adım 21(d) bunu bir
ablasyonun neden geçersiz olduğunu açıklamak için doğru yönde kullanmıştı,
ama sayıyı hiç ölçmemişti; ben de aynı ölçülmemiş varsayımı devralıp
"MATLAB regresyonu gereksiz" sonucuna vardım.

**Kalıcı durum:** `TILT_SLEW_BOX_RATE = 3.00f` (PX4), `p.tilt.slew_box_rate
= 4.8` (MATLAB + Simulink) — üç uygulama artık aynı etkin slew'da. Kaynak
ile derlenmiş ikili uyumlu, `logger_topics.txt` geçici log profili
**silindi** (varsayılan profil geri geldi), süreçler temiz.

**Genel ders (yeniden kullanılabilir):** *Bir geç/kal kriteri, testin kendisinin
komut ettiği bir manevrayı ölçüyor olabilir. Kriteri yazarken başlangıç
koşulunun (burada: spawn heading'i) setpoint'e eşit olduğunu doğrulayın —
yoksa "kriter kalıyor" diye aylarca var olmayan bir kusur kovalanır.* Bu,
projedeki **dördüncü ölçüm tuzağı** (önceki üçü: yaw'ı açıdan örnekleme,
p99 yerine max okuma, bağlam değişkenini kaydetmeme).

---

### Adım 28 — Madde (N) ve (P) ÇÖZÜLDÜ: yatay pozisyon döngüsü + Fx trim (2026-07-29)

İki madde aynı kökten geliyordu (**tek yönlü tilt aralığı**) ve **sıralı bir
bağımlılıkları** olduğu ortaya çıktı: (P)'nin çözümünün bedeli kalıcı bir +Fx,
ve onu ancak (N)'in çözümü taşıyabiliyor. Bu yüzden önce (N) yapıldı.

#### (a) Madde (N) — yatay pozisyon döngüsü (`position_loop.m` / `positionLoop()`)

Yapı `altitude_loop.m`'i birebir izliyor: **P (konum) → PI (hız) → attitude
setpoint**, aynı `Ts_pos = 1/50` decimasyonuyla.

**Neden Fx değil attitude üretiyor:** tiltrotor'un "doğal" yatay kanalı
rotorları eğip gövde-x kuvveti üretmek, ama tilt aralığı tek yönlü olduğu için
bu yalnızca **ileri** itebilir — frenleyemez. Gövdeyi eğmek iki yönde de çalışır
ve donanıma doğrudan aktarılabilir. Adım 15'in elle `pitch_sp = +0.061 rad`
geçici çözümü zaten bunun manuel hâliydi; bu döngü onu kapalı çevrime aldı.
İşaret kuralı: **+pitch = burun yukarı = geri kuvvet**, yani ileri ivme negatif
pitch ister (Adım 15'in ölçümü bunu doğruluyor).

**MATLAB A/B** (`run_station_keeping_test.m`, yeni): sürüklenme
**137.15 m → 0.36 m** (40 s), max yatay hız 3.93 → 0.35 m/s.

**SITL:** `pos_hold_enable` alanı `TiltrotorIndiSetpoint.msg`'e eklendi; hedef
`false→true` kenarında yakalanıyor (test kendi koordinatını bilmek zorunda
değil). Ölçülen: hedeften sapma **ortalama 0.06 m, max 0.17 m**, yatay hız
ortalama 0.08 m/s — önceki **25 s'de 235 m** yerine.

**Bunun asıl kazancı ölçüm tarafında:** Adım 16 "yatay döngü olmadığı için her
hover doğrulaması aslında ~10 m/s seyir testiydi, yaw'ın EN KÖTÜ koşulu olan
gerçek duruş hiç test edilmedi" demişti. **Artık test edildi:** gerçek duruşta
(v_h = 0.01 m/s) +30° yaw adımı ~9 s'de oturuyor, yaw hatası **−0.33…+1.26°**,
yaw hızı RMS 0.014, salınım yok, 0 BIG_M.

#### (b) Madde (P) — Fx trim; ve teşhisim bir kez yanlış çıktı

**Önce yanlış hipotez (kayıt için).** (P) için WLS'in `du_pref`'ine "tilt'i
bias'a çek" terimi eklendi. **Tamamen etkisiz çıktı:** bias 0/5/10° ve
`bias_tau` 3.0/1.0/0.5/0.2 (kutunun **5 katına** çıkan pull dahil) — δ1 her
durumda tam 0.00° ve taban oranı %100. Ek olarak ilk uygulamada ölçekleme de
yanlıştı (`Ts/bias_tau` yerine `tau_tilt/bias_tau` olmalıydı) — bu **Adım
26'nın çift-sayma tuzağının aynısı**: artımlı komuta oran uygulamak onu `dt/τ`
ile ölçekler.

**Gerçek mekanizma, tahsisat enstrümante edilerek bulundu.** Dengede
`nu_des(Fx) = **−2.91 N**` ve **kısıtsız çözüm üç tilt için de negatif**
(`du_free(4:6) = [−0.0084, −0.0089, −0.0017]`). Yani araç δ1'i 0'ın **altına**
indirmek istiyor ve tabana çakılı kalıyor (`sat_flag = 1`). Sebep bir tercih
zayıflığı değil: **tahsisattan `Fx = 0` istemek, aracın yapısal olarak iptal
edemeyeceği bir kuvveti iptal etmesini istemek.** (P)'nin yön asimetrisi tam
olarak budur — −yaw serbest, +yaw sınıra dayalı.

**Çözüm:** talebe ulaşılabilir trim'i eklemek. Ölçüldü (±30° yaw adımı):

| `fx_trim` | +30 aşım | −30 aşım | asimetri | δ1 tabanda |
|---|---|---|---|---|
| 0 | %14.5 | %10.9 | **1.33×** | %100 |
| **2.9** | %13.5 | %13.3 | **1.02×** | **%0** |
| 4.0 | %13.5 | %13.0 | 1.03× | %0 (ama Fx 3.84 N) |

2.9 N **doğal olarak ulaşılabilir denge değeri** (ölçüldü), uydurma bir sayı
değil. Adım 17'de bu asimetri **7.4×** idi; Adım 27'nin kutu düzeltmesi 1.33×'e
indirmişti, şimdi **1.02×**.

**Trim nerede duruyor ve neden:** ilk denemede doğrudan tahsisata konuldu ve
`run_hover_gust_test` regresyonunu **q RMS 0.0004 → 0.0013 (4.3×)** bozdu —
çünkü o test pozisyon döngüsü kullanmıyor ve oluşan ileri kuvveti taşıyacak
hiçbir şey yok. Bunun üzerine trim `position_loop.m`'e taşındı ve `F_sp(1)`
olarak veriliyor: **(P)'nin çözümü artık yapısal olarak (N)'in aktif olmasına
bağlı.** Bu, hem regresyonu tamamen kaldırdı hem de bağımlılığı kodun
yapısına gömdü. Geçiş boyunca `sched.smooth` ile sönüyor.

#### (c) Doğrulama

**MATLAB regresyonu — değişiklik ÖNCESİ ile birebir aynı** (düzeltmeler
eklemeli olduğu için): hover-gust 0.0014/0.0003 (LESO açık) ve 0.0046/0.0008
(kapalı); transition tilt 9.5°, u 6.70 m/s, Δirtifa −0.08 m, max|ω| 0.0126;
yaw adımı %14.5/%11.0. `tiltrotor_indi.slx` yeniden derlendi.

`run_station_keeping_test.m` (pozisyon döngüsü AÇIK, N+P birlikte):

| | sürüklenme | v_h(adım) | +30 aşım | −30 aşım | asimetri |
|---|---|---|---|---|---|
| döngü KAPALI | 137.15 m | 3.79 m/s | %14.5 | %10.9 | 1.33× |
| **döngü AÇIK** | **0.36 m** | **0.01 m/s** | %13.5 | %13.3 | **1.02×** |

**SITL** (duruş + üç ardışık yaw manevrası, tek uçuş):

| adım | aşım | oturma | max sürüklenme | max v_h | δ1 tabanda |
|---|---|---|---|---|---|
| +30° | %18.5 | 2.99 s | 0.28 m | 0.22 m/s | %0.0 |
| −30° | %18.3 | 3.09 s | 0.22 m | 0.32 m/s | %1.0 |
| −30° | %17.7 | 4.09 s | 0.29 m | 0.38 m/s | %0.2 |

**SITL asimetri 1.05×** (MATLAB 1.02×). δ1'in taban süresi Adım 27
uçuşlarındaki **%86.8'den %0.3'e** düştü — (P)'nin mekanizması SITL'de de
kapandı. İtki `sat_flag` %0.0, **0 BIG_M**.

#### (d) Kalan maddelerin durumu

- **(O)** — kapalı; Adım 27 eşiği daralttı: iniş kademeleri **1.0 m** olmalı
  (1.5 m 13 BIG_M üretti). Bu bir işletim prosedürü, kontrol kusuru değil.
- **(M) `WS_YAW`** — **gereksiz olarak KAPATILDI.** Yaw kriteri, ağırlıklara
  hiç dokunmadan geçti (Adım 27) ve asimetri de ağırlık değiştirmeden çözüldü.
- **(J) artık irtifa hatası** — kendiliğinden çözüldü; duruş uçuşlarında
  irtifa tutuş RMS 0.052-0.087 m.
- **Donanım hâlâ 🔴 NO-GO.** Pozisyon döngüsü SITL'de doğrulandı ama gerçek
  donanımda hiç uçmadı; ayrıca Adım 11'in itki eşlemesi düzeltmesi SITL'e özgü.
  Bir sonraki iş bu değil, **gerçek donanım kalibrasyonu ve kademeli uçuş
  testi** olmalı.

#### (e) İki test boşluğu kapatıldı: rüzgâr reddi ve geçiş, güncel ayarlarla

Bunlar Adım 27-28 konfigürasyonuyla SITL'de hiç koşulmamıştı. **Her ikisi de
artık `pos_hold` ile, yani gerçek hover'dan başlayarak koşuluyor** — daha önce
ikisi de yapısal olarak seyir testiydi (Adım 16). Sürücüler güncellendi
(`indi_sitl_common.py`'ye `pos_hold` argümanı, `run_hover_gust_test.py`'ye
zorunlu hız/sürüklenme bağlamı).

**Rüzgâr reddi** (`run_hover_gust_test.py`, 0.4 + 0.15·sin(2π·0.3t) N·m roll
bozucusu, 20 s, iki konfigürasyon):

| konfig | RMS p | RMS q | v_h | max sürüklenme |
|---|---|---|---|---|
| LESO açık (roll+pitch) | 0.0719 | 0.0808 rad/s | 0.01-0.42 m/s | 0.36 m |
| LESO kapalı | 0.0625 | 0.0945 rad/s | 0.07-0.32 m/s | 0.36 m |

**Sonuç: araç sürekli bir roll bozucusu altında duruşunu koruyor** — ıraksama
yok, aktüatör yapışması yok, **0 BIG_M**, EKF %100 sağlıklı. İki dürüst not:
(1) LESO açık/kapalı farkı burada belirsiz (p'de biraz kötü, q'da biraz iyi),
oysa MATLAB'da LESO 3× kazandırıyor — bu fark açık bırakılıyor, bir engelleyici
değil. (2) SITL RMS'leri MATLAB'ın 50-250× üzerinde, ama **karşılaştırılabilir
değiller**: burada pozisyon döngüsü de aktif olarak attitude komutluyor, yani
ölçülen hız aktivitesi yalnızca bozucu yanıtı değil. Ayrıca örnekleme kaba
(~1.1-1.3 s/adım). Eski koşularla da birebir karşılaştırma yok, çünkü onlar
seyirde yapılmıştı.

**Geçiş** (`run_transition_test.py`, duruşta 14 s → `pos_hold` bırakılır →
Fx rampası 0→10 N / 12 s):

| pencere | rate RMS p/q/r | max\|ω\| | v_h |
|---|---|---|---|
| duruşun son 6 s'si | 0.2123/0.0773/0.0425 | 0.3642 | 0.01-0.68 m/s |
| **devir (±2 s)** | **0.1503/0.0912/0.0450** | **0.2538** | 0.11-0.47 m/s |
| Fx rampası (+14 s) | 0.0446/0.0689/0.0264 | 0.2569 | 0.11 → **10.86 m/s** |

**Devir temiz: `pos_hold` bırakıldığında hız aktivitesi ARTMIYOR, azalıyor**
(p RMS 0.2123 → 0.1503, max|ω| 0.3642 → 0.2538). Yani roll/pitch sahipliğinin
ve `fx_trim`'in el değiştirmesi bir basamak üretmiyor — bu, eklenen kodun en
riskli tarafıydı ve ölçümle kapandı. Rampa boyunca: itki `sat_flag` **%0.0**
(üç kanal, 9.9-19.4 N), tilt0 9.1→29.3°, tilt1 0→19.9°, kuyruk dikey kalıyor
(0→0.7°), irtifa 0→11 m/s ivmelenme boyunca 0.4 m içinde, **0 BIG_M**.
Sonuç: ortalama tilt 17.5°, ileri hız 11.49 m/s, Δirtifa 0.39 m, max|ω| 0.2500.

**Bir beklentim ölçümle düzeldi:** "geçiş sırasında `fx_trim` sönümleniyor,
bu birleşim hiç uçurulmadı" demiştim. Aslında **bu yol normal bir geçişte hiç
tetiklenmiyor**: `fx_trim` yalnızca `pos_hold` açıkken üretiliyor ve geçişte
`pos_hold` kapalı. Üstelik 11 m/s'de bile `gain_schedule_smooth` yalnızca
**0.090**'a çıkıyor, yani sönümleme terimi neredeyse hiç devreye girmiyor.
Fade bir güvenlik payı; aktif bir mekanizma değil. Pozisyon tutarken tilt'in
büyüdüğü bir senaryo (ör. şiddetli rüzgârda duruş) hâlâ test edilmedi.

**Genel ders:** *İki açık maddenin ortak kökü varsa, çözümlerinin arasında bir
SIRA da olabilir — burada (P)'nin bedelini ancak (N)'in çözümü taşıyabiliyordu.
Bağımlılığı yorumla değil, kodun yapısıyla ifade edin (trim'i inner
controller'a değil pozisyon döngüsüne koymak), yoksa bağımsız sanılıp yanlış
yerde regresyona yol açar.*

---

### Adım 29 — GUI gösteri uçuşu Adım 28'in pozisyon döngüsünde GERÇEK BİR KUSUR buldu: döngü yalnızca hover içindir (2026-07-29)

Gazebo GUI'li bir gösteri uçuşunda, geçişten sonra `pos_hold`'u **14.5 m/s'de
yeniden devreye almayı** denedim. Bu senaryo Adım 28'in hiçbir testinde yoktu
(orada `pos_hold` ya hover'da açılıyor ya da bırakılıyordu, hızdayken hiç
açılmıyordu). **Araç düştü.**

**Ölçülen davranış** (ulog, devreye alma anından itibaren):

| t (s) | v_h (m/s) | vz (m/s) | z (m) | pitch (°) |
|---|---|---|---|---|
| 0 | 14.49 | 0.19 | −9.87 | 0.06 |
| 3 | 8.19 | −0.89 | −13.66 | 18.53 |
| 6 | 7.62 | −0.54 | −16.05 | 15.14 |
| 12 | 9.61 | −1.32 | −23.74 | 15.30 |
| 20 | 9.60 | −1.15 | −35.03 | 15.02 |
| 35 | 9.62 | −1.01 | −54.03 | 15.01 |

Pitch **örneklerin %94'ünde `POS_TILT_MAX` = 15°'de doydu**. Fren başta çalıştı
(14.5 → 7.6 m/s), sonra hız **9.6 m/s'de kilitlendi** ve araç 35 saniye boyunca
**~1.1 m/s ile sürekli tırmandı**. Sonraki elle frenleme denemesi (7° burun
yukarı, 18.6 m/s'de) aracı stall'a sokup düşürdü.

**Kök neden — kendi eklediğim kodda bir tasarım hatası.** `positionLoop()` düz
multikopter bağıntısını kullanıyor:

```
theta_sp = -atan2(ax_b, g)
```

Bu, "yatay kuvvet üretmenin tek yolu itki vektörünü eğmektir" varsayımıdır ve
hover'daki bir multikopter için doğrudur. **Bu airframe'in kanadı var.**
~5-6 m/s üstünde kanat baskın hâle gelir ve burun yukarı komutu bir *frenleme*
değil bir **enerji/tırmanış** komutuna dönüşür. İrtifa döngüsü bunu telafi
edemez: kanat taşımasına karşı yapabileceği tek şey rotor itkisini azaltmaktır,
ve itki sıfırlansa bile kanat taşımaya devam eder. Denge, 9.6 m/s'de sabit
hızlı tırmanıştır — klasik bir enerji yönetimi arızası. Döngüde **hava hızı
farkındalığı ve irtifa döngüsüyle enerji bağlaşımı yok.**

**Düzeltme — devreye alma kapısı.** Doğru enerji yönetimi (TECS benzeri bir
kontrolcü) olmadan güvenli otomatik kurtarma yok, o yüzden döngü **yalnızca
doğrulandığı zarfta** devreye alınabiliyor: `POS_ENGAGE_V_MAX = 3.0 m/s`.
Üstünde devreye alma **reddediliyor** (bir kez uyarı basılır). Zaten devrede
olan bir döngü kapatılmıyor — uçuş ortasında kendiliğinden bırakmak kendi
başına bir tehlike. MATLAB tarafında da sınır `position_loop.m` başlığına
yazıldı (MATLAB testleri hep hover'dan başladığı için bu sınıra hiç dayanmadı).

**SITL doğrulaması (yeniden derlenmiş ikili, GUI'li uçuş):**

| test | sonuç |
|---|---|
| hover'da devreye alma | ✅ kabul edildi, `holding x=0.10 y=0.29`, sapma 0.04 m |
| **9.4 m/s'de devreye alma** | ✅ **`pos_hold REFUSED: 9.4 m/s > 3.0 m/s limit`** |
| reddedildikten sonra irtifa | ✅ −10.13 → −9.92 m, vz ≈ 0.15 — **kaçış yok** |

**İkinci bulgu — madde (O) keskinleşti.** Aynı uçuşun sonunda, **13-21 m/s ileri
hızla** inişe geçildiğinde 98 + 51 BIG_M yapışması sayıldı; log'da tetikleyici
görülüyor: `nu_des(Fz) = +6.48` (sert alçalma talebi) altında itki kanalı 0
alt sınırına çakılıyor (`du0 = dmin0 = −1.79`). Bu **doğrulanmış zarfın
dışında** (doğrulanan iniş: hover'dan, 1.0 m kademelerle → 0 BIG_M, Adım 27-28).
Ama kayda değer: **hızlıyken alçalmak bir yapışma senaryosudur** — kanat
taşıması alçalmaya direndiği için irtifa döngüsü itkiyi agresif biçimde kısmak
zorunda kalıyor. İnişe daima düşük hızda girilmeli.

**Genel ders (bu projede tekrar eden temanın yeni bir örneği):** *Bir dış
döngüyü, altındaki aracın fizik rejimi değişebiliyorsa, o rejimlerin
tamamında düşünmek gerekir. `theta -> yatay ivme` bağıntısı bir varsayımdır,
kimlik değil; kanatlı bir araçta hızla birlikte anlamı değişir. Ve bunu bulan
şey bir test değil, izlenmek üzere yapılan bir gösteri uçuşu oldu — senaryo
çeşitliliği, senaryo derinliği kadar değerli.*

---

### Adım 30 — Seyirden hover'a dönüş: **DENENDİ, GERİ ALINDI** — ve frenleme otoritesinin pitch değil TILT olduğu ölçüldü (2026-07-29)

Adım 29 tehlikeyi bir kapıyla engelledi (`POS_ENGAGE_V_MAX = 3 m/s`) ama
**yeteneği eklemedi**: aracın seyirden hover'a dönmek için hâlâ hiçbir yolu
yoktu. Bu adım o yeteneği eklemeyi denedi.

**Önce mekanizma doğrulandı** (bu projenin kendi kuralı: bir ortamda test
etmeden önce mekanizmanın orada aktif olduğunu göster). Adım 29'un çöküş
log'undan: tırmanış boyunca toplam rotor itkisi **13 N**, ağırlık **49.1 N**,
kuyruk rotoru tamamen kapalı — yani irtifa döngüsü otoritesini çoktan
tüketmişti ve aracı **kanat taşıyordu**. Demek ki hızlıyken büyük burun yukarı
komutu itkiyle telafi edilemez.

**MATLAB bu arızayı yapısal olarak üretemiyor** (ölçüldü): MATLAB plant'i tek
bir boylamsal yüzey kullanıyor (`p.aero.area = 0.5 m²`); 12 m/s ve 15°'de
taşıma yalnızca ~25 N, yani 49 N'luk ağırlığı kaldıramaz ve kaçış tırmanışı
oluşmaz. Gazebo modelinde beş lift-drag yüzeyi var. **Adım 11/12/21/27 ile
aynı sınıf bir ortam farkı** — bu yüzden yasa MATLAB'da tasarlanamaz, yalnızca
SITL'de.

**Denenen tasarım** (`decel_loop.m` + `decelLoop()`): hıza orantılı burun
yukarı, "tırmanıyorsan pitch'i kıs" şeklinde uyarlanabilir bir tavanla
sınırlı. Model-bağımsız seçildi (aero katsayılarından hesaplamak MATLAB/Gazebo/
gerçek araç arasında farklı olurdu — bu projede tam bu sınıf dört kez arızaya
yol açtı).

**SITL sonucu: BAŞARISIZ.** Araç hiç yavaşlamadı (hız ~27 s boyunca
11.7-19.2 m/s'de kaldı) ve sonunda düştü. Log sebebi gösteriyor ve **sebep
pitch yasası değil**:

| t (s) | v_h | pitch | tilt0 | tilt1 |
|---|---|---|---|---|
| 0 | 15.00 | −0.16° | 43.0° | 36.3° |
| 12 | 10.94 | +1.25° | 56.3° | 48.9° |
| 24 | 13.50 | +3.09° | 75.2° | 66.8° |
| 30 | 16.74 | +0.07° | **79.9°** | **68.9°** |

**Frenleme sırasında tilt'ler geriye değil İLERİYE kaçtı** (43° → 80°).
`fx_sp = 0` vermek tilt'leri geri çekmiyor, çünkü Fx çok zayıf bir WLS amacı
(`Ws_Fx = 0.05`) — ve bu ağırlık **bilinçli**: yaw'ı trimleyen diferansiyel
tilt aynı zamanda Fx üretir, Fx'i roll/pitch mertebesinde ağırlıklandırmak
yaw'ı bozar (Adım 7'de denenmiş ve geri alınmıştı). Üstelik gain schedule tilt
büyüdükçe onu **daha ucuz** yapıyor (`wu_tilt` 3.0 → 1.5). Bu arada
uyarlanabilir tavan görevini yapıp pitch komutunu ~0-4°'ye indirdi, yani
geriye hiç frenleme otoritesi kalmadı.

**Çıkarım:** *bir tiltrotor'da geri geçişin frenleme otoritesi **TILT**'tir,
pitch attitude'u değil.* Gerçek bir back-transition tilt'i **doğrudan** komut
etmeli (bir tilt setpoint'i, ya da manevra boyunca `Ws_Fx`'i yukarı planlamak);
tahsisatın bunu `fx_sp`'den çıkarmasını ummak yetmiyor. Bu bir ayar değil,
bilinçli bir tasarım değişikliği — **önerilen sonraki adım budur.**

**Geri alındı.** Çalışmayan bir otomatik mod uçuş kontrolcüsünde bırakılmadı;
Adım 29'un doğrulanmış reddetme kapısına dönüldü ve yeniden doğrulandı:
hover'da devreye alma ✅ (x=0.13, y=0.16, |v| = 0.13 m/s), 10.1 m/s'de
**`pos_hold REFUSED`** ✅, irtifa −7.2…−7.5 m'de sabit, kaçış yok ✅.
`decel_loop.m` ve `decelLoop()` **kullanılmadan, gerekçesiyle birlikte
bırakıldı** (Adım 24'teki `TILT_STICTION_BAND` ile aynı disiplin) — aynı fikir
körlemesine tekrar denenmesin diye.

**Genel ders:** *Bir aktüatörü dolaylı olarak (zayıf bir amaç terimiyle)
sürmeye çalışmak, o aktüatörün gerçekten gereken kontrol otoritesi olduğu
durumda işe yaramaz. `fx_sp` bir tercihti, komut değil — ve tahsisat onu
haklı olarak görmezden geldi.*

---

### Adım 31 / Faz 0 — Tilt kaçışının SEBEBİ ölçüldü: sürücü Fx değil, **irtifa (Fz) kanalı**. Adım 30'un açıklaması eksikmiş (2026-07-29)

Adım 30 doğru bir gözlem yaptı (`fx_sp = 0` tilt'leri geri çekmiyor) ama
bunu yanlış bir nedene bağladı. **Göz ardı edilen bir terim bir aktüatörü
sürmez, sadece sürmez** — tilt'i aktif olarak ileri iten başka bir talep
olmalıydı.

**Yöntem — kontrol yasası denemesi değil, saf ölçüm.** Adım 30'un log'unda
`tiltrotor_indi_status` yok (varsayılan profil onu içermiyor), yani `nu_des`
hiç kaydedilmemiş. Rejim `decelLoop` **olmadan** yeniden üretildi: seyire
çıkıp `fx_sp = 0`, `roll_sp = pitch_sp = 0` bırakmak yeterli — Adım 30'un
giriş koşulu buydu ve burun-yukarı yasası tilt kaçışının sebebi değildi
(rapor tablosu: pitch komutu ~0-4°'ye kısılmıştı, tilt yine kaçtı). Yeni kod
yok, rebuild yok, çarpma yok. `sitl/run_backtrans_probe.py`.

**Olay iki bağımsız uçuşta yeniden üretildi** (Adım 20 kuralı), 15 m/s
girişten 32 s gözlem:

| | uçuş A | uçuş B |
|---|---|---|
| kanat tilt δ0 | 48.0 → 68.1° | 47.9 → 67.7° |
| kanat tilt δ1 | 39.9 → 65.5° | 40.2 → 64.4° |
| **kuyruk tilt δ2** | **0.6 → 0.6°** | **0.5 → 0.6°** |
| yatay hız | 15.0 → 16.0 m/s | 15.1 → 16.0 m/s |
| sürüklenme hızı | +0.721 °/s | +0.693 °/s |

**Atıf — korelasyon değil, doğrudan yeniden çözüm.** WLS tahsisatı her örnek
için çevrimdışı yeniden çözüldü (`sitl/analyze_backtrans_probe.py`,
`effectivenessMatrix` + `wlsAllocate` birebir portu, girdi olarak log'lanan
`u_actual` ve `nu_des`). Model önce doğrulandı: çevrimdışı çözüm log'lanan
`du`'yu tilt kanalında **%5.2 / %9.0 RMS** hatayla yeniden üretiyor, ve ima
ettiği sürüklenme (+0.644 / +0.757 °/s) ölçülenle örtüşüyor. Sonra `nu_des`
kanalları tek tek sıfırlandı:

| senaryo | ima edilen δ̇ (uçuş A) | (uçuş B) |
|---|---|---|
| baseline (tüm talepler) | **+0.644 °/s** | **+0.757 °/s** |
| **Fz talebi sıfır** | **−0.446 °/s** | **−0.346 °/s** |
| Fx talebi sıfır | +0.680 | +0.787 |
| τ_y (pitch) sıfır | +4.584 (kutu doygun) | +4.584 |
| τ_x, τ_z sıfır | +0.622 / +0.753 | +0.759 / +0.729 |
| **yalnız Fz talebi** | **+4.584 (kutu doygun)** | **+4.584** |

**Sonuçlar:**

1. **Sürücü irtifa kanalı.** Fz talebi kaldırıldığında sürüklenme **işaret
   değiştiriyor** — tilt kendiliğinden geri çekilirdi. Ortalama
   `nu_des(Fz) = +2.9 N`, yani irtifa döngüsü sürekli "taşımayı AZALT"
   diyor (15 m/s'de kanat yükün bir kısmını taşıyor). `∂Fz/∂δ = T·sin δ`
   olduğu için bunu yapmanın en ucuz yolu tilt'i ileri almak, ve
   `∂Fz/∂T = −cos δ` δ büyüdükçe sıfıra gittiği için itki dikey eksende
   giderek işlevsizleşiyor. **Yalnız Fz talebi tek başına kutuyu doyuruyor.**
2. **Fx gerçekten atıl.** Talep büyük (`nu_des(Fx) = −10.7 N` ortalama, yani
   "ileri kuvveti 10.7 N azalt") ama sıfırlamak sürüklenmeyi neredeyse hiç
   değiştirmiyor (+0.644 → +0.680). `Ws_Fx = 0.05` vs `Ws_Fz = 20`, amaç
   fonksiyonunda **160.000× ağırlık farkı**. Adım 30'un gözlemi doğru,
   nedeni eksikti.
3. **Pitch momenti hipotezi çürütüldü — ve ters yönde çalışıyor.** τ_y
   sıfırlanınca tilt kutuya kadar fırlıyor, yani kalıcı +0.53 N·m burun-yukarı
   talebi kaçışı **frenliyordu**. Kuyruk tilt'inin 0.6°'de kalması da bunu
   doğruluyor (H2 kuyruğun ileri gitmesini öngörüyordu).
4. **Tasarım için belirleyici sonuç:** `Ws_Fx`'i yükseltmek, ağırlığı 400×
   büyük bir kanalla kavga etmektir — ve o mertebeye çıkarmak Adım 7'de yaw'ı
   bozdu. **Ağırlıkların tartışamayacağı tek şey kutu kısıtıdır**, yani tilt
   tavanı (`abs_hi = δ_ceil − u_actual`). Adım 19'un dersiyle aynı sınıf:
   *artımlı bir tahsisatta bir aktüatör konfigürasyonunu tercihle dayatamazsın,
   amacı ya da kısıtı değiştirmelisin.*
5. **Ve yeni bir kuplaj öngörüsü:** tavan tilt'i aşağı zorladığında irtifa
   döngüsü tercih ettiği "taşıma boşaltma" aktüatörünü **kaybedecek**; geriye
   kalan itki ise yüksek tilt'te zayıf (`cos δ`). Yani geri çekme sırasında
   araç tırmanmaya eğilimli olacak. Dikey eksen politikası bu yüzden isteğe
   bağlı bir ek değil, tasarımın **bağlı bir parçası**.

**Yan bulgu — ölçüm aracında sessiz bir hata (düzeltildi).**
`indi_sitl_common.py`'deki `_ARRAY_RE`, `px4-listener`'ın dizi satırlarına
eklediği `(Roll: … deg)` açıklamasını tolere etmiyordu, bu yüzden `q` alanı
**sessizce düşüyor** ve `attitude_euler_deg()` her çağrıda nan dönüyordu.
İlk deneme uçuşunda bu nan `yaw_sp` olarak yayınlandı ve tüm kontrolcüyü
zehirledi (`nu_des`/`G`/`du` hepsi nan, hiç itki yok, araç yerden kalkmadı) —
px4 log'unda bu bir WLS arızası gibi görünüyor. `_FLOAT_RE` aynı eki zaten
tolere ediyordu. İkisi düzeltildi: regex, ve `set_setpoint()` artık
sonlu-olmayan bir setpoint yayınlamayı **reddediyor**. *Not: bu yol Adım
28/30'un python tarafındaki attitude çıktılarında da nan üretmiş olmalı;
rapordaki pitch değerleri ulog'dan geldiği için etkilenmemiş.*

**Yan bulgu — binary kaynakla eşleşmiyordu (düzeltildi).** Çalışan
`bin/px4`, kaynakta artık bulunmayan debug print'ler içeriyordu ve
`MulticopterIndiTiltrotor.cpp`'nin son düzenlemesinden **4 saniye** sonra
linklenmişti, ki bu bir derleme için imkânsız. `make px4_sitl_default`
modülü gerçekten yeniden derledi. Ölçüm doğrulanmış binary ile tekrarlandı
ve sonuç değişmedi (47.6→70.3 vs 47.7→69.8), yani fark bu ölçüm için
davranışsal değildi — ama bu **varsayılmadı, ölçüldü**. *Genel kural: bir
SITL ölçümünden önce binary'nin kaynaktan yeni olduğunu doğrula.*

### Adım 31 / Faz 1 — Tilt tavanı: mekanizma **ÇALIŞIYOR** ama bedeli yaw ekseni. Terminal hız ile yaw otoritesi **aynı sayıyı** paylaşıyor (2026-07-29)

**Ne eklendi.** `MulticopterIndiTiltrotor.cpp`'ye kanat rotorlerinin tilt'i için
bir **üst kutu sınırı** (`abs_hi = δ_ceil − u_actual`, yalnızca i = 0,1) ve
uçuş içinde süpürmek için `tiltceil <deg>` konsol komutu (`slewbox` deseni,
`px4::atomic<int32_t>` milideg). Varsayılan `TILT_MAX`, yani **davranışsal
olarak nötr** — Adım 22'nin "önce mekanizmayı indir, davranışı sonra"
disiplini. Kuyruk rotoru **bilerek kısıtlanmadı**: Faz 0'da iki uçuşta da
0.5-0.7°'de duruyor ve Fx'e katkısı yok, buna karşılık üçünün en güçlü pitch
aktüatörü.

**Bir tuzak yakalandı ve düzeltildi (uçmadan önce).** Tavan mevcut tilt'in
altına indiğinde `abs_hi` negatif oluyor ve `rate_lo`'nun altına düşebiliyor,
yani kutu **boşalıyor**. Mevcut genel güvenlik satırı (`du_min = fminf(du_min,
du_max)`) bunu `du_min`'i `abs_hi`'ye çekerek çözerdi — kalan tüm farkı **tek
bir tick'te** komut ederek (0.012 rad'lık kutuya karşı −0.1 rad, 8×). O satır,
`abs_hi = TILT_MAX − u ≥ 0` iken kutunun her zaman 0'ı içerdiği varsayımıyla
yazılmıştı. Ters yönde çözüldü: `du_max` `du_min`'e çekiliyor, yani araç tilt
tavanın altına dönene kadar **tam olarak tahsisat slew hızıyla** geri çekilir.
Ölçümle doğrulandı: geri çekme boyunca tilt `|du|` **maksimumu 0.01200 rad** =
kutunun tam kendisi.

**Sonuç: mekanizma çalışıyor.** 15 m/s seyirden, tavan 2.0 °/s ile indirilerek:

| | uçuş 1 | uçuş A | uçuş B |
|---|---|---|---|
| nötrlük kontrolü (tavan 90°) | tilt 47.5 → 52.5° ileri | 50.4° | 52.2° |
| geri çekme sonu | 9.4° | 9.0° | 5.0° |
| irtifa bandı | 0.84 m | 0.34-0.57 m | 0.23-0.29 m |
| itki doyumu | **%0.0** | **%0.0** | **%0.0** |

Nötrlük kontrolünde Faz 0'ın ileri sürüklenmesi aynen göründü, tavan
devreye girince tilt tavanı izledi, **irtifa tutuldu** ve hiçbir fazda itki
kanalı doymadı (kilitlenme yok). **Faz 0'da tırmanma öngörmüştüm — olmadı.**

**Ama bedeli ölçüldü: tavan yaw'ı aç bırakıyor.** Tavan tabanı süpürüldü,
**ikinci uçuşta sıra ters çevrilerek** (Adım 20 kuralı — zaman kayması
karıştırıcısını elemek için):

| taban | uçuş A (9→7→5) | uçuş B (5→7→9) | tilt farkı δ0−δ1 | terminal v_h (A) |
|---|---|---|---|---|
| 9° | +0.0205 rad/s | +0.0069 | 8.7° | 7.49 m/s |
| 7° | +0.0308 | +0.0261 | 6.6° | 6.53 |
| 5° | +0.0384 | +0.0390 | 5.0° | 5.99 |

**Doz-yanıt sırayla birlikte tersine döndü**, yani sürücü taban, geçen süre
değil. Uçuş A'nın 5° fazından sonra araç **kendiliğinden yaw kaçışına girdi**
(10 s'de 610°, ortalama +1.10 rad/s). `pos_hold` bunu tetiklemedi — kapı doğru
çalıştı (`pos_hold REFUSED: 5.9 m/s`).

**Mekanizma, ve Faz 1'in tasarım varsayımımın çürütülmesi.** Tavanı önerirken
"kutu diferansiyel serbestliği korur, çünkü iki kanat da tavanın *altında*
farklı açılarda durabilir" demiştim. **Yanlış.** Her koşuda **δ1 = 0.0°'da,
`TILT_MIN` tabanında çakılı** kalıyor (madde (P)'nin tek yönlü aralığı), yani
diferansiyel **tam olarak tavana eşit**; altında serbestlik yok. Ve δ1 = 0
iken:

```
τ_z(tilt) = −0.25·T0·sin δ0 + 0.25·T1·sin δ1 = −0.25·T0·sin δ0
Fx        =       T0·sin δ0 +       T1·sin δ1 =        T0·sin δ0
⇒  τ_z = −0.25·Fx
```

**Yaw trim torku ile kalan ileri kuvvet aynı fiziksel büyüklüktür.** Terminal
hızı düşürmek için tavanı indirmek, yaw trim otoritesini birebir aynı oranda
kesmek demektir. Bu bir ayar sorunu değil, airframe'in tek yönlü tilt
aralığının doğrudan sonucu — ve Adım 28'in `fx_trim = 2.9 N`'inin neden var
olduğunun da açıklaması (δ0 = 9°'de ölçülen `T0·sin 9° ≈ 2.5 N` ile örtüşüyor).

**Faz 1'in bıraktığı sınır.** Tavan tek başına 15 → **~7.5 m/s** (taban 9°,
yaw kabul edilebilir) getiriyor. Tabanı 5°'ye indirmek ~4-6 m/s'ye indiriyor
ama yaw'ı kaçış eşiğine getiriyor. `POS_ENGAGE_V_MAX = 3 m/s` hâlâ
ulaşılamıyor. **Kalan 7.5 → 3 m/s aralığı tavanla kapatılamaz** — yapısal
olarak gövde pitch'i gerektirir, ki Adım 29 onu ~5-6 m/s ile sınırlamıştı.
Faz 2'nin çözmesi gereken şey tam olarak bu boşluktur, ve bu artık
niceliksel: **7.5 m/s ile 5 m/s arası, iki mekanizmanın da tek başına
kapatamadığı bant.**

**Genel ders:** *Bir kutu kısıtı bir aktüatörü "serbest bırakıyor" görünse
bile, o aktüatörün diğer sınırı zaten bağlıysa serbestlik yoktur. Tavanı
önermeden önce δ1'in `TILT_MIN`'de olduğunu biliyordum (madde (P)) ve yine de
diferansiyelin korunacağını varsaydım — iki bağlı sınır bir serbestlik derecesi
bırakmaz.*

### Adım 31 / Faz 2 ön-ölçümü — ⭐ **GERİ GEÇİŞ UÇTU.** Eksik parça tavanı zamanında **BIRAKMAK**tı (2026-07-29)

Faz 1 geriye 7.5 → 5 m/s'lik kapatılmamış bir bant bırakmıştı. Bu adım onu
ölçtü ve kapattı.

**(a) Gövde pitch'i bu bantta gerçekten frenliyor.** Tavan 9°'de sabit
tutulup (tek değişken pitch olsun diye) `pitch_sp` adımlandı:

| pitch_sp | v_h | vz |
|---|---|---|
| 0° | 5.74 m/s | +0.16 |
| +2° | 3.01 | +0.23 |
| +4° | 0.10 | +0.02 |

**Adım 29'un arızası hıza özgüymüş.** Orada devreye alma 14.5 m/s'de, doymuş
pitch ile ve tilt kaçarken olmuştu; burada araç 7 m/s'den düz ve irtifasını
tutarak giriyor ve 4° burun yukarı onu durduruyor — kaçış tırmanışı yok
(`vz` ±0.23 içinde).

**(b) AMA tavan takılıyken araç yaw'da kaçtı.** Aynı koşuda yaw dönüşü:
pitch +2'de +42.7°, **pitch +4'te +981°**, **pitch +6'da +2117°** (ulog'dan
hızdan ölçüldü; açı örnekleri aliaslanmıştı — ölçüm tuzağı #1). Mekanizma:
taban 9°'de **δ0 tavana, δ1 `TILT_MIN`'e çakılı** — yani diferansiyel yalnızca
küçük değil, **değiştirilemez**. Yaw'ın kontrol otoritesi sıfır, elde yalnızca
sabit bir trim var. Seyirde bunu aero weathervane sönümü maskeliyor
(Adım 20: 11.6 m/s'de 9.1 Nm/rad, 2 m/s'de 0.27) — yavaşlayınca maske kalkıyor
ve madde (Q)'nun rejimine, üstelik aktüatörü kelepçelenmiş hâlde giriliyor.

**(c) Çözüm bir ödün değil: tavanın gerekçesi de yavaşken yok oluyor.** Taban
fazlarında ölçülen `nu_des(Fz) ≈ 0.00` — kanat taşıması bittiği için tavanı
gerektiren Fz kaynaklı tilt kaçışı zaten yok. Yani **tavan yalnızca hızlıyken
gereklidir ve yavaşlarken bırakılmalıdır.**

**(d) Tavan bırakılarak tam dizi, İKİ bağımsız uçuşta:**

| faz | uçuş 1 | uçuş 2 |
|---|---|---|
| geri çekme (tavan açık→9°) | 15 → 8.14 m/s | 15 → 8.27 |
| pitch +2° (tavan BIRAKILDI) | 6.91 → 2.71, dönüş **+0.5°** | 6.16 → 2.86, **−1.9°** |
| pitch +4° | 4.45 → 0.63, dönüş **−0.2°** | 4.32 → 0.18, **+0.7°** |
| **`pos_hold`** | **devrede, 0.23 m/s**, −2.8° | **devrede, 0.11 m/s**, −0.7° |
| irtifa bandı | ≤0.86 m | ≤0.76 m |
| **itki doyumu** | **%0.0** | **%0.00 (tüm uçuş)** |

Tavan kapalıyken faz başına 981-2117° dönen araç, tavan bırakılınca **±2°
içinde** kalıyor. **Bu, hover → seyir → hover'ın SITL'de ilk kez tamamlanmasıdır.**

**(e) Aşırı uygulama uyarısı:** araç durduktan sonra +6° komut etmek `v_h`'yi
0.63 → 5.77 m/s'ye çıkarıyor — burun yukarı artık GERİYE hızlandırıyor. Pitch,
hız sıfıra yaklaşırken bırakılmalı; `pos_hold` bunu zaten yapıyor.

**Faz 2'nin tasarımı artık ölçümle belirlendi** — dört durum, her birinin
eşiği ve mekanizması ölçülmüş:

```
RETRACT   v_h > ~8 m/s : tavan ~2 deg/s ile 9 deg'e iner   (Fz kaçışını kes)
RELEASE   v_h < ~8 m/s : tavan TILT_MAX'e bırakılır         (yaw aktüatörünü geri ver)
BRAKE     v_h < ~8 m/s : sınırlı burun yukarı (+2..+4 deg)  (gövde pitch'i frenler)
HANDOFF   v_h < 3 m/s  : pos_hold devreye                   (mevcut, doğrulanmış)
```

**Yan bulgu — bir koşu KÖR UÇTU (araçta düzeltme yapıldı).** Bir denemede
`px4-listener` / `tiltceil` istemci yolu tümüyle sessizce başarısız oldu: her
okuma nan döndü, tavan komutları hiç gitmedi, araç 24 m/s'ye hızlanıp 44 m
tırmandı ve roll'de kaçtı. Betiğe telemetri-nan iptali eklendi (okuma yoksa
manevraya devam etmek anlamsız). Ayrıca `kill_sitl()`'in `pkill` temizliği bir
kez daha tutmadı ve yetim bir `gz sim` kaldı — PID ile öldürmek gerekti; bu
zaten bilinen bir tuzak, tekrarlandığı için not ediliyor.

**Genel ders:** *Bir kısıt, çözdüğü problem ortadan kalktıktan sonra da
yürürlükte kalırsa saf zarara döner. Tavan hızlıyken hayat kurtarıyor,
yavaşken yaw'ı öldürüyordu — ve iki rejimi ayıran sinyal zaten ölçülüyordu
(`nu_des(Fz)` → 0). Bir kısıt eklerken onu ne zaman KALDIRACAĞINI da tasarla.*

### Adım 31 / Faz 2 — ✅ **GERİ GEÇİŞ BİR KONTROL YASASI OLDU.** Durum makinesi kuruldu, bir kusur uçuşta bulundu ve düzeltildi (2026-07-29)

Faz 2 ön-ölçümü diziyi konsoldan sürerek kanıtlamıştı. Bu adım onu modüle
taşıdı: `backtrans_loop.m` / `backTransition()`, dört durumlu, `bt_enable`
bayrağıyla sürülen bir makine.

**`safe-control-change` sırası izlendi**, tek bilinçli sapmayla: MATLAB bu
manevrayı yapısal olarak üretemediği için (Adım 30'da ölçüldü) MATLAB portu
**yalnızca senkron ve regresyon** içindir. Regresyon nötr çıktı — hover-gust
RMS p/q **0.0014/0.0003**, transition max|ω| **0.0126** — yani kayıtlı
referansla birebir aynı. *Bilinen senkron boşluğu: `sf_wls_alloc.m` /
`tiltrotor_indi.slx` tavan girdisini almadı. Codegen blok arayüzünü
değiştirmek, orada hiç uyarılamayacak bir yasa için gereksiz risk; bilerek
ertelendi ve burada işaretlendi.*

**Uçan ilk otomatik sürüm BAŞARISIZ oldu — ve nedeni öğreticiydi.** RETRACT
çalıştı (13.4 → 8.4 m/s, tilt 9°/0°), ama BRAKE tavanı `TILT_MAX`'e **tamamen
bıraktığı** için Fz kaçışı yeniden başladı: tilt 24 → 90°, hız 6.3 → 12 m/s.
**Manevra kendini bozdu.**

Elle kosular bunu neden yaşamamıştı? Çünkü onlar tavanı **daha geç**, oturmuş
5.7-6.9 m/s'de bırakmıştı; 8.4 m/s'de kanat taşıması ~1.5× daha büyük.
Ama düzeltme yalnızca eşiği düşürmek değil — tabandaki terminal hız 5.7-7.9 m/s
arasında değişiyor, yani düşük bir eşik RETRACT'i kilitleyebilir.

**Asıl teşhis daha derindi ve önceki adımın kendi çıkarımını düzeltti.** Yaw'ı
aç bırakan şey tavanın **varlığı** değil, tavanın **trim değerinde oturup δ0'a
hiç yer bırakmamasıydı**. δ1 zaten `TILT_MIN`'de olduğundan diferansiyel = δ0;
tavan 9°'deyken δ0 da 9°'de kelepçeli, yani modüle edilemiyor. Tavanı trim'in
belirgin üstüne çıkarmak (`BT_BRAKE_CEIL = 20°`) δ0'ı [0, 20°] arasında serbest
bırakır — yaw otoritesi tam geri gelir — ve bir tavanın var olmaya devam etmesi
kaçışı yine de sınırlar. 20° uçuş verisinin desteklediği değer: başarılı elle
koşuda tilt bırakma sonrası geçici 18.8°'ye çıkıp ~10°'ye dönmüştü.

Nihai durum makinesi:

| durum | koşul | tavan | pitch |
|---|---|---|---|
| RETRACT | giriş | mevcut tilt → 9°, 2°/s | 0 |
| BRAKE | tavan tabanda **ve** v_h < 8 | **20°** (bırakma değil, yükseltme) | ≤4°, hızla sönen |
| HANDOFF | v_h < 3 | `TILT_MAX` (artık gerçekten kalkar) | ≤4°, `pos_hold` devralır |

**Sonuç — iki bağımsız otomatik uçuş, elle hiçbir şey sürülmeden:**

| | uçuş 1 | uçuş 2 |
|---|---|---|
| giriş | 13.5 m/s | 14.5 m/s |
| v_h < 1 m/s'ye süre | 41.6 s | 32.9 s |
| son hız | 0.12 m/s | 0.09 m/s |
| yaw toplam dönüş | **−10.1°** | **−8.1°** |
| irtifa bandı | **0.96 m** | 0.95 m |
| itki doyumu / BIG_M | **%0.00 / 0** | **%0.00 / 0** |

Durum geçişleri log'da tam tasarlandığı gibi: `0→1 (v_h=13.5, ceil=36.3°)`,
`1→2 (v_h=8.0, ceil=9.0°)`, `2→3 (v_h=3.0, ceil=20.0°)`, ardından
`pos_hold: holding`. **`bt_enable` dışında hiçbir komut verilmedi.**

**Zorunlu `sitl-lockup-check` GEÇTİ** (`sitl/run_lockup_check.py`, artık
betikli: `yaw_sp` = arm heading, `pos_hold` açık, status log profili kurulu —
üçü de daha önce elle atlanabilen adımlardı). 30 s pencerede: itki `sat_flag`
**%0.00**, BIG_M **0**, itki aralığı 10.39-19.59 N, yaw toplam dönüş −9.59°,
|vz|max 0.530 m/s, irtifa hata RMS **0.065 m**, v_h 0.22 m/s (gerçek durus).

**Yan olay — GUI gösteriminde iniş batırıldı (ölçüm değil, operasyon hatası).**
Kademeli inişin sonunda araç **2.1 m'deyken disarm edildi**; proje belleğindeki
"irtifadayken disarm etme" uyarısı atlandı. Araç düştü, takla attı ve EKF
ıraksadı (z = 6564 m, vz = 355 m/s okudu). O koşunun log'undan alınan
`Wu*=1000000` sayıları (78/44) bu yüzden **geçersizdir** — çarpma sonrası
çöp içerirler. Kriter bu nedenle temiz, enstrumanlı bir koşuyla yeniden
alındı. `run_lockup_check.py` artık inişi yerde bitirip öyle disarm ediyor.

**Genel ders:** *Bir kısıtı kaldırırken de eklerken olduğu kadar dikkatli ol.
"Sorun geçti, kısıtı kaldır" ile "sorun geçti, kısıtı gevşet" aynı şey değil —
burada tam kaldırma manevrayı geri sardı, trim'in üstüne yükseltme ise hem
yaw'ı kurtardı hem kaçışı önledi.*

**B5 kapanmadı, daraldı:** geri geçiş artık var ve otomatik, ama yalnızca
SITL'de doğrulandı. Donanım hâlâ NO-GO.

---

### Adım 32-33 — Kademeli failsafe ve pilot girişi YAZILDI (2026-07-29, belgelenmesi Adım 34'te)

Bu iki adım engelleyici **B1**'e (pilot girişi / failsafe yok) girişti ve
oturum belgeleme yapmadan bitti; aşağısı koddan geri okunarak yazıldı.
Ayrıntılı hâli `HARDWARE_READINESS_CHECKLIST.md` §0/B1'de.

**Adım 32 — kademeli failsafe.** Eskiden `Run()` her bozulmuş girdiye aynı
cevabı veriyordu: `if (!ekf_ok || !att_ok || !lpos_ok) { publishDisarmed(); }`
— yani havada **motorları kesmek**. Üstelik üç ilgisiz sinyalin OR'u idi ve
biri (`lpos`) hiç tazelik kontrolü içermiyordu, dolayısıyla *durmuş* bir EKF
sonsuza kadar sağlıklı görünürken *titreyen* bir EKF motorları kesiyordu.
Yerine dört seviye kondu (`FsLevel`), her biri yalnızca **girdisi kaybolan
döngüyü** kapatıyor: NONE / NO_POS / NO_ALT / RATE_ONLY. Tasarımı mümkün kılan
yapısal olgu: **INDI hız döngüsü yalnızca ω ve ω̇ istiyor**, ikisi de
`vehicle_angular_velocity`'den geliyor — Run()'ı zaten o topic'in callback'i
tetikliyor, yani iç döngünün ihtiyacı tanım gereği mevcut.

> **İleri referans:** bu yapısal olgu doğru, ama yeterli değil. Adım 35
> (2026-07-30) RATE_ONLY'yi **kaldırdı**: iç döngünün girdisi mevcut olmak,
> onunla uçulabildiği anlamına gelmiyor — ölçüldü, gelmiyor. Aşağıdaki dört
> seviyeli anlatım adım 34'e kadar geçerlidir.

Aynı adımda **gerçek bir hata bulundu**: eski tazelik testi
`(now - att.timestamp) < 50_ms` iki `uint64` üzerinde **işaretsiz taşma**
yapıyordu. `vehicle_attitude` bu tick'in gyro örneğinden ~4 ms *ileride*
yayınlanıyor (ölçüldü: yaş = −4000 µs), çıkarma ~1.8e19'a sarıyor, test
başarısız oluyor ve cevap `publishDisarmed()` — **uçuş başına 7-13 kez, havada,
tüm motorlara NaN**. Tek bir 4 ms NaN tick'i aşağıda absorbe edildiği için
hayatta kalınmış, ve hiçbir şey loglamadığı için görünmezdi. Toplama soldan
yazılarak düzeltildi.

**Adım 33 — pilot girişi.** `manual_control_setpoint` → roll/pitch açı komutu
(`MAN_TILT_MAX = POS_TILT_MAX`, aracın hiç uçtuğu tek genlik), yaw **hız**
komutu + heading leash (`MAN_YAW_RATE = 0.35`, `RATE_SP_LIMIT[2] = 0.5`in
%70'i — leash Adım 13'ün ıraksamasının ikinci yarısına karşı), throttle →
tırmanma hızı + z leash, **çubuklar ortada → `pos_hold`** (madde (N) yüzünden
eller-serbest hover'ın *varsayılan* olması gerekiyor), VTOL geçiş anahtarından
`bt_enable` (B5'in "pilot başlatamıyor da iptal edemiyor da" eksiğini kapatır).
Çubuklar bir FlightTask üzerinden değil doğrudan modülde okunuyor, çünkü
airframe `.post` betiği `flight_mode_manager` + `mc_pos_control`'ü durduruyor ve
onları geri açmak Adım 29'un **ölçtüğü** yanlış pozisyon yasasını
(`theta_sp = -atan2(ax_b, g)`) devreye sokardı.

`sitl-lockup-check` bu derlemeyle geçti (itki doyumu %0.00, BIG_M 0, yaw
+2.40°, irtifa RMS 0.068 m) — **ama o koşuda `failsafe_level` %100 sıfırda
kaldı**, yani üç bozulma dalı da ve pilot yolu da hiç çalıştırılmamış koddu.

### Adım 34 — Failsafe dalları ilk kez TETİKLENDİ: 1. seviye çalışıyor, **3. seviye aracı düşürüyor** (2026-07-30)

Yeni araç: `sitl/run_failsafe_test.py` (seviye başına bir uçuş) +
`indi_sitl_common.py`'ye `param_set` / `stop_module` / `lockdown` /
`gz_truth_z`. **Tasarım kararı:** seviyeyi zorlayan bir konsol kancası
(`slewbox`/`tiltceil` tarzı) *kullanılmadı* — kod her döngüde o döngünün
**kendi** girdisinin geçerliliğine bakıyor (`alt_ok`, `pos_ok`), `_fs_level`
yalnızca en-kötü özeti, dolayısıyla seviyeyi zorlamak dal gövdelerini
çalıştırmazdı. Bunun yerine kestirim gerçekten elinden alındı; rebuild
gerekmedi. `gz_truth_z()` şart oldu çünkü **her PX4 irtifa sinyali tam da
bozduğumuz kestirimden geçiyor**.

**Seviye 1 (NO_POS), `EKF2_GPS_CTRL = 2` — ✅ GEÇTİ.** 5 s'de (EKF2_NOAID_TOUT)
fs=1; motorlar kesilmedi (%0.0 NaN), **irtifa korundu** (gz 20.5-21.1 m),
açısal hız baseline'ın *altında* (|ω|max 0.081 vs 0.131), itki 13.1-19.2 N,
%0.0 doyum, 0 BIG_M; geri alındığında ~3 s'de temizlendi. Ayrıştırılmış tanı
mesajı tam olarak tek terimi işaretledi:
`failsafe 1 (att=1[cp=1 fresh=1 tilt=1] alt=1 xy=0 yaw_ref=1)`.

**Yan bulgu — failsafe kendi kendine TOPARLANMIYOR.** Bozulma sırasında yatay
sürüklenme yapısal olarak kaçınılmaz (xy geçersizken pozisyon döngüsü kapanmak
zorunda, ve madde (N) yüzünden yaw trim'i aracı ileri iter): 14 s'de
0.1 → 2.6 m/s. Kestirim geri geldiğinde log şunu yazdı:
`failsafe CLEARED` → hemen ardından `pos_hold REFUSED: 4.8 m/s > 3.0 m/s`.
Yani Adım 29'un kapısı doğru çalışıyor ama duruş geri kazanılamıyor; geri
geçiş (`bt_enable`) toparlanma yoluna bağlanmalı.

**Seviye 3 (RATE_ONLY), `ekf2 stop` — ❌ ARAÇ DÜŞTÜ, ve kodun bir varsayımı
çürütüldü.** Modül doğru davrandı:
`failsafe 3 (att=0[cp=0 fresh=0 tilt=0] alt=0 xy=0) -- degrading, NOT cutting`.
**0.05 s sonra** `actuator_armed.lockdown` true oldu (ulog: ilk fs=3 örneği
t=48.96 s, lockdown t=49.01 s, `actuator_motors` aynı anda %100 NaN) ve araç
35 m'den **serbest düştü** — gz yer gerçeği 34.84 → 0.10 m, ~2.7 s, ki
0.5·9.81·2.7² ≈ 36 m, yani tam serbest düşüş. Bizim `lockdown` komutumuz ~16 s
sonraydı; araç o zaman çoktan yerdeydi.

Sebep: `terminationCommanded()`'ın yorumu "iki nedeni de *komut edilmiş*,
çıkarsanmış değil" diyor — **ikisi de değil**:

```
Commander.cpp:1881             armed.lockdown = (nav_state == NAVIGATION_STATE_TERMINATION)
                                                || HIL || throw_launch
ModeUtil/control_mode.cpp:119  flag_control_termination_enabled = true
                                                <- yine yalnızca nav_state == TERMINATION
```

ve commander oraya **otomatik** giriyor: attitude geçersizken
`FailsafeBase::modeCanRun()` başarısız, yedek mod yok (airframe `.post`
`flight_mode_manager` + `mc_pos_control`'ü durduruyor), failsafe
`Action::Terminate`'e yükseliyor (`Commander.cpp:2337`). Gerçekten insan kararı
olan tek şey `manual_lockdown` (`action_request_s::ACTION_KILL`). Ayrıca:
`gz_bridge` `actuator_armed`'a **hiç** bakmıyor, yani aracı düşüren NaN'ı
modülün kendisi yazdı — karar bizim tarafta, düzeltilebilir.

Sonuç: **RATE_ONLY, kademeli tasarımın var olma sebebi olan seviye, tek bir
4 ms tick'ten uzun hiç çalışmadı.** Düzeltme otomatik sonlandırmayı görmezden
gelmeyi gerektiriyor, yani stok PX4 emniyet davranışından bilinçli bir sapma —
ölçülmeden ve karar verilmeden yapılmadı; üç seçenek ve önce yapılması gereken
MATLAB ölçümü checklist §0/B1'de.

**Seviye 2 (NO_ALT) — ⚠️ TETİKLENEMEDİ, ve muhtemelen ÖLÜ DAL.** İlk deneme
(`EKF2_GPS_CTRL = 4`) GPS 3B hızını açık bıraktı, dikey hız oradan besleniyordu,
18 s boyunca fs=0. İkincisi (baro kapalı + `GPS_CTRL = 1`) tüm dikey yardımı
kesti ama `z_valid` 12 s'de hiç düşmedi (ulog: 10317 örnekte 0 kez).
Yapısal sebep: `EKF2.cpp:1589-1590`'da

```
lpos.z_valid   = isLocalVerticalPositionValid() || isLocalVerticalVelocityValid();
lpos.v_z_valid = isLocalVerticalVelocityValid() || isLocalVerticalPositionValid();
```

**aynı OR ifadesi** — yani modülde `alt_ok ≡ vz_ok` her zaman. İki sonucu var:
(a) NO_ALT ancak attitude sağlamken dikey geçerlilik düşerse oluşur, (b) irtifa
dalındaki orta kol (`else if (vz_ok)` → `altitudeLoopVz`, `FS_DESCENT_VZ` ile
kontrollü alçalma) `!alt_ok` iken **asla** koşamaz; yalnızca RATE_ONLY + geçerli
lpos hâlinde, o da attitude kaybının lpos'tan bağımsız olmasını gerektirir.
`estimator_status_flags` ≥1 Hz yayınlandığı için (`EKF2.cpp:1852`)
`tilt_aligned`'ın 3 s penceresi o kapıyı da kapatıyor — yani orada bir hata
*yok*, ama `FS_DESCENT_VZ`'nin irtifa dalındaki kullanımı ölü görünüyor.

**Testin kendisi de bir yanlış geçiş üretti ve düzeltildi.** İlk sürüm seviye 3
koşusunu **GEÇTİ** saydı: bozulma penceresini `failsafe_level != 0` aralığından
çıkarıyordu, termination `_fs_level`'i NONE'a sıfırladığı için pencere **tek
örneğe** çöktü ve her ölçüt o tek örnekte hesaplandı — araç 35 m düşerken.
Eklenen zorunlu ölçütler: (1) pencere niyet edilen sürenin en az yarısı kadar
olmalı, (2) bozulma **sonlandırmayla öncelenmemiş** olmalı (ön-alma *niyet
edilen* süreye göre ölçülür, gözlenene göre değil — gözlenen pencere lockdown
ile birlikte kapandığı için kendisiyle karşılaştırılamaz), (3) gz yer
gerçeğinden alçalma < 2 m/s.

*Genel ders 1: bir alt sistemin "bozulmuş girdiyle uçmaya devam ederim"
kararı, ÜST sistem aynı bozulmayı görüp sonlandırmaya karar veriyorsa geçersiz
— kendi failsafe'inizi yazarken çevreleyen otoritenin aynı sinyale ne yaptığını
ölçün. Adım 11/12/21/27'nin kontrolcü↔plant arayüz hatalarıyla aynı sınıf, bu
kez arayüz **commander**.*
*Genel ders 2: bir test, ölçüt penceresini ölçtüğü sinyalin kendisinden
çıkarıyorsa, o sinyali sıfırlayan bir arıza pencereyi yok eder ve test sessizce
GEÇER — pencerenin uzunluğunu ayrı bir geçerlilik ölçütü yapın.*

---

### Adım 35 — RATE_ONLY ölçüldü, savı ÇÜRÜTÜLDÜ ve seviye KALDIRILDI (2026-07-30)

Adım 34, `FsLevel::RATE_ONLY`'nin SITL'de hiç çalışamadığını gösterdi ama
**neden** çalışamadığını değil: commander 50 ms'de sonlandırdığı için seviyenin
kendisi hiç sınanmamıştı. Geriye üç seçenek kalmıştı ve seçim, seçenek 1'in
(commander'ın otomatik sonlandırmasını görmezden gelmek) gerekçesine
dayanıyordu: *"modülün hız sönümlemesi kesinlikle daha iyi."* Bu bir
**kontrol-yasası savı**, yani commander'ın olmadığı bir yerde ölçülebilir.

**Ölçüm — `run_rate_only_test.m` (MATLAB).** PX4 dallarının birebir karşılığı
kuruldu, kontrolcüde hiçbir değişiklik gerekmedi:
`att_sp = att` ⇔ `omega_sp = 0` (çünkü `indi_attitude_controller.m:35-49`
`omega_sp = Kp_att·(att_sp − att)` hesaplıyor), irtifa döngüsü kapalı ve
`Fz = −m·g·FS_FZ_OPENLOOP`, `fx_sp = 0`. Bozulma 35 m'de, t = 4 s'de — adım
34'ün SITL koşusuyla aynı irtifa. Üç senaryo, çünkü savın kendisi ("hangi
yatışta ise onu korur") başlangıç koşuluna bağlı ve bu projede tek senaryodan
genelleme defalarca yanlış çıktı (Adım 20'nin dersi):

| senaryo | >30° yatış | >90° | max \|roll\|/\|pitch\| | yaw savrulma | ort/max alçalma | **çarpma** |
|---|---|---|---|---|---|---|
| 1) temiz hover | 45.3 s (18.8 m) | 47.4 s | 179.6° / 76.4° | +287.2° | 0.71 / 12.37 m/s | 11.66 m/s |
| 2) gust + roll bozucusu | 12.2 s (33.2 m) | 13.8 s | 154.2° / 62.0° | +207.7° | 2.13 / 23.66 m/s | 23.66 m/s |
| 3) 10° yatık girildi | 1.7 s (34.9 m) | 22.4 s | 122.1° / 77.9° | +391.0° | 1.49 / 19.57 m/s | 19.57 m/s |

Referans, alternatifin kendisi: **35 m'den serbest düşüş = 2.67 s, 26.2 m/s.**
Süreler bozulma anından itibaren. Grafik: `rate_only_test.png`.

**Sonuç: sav yanlış.** Üç senaryonun üçü de ters döndü ve yere çarptı. "Motorları
kesmekten kesinlikle daha iyi" iddiası nicel olarak sınırda (11.7 vs 26.2 m/s),
nitel olarak yanlış — bu kurtarılmış bir uçuş değil, biraz yavaşlatılmış bir
düşüş. **En temiz senaryo bile kurtarmıyor:** bozucu yok, düz girildi, ve yine
179.6° roll. Yani başarısızlık bozucuya ya da kötü bir başlangıca bağlı değil,
yasanın kendisinde.

**Mekanizma** adım 29/30'un bulduğuyla aynı ve burada dördüncü kez görülüyor:
**hız sönümlemesi duruş tutmak değildir.** Artık torklar duruşa sınırsız
integre olur (sönümleme yalnızca türevi cezalandırır, konumu değil); araç
yattıkça açık çevrim Fz yer çekimine karşı koymayı bırakır; hız arttıkça kanat
momentleri devreye girer ve süreç hızlanır. Senaryo 3'ün 1.7 s'de 30°'yi geçip
90°'ye 22.4 s'de varması da bunu doğruluyor — başlangıçtaki 10° korunmuyor,
**büyüyor**.

**Ayrım: tasarımın bir yarısı ÇALIŞTI.** `FS_FZ_OPENLOOP = 0.97` temiz senaryoda
49.5 s boyunca ortalama **0.71 m/s** alçalma verdi, yani irtifayı neredeyse
tuttu. Başarısız olan `omega_sp = 0`. Açık çevrim itki "z gitti"nin iyi bir
cevabı; hız sönümlemesi "duruş gitti"nin cevabı değil. Bu ayrım önemli, çünkü
seviyeyi tümden atmak yerine hangi parçasının korunacağını söylüyor: sabit
kaldı, `!alt_ok` cevabı olarak yaşamaya devam ediyor.

**Uygulanan değişiklik — seçenek 2 (dürüstçe geri çekme).**
- `FsLevel` = NONE / NO_POS / NO_ALT. **Değer 3 bir daha kullanılmayacak**:
  adım 35 öncesi loglarda "duruş gitti ama hâlâ uçuyor" demek. `.msg`'de de
  böyle işaretlendi.
- Duruş kestirimi artık kademelendirilebilir bir girdi değil, **sert ön koşul**.
  `!att_ok` → tek seferlik `attitude LOST` hatası, `resetState()`,
  `publishDisarmed()`, return. `resetState()` şart: bu return LESO güncellemesini
  ve INDI artımını atlıyor, yani `_u_actual`/`_prev_u_leso`/integraller
  **donardı** ve duruş geri gelseydi artımlı yasa gerçeği tarif etmeyen bir
  gölgeden devam ederdi — bu projenin bedelini zaten ödediği bayat-gölge tuzağı.
- `terminationCommanded()`'ın "iki nedeni de komut edilmiş" yorumu düzeltildi.
- Dış attitude P döngüsündeki `if (_fs_level != RATE_ONLY)` koşulu kalktı;
  döngü koşulsuz.
- `altitudeLoopVz` artık **kanıtlanabilir biçimde ulaşılamaz** (tek girişi
  RATE_ONLY'ydi; `EKF2.cpp:1589-1590` yüzünden `alt_ok ≡ vz_ok`). Silinmedi,
  gerekçesiyle bırakıldı — `decelLoop` / `TILT_STICTION_BAND` disiplini.

**Bedeli açıkça yazıyorum:** artık *geçici* bir duruş boşluğu (>50 ms, sonra
düzelecek olan) da çıkışı kesiyor; eskiden hız sönümlemeli bir köprü alırdı.
Kabul edildi çünkü (a) commander aynı sinyalle ~50 ms sonra zaten sonlandırıyor
— modül onunla yarışmıyor, aynı fikirde olduğunu söylüyor, (b) ölçüm o
köprünün bir yere çıkmadığını gösteriyor. Donanımda gerçek EKF boşluklarının
sıklığı/uzunluğu bilinmiyor; kontrol listesine ayrı bir kalem olarak yazıldı.

**SITL doğrulaması.** `run_failsafe_test.py --level 3` yeniden yazıldı: artık
bir failsafe seviyesi değil, bir senaryo ("duruş kaybı") ve ölçütleri tersine
çevrildi. Ölçülen (ulog `06_38_47.ulg`):

| ölçüt | sonuç |
|---|---|
| son `vehicle_attitude` örneği | 48.832 s |
| modülün kesmesi (ilk tam-NaN) | 48.868 s → **36 ms gecikme** |
| commander `lockdown` | 48.93 s → kesmeden **64 ms SONRA** |
| kesmeden sonra NaN | %100 (kesme tam, titremiyor) |
| arm→enjeksiyon arası havada NaN | **%0.000** (11226 örnek) — adım 32 taşma regresyonu |
| `failsafe_level = 3` görüldü mü | hayır |

Yani karar gerçekten modülün: commander'ı beklemiyor. Ve seviye 1 (NO_POS)
yeniden koşuldu, **hâlâ GEÇİYOR** (fs=1, %0.0 NaN, gz alçalma 0.07 m/s, 0
BIG_M, |ω|max 0.123, geri alındığında temizlendi) — kademeli failsafe'in
kalan savı ayakta. Zorunlu `sitl-lockup-check` de geçti: itki sat %0.00,
BIG_M 0, itki 10.33-19.43 N, yaw +4.06°, |vz|max 0.517, irtifa RMS 0.059 m —
adım 31 taban çizgisiyle denk.

**Ölçüm tuzağı (kendi analiz betiğimde).** İlk `analyze_att_loss` sürümü duruş
kaybı anını "en büyük yayın boşluğu" diye arıyordu ve `nan` döndü: `ekf2 stop`
sonrası `vehicle_attitude` bir daha **hiç** yayın yapmıyor, yani seride bir
boşluk oluşmuyor — seri **bitiyor**. Doğrusu son örnek. Ayrıca NaN oranını
ölçerken arm'dan öncesini dışlamak gerekti; disarmed'ken `publishDisarmed()`
zaten sürekli NaN yazıyor ve bu doğru davranış.

*Genel ders: bir emniyet mekanizmasının GEREKÇESİ de bir iddiadır ve
ölçülebilir. "X, Y'den kesinlikle daha iyidir" cümlesi kodda yorum olarak
durduğu sürece sınanmamış bir varsayımdır — ve burada yanlış çıktı. Bir
mekanizmayı savunmadan önce onun neyi başardığını ölçün; ölçüm mekanizmanın
hangi yarısının işe yaradığını da söyler (burada dikey politika kaldı,
hız sönümlemesi gitti).*

---

### Adım 36 — Seviye 2 / `altitudeLoopVz` kararı: ölü KOL kaldırıldı, ve adım 34'ün iki hükmü de düzeltildi (2026-07-30)

Adım 35'ten geriye tek açık failsafe kalemi kalmıştı: `altitudeLoopVz` ölü mü,
ulaşılabilir mi kılınsın yoksa kaldırılsın mı. Karar vermeden önce iddiaların
ikisi de kaynaktan doğrulandı — ve **ikisi de yanlış çıktı.**

**Düzeltme 1 — `altitudeLoopVz` ölü kod DEĞİL.** `altitudeLoop()` onu her
`ALT_TS` çevriminde çağırıyor (`TiltrotorIndiControl.hpp:334`), yani her uçuşta
nominal yolda koşuyor. Ölü olan fonksiyon değil, **ikinci çağıranıydı**:
`else if (vz_ok)` failsafe kolu. Adım 34 bunları birbirine karıştırmış.

**Düzeltme 2 — NO_ALT ulaşılamaz DEĞİL.** Adım 34 iki denemeden sonra
"tetiklenemedi" demişti. `fs=2` ulog'da **görüldü** (`07_05_51.ulg`, seviye
dizisi `[0, 1, 2]`). Doğru enjeksiyon `EKF2_GPS_CTRL = 0`: **tüm** yardımın
kesilmesi. Adım 34'ün iki denemesi de en az bir kaynağı ayakta bırakıyordu.

**Ölçüm — `probe_no_alt.py`.** Adım 34 iki kez tahmin edip iki kez tutturamadı,
üstelik hangi kaynağın ayakta kaldığını hiç görmeden. Bu probe önce EKF2'nin
dikey geçerlilik mantığını kaynaktan çıkarıyor:

```
z_valid = isLocalVerticalPositionValid() || isLocalVerticalVelocityValid()
isLocalVertical{Position,Velocity}Valid() = !{...}_deadreckon_exceeded && !fake_hgt
=> z_valid FALSE  <=>  fake_hgt  VEYA  vvel_deadreckon_exceeded
```

sonra `estimator_status_flags`'ı örnekleyerek hangi kaynağın füzyonda olduğunu
**görüyor**. Üç konfigürasyon:

| aşama | param | `z_valid` | EKF `z` | gz yer gerçeği | fs |
|---|---|---|---|---|---|
| A | `BARO_CTRL=0`, `GPS_CTRL=1` | **TRUE** (22 s) | −20.04 **sabit** | 19.95 → **10.15 m** | 0 |
| C | A + `HGT_REF=2` (var olmayan kaynak) | **TRUE** (21 s) | −20.39 **sabit** | 20.01 → **17.89 m** | 0 |
| B | `BARO_CTRL=0`, `GPS_CTRL=0` | **false** (anında) | — | düşüş | **2** |

**Sonuç: "z geçersiz, xy geçerli" durumu PX4 param yüzeyinden ÜRETİLEMİYOR.**
Yükseklik yardımı kesilip yatay yardım devam ederse EKF2 dikey kestirimi geçerli
ilan etmeye devam ediyor; `fake_hgt`'e ancak yatay yardım da kesilince düşüyor.

**Karar: ölü kol KALDIRILDI, ulaşılabilir kılınmadı.** Kolun ihtiyaç duyduğu
durum üretilemiyor, ve sebebi bizim tarafımızda değil: `EKF2.cpp:1588-1590`
`z_valid` ile `v_z_valid`'i **tek bir OR**'dan türetiyor, üstelik *"bazı
tüketiciler ikisinin farklı olmasını doğru işlemiyor"* diyen bir TODO ile. Kolu
ulaşılabilir kılmak, kestirimcinin **açıkça yapmayı reddettiği** bir ayrımı
uydurmak olurdu — yani sınanmamış bir failsafe varsayımı, ki adım 35 tam olarak
onun bedelini ölçtü. `decelLoop`'tan farkı da bu: `decelLoop` **ölçülmüş
olumsuz bir sonucu** kaydediyor (tekrarlanmasın diye saklanır), bu kol ise hiç
sınanmamış bir varsayımdı. `FS_DESCENT_VZ` kaldı; tek kullanıcısı artık **link
kaybı** yolu.

**Seviye 2'nin gerçek tavanı ölçüldü.** NO_ALT'a yalnızca tüm yardımı keserek
varılıyor, o da xy'yi götürüyor, o da commander'ı TERMINATION'a sokuyor:
**fs=2 penceresi 0.02 s**, `lockdown` 20 ms sonra. **RATE_ONLY'yi öldüren
yapısal tavanın aynısı** — yani kademeli failsafe'in ölçülerek doğrulanmış tek
seviyesi hâlâ yalnızca NO_POS. `--level 2` ölçütleri buna göre modülün kontrol
ettiği şeye indirgendi (seviye raporlandı mı ✅, o sırada motorları kesmedi mi
✅, BIG_M yok ✅); commander'ın sonlandırması başarısızlık değil, kaydedilen bir
tavan.

**⚠️ ASIL BULGU, ve sorulan sorudan daha ciddi: kısmi yükseklik kaybında
kestirim SESSİZCE YANLIŞ.** A ve C aşamalarında EKF'in `z`'si **donuyor ve
`z_valid` TRUE kalıyor**, oysa araç gerçekten alçalıyor — A'da **10 m**, 22 s
boyunca, hiçbir failsafe tetiklenmeden. **Modülün geçerlilik kapısı bunu
yapısal olarak göremez**: `z_valid` de tazelik testi de "sinyal var ve taze"
diyor; yanlış olan **değerin kendisi**. İrtifa döngüsü bir kurguyu uçuruyor.
Bu, adım 32'nin düzelttiği "durmuş kestirim sağlıklı görünüyor" hatasının bir
üst katmanı: orada sinyal **bayattı**, burada **taze ve yanlış**. Donanımdaki
muadilleri baro tıkanması ve GPS yükseklik kaybı. Kontrol listesine yeni bir
kalem olarak yazıldı (B1-a): bağımsız bir yükseklik kaynağıyla çapraz kontrol
gerekiyor — modülde şu an **hiçbir tutarlılık kontrolü yok**, yalnızca
geçerlilik bayrağı ve tazelik.

**Ölçüm tuzağı — PX4 SITL param'ları KALICI.** Probe'un enjeksiyonu
`rootfs/parameters.bson`'a yazıldı ve bir sonraki koşuya sızdı: SITL
"Preflight Fail: Yaw estimate error" ile 3 launch denemesi boyunca hiç açılmadı.
Kurtarma `rm rootfs/parameters*.bson`; kalıcı çözüm probe'un `finally`'sinde
param geri alma. **Kestirimci param'ı değiştiren her betik onu geri almak
zorunda.**

**Regresyon.** Zorunlu `sitl-lockup-check` değişiklikten sonra tekrar geçti
(`07_19_31.ulg`): itki sat %0.00, BIG_M 0, itki 10.05-19.56 N, yaw **−4.09°**,
|vz|max 0.502, irtifa RMS 0.182 m, v_h 0.15 m/s.

> **Bu koşu iki kez yapıldı ve BİRİNCİSİ GEÇERSİZDİ — kendi tuzağıma düştüm.**
> İlk lockup koşusu (`07_11_04.ulg`) probe'un geri alamadığı param'larla, yani
> `BARO_CTRL=0, GPS_CTRL=1, HGT_REF=2` ile koştu ve "GEÇTİ" dedi: itki sat %0.00,
> yaw +0.07°, irtifa RMS **0.040 m**. Ama irtifa RMS'i EKF'in `z`'sinden
> hesaplanıyor ve bu tam da yukarıda ölçtüğüm **donmuş z** konfigürasyonu —
> yani "mükemmel irtifa tutuşu" sayısı, sabit bir sayının kendisiyle
> karşılaştırılmasıydı. Temiz ortamdaki gerçek değer 0.182 m. **Ders, kendi
> bulgumun aynısı: `z_valid` doğruluk garantisi değil, ve bir kestirimi bozan
> her koşudan sonra ortamın gerçekten temiz olduğu DOĞRULANMALI.**
>
> İkinci tuzak aynı zincirde: `parameters.bson` silindikten sonraki **İLK**
> koşu da geçersiz çıktı (`07_15_58.ulg`). Kalibrasyon/manyetik sapma
> param'ları sıfırdan öğrenildiği için EKF geç yakınsadı, araç arm sonrası
> 3 m/s'yi aştı, `pos_hold` REDDETTİ ve test bir **seyir** koşusuna dönüştü:
> baseline `v_h` 5.8 m/s, itki doygunluğu %16, BIG_M 949. Aynı test bir sonraki
> boot'ta temiz geçti (`07_17_54.ulg`, baseline `v_h` 0.1 m/s, BIG_M 0).
> **Param dosyasını sildikten sonra ilk koşuyu ısınma sayın, ölçüm saymayın.**

*Genel ders: "ulaşılamaz" ve "ölü" iki ayrı iddiadır ve ikisi de doğrulanmadan
kayda geçmemeli — burada biri (fonksiyon ölü) yanlıştı, diğeri (seviye
ulaşılamaz) da yanlıştı, ve ikisini düzeltmek üçüncü ve en ciddi bulguyu
ortaya çıkardı. Bir dalın neden çalışmadığını ararken KAYNAĞI okuyun ve
çevreleyen sistemi ÖRNEKLEYIN; tahmin etmek adım 34'te iki uçuş, adım 36'da
sıfır uçuş harcadı.*

---

### Adım 37 — Geri geçiş GUI'li yeniden koşuldu: **manevra devir hızının ÜSTÜNDE takılıyor.** Adım 31'in "doğrulandı" sonucu MARJİNALMİŞ (2026-07-30)

Amaç yeni bir özellik değildi: mevcut tilt geçişlerini Gazebo arayüzü açık
şekilde yeniden koşup doğrulamak. Bu, adım 29'un dersini tekrarladı —
**senaryo çeşitliliği kadar, aynı senaryoyu TEKRAR koşmak da kusur buluyor.**

**(a) Önce altyapı: GUI artık tek anahtar, ölçütler artık makineyle.**
`INDI_SITL_GUI=1` → `launch_sitl()` `HEADLESS` vermez, `DISPLAY`i geçirir ve
`gz_follow.sh`'i kendisi çağırır (kamera kilidi bu araçta zorunlu — yatay
sürüklenme). Uçuş birebir aynı; yalnızca render ekleniyor.

Daha önemlisi: `run_backtrans_test.py`'nin docstring'i **beş** ölçüt sayıyordu
ama koşu sırasında yalnızca üçü görülebiliyordu (v_h, irtifa bandı, konsolda
`pos_hold`); yaw toplam dönüşü ve itki doyumu elle bakılıyordu. Yeni
`sitl/analyze_backtrans.py` beşini de ulog'dan hesaplıyor ve **pencereyi
ölçtüğü sinyalden bağımsız** bir sinyalden (`bt_state`) kuruyor — adım 34(e)'nin
tuzağının tekrarlanmaması için. Doğrulaması: adım 31'in üç uçuşunda çalıştırıldı,
kayıtlı sayıları yeniden üretti (13.5 m/s → 0.26, bant 0.96, yaw −10.8;
14.5 → 0.14, bant 0.95) **ve bilinen BAŞARISIZ ilk otomatik uçuşu doğru şekilde
"KALDI" verdi** — yani ayırt ediyor.

**Telemetri eklendi (`TiltrotorIndiStatus.msg`): `bt_state` + `bt_tilt_ceil`.**
Kontrol yoluna girmiyor. Gerekçesi ölçüm: durum geçişleri yalnızca zaman damgası
olmayan `PX4_INFO` satırlarındaydı, yani bir test RETRACT'ın nerede bitip
BRAKE'in nerede başladığını **tilt izinden ÇIKARSAMAK** zorundaydı; tavan ise
manevranın gerçek aktüatörü (WLS'e kutu sınırı olarak giriyor, amaç terimi
olarak değil), o yüzden tavansız bir log "tilt tavanı mı izledi, tavan tilti mi"
sorusunu cevaplayamaz. Adım 31/Faz 0'da `nu_des` loglanmadığı için bir uçuş
harcanmıştı — aynı disiplin.

**(b) Bulgu: iki bağımsız uçuşta manevra TAMAMLANMADI.** Girişler 14.09 ve
13.82 m/s, dizinin ilk yarısı kusursuz (RETRACT 25.8/21.0 s, tavan 41.6/41.1 →
9.0 deg, BRAKE'e 8.00 m/s'de geçiş), sonra **v_h 3.2-3.5 m/s'de KARARLI bir
dengede kilitlendi ve 90+ s boyunca orada kaldı** — `BT_HANDOFF_V = 3.0`'a hiç
inemedi, dolayısıyla `pos_hold` hiç istenmedi. Diğer her şey temizdi: itki
doyumu %0.00, 0 BIG_M, irtifa bandı 0.93-0.95 m, yaw −14.5/−14.6 deg, havada
NaN çıkış 0.

| uçuş | giriş | BRAKE süresi | son v_h | sonuç |
|---|---|---|---|---|
| 10_46_58 | 14.09 m/s | 94.5 s | 3.54 ± 0.04 | ❌ HANDOFF yok |
| 10_51_38 | 13.82 m/s | 99.1 s | 3.64 | ❌ HANDOFF yok |

**(c) Mekanizma — tahmin değil, plato üzerinde kuvvet dengesi.**
40 s'lik platoda ölçüldü: `delta1 = delta2 = 0.00 deg` (madde (P), tek yönlü
tilt aralığı), `delta0 = 10.47 deg` ve tilt'ten doğan **ileri kuvvet 3.13 N**
(`nu_des(Fx) = −3.13 N`, yani tahsisata "bunu iptal et" deniyor ve
yapısal olarak edemiyor — adım 28'in `fx_trim = 2.9 N` bulgusunun aynısı).
Fren yasası ise `pitch = 4 deg · v_h / 5`, yani 3.5 m/s'de yalnızca
**2.83 deg = 2.42 N geri**. Aradaki 0.70 N'u aerodinamik sürüklenme
dengeliyor → tam bir denge noktası.

Yani **frenleme otoritesi hız düştükçe azalırken, yenmesi gereken kuvvet
sabit kalıyor.** Sadece "durabilmek" için gereken burun yukarı açısı
`asin(3.13/(m·g)) = 3.66 deg` — `BT_PITCH_MAX = 4.0 deg`'nin hemen altında.
Yasa 3.0 m/s'de 2.05 N veriyor, gereken ~3.1 N. Manevra devir hızına ancak
sürüklenme sayesinde inebilir.

**(d) Bu, adım 31'in sonucunu da düzeltiyor.** O iki uçuş gerçekten tamamlandı,
ama **ancak kıl payı**: aynı ölçüm betiği o loglara uygulandığında biri son
0.5 m/s'yi **12.1 s**'de, diğeri 2.7 s'de geçmiş. Yani "doğrulandı" diye
kaydedilen manevra marjinal bir manevraydı ve bugün üçüncü/dördüncü uçuşta
düştüğü tarafa düştü. **Adım 20'nin dersi (marjinal kararlı bir sistemde tek
uçuştan sonuç çıkarma) buraya da aynen uyuyor — ve bu kez dersi kendi
sonucumuza uygulamak gerekti.**

**(e) Düzeltme: `BT_BRAKE_V_FULL` 5.0 → `BT_HANDOFF_V` (3.0), sayı olarak
değil BAĞIMLILIK olarak.** Sönceleme yasasının kendisi doğru (durmuş araca
+6 deg onu GERİYE 5.77 m/s'ye hızlandırıyor, adım 31/Faz 2). Yanlış olan
NEREDE söncelendiği: tam güç frenleme **en az devir hızına kadar** sürmeli,
çünkü onun altında zaten `pos_hold` pitch'in sahibi oluyor. İki sabitin
birbirinden ayrı sürüklenememesi için kodda bağımlılık olarak yazıldı
(adım 28'in dersi: bağımlılığı yoruma değil yapıya yaz).
`safe-control-change` sırasıyla: MATLAB (`p.bt.brake_v_full = p.bt.handoff_v`)
→ regresyon **tam nötr** (RMS p/q 0.0014/0.0003, transition max|ω| 0.0126 =
kayıtlı taban çizgisi) → `sf_*.m`/`.slx` **etkilenmiyor** (bt sabitleri codegen
yolunda yok, doğrulandı) → PX4 → SITL.

**(f) İlk düzeltme YETMEDİ — ve nedeni ikinci uçuşta ölçüldü.** İki doğrulama
uçuşundan biri geçti (10_56_56: 15.33 m/s → 0.09, BRAKE 16.1 s, beş ölçüt de
✅), **diğeri yine takıldı — bu kez 4.9 m/s'de** (10_59_08, BRAKE 92.0 s).
Yani düzeltme eşiği kaydırdı, sorunu kaldırmadı. **İki uçuş kuralı burada
kendini ödedi: tek uçuşla "çözüldü" denecekti.**

Platoların karşılaştırması sebebi doğrudan gösteriyor:

| uçuş | BRAKE | δ0 | tilt'ten ileri kuvvet | fren pitch'i | sonuç |
|---|---|---|---|---|---|
| 10_56_56 ✅ | 16.1 s | 13.19° | 3.30 N | +3.15° = 2.70 N | tamamlandı |
| 10_59_08 ❌ | 92.0 s | 14.86° | **4.11 N** | +4.00° = **3.42 N** | 4.9 m/s'de takıldı |

**`BT_PITCH_MAX = 4 deg` (3.42 N), yapısal ileri kuvvetin dağılımının (3.1-4.1 N)
TAM İÇİNDE.** Yani otorite bozucuyla aynı mertebede: bazı uçuşlarda yetiyor,
bazılarında yetmiyor. `BT_BRAKE_V_FULL` düzeltmesi gerekliydi ama yeterli değildi.

**(g) Asıl düzeltme: fren yasası AYRIŞTIRILDI — "yerinde durma trimi + frenleme
payı".** Fizik zaten böyle: δ1/δ2 tabanda çakılı olduğu için yaw trimi kalıcı
bir ileri itki üretiyor, dolayısıyla *durmak* bile bir burun yukarı açısı
istiyor. Yeni yasa

```
pitch_sp = BT_TRIM_PITCH + BT_PITCH_MAX · min(1, v_h / BT_BRAKE_V_FULL)
           └ asin(FX_TRIM/(m·g)) = 3.39°, SÖNMEZ      └ 4°, sönen frenleme payı
```

Üç kazanç: (1) toplam otorite 6.31 N, ölçülen en kötü 4.11 N'un 1.53 katı;
(2) `v_h → 0`'da yasa 0'a değil 3.39°'ye iniyor — yani araç yeniden ileri
kaçmıyor, ve "durmuş araca +6 deg GERİYE hızlandırır" ölçümüyle çelişmiyor
(bu yasa durmuş araca 6° vermiyor); (3) ilk terim `FX_TRIM`'e bağlı olduğu
için **gerçek araçta `fx_trim` yeniden ölçüldüğünde yasa kendiliğinden
düzeliyor** (donanım kontrol listesinde zaten açık bir madde).

**(h) Doğrulama — iki bağımsız uçuş, ikisi de tam dizi:**

| uçuş | giriş | RETRACT | BRAKE | HANDOFF | son v_h | bant | yaw | doyum / BIG_M |
|---|---|---|---|---|---|---|---|---|
| 11_06_40 | 15.38 m/s | 21.9 s | **6.1 s** | 22.6 s | 0.13 m/s | 1.14 m | −3.4° | %0.00 / 0 |
| 11_08_37 | 14.66 m/s | 25.1 s | **5.7 s** | 22.5 s | 0.07 m/s | 1.11 m | +1.1° | %0.00 / 0 |

BRAKE süresi 92-99 s'lik takılmadan **5.7-6.1 s**'ye indi (adım 31'in en iyi
uçuşunda bu evre ~12-25 s sürüyordu). Bedeli ölçüldü ve kabul edildi: irtifa
bandı 0.66-0.95 m'den **1.11-1.14 m**'ye çıktı (ölçüt 3.0 m) — daha büyük burun
yukarı açısının beklenen etkisi. Yaw dönüşü **−3.4/+1.1 deg**, yani önceki
−9…−14.6 deg'den daha da iyi. Havada NaN çıkış 0 (bir uçuşta arm anında bir
tick — `actuator_armed` ile modülün `flag_armed`'ı aynı tick'te değişmiyor;
ölçüt bunu ayrı raporluyor).

**Zorunlu `sitl-lockup-check` GEÇTİ** (11_11_29, GUI'li): itki doyumu %0.00,
BIG_M 0, itki 10.30-19.31 N, yaw toplam −0.20°, |vz|max 0.185 m/s, irtifa RMS
**0.085 m**, v_h 0.14 m/s. Sabahki taban çizgisiyle (10_38_06: yaw −1.88°,
|vz| 0.514, RMS 0.170) karşılaştırıldığında hepsi eşit ya da daha iyi —
beklenen, çünkü fren yasası yalnızca BRAKE/HANDOFF'ta koşuyor, hover'a hiç
dokunmuyor.

**(i) 🔶 AYNI HASTALIK BİR EVRE ÖNCE — YENİ AÇIK MADDE (R): `BT_RELEASE_V`
RETRACT'ın TERMİNAL HIZININ ÜSTÜNDE DEĞİL.** Fren düzeltmesi doğrulandıktan
sonraki bir teyit uçuşunda (11_16_48) manevra yine tamamlanmadı, ama **başka
bir evrede ve başka bir sebeple**: RETRACT **111.4 s** sürdü (diğer bütün
uçuşlarda 21.9-28.2 s). Sebep ölçüldü — tavan 22.3 s'de 9°'ye indi, sonra araç
oraya **asimptotik olarak yaklaştı**:

| RETRACT içi t | 22.3 s | 39.0 s | 55.7 s | 78.0 s | 111.4 s |
|---|---|---|---|---|---|
| v_h | 9.50 | 8.62 | 8.29 | 8.19 | **8.00** |

9° tavanda, `pitch = 0` (RETRACT'ta bilerek — adım 29'un yüksek hız burun
yukarı = tırmanış tehlikesi) ve `Fx ≈ 2.4 N` ile **terminal hız ≈ 8.0-8.6 m/s**,
yani `BT_RELEASE_V = 8.0`'ın ta kendisi. Eşik bir dengenin sınırına konmuş
durumda: geçiş 22 s'de de olabiliyor, 90+ s'de de. (Parametre yorumu terminal
hızı "5.7-7.9 m/s" diye kaydediyor; bugünün ölçümü 8.0+ olabildiğini gösteriyor.)

İlk bakışta bu yalnızca bir yavaşlık gibi göründü — aynı uçuşta BRAKE'e
geçildiğinde araç 8.7 s'de 8.00 → 3.28 m/s yavaşladı (fren düzeltmesi
çalışıyor), yalnızca testin 120 s'lik bütçesi bitmişti. **Bütçe 200 s'ye
çıkarılıp tekrarlandığında madde (R) BUNDAN DAHA CİDDİ çıktı** (11_20_38):
RETRACT **200.3 s boyunca hiç çıkmadı**, araç **8.97 m/s'nin altına inmedi**
ve — asıl önemlisi — **yaw toplam dönüşü +117.7° oldu** (ölçüt ≤45°).

Yaw'ın kaçması sürpriz değil, adım 31/Faz 2'nin ölçtüğü mekanizmanın ta
kendisi: tavan **trim değerinde** (9°) otururken δ1 zaten `TILT_MIN`'de olduğu
için diferansiyel δ0'a eşit ve δ0 tavana çakılı — yani **yaw'ın modülasyon
otoritesi sıfır**, yalnızca sabit bir trim var. Kısa süre dayanılabilir; 200 s
dayanılamaz. **Yani RETRACT'ın çıkış koşulu sağlanmazsa araç, yaw'ı aç bırakan
konfigürasyonda SINIRSIZ SÜRE kalıyor.** Terminal hız uçuştan uçuşa 8.0-9.3 m/s
arasında değişiyor (ölçüldü) ve `BT_RELEASE_V = 8.0` bu aralığın ALTINDA —
demek ki tamamlanan uçuşlar eşiği **ilk yavaşlama geçicisinde** geçmiş, dengeye
oturduktan sonra değil.

**Şimdilik yapılan, bilinçli olarak yalnızca ÖLÇÜM tarafı:** `BT_TIMEOUT_S`
120 → 200 s (bir kontrol arızasını yavaşlıktan ayırt edebilmek için; ölçüt
süresi, ölçülen manevranın en yavaş meşru hâlinden kısaysa test kendi
penceresini ölçer — nitekim bu değişiklik hemen daha kötü bir gerçeği
gösterdi). **Kontrol tarafı AÇIK BIRAKILDI, iki seçenek de ölçüm istiyor:**
1. **`BT_RELEASE_V`'yi terminal hızın üstüne çıkarmak** (ölçülen 8.0-9.3 →
   ~10 m/s). Riski: BRAKE tavanı 20°'ye çıkarıyor ve daha hızlıyken kanat
   taşıması daha büyük — adım 31'de **tam** bırakma 8.4 m/s'de manevrayı geri
   sarmıştı (o zaman 90°'ye bırakılıyordu, şimdi 20° tavan var, yani aynı şey
   değil ama ölçülmeden varsayılamaz).
2. **RETRACT'ta sınırlı bir bekleme süresi + iptal**: çıkış koşulu sağlanmazsa
   tavanı bırakıp IDLE'a dönmek. Kısıtı kaldırmak güvenli yön (kodun kendi
   notu: "leaving it clamped unsupervised kills yaw") ve 117.7°'lik sapmayı
   yapısal olarak imkânsız kılar. Ama tetiklenmesi rastlantısal olduğu için
   doğrulanabilmesi bir test kancası gerektiriyor.
**Bu oturumda hiçbiri uygulanmadı**, çünkü ikisi de en az iki uçuşluk
doğrulama istiyor ve bu projenin kuralı doğrulanmamış kontrol değişikliği
bırakmamak. **Pratik sonuç, donanım için:** geri geçiş SITL'de bile
**garanti tamamlanan bir manevra değil** — 8 uçuşun 5'i tamamladı.

**(j) Yan ölçüm — kontrol listesinin açık maddesi kapandı (SITL için).**
Yeni `sitl/check_output_cuts.py` her uçuşta iki şeyi tarıyor: havada motorlara
NaN gidip gitmediği ve `vehicle_attitude` boşluklarının uzunluğu (adım 35'ten
sonra >50 ms bir boşluk çıkışı kesiyor ve bu boşluğun istatistiği
**bilinmiyordu**). Bugünün tüm uçuşlarında: **0 NaN**, boşluk max 8-16 ms,
p99 4 ms → `FS_ATT_TIMEOUT_US`'a **3.1-6.2× pay**. Ayrıca aynı tarama adım
31'in loglarında **uçuş başına 3 adet tek-tick NaN** buldu (o zamanki ikili,
adım 35 öncesi) — kimse bakmadığı için görülmemişti. Donanımda bu ölçüm
yeniden yapılmalı; SITL yalnızca bir alt sınır veriyor.

*Genel ders 1: bir manevranın "tamamlandı" kaydı, onun ne kadar PAYLA
tamamlandığını söylemez. Marjinal bir başarı ile sağlam bir başarı loglarda
aynı görünür — ayırt eden şey son metrelerin ne kadar sürdüğüdür. Bir eşiğe
yaklaşırken otoritesi AZALAN bir yasa yazdıysanız, o otoritenin eşikte hâlâ
yenmesi gereken kuvvetten büyük olduğunu ayrıca gösterin.*

*Genel ders 2 (asıl olan): bir kontrol yasasını "bozucu ne kadar, otoritem ne
kadar" diye AYRIŞTIRIN. Buradaki bozucu sönmüyordu (yaw trimi kalıcı ileri
itki üretiyor) ama yasa sönüyordu — tek terimli yazıldığı için bu uyumsuzluk
görünmüyordu bile. Ayrıştırılmış hâlde "durmak için 3.39°, frenlemek için
+4°" cümlesi kendi kendini denetliyor ve terimlerden biri ölçülmüş bir sabite
(`FX_TRIM`) bağlandığı için donanımda o sabit güncellendiğinde yasa da
güncelleniyor. Adım 30'un dersinin ikizi: orada zayıf bir AMAÇ terimiyle bir
aktüatörü sürmeye çalışmak, burada sönen bir OTORİTEYLE sönmeyen bir kuvveti
yenmeye çalışmak.*

---

### Adım 38 — Madde (R) KAPATILDI (iki terimli çıkış), ve doğrulama sırasında **aynı hastalığın ÜÇÜNCÜ tekrarı** bulundu (2026-07-31)

Görev, Adım 37(i)'de açık bırakılan madde (R)'yi çözmekti: RETRACT'ın çıkış
koşulu `v_h < BT_RELEASE_V = 8.0` bir DENGENİN SINIRINDAYDI ve sağlanmayabiliyordu.

**(a) Neden iki seçenekten biri değil, İKİSİ BİRDEN.** Adım 37 iki aday
bırakmıştı. Tek başına hiçbiri yeterli değil, çünkü ikisi **farklı şeyleri**
garanti ediyor:

1. **`BT_RELEASE_V` 8.0 → 10.0**: eşiği ölçülen terminal hız aralığının
   (8.0-9.3) dışına çıkarır. Manevranın tamamlanmasını sağlar — ama bu sayı
   Gazebo'nun beş lift-drag yüzeyinden ölçüldü. Gerçek kanat farklı sürüklerse
   eşik yine dengenin içine düşer: **donanımda B2/B3 ile aynı sınıf risk.**
2. **`BT_FLOOR_DWELL = 20 s` (YENİ)**: tavan tabana vardıktan sonra bu süre
   geçerse hıza **hiç bakmadan** BRAKE'e geçilir. Aero-bağımsızdır, yani
   terminal hız ne olursa olsun geçerlidir. **Sınırsız bir arızayı sınırlı bir
   gecikmeye çevirir.**

**İptal değil, BRAKE.** Adım 37'nin 2. seçeneği "IDLE'a dön" idi. Ölçüm bunu
desteklemiyor: bekleme, RETRACT'ın verebileceği en düşük hızda dolar (hız
asimptota oturmuştur) ve BRAKE tavanı 20°'ye **yükselterek** yaw'ın modülasyon
otoritesini geri verir — yani takılmanın asıl zararını (200 s'de +117.7° yaw)
doğrudan kaldırır. IDLE'a iptal yaw'ı kurtarır ama aracı seyirde, eve dönüş
yolu olmadan bırakırdı.

**20 s ölçümden:** taban tavana varıldıktan sonra hız 16.7 s'de 9.50 → 8.62,
33.4 s'de 8.29, 89.1 s'de 8.00 — yani 20 s'de araç terminal hızın ~0.4 m/s
yakınındadır, daha uzun beklemek kazanç getirmez. Maruz kalınan yaw
sürüklenmesi ~0.59 °/s × 20 s ≈ 12°, ölçütün (≤45°) rahat altında.

**(b) MATLAB tarafı: plantsız bir birim testi (`run_backtrans_sm_test.m`).**
`backtrans_loop.m` bu manevrayı MATLAB'da üretemez — ama madde (R)'nin sorusu
manevra değil, *"çıkış koşulu sağlanmazsa durum makinesi ne yapar?"*. Bu saf
mantık. Sentetik `v_h` izi gerçek aerodinamikten **daha iyi**: arızalı rejim
(terminal hız eşiğin üstünde) istenildiği gibi kurulabiliyor, oysa SITL'de o
rejim rastlantısaldı. Altı kontrol de geçti; ayrıca **emniyetsiz mantık aynı
izde 300 s takılı kalıyor**, yani regresyonun gerçekten var olduğu gösterildi.

Testin ilk sürümü kendi tuzağına düştü ve düzeltildi: `t_floor` "durum hâlâ
RETRACT iken" diye aranıyordu, oysa hız koşulu zaten sağlanmışsa tavanın tabana
vardığı tick ile BRAKE'e geçilen tick **aynıdır** — ölçüm penceresi yine
ölçtüğü şeyden türetilmişti (Adım 34e/37a'nın tuzağı).

**Zorunlu MATLAB regresyonu birebir nötr:** RMS p/q = 0.0014/0.0003,
max|ω| = 0.0126 — Adım 37'nin kayıtlı değerleriyle aynı. (Simulink portu
yeniden kurulmadı: geri geçiş `.slx`'te **hiç yok**, `sf_*` dosyalarının
hiçbiri `tilt_ceil`/`bt_*` kullanmıyor.)

**(c) SITL — üç normal uçuş, üçünde de manevra tamamlandı:**

| uçuş | ulog | giriş | RETRACT | BRAKE girişi | BRAKE | HANDOFF | son v_h | bant | yaw | doyum/BIG_M |
|---|---|---|---|---|---|---|---|---|---|---|
| A | 13_58_13 | 15.20 | 17.6 s | **9.99** | 8.9 s | 22.0 s | 0.12 | 1.56 m | −6.0° | %0.00 / 0 |
| B | 14_00_27 | 14.61 | 16.5 s | **9.79** | 9.0 s | 22.6 s | 0.24 | 1.38 m | −3.6° | %0.03 / 4 |
| C | 14_03_33 | 14.29 | 20.9 s | **10.00** | 9.8 s | 21.6 s | 0.09 | 1.54 m | −2.4° | %0.00 / 0 |

(Adım 37 referansı: RETRACT 21.9/25.1 s, BRAKE girişi **8.00**, 6.1/5.7 s,
bant 1.11-1.14 m.)

**Yapısal kazanç, sayılardan daha önemli:** eskiden çıkışı belirleyen şey
aracın bir AERODİNAMİK ASİMPTOTA inmesiydi; artık **tavanın 2 °/s ile tabana
inmesi** belirliyor — deterministik bir süreç. RETRACT 4-7 s kısaldı. Bedel
ölçüldü ve kabul edildi: BRAKE daha yüksek hızda başladığı için 3 s uzadı ve
irtifa bandı 1.11-1.14 → 1.38-1.56 m (ölçüt 3.0).

**(d) B'nin 5. ölçütü KALDI — ama manevraya ait değil.** 4 BIG_M örneğinin
tamamı pencerenin açılışından **0.05 s içinde**; aynı olayın 7 örneği daha
`bt_enable`'dan **önceki** seyir fazında (min itki 1.92 N; A: 3.53, C: 3.62,
Adım 37 uçuşları: 4.05-4.71, hepsinde 0 olay). `sat_flag` mutlak tabana değil
**WLS artımının kutu sınırında kırpılmasına** set oluyor ve hiçbir rotor 1 N'un
altına inmedi. Değiştirilen kod yolu seyirde zaten çalışmıyor (`bt_state` =
IDLE, tek fark `floor_dwell = 0`), yani mekanizma olarak bu değişikliğe
bağlanamaz. **Kaydedildi, kapatılmadı:** 6 uçuşta ilk kez görüldü, tekrarlarsa
kendi maddesi olmalı.

**(e) YENİ ÖLÇÜM ARACI — `sitl/brake_ceiling_margin.py`: BRAKE tavanının payı.**
Adım 37'nin 1. genel dersi ("tamamlandı kaydı, ne kadar PAYLA tamamlandığını
söylemez") bu değişikliğe doğrudan uygulandı, çünkü BRAKE artık daha hızlı
giriliyor ve kaçışı tutan şey bir kutu kısıtı:

| uçuş | BRAKE girişi | tavana dayalı süre | en küçük pay |
|---|---|---|---|
| 11_06_40 (Adım 37) | 8.00 m/s | 0.0 s (%0.0) | +2.24° |
| 11_08_37 (Adım 37) | 8.00 m/s | 0.0 s (%0.0) | +4.39° |
| A (Adım 38) | 9.99 m/s | **0.9 s (%10.7)** | **+0.00°** |

Yani `BT_RELEASE_V = 10.0` **`BT_BRAKE_CEIL`'in payını tamamen yiyor.** Zarar
vermedi (doyum %0, kaçış yok, hız monoton düştü) — bir kısıtın *aktif olması*
ile *aşılması* aynı şey değil. Ama **10.0 pratik üst sınırdır: `BT_RELEASE_V`
bundan daha yukarı, `BT_BRAKE_CEIL` yeniden türetilmeden çekilmemelidir.**
(Betik, geçiş tick'indeki sınır artefaktını ayıklar — ayıklanmazsa her uçuş
"tavana dayandı" görünüyordu.)

**(f) Emniyet yolu kancayla tetiklendi (`BT_RELEASE_V = 5.0f` geçici build).**
Gerçek değerle (10.0) emniyet SITL'de hiç çalışmaz, çünkü hız koşulu her zaman
sağlanır. **İlk kanca uçuşu (14_06_01) KİRLENDİ ve bu kayda geçirilmelidir:**
manevranın ortasında `ERROR attitude LOST (cp=1 fresh=1 tilt=0)` tetiklendi,
Adım 35'in sert ön koşulu `resetState()` çağırdı, durum makinesi IDLE'a düştü
ve manevra 10.1 m/s'de, tavan 90°'ye bırakılmış hâlde **sıfırdan yeniden
başladı**. Kuyruk tilt'i (tavan yalnızca kanat rotorlarına uygulanır) 0 → 49°
büyüdü ve araç yeniden hızlandı. *Bu oturumda önce "uzun bekleme kuyruk
kaçışına yol açıyor" diye okundu — YANLIŞ okuma.* Temiz kanca uçuşunda
(14_13_11) 20 s'lik beklemenin tamamı boyunca kuyruk **0.0°'de kaldı** ve
tarihsel 111 s'lik RETRACT'ta (11_16_48) da 0.0°'de kalmıştı. Ders: **bir uçuşu
sonuç olarak kullanmadan önce px4 log'unda WARN/ERROR ara** — burada tek bir
ERROR satırı, tüm nedensel okumayı tersine çevirdi.

**Temiz kanca uçuşu (14_13_11) emniyeti DOĞRULADI:** tavan 15.3 s'de tabana
indi, bekleme 20 s'de doldu ve
`back-transition: state 1 -> 2 (v_h=6.6 m/s) [via FLOOR_DWELL backstop]` +
`WARN: RETRACT never reached BT_RELEASE_V (5.0 m/s) in 20.0 s at the floor`
basıldı. Yeni tanı çıktısı bilerek eklendi: donanımda bu satır, `BT_RELEASE_V`'nin
gerçek aeroya göre yeniden türetilmesi gerektiğinin **sinyalidir**.

**(g) 🔴 YENİ AÇIK MADDE (S) — HANDOFF eşiği YANLIŞ SİNYALE bakıyor; fren
yasası aracı GERİ kaçırıyor.** Aynı temiz kanca uçuşunda, emniyet çalıştıktan
sonra manevra tamamlanmadı. Ölçüm (`sitl/diag_brake_reversal.py`):

| BRAKE evresi | min \|v_h\| (eşik 3.0) | gövde ileri u | yanal v | sonuç |
|---|---|---|---|---|
| 69.9-117.6 s | **3.08** | **−0.51** | **+3.04** | geri kaçış → 12.82 m/s |
| 137.0-177.0 s | 4.73 | −4.42 | −1.68 | geri kaçış → 13.38 m/s |
| 205.2-215.6 s | 4.63 | −4.45 | −1.27 | geri kaçış → 7.85 m/s |

**Mekanizma:** `BRAKE → HANDOFF` koşulu `v_h < BT_HANDOFF_V` ve `v_h`
**bir BÜYÜKLÜK** (`hypot(vx,vy)`). Fren yasası ise yalnızca **ileri** ekseni
kontrol eder. İlk satırda araç ileri yönde çoktan durmuştu (u = −0.51) ama
**3.04 m/s yanal sürüklenme** büyüklüğü eşiğin üstünde tuttu → handoff hiç
istenmedi → sönmeyen fren pitch'i (3.39° trim + 4.0° marj) itmeye devam etti →
araç geri yönde 12.8 m/s'ye kaçtı. Beş kez tekrarlandı.

Yanal sürüklenmenin kaynağı yapısal: **RETRACT/BRAKE boyunca yanal eksende
hiçbir kontrol yok** — pozisyon döngüsü ancak HANDOFF'ta devreye giriyor. Yani
manevra ne kadar uzarsa, eşiği bloke eden bileşen o kadar büyüyor.

**Bu, aynı hastalığın ÜÇÜNCÜ tekrarı** (BRAKE'in fren yasası → Adım 37,
RETRACT'ın çıkışı → madde (R), şimdi HANDOFF'un eşiği) ve Adım 37'nin kendi
dersinin — *"bu soruyu durum makinesindeki HER geçişe sor"* — ilk kez
uygulandığında bulundu. **Madde (R)'nin sebebi DEĞİL, ondan bağımsız ve önceden
var:** üç normal uçuşta görülmedi çünkü BRAKE 8.9-9.8 s sürdü ve yanal
sürüklenme birikemedi. Ama **emniyet devreye girdiğinde manevra uzuyor, yani
(R)'nin çözümü (S)'yi daha ERİŞİLEBİLİR kılıyor.** Aday çözüm (ölçülmedi,
uygulanmadı): eşiği gövde ileri hızına bağlamak ve/veya işaret değişiminde
pitch'i kesmek. **Bu oturumda uygulanmadı** — kendi doğrulama uçuşlarını
istiyor ve proje kuralı doğrulanmamış kontrol değişikliği bırakmamak.

**(h) 🟠 İKİNCİ AÇIK SORU — `attitude LOST (tilt=0)` uçuş ortasında tetikleniyor.**
İki kanca uçuşunda toplam üç kez görüldü (normal uçuşların hiçbirinde yok).
`tilt_aligned` iki şarta bağlı: `estimator_status_flags` 3 s'den taze **ve**
`cs_tilt_align` set. EKF2 bu topic'i **~1 Hz** yayınlıyor (`PublishStatusFlags`,
"publish at ~1 Hz or immediately if status changes"). Yani iki mekanizma da
mümkün: (i) topic bayatladı ve sağlıklı bir EKF'de çıkış kesildi, (ii) şiddetli
uçuş durumunda EKF hizalamayı gerçekten düşürdü. **Ayırt edilemedi, çünkü
`estimator_status_flags` log profilinde yok.** Sonucu ağır: uçuş ortasında
çıkış kesiliyor ve geri geçiş durum makinesi sıfırlanıyor. Bir sonraki adım
topic'i profile ekleyip tekrar ölçmelidir — **tahmin edilmemeli** (Adım 36'nın
dersi).

**(i) Zorunlu `sitl-lockup-check` GEÇTİ** — nihai binary ile (kanca geri
alındı, `BT_RELEASE_V = 10.0f` doğrulandı, binary kaynaktan yeni; A/B/C
uçuşlarının binary'siyle **kaynak olarak birebir aynı**):
itki doyumu **%0.00**, BIG_M **0**, itki 9.49-21.95 N, yaw toplam **−6.80°**,
|vz|max 0.543 m/s, irtifa RMS 0.293 m, v_h 0.23 m/s (`pos_hold` doğrulandı).

**Ama ilk koşu KALDI ve sebebi ortamsaldı — kayda geçirilmeli.** İlk deneme
%3.21 doyum / 614 BIG_M / itki 0-45 N (kilitlenme imzası) / v_h ort **11.94 m/s**
verdi. Log'da sebep açıktı: `Preflight Fail: No valid data from Compass 0` —
yani test hover'ı değil seyri ölçmüştü. Tetikleyici, bu oturumda arızalı
koşuları toparlamak için yapılan tekrarlanan `kill -9`'lardı; `parameters.bson`
bozuk kaldı. Adım 36'nın reçetesi uygulandı (yedekle → sil → **ilk boot ısınma
sayılır**): ısınma koşusu 0.00%/0 ile geçti (irtifa RMS 0.443 m), ölçüm koşusu
0.443 → **0.293 m** ile daha da iyileşti — EKF kalibrasyonu öğrendikçe.
Sabahki en iyi kayıt 0.085 m olduğuna göre bu hâlâ onun üstünde, ama **bu
değişiklik hover yoluna hiç dokunmuyor** (fren/emniyet yalnızca BRAKE ve
RETRACT'ta koşar) ve iki ardışık koşunun trendi param sıfırlamasıyla
açıklanıyor. *Ders (Adım 36'nın tekrarı): bir `kill -9` serisinden sonra ilk
kriter koşusu ölçüm değildir.*

*Genel ders 1: bir eşiği bir dengenin dışına taşımak onu SITL'de düzeltir,
donanımda düzeltmez. Kaleme alınmış her eşik bir ORTAMIN ölçümüdür; yanına
o ortamdan bağımsız, tek yönlü bir emniyet koymak — hıza hiç bakmayan bir
süre gibi — sınırsız bir arızayı sınırlı bir gecikmeye çevirir. İki terim
farklı şeyler garanti eder, biri diğerinin yerine geçmez.*

*Genel ders 2 (asıl olan): **bir eşiğin baktığı SİNYAL, kontrol edilen eksenle
aynı olmalıdır.** Fren yasası ileri ekseni kontrol ediyor, çıkış eşiği ise hız
BÜYÜKLÜĞÜNE bakıyordu; kontrol edilmeyen yanal bileşen böylece eşiği sonsuza
kadar bloke edebildi ve sönmeyen bir komut, durmuş bir aracı geri kaçırdı. Bu
üç adımda üçüncü kez aynı aile: Adım 37 otoritenin sönmesiydi, madde (R)
eşiğin dengede oturmasıydı, madde (S) eşiğin yanlış eksene bakmasıydı.*

*Genel ders 3: bir doğrulama uçuşunu kanıt olarak kullanmadan önce px4 log'unu
WARN/ERROR için tara. Burada tek bir `attitude LOST` satırı olmasaydı, bu adım
"emniyet kuyruk kaçışına yol açıyor" diye YANLIŞ bir sonuçla kapanacaktı.*

---

### Adım 39 — Madde (S) KAPATILDI: eşik ve fren marjı artık **gövde ileri hızını** okuyor; ve aynı hatayı bu kez KENDİ ÖLÇÜTÜMDE tekrarladım (2026-08-03)

**Yapılan değişiklik iki satır, gerekçesi tek cümle: bir eşik, yasasının
KONTROL ETTİĞİ ekseni okumalıdır.**

`BRAKE → HANDOFF` koşulu `v_h < BT_HANDOFF_V` idi ve `v_h` bir BÜYÜKLÜK
(`hypot(vx, vy)`). Fren yasası ise burun yukarı pitch komut ediyor, yani
yalnızca **gövde ileri** eksenini kontrol ediyor; yanal eksende RETRACT/BRAKE
boyunca hiçbir kontrol yok (pozisyon döngüsü ancak HANDOFF'ta devreye giriyor).
Adım 38'de ölçülmüştü (log 14_13_11): `min|v_h| = 3.08` m/s iken gövde ileri
hızı `u = −0.51`, yanal hız `v = +3.04` m/s — yani araç ileri yönde çoktan
durmuştu, eşiği tutan şey manevranın kaldıramadığı bileşendi. Handoff hiç
istenmedi, sönmeyen fren pitch'i (3.39° trim + 4.0° marj = 6.31 N, yendiği
3.1-4.1 N'un üstünde) itmeye devam etti ve araç **geri yönde 12.8 m/s**'ye
kaçtı. Büyüklük geri hızlandıkça BÜYÜDÜĞÜ için eşik gitgide daha ulaşılmaz
oluyordu: pozitif geri besleme.

**Düzeltme iki yerde, ikisi de aynı ilkeden** (`backtrans_loop.m`,
`TiltrotorIndiControl.hpp`):

1. Çıkış koşulu **işaretli** gövde ileri hızını okur: `v_fwd < BT_HANDOFF_V`.
   İşaretli olması şart — `u < 0` ise BRAKE'in işi ZATEN bitmiştir, mutlak
   değer almak aynı hatayı tekrarlardı.
2. Frenleme **marjı** da `max(0, v_fwd)` ile söner, yani araç geri gitmeye
   başladığında marj tam olarak sıfırlanır ve geriye yalnızca duruş trimi
   kalır. **Kaçışı fiilen durduran terim budur**; (1) tek başına durumu
   değiştirir ama itmeyi kesmez.

Duruş trimi terimi (adım 37) SÖNMEZ: yendiği ileri kuvvet hızdan bağımsızdır.

**RETRACT bilerek `v_h` okumaya devam ediyor.** Oradaki soru "frenlemem gereken
ileri hızım var mı" değil, "kanat hâlâ taşıyor mu, yani Fz kaynaklı tilt kaçışı
mümkün mü" — bu aerodinamik bir sorudur ve cevabı hava hızının BÜYÜKLÜĞÜDÜR.
İki evrenin farklı sinyallere bakması bir tutarsızlık değil, tasarımın kendisi:
**her eşik KENDİ yasasının sorusunu sormalı.**

**Doğrulama.**

| koşu | giriş | RETRACT | BRAKE | son v_h | yaw | irtifa bandı | doyum/BIG_M | yeniden-hızlanma | handoff→hold |
|---|---|---|---|---|---|---|---|---|---|
| A (normal) | 15.21 | 18.8 s | 8.5 s | 0.21 | −2.2° | 1.53 m | %0.00 / 0 | **0.00** | 0.56 s |
| B (normal) | 15.76 | 19.1 s | 9.1 s | 0.12 | −2.6° | 1.54 m | %0.00 / 0 | **0.00** | 0.22 s |
| C (kancalı, `BT_RELEASE_V=5`) | 15.24 | 37.8 s | 4.7 s | 0.14 | −3.0° | 0.97 m | %0.00 / 0 | **0.00** | 0.22 s |

Kancalı koşu emniyeti yine tetikledi (`[via FLOOR_DWELL backstop]`, 6.64 m/s'de,
tabanda 20.0 s) — madde (R) çalışmaya devam ediyor. MATLAB tarafında durum
makinesi testi 6 → 9 gruba çıkarıldı (13 kontrol, hepsi geçti); **test 8 eski
BÜYÜKLÜK mantığını aynı iz üzerinde koşturuyor ve 300 s boyunca BRAKE'te
kalıyor**, yani regresyonun varlığı gösterildi, varsayılmadı. MATLAB regresyonu
birebir nötr (RMS p/q **0.0014 / 0.0003**, `max|omega| = 0.0126`) ve zorunlu
`sitl-lockup-check` GEÇTİ (%0.00 doyum, 0 BIG_M, yaw −1.37°, |vz|max 0.502,
irtifa RMS 0.275 m, v_h 0.13 m/s).

**⚠️ AMA KANCALI UÇUŞ MADDE (S)'Yİ SINAMADI, ve bunu fark etmek asıl işti.**
Manevra temiz tamamlandı, fakat yanal sürüklenme yalnızca **1.28 m/s**'ye
ulaştı — oysa arıza 3.04 m/s'de olmuştu. Yani o uçuş rejime HİÇ GİRMEDİ; "geçti"
demek, hiç kurulmamış bir sınavı geçmek olurdu (Adım 21d: bir ortam ancak
hedeflenen mekanizma orada AKTİFSE bir şey kanıtlar). Adım 38'de yanal
sürüklenmenin birikmesinin sebebi de rastlantısaldı: durum makinesi
`attitude LOST` ile beş kez sıfırlanmış, manevra baştan başlamıştı.

Bu yüzden rejimi **deterministik olarak kuran** yeni bir probe yazıldı —
`sitl/probe_lateral_handoff.py`: frenleme sırasında heading +90° çevrilir, yani
mevcut hız vektörü gövde çerçevesinde ileriden yanala döner. Ölçülen arızanın
geometrisi de tam budur (`u = −0.51`, `v = +3.04` ≈ ~90° dönmüş araç). **Sonuç:
yanal 4.76 m/s'ye çıktı ve HANDOFF `|v_h| = 5.04` m/s'de gerçekleşti
(`v_fwd = +2.87`, `v_yanal = −4.14`) — yani ESKİ büyüklük koşulu o anda
SAĞLANMIYORDU.** Karşı-olgusal, sentetik iz üzerinden değil gerçek uçuş
verisinden. Manevra 0.17 m/s'de tamamlandı.

**🔎 VE PROBE, BENİM KENDİ ÖLÇÜTÜMDE AYNI HASTALIĞIN DÖRDÜNCÜSÜNÜ BULDU.**
Madde (S)'yi yakalasın diye `analyze_backtrans.py`'ye eklediğim 6. ölçüt
"işaretli gövde ileri hızı ≥ −2.0 m/s" idi. Probe'da **KALDI: min v_fwd
= −3.95 m/s.** Ama araç geri kaçmamıştı: aynı pencerede `v_h` **9.74 → 3.00
m/s** ile MONOTON azaldı ve yeniden-hızlanma **0.00** m/s idi. Sebep,
`v_fwd`'nin **DÖNEN bir çerçeveye izdüşüm** olması — heading o pencerede
**+186.7°** döndü, yani sabit bir hız vektörü bile işaret değiştirir. Yani
ölçütüm, kontrol yasasının yaptığı hatanın aynasını yapıyordu: *yanlış sinyali
okuyordu.*

Ölçüt 6 bu yüzden **dönmeden etkilenmeyen** bir büyüklüğe çevrildi: fren
yasasının pitch'e sahip olduğu pencerede `v_h`'nin **kendi koşan minimumundan
yükselişi** (yeniden-hızlanma), sınır 1.0 m/s. **Gerçek arıza log'una karşı
doğrulandı**, ki asıl kanıt budur:

| log | yeniden-hızlanma | ölçüt 6 |
|---|---|---|
| 14_13_11 (Adım 38, madde (S) arızası) | **10.31 m/s** | **KALDI** ✅ (doğru) |
| A / B / C / probe (Adım 39) | 0.00 m/s | GEÇTİ |

`min v_fwd` ve `max |yanal|` **teşhis** olarak basılmaya devam ediyor; ölçüt
değiller. Simetri kayda değer: **"ne zaman devredeyim" sorusunun doğru sinyali
İŞARETLİ ileri hız (yasanın kontrol ettiği eksen), "kaçtım mı" sorusununki ise
BÜYÜKLÜK (çerçeveden bağımsız). İkisi de gerekli ve yerleri değiştirilemez.**

**Ölçütün pencere seçimi de düzeltildi.** İlk halinde bütün bt penceresine
bakıyordu ve ilk koşuda −1.99 m/s ile sınırın dibinden geçti; incelendiğinde o
değerin **pos_hold devraldıktan 2.8 s sonra** oluştuğu görüldü (BRAKE hiç
+2.98'in altına inmemişti) — yani ölçüt başka bir yasayı suçluyordu. Pencere,
"fren yasasının pitch'e gerçekten sahip olduğu" aralığa daraltıldı; bunu
mümkün kılmak için `TiltrotorIndiStatus.msg`'e **`pos_hold_active`** eklendi
(telemetri; `bt_state == HANDOFF` yalnızca hold'un İSTENDİĞİNİ söyler,
DEVRALDIĞINI değil).

**Kapatılmayan, ölçülen kalıntı.** `POS_ENGAGE_V_MAX` hâlâ bir BÜYÜKLÜK
kapısıdır, yani handoff ileri eksende istenip hold tarafından reddedilebilir ve
her tick yeniden denenir. Artık tahmin değil ölçüm: **normal uçuşlarda 0.22 /
0.56 s, ağır yanal sürüklenmeli probe'da 3.23 s** — ve o 3.23 s boyunca `v_h`
5.04 → 3.00 ile azalmaya devam etti, pitch trimde kaldı, kaçış olmadı. Bu bir
tercih: kapıyı yön duyarlı yapmak Adım 29'un ölçtüğü güvenlik gerekçesine
dokunmak demektir ve kendi ölçümünü hak eder (modül `_bt_handoff_wait` ile
0.5 s'yi aşan her beklemeyi artık WARN olarak da yazıyor).

**Madde (T) ÖLÇÜLDÜ ama AYRIŞTIRILAMADI — ve sebebi ilginç.** Topic log
profiline eklendi (`logger_topics_shadow.txt`). Dört uçuşun hiçbirinde
`attitude LOST` görülmedi; `estimator_status_flags` yayın aralığı ortalama
0.93-0.96 s, **max 1.00 s (3 s'lik pencerenin çok altında, >3 s olan: 0)** ve
`cs_tilt_align` her örnekte 1. Yani bu koşullarda "topic bayatladı" mekanizması
için kanıt YOK. Ama (T)'nin görüldüğü uçuşlar **madde (S)'nin kaçış
uçuklarıydı** — 12.8 m/s geri kaçış artık oluşmadığı için tetikleyici koşul da
oluşmadı. **(T) hâlâ açık, ama artık enstrümanlı**: bir daha görülürse hangi
mekanizma olduğu tahmin edilmeyecek, log'dan okunacak.

*Genel ders: **bir ölçüt de bir kontrol yasası kadar sinyal seçer, ve aynı
hatayı yapabilir.** Madde (S)'yi yakalamak için yazdığım ölçüt, tam olarak
madde (S)'nin hatasını yaptı — çerçeveye bağlı bir niceliği çerçeveden bağımsız
bir soruya cevap sanmak. İki koruma işe yaradı: (i) ölçütü BİLİNEN ARIZA
log'una karşı koşmak (geçmesi gereken uçuşlarda geçmesi yetmez, kalması gereken
uçuşta KALMALI), (ii) sınıra yakın bir "GEÇTİ"yi (−1.99 vs −2.00) kutlamak
yerine incelemek — Adım 20'nin kuralı, bu kez kendi test altyapıma uygulandı.*

*İkinci ders: bir düzeltmeyi doğrulayan koşu, düzelttiği REJİMİ kurmuyorsa
kanıt değildir. Kancalı uçuş beş ölçütü de geçti ve hiçbir şey kanıtlamadı;
kanıtı, rejimi bilerek kuran 30 satırlık probe verdi. "Arıza tekrarlanamadı"
ile "arıza koşulları oluşturulup tekrarlanmadı" aynı cümle değildir.*

#### Adım 39b — GUI'li yeniden koşum: madde (S) rejimi NORMAL bir uçuşta KENDİLİĞİNDEN oluştu

Adım 37'nin disiplini (izlenen bir yeniden koşum, yeni bir senaryo kadar
değerli) tekrar uygulandı: iki geri geçiş + iki probe, Gazebo penceresi açık.
Dördü de geçti — ama biri yeni bir bilgi verdi.

**`side` kameralı normal uçuşta yanal sürüklenme kendiliğinden 3.60 m/s'ye
çıktı** — madde (S) arızasını üreten 3.04 m/s'nin ÜSTÜNDE, hiçbir kanca ya da
komut olmadan. HANDOFF `v_h = 3.58`, `v_fwd = 2.99`'da gerçekleşti, yani
**eski BÜYÜKLÜK koşulu o anda da sağlanmıyordu**: düzeltme olmasaydı bu sıradan
uçuş da BRAKE'te takılırdı. Sonuç temiz (yeniden-hızlanma 0.00, yaw +0.8°,
irtifa bandı 1.23 m, %0.00 doyum), handoff→hold beklemesi 1.92 s.
**Bu, madde (S)'nin sanıldığı kadar egzotik olmadığını gösteriyor:** Adım
38'de yalnızca durum makinesi beş kez sıfırlandığı için görünmüştü, ama
tetikleyici koşul (yanal sürüklenmenin birikmesi) normal manevrada da
oluşabiliyor. Dört normal uçuşta ölçülen yanal: 1.04 / 1.28 / 1.49 / 1.90 /
**3.60** m/s — yani dağılımın kuyruğu eşiği rahatça aşıyor.

**İki test altyapısı düzeltmesi daha (ikisi de GUI'de görünür oldu):**

- **Kademeli iniş.** `run_backtrans_test.py` ve `probe_lateral_handoff.py`
  manevra bitince **25 m irtifada disarm ediyordu**, yani izlenen son şey aracın
  düşüp takla atmasıydı — üstelik bu projede zaten belgelenmiş bir tuzak.
  `run_lockup_check.py`'nin kanıtlanmış deseni (1.0 m kademe, iki terimli çıkış
  şartı, yerde disarm) taşındı. Ölçütler etkilenmez: `bt_enable` inişten ÖNCE
  bırakılıyor, yani ölçüt penceresi eskisi gibi kapanıyor — açık bırakılsaydı
  25 m'lik iniş, irtifa bandı ölçütünü (≤ 3 m) haksız yere düşürürdü.
- **Probe'un kendi çıktısındaki iki kusur.** (i) Probe hâlâ terk edilen
  çerçeveye-bağlı ölçütü basıyor ve sahte bir "KALDI -- GERİ KAÇIŞ" veriyordu
  (aynı düzeltme `analyze_backtrans.py`'de yapılmış ama probe'un kendi kopyası
  unutulmuştu — **bir ölçütü düzeltirken onun KOPYALARINI arayın**). (ii)
  "son 10 s" artık inişi ölçüyordu (0.15 yerine 2.01 m/s yazdı); pencere
  manevranın sonundan alınacak şekilde düzeltildi — ölçüm penceresini ölçtüğü
  şeyden bağımsız kurma kuralının küçük hâli.

**Kamera artık tilti gösteriyor.** Eski varsayılan `chase` (2 m arkadan)
naselleri tam profilden gizliyordu, yani bu projenin uçuşlarını harcadığı asıl
olay — tavanın 45° → 9° → 20° rampası — izlenemiyordu. `gz_follow.sh` dört
hazır görüş alıyor (`front` varsayılan / `nose` / `side` / `chase`),
`INDI_GZ_CAM` ile her koşuda değiştirilebiliyor ve `follow_model()` üzerinden
**bütün** testlere uygulanıyor. Tilt açısını en doğrudan okutan `side`
(nasel gövde X-Z düzleminde döner); `nose` en temiz kadraj ama ara açıları
(9° ile 20°) ayırt ettirmez.

---

### Adım 40 — B1'in pilot yolu İLK KEZ ÇALIŞTIRILDI: yol çalışıyor, ama link kaybı dalı savını tutmuyor (madde (U)) — 2026-08-03

Pilot girişi Adım 33'te yazılmıştı ve **bugüne kadar bir kez bile
çalıştırılmamıştı**; kontrol listesinde B1 tam olarak bunun için 🔴 duruyordu.
Bu projede çalıştırılmamış kod yolu iki kez pahalıya patladı (Adım 34'te
RATE_ONLY ilk tetiklendiğinde araç düştü; Adım 36'da "ulaşılamaz" ve "ölü"
hükümlerinin ikisi de yanlıştı).

**Önce ulaşılabilirlik ölçüldü, uçmadan** (`probe_pilot_link.py`). Modülün
pilot dalı iki koşula birden bağlı ve **asıl risk bizim gönderdiğimizde değil,
commander'ın verdiğindeydi**: `flag_control_manual_enabled`. Ölçüm: akış yokken
`False`/`valid=False`/nav_state 4, MAVLink `MANUAL_CONTROL` akışı başlayınca
**`True`/`valid=True`/source=3/nav_state 2**. Param değiştirmeye gerek yok —
`COM_RC_IN_MODE` varsayılanı 3 zaten joystick kabul ediyor (SITL param'ları
kalıcı olduğu için bu bilinçli bir tercih). `manual_control_switches` yalnızca
`RC_CHANNELS_OVERRIDE` ile geliyor, yani VTOL anahtarı → `bt_enable` yolu bu
akışla değil RC ile sınanmalı (henüz sınanmadı).

**🔴 ÖNCE DÖRT GİZLİ TAŞMA DÜZELTİLDİ.** Probe sırasında görüldü ki Adım 32'nin
bulduğu `(now - topic.timestamp) < TIMEOUT` deseni — iki `uint64`, ve
`now = angular_velocity.timestamp_sample` — **hâlâ dört yerde duruyor**
(satır 216, 253, 372, 547); Adım 32/33 düzeltmeyi yalnızca ısırdığı iki yere
uygulamış (390, 454). Ölçüm, tahmin yerine: bu uçuşlarda
`estimator_status_flags` **hiç taşmıyor** (39217 tick'te 0), ama
`vehicle_attitude` **düzeltme olmasaydı %0.03 taşacaktı** — yani form gerçekten
hatalı, SITL onu maskeliyor: gz gyro'sunun publish−sample farkı ~0 µs, çünkü
lockstep. **Gerçek sensör hattında o gecikme gerçektir**, yani bu, SITL'in
yapısal olarak gösteremeyeceği ama donanımın göstereceği bir hata. Dördü de
toplama formuna çevrildi. En kritik ikisi: `tilt_aligned` (Adım 35'ten beri
`att_ok` false ⇒ **çıkış kesilir**, yani bir taşma gerçek duruş kaybından
ayırt edilemez — madde (T)'nin imzası; **bu (T)'yi KAPATMAZ**, çünkü ölçüm
taşmanın SITL'de olmadığını söylüyor, ama Adım 38'in düşünmediği ÜÇÜNCÜ bir
aday mekanizmayı ortadan kaldırır) ve pilot tazeliği (orada
`manual_control_setpoint.timestamp` **gerçek zamanla** damgalanıyor, `now` ise
bir örnek zamanı — yani "topic now'dan yeni" burada istisna değil NORMAL durum;
taşan formda bir çubuk girdisi sessizce yok sayılırdı).

**Sonra uçuruldu** (`run_pilot_input_test.py`, iki bağımsız GUI'li uçuş).
Yedi ölçüt, her biri modülün bir dalına karşılık geliyor. **Yol baştan sona
çalıştı:** devralma tam bir kez ve basamaksız; çubuklar ortada → `pos_hold`
kendiliğinden devrede (koşunun %28-30'u); roll/pitch komutu izleniyor ve
**pilot sahipken max |roll| 8.2-8.3°, |pitch| 8.4-8.5°** — 0.5 çubuk ×
`MAN_TILT_MAX` 15° = 7.5° komutunun tam karşılığı; yaw çubuğu heading'i
120-164° döndürüyor; throttle 13.7-13.9 m irtifa aralığı veriyor; havada NaN
çıkış 0, BIG_M 0, `attitude LOST` 0.

**🔴 MADDE (U) — LİNK KAYBI DALI, TUTAMADIĞI BİR ŞEYİ İDDİA EDİYOR.** Dal
tetikleniyor ve `pilot input LOST -- holding position and descending` yazıyor.
Ölçüm ise: **alçalma boyunca `pos_hold_active` HİÇ 1 olmuyor** ve araç
**4.8-6.1 m/s** ile uçarak iniyor, ~100 m ileride yere değiyor. Mekanizma
zincirleme ve tamamen bilinen parçalardan: çubuk evrelerinde madde (N)'nin
yapısal ileri ivmesi aracı 9.3 m/s'ye çıkarıyor → `pos_hold REFUSED:
9.3 m/s > 3.0` → link kaybı dalı `req_pos_hold = true` diyor ama **bu yalnızca
bir İSTEK**, `POS_ENGAGE_V_MAX` onu reddetmeye devam ediyor → ve dal, aracı
yavaşlatabilecek tek mekanizmayı **açıkça kapatıyor**: `req_bt = false`.
Kontrol listesi bunu Adım 34(d)'de zaten yazmıştı — *"Station keeping must be
handed to `bt_enable` on recovery — not wired"* — ama o zaman kurtarma yolunda
görülmüştü; **burada link kaybında, yani müdahale edecek kimsenin olmadığı
durumda.** Madde (S) ile aynı yapısal desen: alıcının reddedebildiği bir istek,
ve reddedilirse yedeği olmayan bir tasarım.
**Uygulanan:** yalnızca log satırı — artık hızı yazıyor ve hold reddedilecekse
`PX4_ERR ... TOO FAST ... descending WITHOUT position control (item (U))`
diyor. **Uygulanmayan:** asıl düzeltme (`req_bt = v_h > POS_ENGAGE_V_MAX`, yani
çok hızlıysa geri geçişe devret). Sebebi disiplin: **alçalırken geri geçiş
yapmak hiç uçurulmadı** ve bu proje doğrulanmamış kontrol değişikliği bırakmaz.
Kendi doğrulama kampanyasını hak ediyor (BT_MIN_ALT = 15 m etkileşimi dahil).

**🔎 VE ÖLÇÜT PENCERESİ HATASI ÜÇÜNCÜ KEZ, BU KEZ ÜÇ AYRI ÖLÇÜTTE.** İlk koşuda
ölçüt 3 (22.1° pitch) ve 7 (283 BIG_M) KALDI verdi. İkisi de **aynı 1.4 s'lik
pencerede, z = −1.7 m'de, `pos_hold` %95 aktifken** — yani **yere temasta**
oluşmuştu. Pilot pitch'e sahipken değer 8.5°; BIG_M havada 0. Ölçütler sahibi
oldukları yasanın aktif olduğu pencereye kısıtlandı (Adım 39'un madde (S)
ölçütünde öğrendiğinin aynısı), ve dışlanan değerler **yine de raporlanıyor** —
ölçütten çıkarmak görmezden gelmek değildir.
**Ters yönde bir hata da vardı ve daha önemliydi:** ölçüt 6 yalnızca mesajın
YAZILDIĞINI kontrol ediyordu, savın tuttuğunu değil — ve bu yüzden iki uçuşta
da GEÇTİ verdi. Sav ölçülür hâle getirilince (alçalırken ortalama v_h ≤
`POS_ENGAGE_V_MAX`) **ikisi de KALDI**. Konsoldan okuduğum "0.07 m/s tutuluyor"
değeri de yanlıştı: o **yere indikten sonraki** anlık değerdi, evre boyunca
5.46 m/s idi. *Bir dalın VARLIĞINI ölçmek, SAVINI ölçmek değildir* (Adım 35'in
dersi), ve *bir evreyi tek bir anlık örnekle yargılama* (Adım 12b'nin dersi,
bu kez hız üzerinde).

*Genel ders: bir kod yolunu "yazıldı" saymakla "çalıştırıldı" saymak arasındaki
fark, bu projede dördüncü kez ölçüldü. Pilot yolunun yedi dalından altısı ilk
denemede doğru çalıştı — ama yedincisi, tam da insanın müdahale EDEMEDİĞİ
durumu ele alan dal, kendi log satırında yazan şeyi yapmıyordu ve bunu yalnızca
savı ölçüte çevirmek gösterdi.*

---

### Adım 41 — Madde (U) KAPATILDI (irtifası olan link kaybı için), ve izlemek yeni bir madde buldu: (V) — 2026-08-03

**Düzeltme.** Link kaybı dalındaki `req_bt = false` şuna dönüştü:

```cpp
req_bt = !_pos_hold_active
         && ((v_h_now > POS_ENGAGE_V_MAX) || (_bt_state != BtState::IDLE));
```

Yani hold kabul edemeyecek kadar hızlıysak araç, işi tam olarak "seyirden
hover'a" olan makineye devredilir. **`_bt_state != IDLE` mandalı şart:** yoksa
hız eşiğin altına düştüğü an `req_bt` düşer, makine IDLE'a sıfırlanır, tavan
bırakılır, bir tick sonra yeniden başlar — manevra bitmek yerine sürekli baştan
başlardı. Bırakma koşulu `_pos_hold_active`, çünkü manevranın varlık sebebi
odur. Alçalma bunun altında bağımsız sürer: geri dönecek pilot yok, iniş
yavaşlamayı beklememeli.

| | link kaybı irtifası | yatay yol (kayıptan temasa) | temas v_h | alçalırken ort. v_h |
|---|---|---|---|---|
| öncesi (2 uçuş) | 7-13 m | **104-110 m** | 4.96-5.39 m/s | 5.43 / 5.46 |
| sonrası (2 uçuş) | 17.8 / 19.3 m | **13 / 12 m** | **0.16 / 0.13 m/s** | **1.33 / 0.97** |

px4 log dizinin tamamını gösteriyor: `pilot input LOST at 5.0 m/s -- too fast
... handing over to the back-transition` → `state 0 -> 1 -> 2 -> 3` →
`pos_hold: holding` → `state 3 -> 0`. İki uçuşta da 7/7 ölçüt geçti; zorunlu
`sitl-lockup-check` geçti.

**⚠️ İLK DENEME BAŞARISIZDI VE SEBEBİ ÖĞRETİCİ.** Düzeltme doğru bağlanmıştı ama
hiç devreye girmedi; log tek satırda söyledi: `back-transition REFUSED: 7.2 m
AGL < 15.0 m minimum`. Test senaryosu link kaybını 7.2 m'de kuruyordu — ve orada
**reddetmek DOĞRU davranıştır**, 30-40 s'lik bir manevraya yer yok. Yani hata
düzeltmede değil, **senaryonun düzeltmenin hedeflediği rejimi kurmamasındaydı**
(Adım 39'un dersinin birebir tekrarı, bu kez ben kurmadan önce fark etmedim).
Tırmanış 13 → ~30 m'ye çıkarıldı. **Kalan sınır dürüstçe açık:** alçak irtifada
hızlı link kaybı hâlâ 100+ m sürüklenmeyle biter ve çözümü geri geçiş değildir.

**🔎 GUI'Lİ İZLEME İKİ ŞEY DAHA BULDU — ikisi de "izlemek ölçmektir"in örneği.**

*(a) Yerdeki debelenme, ölçüt özetlerini kirletiyordu.* Kullanıcı "kamera
uzaklaştı, drone kontrolü mü kaybediyor?" diye sordu. Ölçüm: **havada değil,
yerde.** Test link kaybından sonra `LINKLOSS_S` boyunca dönüyordu ve araç yere
değdikten sonra ARMED kalıp yerde sürünüyordu — o evrede 3294 BIG_M, tilt
48.6°'ye, yaw **823°**'lik bir span'e gidiyor. Havadaki gerçek değerler: 0
BIG_M, tilt 3.7-15.1°, yaw span **133°** (yaw çubuğunun komutu). Aynı tuzak
`run_lockup_check.py`'de çözülmüştü, pilot testine taşınmamıştı; temasta duran
iki terimli çıkış eklendi. **Kameranın "uzaklaşması" ise kontrol kaybı değil:
araç çubuk evrelerinde 10.4 m/s'ye çıkıyor (madde (N), yapısal ileri ivme,
`pos_hold REFUSED: 10.0 m/s`) ve offset model çerçevesinde olduğu için hızlı
hareket + yaw kamerayı savuruyor.** Görüşler yakınlaştırıldı (`nose` artık
+1.8 m ve varsayılan).

*(b) 🟠 MADDE (V) — pilot İLERİ geçişi hiç komut edemiyor.* Kullanıcının
"tilti hiç görmedim, drone konseptiyle mi uçuyor?" sorusu doğru bir gözlemdi:
pilot uçuşlarında kanat tilti havada yalnızca 3.7-15.1° oynuyor. Sebep kodda
tek satır — pilot dalı **`fx_sp = 0.0f` sabitliyor**. Çubuklar yalnızca duruş
(≤15°) ve tırmanma hızı veriyor; tek geçiş kontrolü VTOL anahtarı → `req_bt`, o
da **geri** geçiş. Yani **pilot, hiç başlatamayacağı bir geçişi bitirebiliyor.**
Koddaki yorum "pilota B5'in eksik dediği iki yarıyı da verir" diyor; doğru ama
*geri geçişin* iki yarısı için, uçuşun iki YÖNÜ için değil — sav ile kapsamın
karıştığı bir yer daha. Düzeltme yönü açık (bir eksen/anahtar → `fx_sp`) ama
tasarım kararı gerektiriyor: `fx_sp`'yi doğrudan pilota vermek, madde (N) ile
birleşince `pos_hold`'un reddedeceği hızlara çıkmanın en kısa yolu, yani madde
(U)'nun tetikleyicisi. Ölçülmeden uygulanmadı.

*Bu vesileyle görev profili (CONOPS) ilk kez tek parça hâlinde yazıldı*
(README "Görev profili" bölümü): kalkış → tilt ile sabit kanada geçiş → seyir →
tilt ile multikoptere dönüş → iniş. **Profilin tamamı SITL'de uçtu — ama bir
hata ayıklama konsolundan, pilottan değil.** Tablo "yasa var mı", "SITL'de uçtu
mu" ve "pilot komut edebiliyor mu" sütunlarını ayrı tutuyor, çünkü B1 tam olarak
bu üçünün karıştırılmasıydı.

*Genel ders: bir düzeltmenin devreye GİRDİĞİNİ de ölçmek gerekir, çalıştığını
değil sadece. İlk koşuda yedi ölçütten altısı geçti ve düzeltme hiç çalışmamıştı
— tek kanıt bir REFUSED satırıydı. Ve izlemenin kendisi bir ölçüm aracıdır:
bu adımda iki bulgu da (yerdeki debelenme, ileri geçişin yokluğu) grafiklerden
değil, birinin pencereye bakıp "bu doğru görünmüyor" demesinden çıktı.*

---

### Adım 42 — İLERİ GEÇİŞ BİR KONTROL YASASI OLDU: tam otonom görev SITL'de uçtu (2026-08-03)

**Gereksinim netleşti:** profil baştan sona otonom olmalı — kalkış multikopter,
seyir sabit kanat/tilt motor, iniş multikopter. Pilot bu profilin sürücüsü değil,
yalnızca müdahale yolu. Bu, madde (V)'yi yeniden çerçeveledi: sorun "pilot
komut edemiyor" değil, **hiçbir şey otonom komut edemiyor**.

**Yazılan:** `forwardtrans_loop.m` / `forwardTransition()`, üç durum
(IDLE → RAMP → CRUISE), `ft_enable` bayrağıyla sürülüyor — `backTransition()`'ın
tam aynadaki karşılığı.

**Ve mekanizması bilerek TERS.** Bu makine **tilt açısı komut etmez**: `fx_cmd`'yi
rampalar, tilti WLS kendi seçer. Gerekçe ölçüm: Adım 31/faz 0, tilti sürenin
tahsisatın kendi tercihi olduğunu gösterdi — **ileri yönde o tercih zaten doğru
yöne bakıyor** (kanat yüklendikçe `dFz/dT` küçülüyor, tiltlemek ucuzluyor), yani
bir amaç terimi yeter. **Geri yönde ters yöne bakıyordu** ve yalnızca bir KUTU
KISITI (tavan) fikrini değiştirebiliyordu. *Genel: aynı aktüatörü hareket
ettirmenin doğru aracı, tahsisatın o an zaten hangi yöne gitmek istediğine
bağlıdır.*

Pitch **her zaman 0** (Adım 29: ~5-6 m/s üstünde burun yukarı bir TIRMANMA
komutudur ve irtifa döngüsü kanat taşımasına karşı koyamaz — hızlanırken o kol en
tehlikeli olduğu andadır). `pos_hold` **bırakılır** (aktifken roll/pitch'in ve
`fx_trim`'in sahibi odur, komutu ezerdi; bırakma handoff'u Adım 28'de temiz
ölçülmüştü: hız aktivitesi DÜŞÜYOR).

**İptal `fx = 0` DEĞİLDİR.** Adım 30 tam olarak bunu denedi ve araç yavaşlamadı;
Fx tiltleri geri çekemeyecek kadar zayıf bir amaç terimi. Ayrıca `v_h`
`POS_ENGAGE_V_MAX`'i geçer geçmez `pos_hold` bir daha kabul etmiyor (Adım 29'un
kapısı) — yani **bu manevra tek yönlü bir kapı** ve hover'a dönüşün tek yolu geri
geçiş. Bu yüzden iptal, **geri geçişi İSTEMEK** (`req_abort` → `req_bt`), ve FT
bloğu BT bloğundan ÖNCE koşuyor ki iptal aynı tick'te devredebilsin.

**İki emniyet, bilerek FARKLI cinsten** (Adım 38'in dersi):
(1) **irtifa bandı** 5 m — aero-bağımlı, Adım 29'un kaçış tırmanışının doğrudan
dedektörü; (2) **süre** 30 s — hıza HİÇ bakmaz, yani gerçek kanat farklı taşırsa
da geçerli. Bir de doygunluk: itki kutudayken `fx` büyütülmez — bu bir iptal
değil bekleme sebebi (Adım 11'in dersi).

**Doğrulama — tam otonom görev, iki bağımsız GUI'li uçuş, elle hiçbir komut yok
(yalnızca iki bayrak):**

| | FT RAMP | FT CRUISE | seyir tilt / v_h | FT irtifa sapması | BT | son v_h |
|---|---|---|---|---|---|---|
| A | 14.4 s (0.19 → 12.30) | 12.0 s (→ 15.24) | **44.1° / 14.10 m/s** | 0.80 m | RETRACT→BRAKE→HANDOFF ✅ | 0.09 |
| B | 14.4 s (0.10 → 13.24) | 11.4 s (→ 15.40) | **39.7° / 14.62 m/s** | 0.58 m | ✅ | 0.12 |

İkisinde de havada itki doyumu %0.00, BIG_M 0, NaN 0, iptal 0. Rampa süresi
(14.4 s) `FT_FX_CRUISE / FT_FX_RATE`'ten türüyor — elle yazılmış bir sayı değil.
Plantsız mantık testi `run_forwardtrans_sm_test.m` **16/16** (normal dizi, iki
emniyetin ayrı ayrı tetiklenmesi, süresiz mantığın aynı izde 120 s takıldığının
gösterilmesi, doygunlukta beklemesi, pitch'in her zaman 0 olması, enable=0
sıfırlaması). MATLAB regresyonu birebir nötr (0.0014/0.0003, 0.0126), geri geçiş
mantık testi 13/13 hâlâ geçiyor, zorunlu `sitl-lockup-check` GEÇTİ (%0.00/0,
yaw −6.97°, irtifa RMS 0.088 m).

**Bu, profilin ilk kez UÇTUĞU anlamına gelmiyor — ilk kez OTONOM uçtuğu anlamına
geliyor.** Aynı fizik Adım 28'den beri uçuyordu; fark, rampayı test betiğinin
değil aracın kendisinin yürütmesi. *Bir yeteneğin var olması ile ona otonom
erişilebilmesi ayrı şeylerdir ve engelleyici B1'in tamamı bu ayrımdır.*

**Kalan (madde (V) tam kapanmadı):** (a) seyirde **enerji yönetimi yok** — hız ve
irtifayı birlikte yöneten bir yasa (TECS benzeri) yok, `fx` sabit tutuluyor;
(b) **görev dizicisi yok** — evreleri hâlâ dışarıdan gelen iki bayrak zincirliyor,
modülün kendi içinde bir "kalkış→geçiş→seyir→dönüş→iniş" dizisi yok;
(c) hiçbiri **gerçek donanımda** uçmadı.

---

### Adım 43 — Seyir zarfı ölçüldü: "tam sabit kanat" bir AYAR değil, MİMARİ bir eksik (2026-08-03)

Adım 42'nin seyri (`FT_FX_CRUISE = 12 N`) izlenirken soruldu: *"tilti hiç
görmedim, drone gibi gidiyor, sabit kanat olmalı."* Ölçüm gözlemi kısmen
doğruladı ve kısmen düzeltti — **ve altından mimari bir sınır çıktı.**

**Önce Adım 42'nin seyri ölçüldü:** 14.4 m/s'de kanat tilti **32°**, kuyruk
tilti **0.6°**, rotorların dikey bileşeni 28.8 N, yani **ağırlığın %59'u hâlâ
rotorlarda, kanatta yalnızca %41.** Bu bir KISMİ geçiş; "seyir = sabit kanat"
diye yazdığım önceki ifade yanlıştı.

**Sonra zarf süpürüldü** (`sitl/probe_cruise_envelope.py`, fx 12→24 N kademeli,
her kademenin son 1/3'ü ölçüldü):

| fx (N) | v_h | tilt0 | tilt2 | toplam itki | rotor dikey | **kanat %** | itki marjı | doyum % | dz |
|---|---|---|---|---|---|---|---|---|---|
| 12 | 15.6 | 44.2° | 0.6° | 31.2 | 27.2 | 45 | 23% | 0.00 | 0.15 |
| 15 | 17.4 | 58.6° | 0.6° | 30.5 | 24.5 | 50 | 23% | 0.21 | 0.18 |
| 18 | 20.7 | 72.0° | 3.0° | 36.2 | 28.0 | 43 | 27% | **20.5** | 0.51 |
| 21 | 27.2 | 75.9° | 19.8° | **104.5** | 35.3 | 28 | **77%** | **99.6** | **21.2** |

**Tilt GERÇEKTEN yatıyor** (44° → 72°), yani gözlemin o yarısı kamera/zamanlama
kaynaklıydı. **Ama kanat payı artmıyor** — ve sebebi rotor-bazında bakınca
görünüyor:

| v_h | T0 | T1 | T2 | δ0 | δ1 | δ2 | rotor dikey | kanat | nu_des(Fz) |
|---|---|---|---|---|---|---|---|---|---|
| 15.6 | 8.0 | 7.4 | 15.5 | 44° | 40° | 0.6° | 26.9 | 22.2 N | +2.4 |
| 17.4 | 9.3 | 8.3 | 14.4 | 59° | 56° | 0.5° | 23.9 | 25.1 N | +1.7 |
| 20.6 | **0.0** | **0.0** | **21.8** | 79° | 77° | 19° | 20.5 | 28.5 N | **+10.5** |

20.6 m/s'de **kanat rotorlarının ikisi de TAM KAPALI** ve naseller 79°'de
yatmış — yani araç kanat-taşımalı olmaya gerçekten başlıyor. Takıldığı yer
şu: **kuyruk rotoru tek başına 21.8 N taşıyor ve kapatılamıyor.**

**KÖK NEDEN — kontrol yüzeyleri hiç kullanılmıyor.**
`MulticopterIndiTiltrotor.cpp:1200`: *"control surfaces (indices 0-4): held at
trim, out of scope for the ported controller"* → `servos.control[0..4] = 0`.
Modelde elevator/aileron/rudder var, kontrolcü hiçbirini oynatmıyor. Sonuç
zinciri: **seyirde bütün duruş otoritesi rotorlardan gelmek zorunda → rotorlar
kapatılamaz → kuyruk rotoru hem pitch'i tutmak hem kalan ağırlığı taşımak için
dik kalmak zorunda → tam kanat-taşımalı uçuş yapısal olarak erişilemez.**
fx = 21 N'de zarf kırılıyor (itki 104.5 N, %99.6 doygunluk, irtifa +21 m).

**Bu bir ayar sorunu DEĞİL.** `FT_FX_CRUISE`'u yükseltmek aracı yalnızca
doygunluğa iter; ölçülen tavan fx ≈ 18 N / ~20 m/s ve orada bile %20 doygunluk
var. Tam sabit kanat için gereken mimari adım belli: **kontrol yüzeylerini WLS
tahsisatına almak** — etkinlik matrisi 5×6'dan 5×(6+N) boyutuna çıkar, yüzey
etkinliği dinamik basınçla (½ρv²S·C) ölçeklenir, ve gain-schedule zaten var olan
`delta_bar` karışımını yüzey/rotor otoritesi arasında paylaştırmak için
kullanılır. O zaman seyirde duruş otoritesi yüzeylere geçer, rotorlar (kuyruk
dahil) kapanabilir ve kanat ağırlığın tamamını alır.

*Genel ders: bir yeteneğin eksik olmasıyla bir MİMARİNİN onu dışlaması ayrı
şeylerdir. "Seyirde kanat neden taşımıyor" sorusunun cevabı ne aerodinamikte ne
de geçiş yasasındaydı — port edilirken kapsam dışı bırakılmış beş aktüatörde.
Zarfı süpürmek bunu iki uçuşta ortaya çıkardı; tek bir çalışma noktasına bakmak
asla çıkarmazdı.*

---

### Adım 44 — Kontrol yüzeyleri tahsisata girdi (1/n): etkinlik matrisi 5×6 → 5×11 (2026-08-03)

Adım 43'ün bulgusunun gereği: yüzeyler WLS'e girmeden tam kanat-taşımalı uçuş
yapısal olarak erişilemez. Bu adım **yalnızca etkinlik matrisini** genişletiyor
— bilinçli olarak küçük ve tek başına doğrulanabilir bir artış.

**Sabitler uydurulmadı, `model.sdf`'ten birebir alındı** ve FLU→FRD çevrimi
açıkça yapıldı (bu projede aynı çevrimi atlamak madde B4'te aylarca süren bir
"işaret çelişkisi" yanılsaması üretmişti):

| servo | yüzey | cp FLU | alan | rad→cl | k = alan·rad_to_cl |
|---|---|---|---|---|---|
| 0/1 | sol/sağ elevon | (−0.05, ±0.30, 0.05) | **0.5** | −4.0 | −2.0 |
| 2/3 | sol/sağ elevator | (−0.70, ±0.15, −0.04) | 0.048 | −12.0 | −0.576 |
| 4 | rudder | (−0.74, 0, 0.12) | 0.032 | −6.0 | −0.192 |

Model: `F = q̄ · k · δ · e_up`, cp'de uygulanır. **Sapmada LİNEER**, yani
Jacobian δ'dan bağımsız; yalnızca `q̄ = ½ρV²` ile ölçeklenir.

**★ VE BU YAPININ ASIL DEĞERİ: MOD DEĞİŞİMİ GEREKTİRMEMESİ.** Hover'da q̄ = 0,
yani yüzey sütunları **tam sıfır**; sıfır etkinlikli bir sütunun WLS maliyeti
yalnızca `Wu·du²`'dir ve minimumu `du = 0`. Yani hover davranışı
bit-birebir eskisi — **ölçülmeden önce KANITLANABİLİR bir nötrlük.** Hız
arttıkça sütunlar büyür ve yüzeyler kendiliğinden devralır. Bu, kontrolcünün
bütün felsefesiyle aynı: gain-schedule gibi sürekli bir karışım, ayrı bir durum
makinesi değil.

**Doğrulama** (`run_surface_effectiveness_test.m`, 11/11): eski 6-elemanlı
çağrı hâlâ 5×6 veriyor; 11-elemanlı çağrı 5×11; rotor sütunları **birebir
aynı** (max fark 0); q̄=0'da yüzey sütunları **tam sıfır**; `nu0` değişmiyor.
İşaretler ELDE türetilip karşılaştırıldı (koddan değil — yoksa test kendi
hatasını onaylar): elevon L `τx = −0.6q̄`, elevon R `+0.6q̄` (ters → diferansiyel
= roll), ikisi `τy = +0.1q̄` (aynı → kolektif = pitch), rudder
`τz = −0.14208q̄` ve pitch üretmiyor. Ölçekleme `q̄ ∝ V²` doğrulandı (hız 2× →
etkinlik 4.0×). MATLAB regresyonu birebir nötr (0.0014/0.0003, 0.0126).

**⚠️ ÖLÇÜLEN VE SONRAKİ ADIMI BELİRLEYEN SAYI — yüzeyler ÇOK güçlü.** 20 m/s'de
tam sapmada: **roll 229 N·m, pitch 38 N·m, yaw 18 N·m.** Karşılaştırma: yaw'ın
rotor otoritesi adım başına ~0.05 N·m (Adım 12g) ve `I_zz = 0.25` — yani 18 N·m
72 rad/s² demek. Sebep görünür: servo_0/1'in alanı **0.5 m²**, yani bunlar küçük
trim tabları değil, ANA KANAT panelleri ve elevon onların flapı (Adım 30'un
"tek boylamsal yüzey, 0.5 m²" ölçümüyle aynı yüzeyler). **Sonraki adımın asıl
tasarım işi bu yüzden ağırlıklar ve hız limitleridir**, etkinlik değil: tahsisat
bu kadar güçlü bir aktüatörü doğru cezalandırılmazsa bang-bang üretir.

**Bu adımda BİTMEYENLER** (her biri kendi doğrulamasını hak ediyor): hava
hızının kontrolcüye bağlanması; `u_actual`'ın 6→11 büyütülmesi (gölge aktüatör
modeli, `hover_trim`, LESO yolu); yüzeyler için `Wu` ve kutu/slew kısıtları;
MATLAB plant'ine yüzey aerodinamiği (yoksa MATLAB tarafı yüzeyleri
DOĞRULAYAMAZ — geri geçişte olduğu gibi doğrulama SITL'de olur); PX4 portu
(`servos.control[0..4]` artık sıfır olmayacak); SITL doğrulaması.

---

### Adım 45 — Yüzeyler tahsisata SOKULDU, İKİ KEZ DENENDİ, İKİSİ DE GERİ ALINDI (2026-08-03)

Adım 44'ün etkinlik matrisi PX4'e taşındı: `N_ACT` 6 → **11**, WLS çözücüsü ve
etkinlik matrisi genişletildi, `q̄` yer hızından hesaplanıp bağlandı, yüzey
kutuları (eklem limiti + slew) ve `WU_SURF = 80` eklendi, `servos.control[0..4]`
artık sıfır değil. **Hover kapısı geçti** (lockup-check: %0.00 doygunluk, 0
BIG_M, irtifa RMS 0.052 m) — q̄≈0'da nötrlük kanıtlandığı gibi çıktı.

Sonra seyirde uçuruldu ve **iki tasarım da ölçümle çöktü.**

**DENEME 1 — yüzeyler tam kuvvet+moment aktüatörü.**

| | seyir tilti | doygunluk | BIG_M |
|---|---|---|---|
| yüzeyler kapalı (Adım 42) | 39.7-44.1° | %0.00 | 0 |
| deneme 1 | **13.3°** | %1.89 | **1958** |

Ölçüm: **üç rotor da `ROTOR_TMAX`'ta** (45.0/45.0/44.9 N), elevonlar +13.3/+13.6°.
Sebep, transkribe ettiğim tablonun içinde duran ama ANLAMINI kaçırdığım bir sayı:
**servo_0/1'in cp'si x = −0.05 m, yani neredeyse AĞIRLIK MERKEZİNDE.** Bunlar pitch
yüzeyi değil, **ana kanadın flapı**: pitch etkinliği 24.5 N·m/rad, Fz etkinliği
490 N/rad — **20 kat fark**. Her kullanım ~107 N aşağı kuvvet (ağırlığın 2 katı)
üretti ve rotorlar bunu yenmek için tavana dayandı. Dahası: **rotorları tiltleyen
mekanizmayı ortadan kaldırdı** — Adım 31 tilti sürenin Fz talebi olduğunu ölçmüştü,
ve o talebi artık flap karşılıyordu.

**DENEME 2 — yüzeyler MOMENT-ONLY (G'nin kuvvet satırları sıfırlandı).** Daha kötü:
**araç TERS DÖNDÜ** — max |roll| 180°, uçuşun **%66.9'u 90°'nin ötesinde**, beş
yüzey de sapma limitlerinde çakılı, 4 ileri geçiş iptali, 22.5 m irtifa sapması.
Sebep bu projenin defalarca ödediği genel hata: **bir satırı sıfırlamak kuvveti
kaldırmaz, yalnızca tahsisattan gizler.** Tahsisat yüzeyleri bedelsiz moment
kaynağı sanıp durdurucuya kadar sürdü, modellenmeyen kuvvet dikey ekseni yıktı.

**GERİ ALINDI ve geri alma DOĞRULANDI** (`SURF_ENABLE = false`): tilt 43.7°,
seyir 14.22 m/s, %0.00 doygunluk, 0 BIG_M, 6/6 ölçüt — Adım 42'nin davranışı
birebir geri geldi. Kod ve ölçülen sonuçlar `decelLoop` disipliniyle korundu:
aynı iki fikir bir daha körlemesine denenmesin diye.

**Üçüncü denemenin çözmesi gereken şey belli:** elevonlar 0.05 m kollu bir
TAŞIMA cihazıdır, moment cihazı değil — yerleri Fz kanalı, ve büyük olasılıkla
bir WLS aktüatörü olarak değil **hızla programlanmış bir flap** olarak. Tahsisatta
yalnızca servo_2/3 (elevator, 0.70 m kol) ve servo_4 (rudder) kalmalı. Bu bir
ağırlık değişikliği değil, tasarım değişikliğidir ve kendi ölçüm kampanyasını
gerektirir.

*Genel ders 1: **tahsisata verilen model fizikle uyuşmak zorundadır.** Bir satırı
sıfırlamak "o etkiyi yok saymak" değil, "tahsisatın onu bedava sanmasını
sağlamak"tır — ve tahsisat aradaki her farkı sömürür. Adım 11 (karesel itki
eşlemesi) ve Adım 30 (fx zayıf amaç terimi) ile aynı aile.*

*Genel ders 2 (asıl olan): **bir sabiti doğru transkribe etmek, ne anlama
geldiğini anlamak değildir.** `cp = (-0.05, ±0.30, 0.05)` satırını Adım 44'te
tabloya doğru yazdım, testle doğruladım, ve "x = −0.05 bu yüzeyin pitch kolu
yok demektir" cümlesini kurmadım. Sayı doğruydu, çıkarım eksikti — ve iki uçuş
gerektirdi.*

---

### Adım 46 — 3. deneme MATLAB'da: **plantin kanadı TERS İŞARETLİYMİŞ**, ve sanal aktüatörler kararlı ama zarf TECS'e takılıyor (2026-08-03)

**PX4'e HİÇ GİTMEDİ.** `safe-control-change` protokolünün 2. adımı gereği MATLAB
kapısı önce koşuldu ve kapı **temiz geçmedi** (aşağıda), bu yüzden SITL'e
gönderilmedi. Adım 45'in iki çöküşünden sonra bu bilinçli bir sıra.

#### (a) Önce bir ön koşul: MATLAB plant'i bu soruyu SORAMIYORDU

İki ayrı kusur ölçüldü (varsayılmadı):

1. **Beş kontrol yüzeyinin hiçbiri plant'te yoktu** ve kanat TEK 0.5 m² panel
   olarak modelleniyordu. Gerçekte gz'de kanat **iki** 0.5 m² yarım panel
   (cp y = ±0.30) ve **beş panelin beşi de** birer kontrol eklemi taşıyor.
   Yüzeysiz bir plant'te yüzey sapması tam sıfır etki üretir — yani madde (V)
   MATLAB'da **yapısal olarak sınanamazdı**.
2. **🔴 Kanadın taşıma işareti TERSTİ.** 15 m/s'de burun yukarı, eski yasa
   **43-67 N AŞAĞI** kuvvet üretiyordu. Zincir: `alpha_eski = atan2(-w,u) =
   -alpha_std` ve `Cl = cla*(alpha - a0)` birlikte doğru değerin tam
   negatifini veriyor. Kuvvet döndürme matrisi de aynı ters işaretle
   yazıldığından hata kendi içinde "tutarlı" görünüyordu. Ayrıca gz'de
   `cd = |cda*alpha|`'dır, sabit değil: seyir hücum açısında (~0.15 rad)
   gerçek değer 0.096, eski model 0.6417 ile **~7 kat fazla** sürüklüyordu.

`aero_panels.m` gz-sim LiftDrag'i birebir port ediyor (stall dahil), `p.aero`
tek kaynak oldu ve `p.surf` ondan **türetiliyor** (kontrolcü modeli ile plant
fiziği arasında sessiz sapma artık yapısal olarak imkânsız).

**Doğrulama — `run_aero_panels_test.m`, 8 başlık, hepsi elde türetilmiş
beklentilere karşı, hepsi geçti.** En güçlüsü bağımsız çapraz kontrol: yeni
plant 14.4 m/s'de θ = −1.5°'de **kanat yükü %40.6** veriyor; Adım 43'ün SITL'de
ölçtüğü bant **%41-50**. Eski plant bu sayıyı hiçbir duruşta üretemiyordu.

Regresyon (protokol adım 2), eski plant yol gölgelemesiyle ölçüldü:

| | ESKİ plant | YENİ plant |
|---|---|---|
| hover RMS p / q (LESO açık) | 0.0014 / 0.0003 | 0.0025 / 0.0027 |
| geçiş: son ortalama tilt | 9.5° | **19.9°** |
| geçiş: ileri hız | 6.70 m/s | **12.91 m/s** |
| geçiş: max \|omega\| | 0.0126 | 0.0126 |

Hover RMS'in artması kontrolcünün kötüleşmesi değil — plant artık **gerçek bir
kuyruğa** sahip, yani bozucu gerçekten büyüdü; `max|omega|` birebir aynı kaldı.
Geçiş tarafında eski plant aynı Fx komutuyla 6.70 m/s veriyordu, SITL'de
ölçülen ~15 m/s; yeni plant 12.91 m/s.

#### (b) 3. tasarım: beş bağımsız yüzey değil, ÜÇ SANAL AKTÜATÖR

Dayanak Adım 31 Faz 0'ın ölçümü: **tilt'i süren şey Fz talebidir.** Dolayısıyla
tahsisata ucuz bir Fz aktüatörü verilemez — verilirse tilt mekanizması ölür,
ki Adım 45/deneme 1 tam olarak buydu.

- **aileron** = antisimetrik elevon → Fz ve τ_y **tam cancel** (sıfırlanarak
  değil, iki gerçek kuvvetin toplamından), saf τ_x = −1.2·q̄
- **elevator** = simetrik servo_2/3 → τ_y = +0.806·q̄, Fz = +1.152·q̄ (satır
  KORUNUR; 0.70 m dürüst kuyruk kolu)
- **rudder** → τ_z = −0.142·q̄
- **simetrik elevon (flap) aktüatör DEĞİL.** SDF'in kendi yorumu da bunu
  söylüyor: *"The x8 elevons act as ailerons here (roll only) - pitch is the
  tailplane's job."*

`run_surface_effectiveness_test.m` bunu **çalıştırılabilir bir gerekçeye**
çevirdi. Ağırlık penceresi kuralı (alt sınır: Fz'yi tilt'ten ucuza çalmamalı;
üst sınır: asıl ekseni için seçilebilmeli):

| sütun | pencere (15 m/s) | seçilen |
|---|---|---|
| elevator | 37.8 < wu < 175.5 | **80** (iki tarafa 2.1-2.2× marj) |
| **simetrik elevon** | **131.2 < wu < 43.5 → TERS, BOŞ** | — (aktüatör değil) |
| aileron | alt sınır YOK (Fz ≡ 0) | 160 |

**Simetrik elevonun penceresinin boş olması, Adım 45/deneme 1'in neden hiçbir
ağırlıkla kurtarılamayacağının sayısal ifadesidir.**

#### (c) Ölçüm: kararlı, ve tam kanat-taşımalı yöne gidiyor — ama zarf sınırlı

fx zarfı tarandı (8/10/12/14 N) ve üç sanal eksen tek tek ablasyona sokuldu
(sütunlar `p.wls.surf_enable` ile **gerçekten kaldırılarak** — ilk denemede
ağırlığı 1e9 yapmak WLS Hessian'ını tekilleştirdi, RCOND 1e-18, o yöntem
atıldı). fx = 10 N, 70 s, tam set:

| | yüzey KAPALI | yüzey AÇIK |
|---|---|---|
| ortalama tilt | 52.1° | **70.6°** |
| kanat yük payı | 98.9% | **106.4%** |
| max \|omega\| | 0.021 | **0.013** (İYİLEŞTİ) |
| kuyruk rotoru T2 | 14.83 N | 13.62 N (zayıf) |

**İki bulgu, ikisi de iki bağımsız fx'te tekrarlandı:**

1. **🔴 Aileron, elevator OLMADAN açıkken ıraksıyor.** fx = 10 ve fx = 12'de
   ıraksayan bütün kombinasyonların ortak yanı bu (`ail`, `ail+rud`). Tam set
   fx = 10'da kararlı.
2. **fx ≥ 12 N'de rejim yüzeyler KAPALIYKEN BİLE marjinal**: doygunluk
   %13.5-16.1 ve kanat tilt'i 89.5°, yani **90°'lik mekanik durakta çakılı.**
   Aracın hızını sınırlayan şey bir kontrol yasası değil, o durak. Yüzeyler
   açılınca bu kazara koruma kalkıyor, araç hızlanmaya devam ediyor ve
   trimleyemediği bir rejime giriyor (t = 29-37 s'de ıraksama).

**(2) bir yüzey kusuru değil, EKSİK BİR KATMANDIR:** seyirde enerji yönetimi
(TECS) yok ve `fx` açık döngü bir hızlandırıcıdır. README bunu zaten "otonomi
için kalan eksik #1" olarak yazıyordu; bu ölçüm onu bir **engelleyiciye**
dönüştürdü — çünkü yüzeyler olmadan o eksiği tilt durağı gizliyordu.

**Kuyruk rotorunun rahatlaması hipotezi yalnızca KISMEN desteklendi:** `ail+ele`
ikilisinde belirgin (14.83 → 11.45 N, fx = 12'de 1.76 N'e kadar), ama rudder
eklenince geri yükseliyor (13.62 N). Ölçülenden fazlası iddia edilmiyor.

#### (d) Durum: PX4'e GİTMEDİ, kod MATLAB'da kalıyor

Protokolün kuralı net: MATLAB temiz değilse SITL'e gönderilmez. `p.wls.surf_enable`
maskesi tam da bu yüzden kalıcı bir özellik olarak eklendi — bir sonraki adım
yüzeyleri kademeli devreye almak ve önce TECS'i kurmaktır, hepsini birden
açmak değil.

---

### Adım 47 — Hız döngüsü kuruldu ve işe yaradı; ama asıl duvar bulundu: **pitch setpoint'i sabit sıfır** (2026-08-03)

Adım 46'nın iki engelinden ikincisi (`fx` açık döngü bir hızlandırıcı) `cruise_speed_loop.m`
ile kapatıldı: PI, bumpless devralma, koşullu integrasyonlu anti-windup.
Kazançlar ölçülen zarftan türetildi (sürükleme eğimi `c = dD/dv ≈ 2.3 N/(m/s)`,
fx 8→10 N ile v 16.01→16.89 m/s'den; `Kp = 1.0` → kapalı çevrim τ = 1.5 s).
Plant'siz mantık testi: `run_cruise_speed_loop_test.m`, 9 denetim, hepsi geçti.

**Ölçüldü (90 s, yüzeyler AÇIK):**

| koşul | u | tilt | kanat% | doygunluk | ıraksama |
|---|---|---|---|---|---|
| açık döngü fx = 10 N | 17.44 | 71.0° | %107.2 | %15.7 | – |
| **açık döngü fx = 12 N** | – | – | – | %60.9 | **36.1 s** |
| **hız döngüsü v_sp = 16** | **15.99** | 36.4° | %89.2 | **%2.5** | – |

Hedef neredeyse tam tutuluyor (15.99 / 16.0) ve **tahsisat doygunluğu
%15.7 → %2.5'e düşüyor** (yüzeysiz + hız döngülü kontrol grubu: %8.0).
Adım 46'nın fx = 12 ıraksaması birebir yeniden üretildi (aynı t = 36.1 s).

#### Ama v_sp ≥ 17'de hâlâ ıraksıyor — ve sebebi ne yüzeyler ne ağırlıklar

`v_sp` tarandı (16…21, `fx_max` geçici olarak 20 N): v_sp = 16 kararlı,
**v_sp ≥ 17 ıraksıyor** (t = 32.0 / 27.2 / 25.7 s). Iraksama imzası Adım 45/
deneme 1'in imzasıyla aynıydı — **tilt 4-8°'ye çöküyor, rotorlar TMAX'ta** — bu
yüzden "elevator'un Fz kanalını çalması" hipotezi kuruldu ve **sınandı:
`wu_ele` = 80 / 160 / 320 / 640.** Sonuç: **hipotez ÇÜRÜDÜ**, dördü de
t ≈ 31.5-32.0 s'de, pratikte aynı anda ıraksadı. 8 kat ağırlık değişimi hiçbir
şey değiştirmedi.

**Asıl sebep, bütün oturumun ölçümlerini tek sayıda birleştiriyor:**

```
theta = 0'da kanadın tam ağırlığı taşıdığı hız:
    V = sqrt( W / (0.5·rho·S·cla·a0) ) = 16.925 m/s
```

Ve ölçülen denge hızları:

| koşul | ulaşılan hız |
|---|---|
| yüzey KAPALI, fx = 8 / 10 / 12 / 14 N | 16.01 / **16.89 / 16.86 / 16.88** |
| yüzey KAPALI, v_sp = 17 / 18 / 19 | **16.89 / 16.90 / 16.85** (hedefe ULAŞAMIYOR) |
| yüzey AÇIK, v_sp = 16 | 15.97 ✅ / v_sp ≥ 17 ❌ |

**Yüzeysiz araç, fx ne olursa olsun her zaman 16.85-16.90 m/s'de duruyordu — bu
bir tilt durağı değil, 16.925 m/s'nin ta kendisi.** Duvarın adı buydu.

**Mekanizma:** `att_sp` pitch bileşeni her yerde SABİT SIFIR. 16.9 m/s üstünde
seviye uçuş NEGATİF hücum açısı gerektirir; araç θ = 0'a çivilendiği için kanat
ağırlıktan fazlasını üretir, tırmanmak zorunda kalır, irtifa döngüsü buna
"taşımayı azalt" diye karşı koyar ve bir şey doyana kadar direnir. Yüzeyler
kapalıyken araç o hıza hiç çıkamadığı için bu hiç görünmedi; yüzeyler açıkken
çıkabiliyor ve duvara toslamış oluyor.

**Yani Adım 46'nın "fx ≥ 12'de rejim marjinal" teşhisi DOĞRUYDU ama
ATIF EKSİKTİ:** sınırlayan şey ne tilt durağı ne de `fx`'in açık döngü
olmasıydı; ikisi de o tek hızın belirtileriydi.

#### Sonuç: TECS'in eksik yarısı pitch

Klasik TECS iki kanalı iki enerjiye bağlar: itki ↔ toplam enerji, **pitch ↔
enerji dağılımı**. Adım 47 bilerek yalnızca ilkini kurdu ve pitch = 0 tercihini
Adım 29'a dayandırdı. **Bu dayanak yanlış genellendi:** Adım 29'un ölçümü
~5-6 m/s rejimindeydi (orada burun yukarı gerçekten bir tırmanma komutudur);
16.9 m/s üstünde ise pitch trimi bir tercih değil bir ZORUNLULUKTUR.

Durum: `p.tecs.v_sp = 16.0` (duvarın güvenli tarafı), `p.tecs.fx_max = 13.0`.
Bu ayarla yüzeyler AÇIK ve kararlı, doygunluk %2.5. **PX4'e hâlâ gitmedi** —
sıradaki iş pitch/enerji-dağılımı yarısı; tam kanat-taşımalı uçuş (rotorların
kapanması) o yarım olmadan 16.9 m/s'nin üstüne çıkamaz.

*Genel ders: **bir sınırın nerede olduğunu ölçmek, neden orada olduğunu bilmek
değildir.** Adım 46 sınırı doğru yerde buldu ("fx ≥ 12 marjinal") ve iki farklı
yanlış sebebe bağladı (tilt durağı, açık döngü fx). Sebep, ikisinin de altında
duran ve tek satırda yazılabilen bir denge koşuluydu. Ve onu bulduran şey bir
teori değil, ÇÜRÜYEN bir hipotezdi (wu_ele taraması) — bu projede Adım 25'in
"premis iki kez çürüdü" kalıbının aynısı.*

---

### Adım 48 — İleri geçiş artık İPTAL OLMUYOR (gereksinim), ve iptalin kaçış yolunun sıfır marjlı olduğu bulundu (2026-08-04)

**GEREKSİNİM (kullanıcı, 2026-08-04):** görev profili tek parçadır — kalkış/iniş
multikopter, uçuş SABİT KANAT, geçişler tilt motorla — ve **ileri geçiş iptal
olmaz.**

#### Önce: iptal yolunun kendisi incelendi ve bir açık bulundu

Soru "geçiş neden iptal edilemiyor" olarak geldi; kod izlendi. İptal **bağlıydı**
(madde (U) gibi boşta bir bayrak değil): `forwardTransition` → `_ft_req_abort`
→ `req_bt = true`. Ama iptalin *ne demek olduğu* ölçüme dayanıyordu: Adım 30
`fx_sp = 0`'ı denemiş ve **araç yavaşlamamıştı** (Fx tahsisatta çok zayıf bir
amaç terimi, tiltleri geri çekmiyor), ayrıca `POS_ENGAGE_V_MAX` 3 m/s üstünde
`pos_hold`'u reddediyor. Yani iptal = **geri geçişi istemek**.

**Ve sabitler o kaçış yolunu sıfır marjla bırakıyordu:**

```
FT_MIN_ALT (20 m)  -  FT_ALT_BAND (5 m)  =  15 m  =  BT_MIN_ALT
```

Minimum irtifada başlayıp **alçalarak** iptal eden bir geçiş, tam olarak geri
geçişin başlamayı reddettiği irtifada iptal ediyordu. O köşede iptal etmek
etmemekten kötüydü: `_ft_state = IDLE`, `fx_cmd = 0` — ve ölçüm bunun aracı
yavaşlatmadığını söylüyor — yani seyir hızında, tiltler önde, 15 m'nin altında,
hiçbir manevranın sahiplenmediği bir araç. **Aynı anda iki bilinen kalıbın
tekrarı:** madde (U) (loglanan eylemin arkasında aktüatör yok) ve madde (R)
(Adım 38: bir çıkışın tam bir sınırın üstünde oturması).

Asimetri önemli: iptal **tırmanarak** tetiklenirse (Adım 29'un imzası) araç
yüksektedir ve geri geçiş sorunsuz devreye girer. Açık, yalnızca alçalan iptale
özgüdür. *Bu bulgu kod okumasıyla elde edildi, uçuşla değil — sav olarak
kaydedilmiştir.*

#### Uygulanan: dedektörler kalır, EYLEM kalkar

`p.ft.allow_abort` / `FT_ALLOW_ABORT = false`. İki emniyet terimi de
**silinmedi**; çalışmaya devam ediyor ve yeni bir `warn_code` çıkışıyla
(1 = irtifa bandı, 2 = süre) dışarı veriliyor, PX4 tarafında yükselen kenarda
`PX4_WARN` ile loglanıyor. Yalnızca `req_abort` üretmeleri kalktı.

**Bedeli açıkça yazıldı:** Adım 29'un kaçış tırmanışı (35 s'de 44 m) artık
otomatik bir tepki üretmez, yalnızca rapor edilir. **Kazancı da ölçülmüştür:**
iptalin garantili bir kaçış yolu zaten yoktu (yukarıdaki sıfır marj). İptal
ileride geri getirilecekse **önce o marj kapatılmalıdır**.

`true` yapmak eski davranışı **birebir** geri getirir ve bu test edilmektedir:
`run_forwardtrans_sm_test.m` artık **iki modu da** koşuyor. Ölçülen: uyarı,
iptalin tetikleneceği **tam aynı anda** çıkıyor (t = 4.56 s her ikisinde de),
yani dedektör değişmedi — sadece eylemi kalktı. 21 denetim, hepsi geçti.

Senkron: MATLAB (`forwardtrans_loop.m`, `tiltrotor_params.m`) + PX4
(`TiltrotorIndiControl.hpp`, `TiltrotorIndiParams.hpp`,
`MulticopterIndiTiltrotor.cpp/.hpp`).

#### Doğrulama (protokolün 3. ve 4. adımları)

**Tam otonom görev — 6/6 GEÇTİ** (`sitl/run_mission_test.py`):

| evre | süre | v_h | tilt |
|---|---|---|---|
| FT RAMP | 14.4 s | 0.21 → 12.43 m/s | 36.1° |
| FT CRUISE | 11.6 s | 12.43 → 14.71 m/s | **42.5°** |
| BT RETRACT | 16.7 s | 14.71 → 9.35 m/s | 42.4° |
| BT BRAKE | 7.3 s | 9.34 → 3.06 m/s | 16.9° |
| BT HANDOFF | 17.2 s | 3.05 → 0.10 m/s | 10.7° |

**iptal ×0**, FT irtifa sapması 0.60 m (band 5 m — yani dedektör hiç
tetiklenmedi), havada NaN 0 örnek, doygunluk/BIG_M %0.00 / 0, duruş boşluğu
max 8.0 ms (pay 6.2×).

**Zorunlu `sitl-lockup-check` — GEÇTİ:** kriter penceresi 30 s, `sat_flag`
%0.00, BIG_M 0, itki 12.16-19.63 N (0/45 kilitlenmesi yok), yaw toplam sapma
**+0.83°**, |vz| max 0.090 m/s, irtifa hata RMS 0.046 m, v_h ort 0.11 m/s.
*Not: log'da 12 adet `Wu1=1000000` var ama hepsi 917 satırın 829-914'ünde,
yani `Disarmed`'ın hemen öncesindeki temas evresinde ve kriter penceresinin
tamamen dışında — yerde rotorun tabana inmesi beklenen davranıştır, skill'in
uyardığı kilitlenme imzası değil.*

---

### Adım 49 — TECS'in pitch yarısı: yasa çalıştı, ve **sıfır itkinin bir uçurum olduğu** ölçüldü (2026-08-04)

Adım 47 duvarı bulmuştu: `theta = 0`'da kanadın tam ağırlığı taşıdığı hız
`V_wb = 16.925 m/s`, ve pitch setpoint'i her yerde sabit sıfırdı. Bu adım o
eksik yarıyı yazdı — `cruise_pitch_loop.m`.

**Yasa:** `theta_dot = -Ki * (Fz_sp - Fz_hedef)`, hız kapısıyla (13→16 m/s
smoothstep) ölçeklenmiş ve ±6° ile sınırlı. Amaç doğrudan: `Fz_sp`
"aktüatörlerin taşıması gereken dikey kuvvet"tir, onu küçültmek kanadın yükü
devralması demektir.

**Kazanç türetmesi (uydurulmadı):** `dL/dtheta = qbar*S*cla = 841 N/rad @ 17
m/s`, kapalı çevrim `tau = 1/(841*Ki)`. İrtifa döngüsünün `tau`'su ~1.7 s
(`Kp_z = 0.6`), trim ondan ~10× yavaş olmalı → `Ki = 5e-5` (tau = 23.8 s).
Plant'siz mantık testi (`run_cruise_pitch_loop_test.m`, 15 denetim) bunu
ölçtü: **ölçülen tau 17.5 s, elde türetilen 17.5 s**; denge pitch'i de elde
türetmeyle birebir (−1.574°).

#### İlk deneme: hedef = 0. **Yasa ÇALIŞTI — ve tam da bu yüzden kırdı.**

`v_sp = 16` (Adım 47'de KARARLI olan nokta) zaman serisi:

| t (s) | u | theta | Fz_sp | tilt | kanat% | T0 |
|---|---|---|---|---|---|---|
| 20 | 15.82 | +0.38° | −16.73 | 41.4° | 93.2 | 5.12 |
| 26 | 16.47 | +0.57° | −12.00 | 59.0° | 104.7 | 4.95 |
| 32 | 15.92 | +0.79° | −9.64 | 65.3° | 106.4 | **0.63** |
| 34 | 17.12 | +0.98° | −7.97 | 67.6° | 120.7 | **0.00** |
| 36 | **27.64** | — | — | — | — | ıraksama |

**t = 32'ye kadar tam olarak istenen şey oluyor:** irtifa sabit (300.7→301.4 m),
hız sabit, tilt 41→65°, kanat yük payı %93→%106, ve gereken pitch yalnızca
**0.8°**. Rotorlar boşalıyor. Sonra `T0` tam **0.00 N**'e değiyor ve araç 2
saniyede ıraksıyor.

**Sebep sayısal değil YAPISAL.** Etkinlik matrisinde tilt sütunu

```
dtau/ddelta = (r x ddir + km*ddir) * T_i
```

yani **itkiyle çarpılıdır**. İtkisi sıfır olan bir tilt rotorunun kontrol
otoritesi de tam sıfırdır — tahsisat o aktüatörü tamamen kaybeder. Yüzeyler
roll/pitch/yaw'ı devralabiliyor ama **Fx kanalının hiçbir yüzeyi yok**; geriye
kalan tek rotor Fz üretmek için kullanıldığında kaçınılmaz olarak `T*sin(delta)`
kadar Fx de üretiyor ve bunu iptal edecek ikinci aktüatör kalmıyor. Hız 27.6
m/s'ye kaçıyor.

**Bu, Adım 43-45'in kök nedeninin ta kendisidir** ("bu airframe'de bütün duruş
otoritesi rotorlardan gelir ve rotorlar kapatılamaz") — ama orada bir mimari
gözlem olarak yazılmıştı; burada bir **sayı** oldu: kırılma `T0 = 0.00 N`'de.

#### Düzeltme: hedef sıfır değil, ölçülmüş bir taban

`p.tecs.pitch_fz_sp = -12.0 N`. Aynı izden okunan değerler: `Fz_sp = -12` →
`T0 ~ 4.95 N` (sağlıklı otorite), `-10` → 1.83, `-8` → kırılma. −12 seçildi:
uçurumdan iki kademe uzakta, **ve orada kanat zaten ağırlığın %105'ini
taşıyor, tilt 59°**.

*Genel ders: **bir amaç fonksiyonunun matematiksel limiti, fiziksel olarak
ulaşılabilir bir nokta olmak zorunda değildir.** "Tam kanat-taşımalı uçuş
= Fz_sp → 0" tanım olarak doğruydu; ama o limitte aracın aktüatörleri yok
oluyor. Doğru hedef limit değil, limitin ölçülmüş güvenli tarafıdır.*

#### Düzeltilmiş yasayla sonuç: 16 m/s'de BÜYÜK kazanç, ama duvar DURUYOR

`v_sp` taraması (yüzeyler açık, pitch kapalı/açık):

| v_sp | pitch | u | tilt | kanat% | T0 | wmax | ıraksama |
|---|---|---|---|---|---|---|---|
| 16 | kapalı | 16.00 | 37.7° | %89.4 | 6.71 | 0.013 | – |
| **16** | **açık** | **16.16** | **60.7°** | **%106.1** | **2.46** | 0.014 | – |
| 17 | kapalı / açık | – | – | – | – | – | 32.0 / 29.9 s |
| 18-20 | kapalı / açık | – | – | – | – | – | ~25-27 s |

**16 m/s'de pitch trimi aranan şeyi veriyor**: aynı kararlılıkla (wmax
0.013→0.014) tilt 37.7 → **60.7°**, kanat yük payı %89.4 → **%106.1**, kanat
rotoru yükü 6.71 → **2.46 N**. Ama **duvarı kaldırmıyor**: 17 m/s üstünde
pitch açık da kapalı da ıraksıyor.

**Bunun geçici rejim mi kalıcı sınır mı olduğu ayrıca ölçüldü.** Trimin
tau'su 17-24 s, ıraksamalar t ≈ 25-32 s'de, yani tam rampa sırasında —
makul bir şüphe. Test: 16 m/s'de 60 s OTUR (trim yerleşsin), sonra
v_sp'yi **90 saniyede** 16 → 19 ramp et (trimden çok daha yavaş):

```
 t=30..50  v_sp=16  u=16.5-16.9  Fz_sp=-11.9..-12.7 (hedefte)  tilt 57-65 deg
           kanat %104-106   T0 = 1.39 .. 3.31 N   fx 7.6-10.2
 t=90      v_sp=17.00  u=16.50  tilt 61 deg  kanat %106  T0=2.78
 -> IRAKSADI t = 96.7 s  (v_sp = 17.22, u = 20.99 m/s)
```

Yavaş rampa da kurtarmadı; üstelik ıraksamadan hemen önce **hız hedefi
aştı** (komut 17.2, gerçek 21.0 m/s). Yani sınır bir trim hızı sorunu değil.

**Ve mekanizma iz boyunca görünüyor:** `T0` hedef etrafında salınıyor
(3.31 → 1.39 → 1.83 → 2.95 → 3.87 → 2.78) ve bir dipte otoritesini
kaybediyor. **Asıl kısıt `Fz_sp` değil, KANAT ROTORU İTKİSİDİR** — `Fz_sp`
çoğunlukla kuyruk rotoru tarafından karşılanıyor (T2 ~ 13 N), dolayısıyla
`Fz_sp = -12 N` hedefi `T0`'ı doğrudan korumuyor.

#### Sıradaki adım, ölçümle işaret edilmiş: kanat rotoruna İTKİ TABANI

`p.rotor.Tmin = 0.0` — yani tahsisatın kanat rotorlarını sıfıra indirmesine
izin var, ve sıfırda otoriteleri de sıfır. İki bağımsız ölçüm aynı yeri
gösteriyor: hedef=0 koşusu tam `T0 = 0.00`'da kırıldı, yavaş rampa ise
`T0 ~ 1.4` dibinde. Doğru araç bir amaç terimi değil, **bir kutu kısıtı**
(seyirde kanat rotorlerine `Tmin > 0`) — Adım 31'in tilt tavanıyla aynı
mekanizma, ve aynı gerekçeyle: *tahsisatın tercihi manevranın tersine
bakıyorsa onu ancak bir kutu değiştirebilir.*

Bu bir kontrol sabiti değişikliğidir; `safe-control-change` protokolü
(hover kapısı dahil) baştan uygulanmalıdır.

---

### Adım 50 — İtki tabanı: **DENENDİ, GERİ ALINDI** — ve duvarın gerçek mekanizması ortaya çıktı (2026-08-04)

Adım 49'un işaret ettiği düzeltme uygulandı: seyirde kanat rotorlarına itki
tabanı (`p.rotor.Tmin_cruise = 4.0 N`, ortalama tilt ile smoothstep açılan,
hover'da tam sıfır). **Daha önce KARARLI olan bir noktayı bozdu** (v_sp = 16,
pitch kapalı: t = 26.9 s'de ıraksama). Zaman serisi sebebi tek başına gösterdi:

| t | T0 | T1 | **T2** | d0 | Fz_sp | alt |
|---|---|---|---|---|---|---|
| 25 | 3.39 | 2.76 | **14.19** | 87.5° | −16.31 | 400.8 |
| 26 | 2.99 | 2.99 | **0.01** | **90.0°** | −5.20 | 400.9 |
| 27 | 3.08 | 3.08 | 0.00 | 90.0° | **+96.42** | 402.5 |
| 28 | 3.17 | 3.17 | 0.30 | 90.0° | +138.07 | 409.9 |

Kanat rotorları tam tabanda çakılı (`T0 = T1 = Tmin_w`), ama **ölen aktüatör
KUYRUK rotoru.** Zincir:

1. Taban, kanat rotorlarını **90° tiltte** itki üretmeye zorluyor.
2. 90°'de bir tilt rotorunun itkisi **hiçbir taşıma üretmez** — yalnızca ileri
   kuvvet. Yani taban, dikey kanala hiçbir şey katmadan araca ivme veriyor.
3. Araç hızlanıyor → kanat ağırlıktan fazlasını taşıyor → irtifa döngüsü
   "taşımayı azalt" diyor → tek dikey aktüatör olan **T2 sıfıra kesiliyor.**
4. Dikey kanal tamamen kayboluyor; `Fz_sp` +96, +138 N'e patlıyor — hiçbir
   aktüatörün üretemeyeceği bir aşağı kuvvet talebi. Araç tırmanarak ıraksıyor.

**GERİ ALINDI** (`Tmin_cruise = 0.0`), kod bilerek bırakıldı (`SURF_ENABLE`
ile aynı disiplin) ki aynı fikir bir daha körlemesine denenmesin.

*Genel ders: **bir aktüatörü "hayatta tutmak" için konan taban, o aktüatörün
o anki YÖNELİMİNİ hesaba katmazsa yalnızca istenmeyen bir kuvvet enjekte
eder.** 90°'de bir tilt rotoru dikey kanal için zaten ölüdür; tabanın koruduğu
şey otorite değil, sadece ileri itki oldu. Adım 45'in "tahsisata verilen model
fizikle uyuşmak zorundadır" dersinin kutu-kısıtı tarafındaki karşılığı.*

#### Duvarın gerçek mekanizması (üç ölçümün ortak noktası)

Adım 49-50'de üç bağımsız ıraksamanın hepsi aynı yere çıkıyor: **araç seyirde
fazla taşımayı ATACAK yol bulamıyor.** Taşma atmanın yalnızca dört yolu var —
(a) rotorları öne yatırmak (90° durağına kadar), (b) itkiyi azaltmak (0'a
kadar), (c) burnu aşağı almak (pitch trimi, ±6°), (d) elevatörle aşağı kuvvet.
(a) ve (b) tükendiğinde `Fz_sp` patlıyor ve dikey kanal kayboluyor.

Bu, `V_wb = 16.925 m/s`'nin neden bir duvar olduğunun tam açıklamasıdır: o
hızın üstünde kanat *sürekli* fazla taşır ve fazlayı atma kapasitesi sonludur.

**Bugün ulaşılabilen ve DOĞRULANAN çalışma noktası:** `v_sp = 16 m/s`, pitch
trimi açık — tilt **60.7°**, kanat yük payı **%106.1**, kanat rotoru yükü
**2.46 N**, `wmax` 0.014 (yüzeysiz referansla aynı). Pratikte "motorlar dik,
kanat taşıyor" budur.

---

### Adım 51 — Duvarın sebebi arandı: **dört aday çürütüldü**, ve kanat oturma açısının KISMİ katkısı ölçüldü (2026-08-04)

Adım 49-50'nin bıraktığı soru: ~17 m/s sınırı bir kontrol açığı mı, yoksa
airframe kapasitesi mi? Dört aday tek tek sınandı — **hepsi çürüdü.**

| aday | sınama | sonuç |
|---|---|---|
| elevator ağırlığı `wu_ele` | 80 / 160 / 320 / 640 | dördü de t ≈ 31.5-32.0 s'de ıraksadı |
| hız döngüsü kazancı `Kp` | 1 / 4 / 8 (Ki orantılı) | üçü de ıraksadı (29.9 / 27.2 / 29.2 s) |
| kanat rotoru itki tabanı | 4.0 N | **daha kötü** — kararlı noktayı bozdu, geri alındı (Adım 50) |
| seyir tilt tavanı | 70° / 65° / 60° | üçü de ıraksadı (31.3 / 31.8 / 32.7 s) |

**Ve duruş kontrolü ıraksamaya kadar KUSURSUZ çalışıyor** (`diag_pitchauth`,
v_sp = 17): t = 28'e kadar `theta` setpoint'i birebir izliyor (+0.25 / +0.25),
pitch talebi `nu_des(tau_y)` ≈ 0.05, elevator sapması 0.11°, `Fz_sp` tam
hedefte (−12.7), kanat yük payı %106.9. Yani kıran şey ne pitch otoritesi ne
de tahsisat doygunluğu.

**Ölçülen mekanizma:** `T0` (kanat rotoru) yavaşça ölüyor —
5.13 → 4.89 → 4.73 → 4.62 → 3.85 → 1.92 → 1.28 → **0.38 N** — ve aynı anda
tilt 56.7° → 64.8°'ye kayıyor. Yüksek tiltte kanat rotorlarının dikey
etkinliği (`cos δ`) düştüğü için tahsisat Fz işini tilt'i ~0° olan KUYRUK
rotoruna devrediyor; kanat rotorları işsiz kalıyor ve `dτ/dδ ∝ T` olduğu için
otoriteleri de onlarla birlikte sıfırlanıyor.

**Tilt tavanı bunu düzeltmedi** — yani tilt kayması bir sebep değil, aynı
dengenin başka bir belirtisi.

#### Kanat oturma açısı: gerçek ama KISMİ bir katkı

Teşhisi iddia olmaktan çıkarmak için `a0` (kanat oturma açısı) düşürüldü —
bu bir kontrol değişikliği değil, **airframe/SDF parametresidir**; amaç
mekanizmayı sınamaktı.

| v_sp | a0 | V_wb | sonuç |
|---|---|---|---|
| 17 | 0.0598 | 16.93 | ıraksama (29.9 s) |
| **17** | **0.030** | **23.90** | **KARARLI** — u 17.29, θ +1.74°, tilt 61.9°, kanat %105.5, T0 3.28 N, wmax **0.013** |
| 20 | 0.0598 | 16.93 | ıraksama (24.9 s) |
| 20 | 0.030 | 23.90 | ıraksama (34.2 s) |
| 23 | 0.0598 | 16.93 | ıraksama (24.9 s) |

**Oturma açısını yarıya indirmek kullanılabilir seyir hızını yükseltti**
(a0 = 0.0598'de 16 kararlı / 17 ıraksıyor → a0 = 0.030'da **17 kararlı** /
20 ıraksıyor). **Ama orantılı DEĞİL:** `V_wb` 16.93 → 23.90'a (1.41×) çıktığı
hâlde sınır ~16.5 → ~18-19 civarında kaldı, ve 20 m/s `V_wb`'nin belirgin
altında olmasına rağmen ıraksadı.

**Dolayısıyla dürüst hüküm: `V_wb` gerçek bir katkıdır ama tek sebep
değildir.** Sınırın geri kalanı AÇIK kalıyor. Bu adımda kesinleşen şey,
sınırın *kontrol tarafındaki dört bariz adayın hiçbiri* olmadığıdır — ki bu,
bir sonraki denemenin nereye BAKMAMASI gerektiğini belirler.

*Genel ders: **dört hipotezi çürütmek bir cevap değildir, ama bir cevaba
giden yolun daraltılmasıdır** — ve bu projede (Adım 25, Adım 45, Adım 49)
çürüyen hipotezler doğru cevabı her seferinde çürümeyenlerden daha çabuk
getirdi. Buradaki fark: bu kez çürüme sonrası kalan aday tek değil, ve
bunu "çözüldü" diye yazmamak sonucun kendisi kadar önemlidir.*

---

### Adım 52 — Kuyruk aşağı yükü: **ilk kez sonucu maddi olarak değiştiren müdahale** (2026-08-04)

Adım 51'in bıraktığı listeden ilk sıradaki aday ölçüldü ve **tuttu** — ama
tam olarak değil.

**Türetme.** Kuyruk panelleri SDF'te `a0 = -0.2 rad` ile kurulu (yorum:
*"a0 = -0.2 gives the download a conventional tail needs to trim a nose-heavy
aircraft"*). Bu, **hız karesiyle büyüyen kalıcı bir AŞAĞI kuvvettir**:

| V | kuyruk aşağı yükü |
|---|---|
| 16 m/s | 14.1 N |
| 20 m/s | 22.0 N |
| 25 m/s | **34.3 N** (ağırlığın %70'i) |

Kontrolcü bu kuvveti **görmez** (aero plant'te, modelde değil), dolayısıyla
tahsisat onu iptal etmeyi hiç denemez — kuvveti sessizce **rotorlar taşır.**
Adım 51'in izinde ölçülen tam da buydu: elevatör sapması yalnızca 0.11°
dururken kuyruk rotoru ~15 N'de sabit, kanat rotorları sıfıra doğru eziliyor.

Elevatör bunu tamamen iptal edebilir ve gereken sapma sabittir (iki terim de
`qbar` ile ölçeklendiği için **tüm hızlarda** geçerli):

```
cla*a0 + rad_to_cl*delta = 0
4.7528*(-0.2) + (-12)*delta = 0   ->   delta = -0.0792 rad = -4.54 deg
```

Bu, madde (P)'nin (`fx_trim`) yüzey tarafındaki karşılığıdır: tahsisatın
göremediği kalıcı bir bias, ancak bir trim terimiyle kapatılabilir.
Uygulama `surf_trim_offset.m` (tek kaynak: hem komut üretiminde hem de
ölçümden sanal koordinata geri projeksiyonda AYNI ofset kullanılır; ayrışırsa
kontrolcü kendi trimini hata sanıp kovalar).

**Ölçüm — v_sp = 17 m/s (trimsiz hâlde t = 29.9 s'de ıraksıyor):**

| trim oranı | ıraksama |
|---|---|
| 0.0 | 29.9 s |
| 0.4 | 32.3 s |
| 0.6 | 34.9 s |
| 0.8 | **75.6 s** |
| 1.0 (tam iptal) | **82.9 s** |

**Monoton ve büyük: 2.8×.** Bu, Adım 51'de elenen dört adayın hiçbirinin
yapamadığı şey — ilk kez sonucu maddi olarak değiştiren müdahale. Ayrıca
**arıza türü değişti**: trimsiz hâlde araç burun aşağı dalıyor (θ ≈ −21°),
trimli hâlde burun yukarı ve yavaşlayarak bitiyor (θ ≈ +20°, u 12 m/s).

**Ama hâlâ ıraksıyor.** Yani kuyruk aşağı yükü büyük bir etkendir, tek etken
değildir — Adım 51'in "sınır çok sebepli" hükmü ayakta kalıyor.

**Tam iptalin ÖTESİ denendi ve daha kötü:** 1.3 → 73.7 s, 1.6 → 64.3 s. Eğri
belirgin bir tepe yapıyor ve **tepe tam olarak analitik iptal değerinde**
(oran = 1.0). Yani sayı ayarlanarak bulunmadı; türetmenin doğruluğunun bağımsız
bir kanıtı çıktı.

#### İki regresyon yakalandı — ikisi de kendi testlerimiz tarafından

1. **Nötrlük garantisi kırıldı.** Ofset ilk yazımda koşulsuzdu; hover'da
   elevatör −4.54°'de duruyordu ve `run_surface_effectiveness_test` E2
   ("hover'da yüzey komutu TAM sıfır") düştü. Aerodinamik olarak zararsızdı
   (`qbar ≈ 0`) ama **garanti garantidir.** Çözüm bir hız kapısı (3 → 8 m/s
   smoothstep) ve **fiziksel olarak bedava**: iptal edilen kuvvet de iptal eden
   kuvvet de `qbar` ile ölçeklendiği için oranları sabit, `qbar = 0`'da ikisi de
   sıfır. E2 yeniden geçti (`max|surf| = 0`).

2. **Harness ile kontrolcü ayrıştı** — ve bu, `surf_trim_offset.m`'in kendi
   başlığında uyardığı tuzağın ta kendisi. `run_cruise_wingborne_test`'te
   ölçümden sanal koordinata geri projeksiyon trimi ÇIKARMIYORDU, yani
   kontrolcü kendi trimini bir hata sanıp kovaladı: t = 8.8 s'de ıraksama,
   elevatör **−29.8°** (sapma limiti), aileron 44.6°, rudder 29.5° — hepsi
   doygun. Düzeltildikten sonra aynı test **7/7 geçti**.

#### Sonuç: trim KALICI (ölçülerek doğrulandı)

`v_sp` 16 m/s çalışma noktasında, yüzeyler açık, trim öncesi → sonrası:

| | trim yok | **trim var** |
|---|---|---|
| ortalama tilt | 70.58° | **70.92°** |
| kanat yük payı | %106.4 | %105.2 |
| kanat rotoru T0 | 1.52 N | **0.40 N** |
| **toplam rotor itkisi** | 16.13 N | **14.14 N** (−%12) |
| max \|ω\| | 0.0126 | 0.0126 |

Trim korunuyor (`p.surf.ele_trim = -0.0792`, hız kapılı). **PX4'e hâlâ
gitmedi** — MATLAB kapısı 17 m/s üstünde temiz değil.

*Genel ders: **bir trim terimi, onu kullanan HER yerde aynı olmalıdır.**
Ofsetin komut tarafına eklenip ölçüm tarafından çıkarılmaması, kontrolcüyü
kendi çıkışını kovalar hâle getirdi — Adım 11/21/27'nin "kontrolcü modeli ile
plant fiziği arasında sessiz sapma" ailesinin trim tarafındaki üyesi. Bu
yüzden ofset tek bir fonksiyonda toplandı.*

#### Son deneme: tilt tavanı + itki tabanı BİRLİKTE — çürüdü

Gerekçe makuldü: ikisi tek başına başarısızdı ama sebepleri birbirini
tamamlıyor gibi görünüyordu — taban 90° tiltte saf ileri kuvvet enjekte ettiği
için (Adım 50), tavan da `T → 0`'ı durdurmadığı için (Adım 51). Birlikte
uygulanırsa tabanın ürettiği itkinin gerçek bir dikey bileşeni olurdu.

**Ölçüm (v_sp = 17, tavan 65°, trim açık):** `Tmin = 0` → t = 9.4 s,
`Tmin = 3` → t = 9.4 s. **İki satır birebir aynı**, yani taban tamamen
etkisiz; zararlı olan tavan — ve tavansız trimli hâlin (82.9 s) **9 katı
daha erken** ıraksıyor.

Tilt tavanı bu rejimde tek başına da, trimle birlikte de **zararlıdır**;
Adım 31'de geri geçiş için doğru olan araç, ileri seyirde doğru araç değil.

*Genel ders: **iki başarısız müdahalenin gerekçeleri birbirini tamamlıyor
görünüyorsa bu, birleşimlerinin çalışacağı anlamına gelmez.** Burada birleşim
her ikisinden de kötü çıktı, ve hangisinin suçlu olduğunu ayıran şey teori
değil, iki satırın BİREBİR aynı olmasıydı.*

#### Bu turda elenen müdahalelerin tam listesi (bir daha denenmesin)

| # | müdahale | sonuç |
|---|---|---|
| 1 | `wu_ele` 80/160/320/640 | hepsi ıraksadı (t ≈ 31.5-32.0 s) |
| 2 | hız döngüsü kazancı Kp 1/4/8 | hepsi ıraksadı |
| 3 | kanat rotoru itki tabanı (4 N) | **daha kötü**, geri alındı |
| 4 | seyir tilt tavanı 70/65/60° | hepsi ıraksadı |
| 5 | kanat oturma açısı `a0` 0.030 | **kısmi** — sınır 16→17, orantısız |
| 6 | tavan + taban birlikte | **çok daha kötü** (9.4 s) |
| ✅ | **kuyruk aşağı yükü trimi** | **2.8× (29.9 → 82.9 s), KALICI** |

Yalnızca (7) sonucu iyileştirdi ve korunuyor; sınırı kaldırmadı.

### Adım 53 — Duvar KALKTI: eksik olan bir kazanç değil, **ileri besleme terimiydi** (2026-08-04)

Adım 49-52 duvarın sebebini beş turda aradı ve altı müdahaleyi çürüttü. Adım 53
duvarı **kanal bazında** ölçtü — her tick'te WLS'in *istenen* (`nu_des`) ve
*başarılan* (`G·du`) sanal kontrolü ayrı ayrı kaydedildi — ve sebep tek bir
cümleye indi:

> **Gereken trim zaten küçüktü ve sınırın çok içindeydi; yasa oraya
> ZAMANINDA varamıyordu.**

19 m/s'de ıraksamadan hemen önce ölçülen: gereken `theta` = **−0.71°**, sınır
6° (otoritenin %12'si), ulaşılan `theta` = **−0.59°**. Yani ne otorite ne
doygunluk sorunu vardı — **yarış kaybedilmişti.** Sebebi yapısal: yasa saf
integraldi (`theta_dot = −Ki·hata`), `tau = 1/(qbar·S·cla·Ki)` ≈ 19-24 s, ve
hız 16 → 19 m/s'ye çıkarken gereken trim de hareket ediyordu. Saf integral,
hareketli bir hedefi tanım gereği geriden izler.

Bu arada Adım 49-52'nin tarif ettiği mekanizma aynen işliyor: kanat fazla
taşıdığı için irtifa döngüsü `Fz_sp`'yi **pozitife** (aşağı kuvvet talebi)
sürüyor — rotorların üretemeyeceği bir işaret — kanat rotorlarının itkisi
sıfıra eziliyor, `dtau/ddelta ∝ T` olduğu için otoriteleri de sıfırlanıyor,
kanat tilt'i 90° mekanik durağa yapışıyor ve araç gidiyor.

**Çözüm bir kazanç değil, eksik terimdi — ve doğru cevap kendi test
dosyamızın içinde yazılıydı.** `run_cruise_pitch_loop_test`'in D2 denetimi
Adım 49'dan beri gereken dengeyi elde türetip yasanın sonucuyla
karşılaştırıyordu:

```
theta_ff(V) = (W + Fz_hedef) / (qbar·S·cla)  −  a0
```

Bu ifade artık yasanın **içinde** (`cruise_pitch_loop.m`); integral yalnızca
**model hatasını** kapatıyor, hedefi kovalamıyor.

#### Ölçüm (kapalı çevrim: hız döngüsü + pitch trim + yüzeyler, 120 s)

| `v_sp` | ileri besleme YOK | **ileri besleme VAR** |
|---|---|---|
| 16 / 17 / 18 | kararlı | kararlı |
| **19** | **ıraksıyor, t = 39.8 s** | **kararlı** |
| **20** | **ıraksıyor, t = 39.4 s** | **kararlı** |
| 22 / 24 | (denenmedi — 19 zaten düşmüştü) | **kararlı** |
| 26 | — | **kararlı**, araç 24.9 m/s'de dengeye oturuyor |

İleri beslemeli halde tilt 51.3° → 77.1°, `Fz_sp` **her hızda tam hedefte**
(−11.8…−12.0 N), `T0` ≈ 7.3 N (sağlıklı otorite), irtifa sapması 1.2 m.

**Ve duvarın kalktığının tanımı budur:** 26 m/s komut edilince araç ıraksamıyor,
**24.9 m/s'de dengeye oturuyor** — çünkü orada hızı sınırlayan şey bir
kararsızlık değil, `p.tecs.fx_max = 13 N`, yani kasıtlı bir emniyet tavanı.
Sınır bir arıza olmaktan çıkıp bir **tercih** oldu.

#### Kazanç artırımı da işe yarıyordu — ve bu yüzden SEÇİLMEDİ

`pitch_ki` 5e-5 → 2e-4 (×4) ölçüldü: 19, 20, 22 ve 24 m/s hepsi kararlı. Yani
teşhis iki bağımsız yoldan doğrulandı. Ama Ki'nin türetmesi bir **garantiye**
dayanıyor: trim, irtifa döngüsünden (`tau` ≈ 1.7 s) en az 10× yavaş olmalı.
×4 bunu 3.5×'e düşürürdü. İleri besleme aynı sonucu **bedava** veriyor:
`pitch_ki` **değişmedi** ve `E1` denetimi `tau`yu hâlâ 17.5 s ölçüyor
(türetilen 17.5 s).

*Genel ders: **bir döngü hedefine geç varıyorsa, ilk refleks kazancı
büyütmektir; ama hedef kapalı formda biliniyorsa doğru cevap ileri beslemedir.**
Kazanç, garantiyi bozarak satın alır; ileri besleme, garantiyi koruyarak verir.
Burada kapalı form zaten TESTIN içinde duruyordu — yasanın içinde değil.*

#### Üçüncü bir yol denendi ve neden yetmediği ölçüldü

`theta` doğrudan analitik değerine sabitlendi (19 m/s için −0.71°, döngü
kapalı): **19 m/s kararlı, 20 m/s t = 53.9 s'de ıraksadı.** Yani gereken trim
hıza bağlıdır ve bir hıza göre ayarlanmış sabit, diğerinde yetmez — ileri
beslemenin `V`'ye bağlı olması bir süs değil, ölçülmüş bir gereklilik.

#### Testin kendisi de değişti (5 denetim güncellendi, 2 yeni)

İleri besleme, testin üç değişmezini geçersiz kıldı — hepsi **mutlak sıfır**
etrafında yazılmıştı, oysa yasa artık `th_ff` etrafında çalışıyor:

- `C1`/`C2` (işaret) ve `C3` (simetri) → ölçüt `th_ff` noktasına göre. 20
  m/s'de `th_ff` zaten −1.574°, yani "burun yukarı" mutlak pozitif olmak değil,
  `th_ff`'in üstüne çıkmak demek.
- `E1` (zaman sabiti) → ölçüm artık ileri besleme uygulandıktan **sonra**
  başlıyor. Aksi halde ilk tick'teki `th_ff` sıçraması hatanın çoğunu bir anda
  kapatıyor ve 1/e eşiği integral hiç çalışmadan aşılıyordu (bir kez
  ölçüldü: 2.4 s, türetilen 17.5 s). Model hatası da artık **açıkça** veriliyor
  (oyuncak plant `a0 + 0.02` ile kuruluyor), yani E1 gerçekten integralin
  hızını ölçüyor.
- **Yeni `D3`:** ileri besleme TEK BAŞINA (`Ki = 0`, geçmiş yok, tek çağrı)
  doğru noktayı vermeli. Bu, D1/D2'nin **yakalayamayacağı** bir hatayı yakalar:
  ters işaretli ya da `a0`'ı eksik bir `th_ff`'i integral yine de yavaşça
  kapatırdı ve D1/D2 geçerdi.
- **Yeni `E0`:** model hatası integralle kapanıyor (`I → −da0`).

#### Ve bir gerçek kusur, testin kendisi tarafından yakalandı

İlk yazımda integratörün sert kırpması `±pitch_max`, koşullu integrasyonu ise
`th_ff + I` üzerindeydi — **iki farklı sınıra bakıyorlardı.** Sonuç sessiz bir
otorite kaybı: `th_ff` ters işaretliyken sert kırpma önce bağlıyor ve toplam
sınıra **hiç ulaşamıyor** (20 m/s'de 6.0° yerine **4.4°**). Eski `F1` bunu
göremezdi çünkü yalnızca `|pitch| ≤ pitch_max` soruyordu — 4.4° de bu şartı
sağlıyor. `F1` "TAM sınırda doyuyor"a çevrildi ve kırpma da toplama göre
yazıldı (`I ∈ [−lim − th_ff, lim − th_ff]`).

*Genel ders: **bir doygunluk denetimi "sınırı aşmıyor" değil, "sınıra
ulaşıyor" da sormalıdır.** Eksik doyma, aşırı doyma kadar gerçek bir arızadır
ve tek taraflı yazılmış bir denetim onu hiç göstermez.*

#### İki dürüstlük notu

1. **Adım 52'nin "17 m/s'de 82.9 s'de ıraksıyor" ölçümü bu turda yeniden
   üretilmedi.** Bu turun probe'unda 17 ve 18 m/s **150 s boyunca kararlı**;
   ıraksama 19'da başlıyor. İki probe birebir aynı değil (Adım 52'nin probe'u
   saklanmadı) — başlangıç irtifası, `fx` rampası ve pitch döngüsünün devrede
   olup olmaması farklı olabilir. **Ölçülen mekanizma aynı** (`Fz_sp` pozitife
   dönüyor, `T0 → 0`, tilt 90° durakta), o yüzden düzeltme aynı arızayı
   hedefliyor; ama duvarın sayısal yeri iki probe arasında farklı ve bu
   raporda 19-20 m/s olarak **bu turun probe'una göre** yazılmıştır.
2. **Kuyruk trimi, bu kapalı çevrim rejiminde aşağı yükü KALDIRMIYOR.**
   Ölçüldü: trim açıkken de kuyruk aşağı yükü 14.8-18.1 N ve fiziksel elevatör
   **nötre yakın** (−0.1…−3.0°), çünkü tahsisat sanal elevatörü ~+4.4°'de
   park ediyor ve trimi fiilen geri alıyor. Trim yine de **kritik**: kapatıldığında
   16 m/s'de kanat tilt'i 90° mekanik durağa yapışıyor ve 20 m/s **t = 49.7
   s'de ıraksıyor**. Yani trimin bu rejimdeki faydası "yükü kaldırması" değil,
   **kontrolcünün modeli ile plant'in aynı şeyi söylemesi** (sanal koordinat
   ofsetin üstündeki sapmadır, dolayısıyla `nu0` aşağı yükü içerir). Adım
   52'nin ölçümü açık çevrim senaryosunda yapılmıştı; ikisi çelişmiyor, ama
   trimin **niçin** işe yaradığına dair açıklama düzeltilmelidir.

#### Doğrulama (safe-control-change protokolü)

- `run_cruise_pitch_loop_test` → **17/17 GEÇTİ** (15 idi, 2 yeni denetim)
- `run_hover_gust_test` → RMS p 0.0025 / q 0.0027 — **birebir aynı**
- `run_transition_test` → tilt 19.9°, u 12.91 m/s, dz 0.42 m, max|ω| 0.0126 —
  **birebir aynı**
- `run_cruise_wingborne_test` → 7/7, kanat %98.9→%105.2, itki 24.0→14.1 N —
  **birebir aynı**
- `run_surface_effectiveness_test`, `run_cruise_speed_loop_test`,
  `run_forwardtrans_sm_test`, `run_backtrans_sm_test` → hepsi GEÇTİ

Birebir aynılık beklenen sonuçtur: değişiklik yalnızca seyir pitch trim yolunu
etkiliyor ve bu testlerin hiçbiri `att_sp`'yi o yoldan almıyor.

**PX4'e GİTMEDİ.** `cruise_pitch_loop`/`cruise_speed_loop`'un PX4 karşılığı
zaten yok (Adım 47-52 ile aynı durum); `mc_indi_tiltrotor` içinde arandı ve
bulunmadı. SITL doğrulaması bu yüzden uygulanamaz — port edildiğinde
`sitl-lockup-check` şarttır.

#### Bundan sonra ne AÇIK kaldı

Duvar kalktı ama **tam kanat-taşımalı uçuş hâlâ değil**: `Fz_sp` hedefi
−12 N olduğu için rotorlar tasarım gereği ağırlığın ~%24'ünü taşımaya devam
ediyor (kanat yük payı %75.6 → %86.3). Bu bir kusur değil, Adım 49'un
**ölçülmüş** kararı: sıfır itki bir uçurumdur (`dtau/ddelta ∝ T`). Rotorların
gerçekten kapanabilmesi için Fx kanalına bir **yüzey** gerekir — bugün hiç yok
— ve bu, madde (V)'nin kalan kısmıdır.

1. **Sorun saf MATLAB'da yeniden üretilmiyor** — yalnızca SITL'e (gerçek
   Gazebo fiziği + PX4 zamanlama) özgü.
2. **Dört bağımsız düzeltme denendi (kutu genişliği, ağırlık oranı,
   mekanik km sabiti, mekanik Y-işareti) — hiçbiri tek başına yeterli
   olmadı**; biri (Y-işareti) MATLAB'da doğrulanmış regresyona bile
   neden oldu ve geri alındı.
3. **LESO yaw ekseninde kapalı** (`leso_enable=[1,1,0]`), yani
   gözlenen yaw savrulması LESO drift'i değil — gerçek bir kalıcı
   fiziksel/geometrik hata ya da düşük `Ws_yaw=3` önceliğinin bunu
   durduramaması olmalı.
4. Kısa (≤10s) testler yanıltıcı olabilir — aday çözüm 3'teki gerçek
   iyileşme yalnızca ~13s sonra ortaya çıkan ikinci, daha kötü bir modla
   maskelendi. **Doğrulama testleri en az 20-25s sürmeli.**
5. **Mekanik model düzeltmeleri (SDF'ye sadakat) tek başına güvenilir
   değil** — km büyüklüğü düzeltmesi zararsızdı (aday çözüm 3), ama Y
   işareti düzeltmesi tahmine dayalıydı ve zararlıydı (aday çözüm 4).
   Herhangi bir mekanik sabit değişikliği önce saf MATLAB'da
   (plant+kontrolcü kontrollü karşılaştırması) doğrulanmadan SITL'e
   taşınmamalı.
6. **"Roll işareti tam ters" hipotezi büyük ölçüde çürütüldü** (Adım 5) —
   sistem ilk ~15s kararlı davranıyor, tam ters işaret anlık kararsızlık
   verirdi. gz-sim'in itki formülü ham kaynaktan doğrulandı; km
   işaret-eşleşmesi doğru, ama pose/frame zinciri statik dosya
   okumasıyla kesin çözülemedi.
7. **`nu_des(2)` (yaw) LESO'nun kolayca telafi edebileceği bir bozucu
   değil** (Adım 6) — LESO açıldığında bile ~17s aynı kalıp sonra FARKLI
   bir kalıcı değere sıçradı, sıfıra hiç yakınsamadı. Ama LESO'yu açmak
   **roll/pitch'i belirgin biçimde iyileştirdi** — bu, tutulmaya değer,
   kısmen faydalı bir bulgu (henüz kalıcı yapılmadı, çünkü yaw kendisi
   hâlâ çözülmedi).
8. **`Ws_yaw` sınırlayıcı faktör DEĞİL** (Adım 7) — 3'ten 6'ya çıkarmak
   yaw'ın tepki HIZINI hiç değiştirmedi (aynı ~14s boyunca aynı
   büyüklükte kaldı) ve yan etkileri (Fx -28N'e kadar, roll daha büyük)
   kötüleştirdi. **WLS ağırlıklandırma katmanında arama yolu artık
   büyük ölçüde tükenmiş durumda** — kalan iki bilinmeyen: (a) hâlâ
   doğrulanamamış bir geometri/frame hatası, (b) `Kp_att[2]`/`Kp_rate[2]`
   (P/rate kazançları) SITL'in gerçek dinamiği için çok düşük/yanlış
   olabilir — bu hiç test edilmedi.
9. **Evet, gerçek bir Gazebo-kaynaklı katkı var, ampirik olarak
   doğrulandı** (Adım 8) — kontrolcünün `_u_actual` gölge aktüatör
   modeli Gazebo'dan HİÇ geri besleme okumuyor; tilt servoları gerçekte
   basit bir gecikme değil, tork-sınırlı (`cmd_max=2`) bir P=100/D=0
   PID + sürtünme (`friction=1.0`) ile çalışıyor. Canlı ölçümde kuyruk
   tilt'inde (δ2) gölge/gerçek arasında 2.6-3.7°'lik, işaret değiştiren
   bir sapma bulundu — kanat rotorlerinde (δ0/δ1) çok daha küçük
   (<0.5°). Bu **saf MATLAB'da yapısal olarak imkânsız** (plant+kontrolcü
   aynı basit modeli paylaşıyor) ve **tek başına yeterli büyüklükte
   görünmese de gerçek bir katkı payı** — muhtemelen Adım 1-7'nin
   bulduğu kırılgan WLS dengesini tetikleyen/büyüten bir faktör.
10. **`TILT_RATE_MAX` düşürmek İLK KEZ net, ölçülebilir bir iyileşme
    sağladı** (Adım 9, KALICI YAPILDI) — roll/pitch iki koşuda da
    sınırlı/küçük kaldı (biri neredeyse mükemmel: 0.1°/0.0°), dikey hız
    kontrolsüzlüğü (`vz=-11 m/s` felaketi) tekrarlanmadı (`vz≈-0.13
    m/s`), yaw sınırsız savrulmak yerine sabit bir platoya oturdu.
    **Ama tam çözüm değil** — bir kanat rotorü hâlâ periyodik olarak
    0'a kilitleniyor, bu sefer sonucu flip riski değil "yetersiz itki,
    tırmanış duruyor" oldu. Adım 8/9, Gazebo'nun gerçek aktüatör
    dinamiğinin sorunun GERÇEK bir katkı payı olduğunu doğruladı —
    kullanıcının "Gazebo ile ilgili olabilir mi" sorusuna kanıtlı EVET.
11. **"Yaw'a daha fazla otorite ver" ailesi ARTIK TÜKENDİ** (Adım 10) —
    `Kp_att[2]`/`Kp_rate[2]` artırmak, MATLAB'da bile roll↔yaw limit
    cycle'ı tetikleyip RMS'i ~10-100× kötüleştirdi (PX4'e hiç gitmedi).
    Bu, `Ws_yaw` artırmanın (Adım 7) başarısızlığıyla birlikte, yaw
    kazancını/önceliğini artırmaya dayalı ÜÇ ayrı yaklaşımın (LESO açma,
    Ws_yaw, Kp) hiçbirinin işe yaramadığını gösteriyor — roll↔yaw
    paylaşımlı-aktüatör kısıtı gerçek bir tasarım duvarı. **Tek başarılı
    yön (Adım 9) yaw'ı GÜÇLENDİRMEK değil, aktüatör-dinamiği
    gerçekçiliğiydi** — sonraki çalışma bu hatta devam etmeli.
    *(Adım 11 sonrası düzeltme: bu üç denemenin hepsi YANLIŞ bir G
    matrisi altında yapılmıştı — bkz. çıkarım 12/15 ve §4 (H). SITL'de
    alınan sonuçları (Adım 6, 7) artık şüpheli saymak gerekir; yalnızca
    Adım 10'un MATLAB'da yakalanan regresyonu bundan bağımsız geçerli.)*

12. **KÖK NEDEN BULUNDU (Adım 11): itki komut eşlemesi karesel Gazebo
    motor modelini doğrusal sanıyordu.** `motors.control[i] =
    u_cmd(i)/ROTOR_TMAX` doğrusaldı, oysa gerçek zincir
    `w = 10 + control·1490` (doğrusal ölçekleme) + `T = 2e-5·w²`
    (karesel) — iki model yalnızca `control=1` uç noktasında uyuşuyor,
    bu yüzden 10 adım boyunca fark edilmedi. İki zararı vardı:
    (a) gerçek toplam itki ağırlığın altında kalıyordu (43.7/28.8/30.0/
    44.6 N vs 49.05 N — araç kalkamazdı), (b) etkinlik matrisi G
    itkiye bağlı biçimde yanlıştı (gerçek `d(T)/du` 5 N'da 0.23,
    45 N'da 1.99; kontrolcü hep 1.0 sanıyordu), bu da düşük itkili
    rotoru tabana iten bir **pozitif geri besleme** yaratıyordu.
    Düzeltildi (`thrustToNormalized()`, karekök tersleme). **Sorunun
    saf MATLAB'da hiç görülmemesinin kesin açıklaması budur** —
    MATLAB plant'i Newton ile sürülüyor, normalize komut katmanı yok.
13. **Kilitlenme roll+Fz üst üste binmesidir, yaw DEĞİL** (Adım 11b,
    eksen-bazlı `WRdbg` atıf logu). Roll talebi iki kanat rotorünü zıt,
    Fz talebi aynı yönde iter; ikisi bir rotorde aynı işaretli
    denk geldiğinde toplanıp onu tabana sürüyor, diğerinde götürüyor.
    `a[yaw]` kanat rotoru itki kanallarında ölçülen 51 örneğin
    50'sinde **tam 0.000** — yaw talebinin bu kanallarda pratikte hiç
    kaldıracı yok. Bu, çıkarım 11'i (yaw-otoritesi ailesinin
    başarısızlığı) ampirik değil **yapısal** olarak açıklıyor.
14. **Bu düzeltmelerden önceki TÜM SITL koşuları yanlış senaryoydu**
    (Adım 11e) — `test_sp`'nin fonksiyon-yerel `uORB::Publication`'ı
    dönerken `orb_unadvertise()` çağırdığı için setpoint kontrolcüye
    HİÇ ulaşmıyordu (60 ardışık yayından sonra bile topic
    "never published"). Aday çözüm 1'den Adım 10'a kadar her koşu
    "6 m tırmanış" değil, "mevcut irtifayı koru + tüm attitude
    setpoint'leri 0" testiydi. Yaw savrulması gerçekti (yaw_sp zaten
    0'dı) ama Adım 9'un "tırmanış durdu" yorumu yanlış atfedilmişti.
15. **Sonuç durumu (Adım 11f, iki bağımsız koşu):** aktüatör
    kilitlenmesi ✅ çözüldü (40s/82 örnek, sıfır BIG_M sabitlemesi),
    dikey hız kontrolsüzlüğü ✅ çözüldü (\|vz\| ≤ 0.20 m/s), irtifa
    takibi ✅ çalışıyor (6 m tırmanış, ~0.15 m hata), roll/pitch ✅
    ≤0.5°. Yaw ❌ hâlâ ±30° kriterini geçmiyor (-56.4°) ama artık
    sınırsız savrulma değil, sönümlü/kendini düzelten gezinme.
    **Adım 6/7/10'un yaw sonuçları artık şüpheli** — hepsi yanlış bir
    G matrisi altında alınmıştı, tekrar denenmeli (bkz. §4 (H)).

16. **KÖK NEDEN #2 (Adım 12): `ROTOR_KM` işaretleri FRD'de tersti.**
    gz gövdeye `tau_z(FLU) = -turningDirection·T·km` uygular; FLU→FRD z
    çevrimiyle bu `+km·T` olur, model ise `m_z = -km·T` diyordu. Adım 3
    yalnızca işaret DESENİNİ eşlemiş, FRD'deki toplam işareti hiç
    karşılaştırmamıştı. `hoverTrim()`'in yaw sıfırlayıcı tilt'i bu yüzden
    dengesizliği **artırıyordu**. Arm anındaki tepe yaw ivmesi ölçümü
    teşhisi doğruladı (+6.45/+6.56 ölçülen vs +6.2 öngörülen; eski model
    ≈0 öngörüyordu) ve düzeltme sonrası **+0.47 rad/s²'ye düştü (14×)**.
    Adım 11 ile aynı sınıftan: MATLAB plant+kontrolcü aynı `km`'yi
    paylaştığı için bu hatayı yapısal olarak göremez.
17. **"±60° sınırlı yaw gezinmesi" bir ÖLÇÜM ARTEFAKTIYDI (Adım 12b).**
    5 s aralıklı `px4-listener` örneklemesi sürekli dönüşü (ort. +0.79
    rad/s, 40 s'de 1818° = 5 tur) rastgele açılara dönüştürmüştü.
    **Yaw ekseni açıdan değil hızdan (`vehicle_angular_velocity.xyz[2]`)
    ve tercihen ulog'dan ölçülmeli.** Adım 11f'in "sönümlü gezinme"
    yorumu bu nedenle geri alındı.
18. **Kalan yaw dönüşünün mekanizması ölçüldü (Adım 12g), artık tahmin
    değil:** (a) tahsisat talep edilen yaw torkunun **%6.8'ini** üretiyor
    çünkü `ddelta` sürekli `TILT_RATE_MAX·dt = 0.005 rad` slew limitinde
    doygun (yaw'ın tek gerçek aktüatörü kanat tilt'i), (b) araç dönerken
    heading sarmalandığı için dış attitude döngüsü yaw hız setpoint'ini
    ±`RATE_SP_LIMIT`=3 arasında sürekli işaret değiştirerek **kendi
    kendini bozuyor**. İki mekanizma birbirini besliyor.
19. **LESO yaw ekseninde AÇILMAMALI — kesin (Adım 12a).** Düzeltilmiş G
    matrisi altında bile 5 saniyede aracı ters çevirdi (`d_hat[2]`→377,
    roll -178°, 160 BIG_M). Adım 6'nın "kısmen olumlu" sonucu geçersiz.

20. **YAW ÇÖZÜLDÜ (Adım 13) — ve çözüm otorite artırmak DEĞİLDİ.**
    Dış döngünün yaw hız setpoint limitini 3.0 → 0.5 rad/s indirmek
    (eksen bazlı) yaw hızı RMS'ini 33×, integre dönüşü 1879°'den 44°'ye
    düşürdü ve yaw'ı ±1.6°'de tuttu; `sitl-lockup-check`'in üç kriteri
    ilk kez aynı koşuda geçti. **Adım 6/7/10'da denenen üç "yaw'a daha
    fazla otorite ver" yaklaşımının (LESO, Ws_yaw, Kp) hepsi yanlış
    soruyu çözüyormuş:** sorun otoritenin azlığı değil, dış döngünün
    ürettiği komutun tutarsızlığıydı. Zayıf otoriteli bir eksende dış
    döngü limiti, o eksenin gerçekten ulaşabileceği hızın ALTINDA
    tutulmalı — yoksa dış döngü iç döngünün sönümlemesini engeller.
21. **Yaw'ı düzeltmek roll/pitch'i de düzeltti** (hız RMS 30-65× daha
    iyi) — dönüş bu eksenlere sızıyormuş. Ölçüm penceresi aynı.
22. **Adım 9'un `TILT_RATE_MAX=2.0` kararı ölçümle DOĞRULANDI** (Adım 14).
    3.0'a çıkarmak hover'da iyileştiriyordu (yaw 21 s yerine 6 s'de
    oturdu) ama +30° yaw adımında aracı tam tura sokan bir aşıma yol
    açtı — iki kez tekrarlandı. Sebep, Adım 8'in ölçtüğü olguydu: yüksek
    varsayılan slew hızıyla WLS, gerçek tork-sınırlı Gazebo servosunun
    tek tick'te veremeyeceği tilt düzeltmelerine bel bağlıyor ve gölge
    model gerçeklikten ayrışıyor. §4 (I)'deki "gerekçesi zayıfladı"
    değerlendirmesi **yanlıştı, geri alındı**.
23. **Bu airframe hover'da GERİ kuvvet üretemez — yapısal** (Adım 15).
    Tüm tiltler `[0, π/2]` aralığında olduğu için net `Fx ≥ 0`. Yaw
    dengesi için gereken diferansiyel tilt kaçınılmaz olarak ~3 N ileri
    itki doğuruyor; `WS_FX=0.05` bunu cezalandırmıyor; kuyruk tilt'i de
    (PY=0) yalnızca +Fx verir. Sonuç: pozisyon döngüsü olmadığı için araç
    sürekli hızlanıp ~10 m/s'ye ulaşıyor (25 s'de 235 m).
24. **EN ÖNEMLİ SINIRLAMA (Adım 16): yaw'ı sönümleyen şey kontrolcü
    değil, ileri hızdaki aerodinamiktir.** Tek uçuşta kontrollü A/B
    (tek değişken hız): 11.6 m/s'de +30° adımı monoton ve temiz (~%13
    aşım), 2.45 m/s'de **±25°'lik sönümsüz salınım**, 15 s'de oturmuyor.
    Pitch trim değişkeni ayrıca elendi. Mekanizma: dikey kuyruk/gövde
    yüzeylerinin (5 `lift-drag` eklentisi) rüzgâr gülü sönümlemesi;
    yaw'ın kendi otoritesi ise ~0.05 N·m/adım. **Çıkarım 23 ile birlikte
    bunun anlamı: bugüne kadarki TÜM "hover" doğrulamaları aslında
    ~10 m/s seyir uçuşuydu.** Gerçek anlamda yerinde duran hover — bir
    pozisyon kontrolcüsünün komut edeceği asıl durum — yaw için en kötü
    koşul ve orada kriter sağlanmıyor. Donanım durumu bu nedenle 🔴
    NO-GO'ya geri alındı.
25. **Yöntem dersi:** Bu oturumda iki kez, verinin ilk okunuşu yanlış bir
    hipoteze götürdü ve ikisini de **eski koşuların ulog'una geri dönüp**
    düzeltmek gerekti (Adım 12b: 5 s'lik açı örneklemesi sürekli dönüşü
    "gezinme" sandırdı; Adım 16: "yüksek hızda bozuluyor" sanılan ilişki
    tam tersiymiş). **Bir hipotezi, kontrol değişkenini aynı uçuşta
    değiştiren bir A/B ile doğrulamadan rapora yazmayın**; ve her koşuda
    hız/hava hızı gibi bağlam değişkenlerini kaydedin.
26. **Adım 16'nın MEKANİZMASI yetersiz, ÖLÇÜMÜ geçerli (Adım 17).** Saf
    MATLAB plant'inde `M_aero(3) ≡ 0` — yani **hiçbir hızda aerodinamik yaw
    momenti yok** (`F_aero(2)=0` ve `r_cp=[-0.05;0;0.05]` olduğundan
    `cross(r_cp,F_aero)` z bileşeni özdeş sıfır). Buna rağmen ±30° yaw
    adımı 3.1-3.7 s'de, **kalıcı salınım olmadan** oturuyor (son 5 s yaw
    hızı RMS ≤ 0.0001 rad/s; SITL'in (Q) koşusunda ±0.5 rad/s). Dolayısıyla
    "düşük hızda rüzgâr gülü sönümlemesi kayboluyor" tek başına (Q)'yu
    açıklamıyor: kontrolcü hiç aero sönümleme olmadan da bu adımı
    sönümleyebiliyor. SITL'de **ek, SITL'e özgü bir kararsızlaştırıcı**
    olmalı; aero sönümleme onu yalnızca maskeliyor. **Genel ders: bir
    ortamda ölçülen korelasyonu, o korelasyonu üretemeyecek bir ortamda
    kontrol etmek, mekanizma iddiasını ucuza sınamanın en hızlı yolu.**
27. **MATLAB ile SITL `TILT_RATE_MAX`'ta ters işaretli — ve bu teşhis
    edici (Adım 17).** MATLAB'da 3.0, 2.0'dan açıkça daha iyi (+30° aşımı
    59.2% → 24.0%); SITL'de (Adım 14) 3.0 aracı tam tura sokuyordu. MATLAB'ın
    tilt servosu komut edilen slew limitine birebir uyduğu için limiti
    yükseltmek orada her zaman yardım eder. Fark, MATLAB'ın sahip olmadığı
    şeyde: `_u_actual` **açık çevrim gölge modeli**
    (`MulticopterIndiTiltrotor.cpp:414-421`, Gazebo'dan sıfır geri besleme)
    ile gerçek tork-sınırlı (`cmd_max=2`, P=100/D=0, `friction=1.0`) Gazebo
    servosu arasındaki sapma (Adım 8: δ2'de 2.6-3.7°). INDI'nin
    lineerleştirme noktası ve WLS'in G matrisi bu `u_actual`'dan türediği
    için sapma doğrudan kontrol yasasını bozar. **Adım 11 ve 12 ile aynı
    sınıf: kontrolcü/plant arayüz uyuşmazlığı — saf MATLAB üçünü de yapısal
    olarak göremez** (MATLAB'da `u_actual = x(14:19)`, yani gerçek durum).
28. **(P) MATLAB'da yeniden üretildi ve mekanizması kanıtlandı (Adım 17).**
    Yön asimetrisi gerçek ve PX4'ün slew limitinde keskinleşiyor (+30° aşımı
    24.0% → 59.2%, −30° 7.7% → 8.0%; 7.4× asimetri). Sebep: hover trim'de
    **δ1 ≡ 0**, tek yönlü tilt aralığının (`p.tilt.min = 0`) tam tabanında.
    `τ_z = -0.25·T0·sin δ0 + 0.25·T1·sin δ1` olduğundan −yaw yalnızca δ0'ı
    artırmayı gerektirir (serbest), +yaw ise δ1'i 0 tabanından kaldırmayı
    (sınıra vuruyor). Artık (P) ucuz ve güvenli bir ortamda çalışılabilir.

29. **Gölge aktüatör modeli sapması ÖLÇÜLDÜ ve (Q)'nun baskın nedeni DEĞİL
    (Adım 18).** Adım 17'nin "en güçlü aday" değerlendirmesi düzeltildi:
    gerçek Gazebo eklem açısıyla `_u_actual` arasındaki fark **p99'da
    ≤ 0.55°**, kalıcı ofset δ1'de −0.41°, δ2'de −0.52°, δ0'da −0.01°. Aynı
    pencerede yaw **73.7°** bantta salınıyor — iki mertebe fark. Sapma
    gerçek ama katkı payı; sürücü değil.
30. **Sınırda YAPISAL bir tahsisat hatası var (Adım 18).** Gölge δ1
    örneklerin %75'inde, δ2 %97'sinde **tam `0.000°`** okurken gerçek
    eklemler **0.53°**'de duruyor — çünkü tork-sınırlı P servosunun
    (`p_gain=100`, `cmd_max=2`, `err_max=0.2`, I=0) kalıcı konum ofseti var
    ve 1. derece gölge model bunu üretemez. `abs_lo = TILT_MIN - _u_actual`
    bu yüzden 0 çıkıyor: WLS "δ1/δ2 aşağı inemez" sanıyor, gerçekte 0.53°
    alan var. Yaw'ın tek gerçek aktüatöründe allocator'dan gizlenen otorite.
31. **(Q) bir ADIM YANITI kusuru değil, DENGE KARARSIZLIĞI (Adım 18).**
    Sürekli salınım `yaw_sp = 0` iken gözlendi: bant 73.7°, periyot 5.18 s,
    r RMS 0.417 rad/s, 1.62 m/s'de. Adım 16 olguyu doğru yakaladı ama adım
    yanıtı üzerinden dar tarif etti. Doğru ifade: **düşük hızda yaw ekseninin
    kararlı bir dengesi yok.**
32. **(P) ve (Q) muhtemelen AYNI KÖK NEDEN (Adım 18).** Salınımın en güçlü
    korelatı δ1'in `TILT_MIN=0` sınırından kalkıp geri çakılması: salınım
    penceresinde 12 kalkış olayı, salınım durduktan sonra sıfır; δ0 aynı
    periyotta (5 s) 8°↔13° testere dişi yapıyor. Tek yönlü tilt aralığı
    +yaw'da sınırdan kalkmayı, −yaw'da sınırın altına inmeyi gerektiriyor —
    ikincisi imkânsız → asimetrik doyum → limit cycle. Bu, Adım 17'nin
    MATLAB'da kanıtladığı (P) mekanizmasının ta kendisi.
33. **Ölçüm tuzağı (bu projede ÜÇÜNCÜ kez): `tiltrotor_indi_status`
    ulog'unda örneklerin %1.5'inde zaman damgası yineleniyor (`dt=0`)**,
    elenmezse interpolasyon 10-12°'lik sahte sapma sivrileri üretiyor —
    Adım 18'in ilk analizi buna kandı. Bu dosyadaki ölçüm dersleri artık
    şöyle: yaw'ı açıdan değil hızdan ölç (Adım 12b); bağlam değişkenlerini
    (hız!) kaydet (Adım 16); **ve p99'a bak, max'a değil.**

34. ~~**İLERİ HIZ, düşük hızdaki yaw salınımını bitiren şey DEĞİL (Adım 19).**~~
    **⛔ GERİ ALINDI (Adım 20c).** Aşağıdaki gözlem doğru ama ondan çıkarılan
    sonuç yanlıştı: sistem kararlılık sınırına çok yakın olduğu için, sınırı
    yeni geçmiş bir sistem hız sabit kalsa da yavaşça söner. İkinci bir uçuş
    (Adım 20b) 1.46-1.87 m/s'de salınımın **112 s boyunca hiç sönmediğini**
    gösterdi; Adım 19'un söndüğü uçuşta hız platosu **2.00-2.10 m/s** idi.
    Doğru ifade → çıkarım 38. Orijinal (hatalı) metin kayıt için aşağıda:
    `yaw_sp = 0` sabit tutulup yalnızca hız değiştirilen tek değişkenli
    A/B'de salınım, hız **2.00-2.09 m/s'de sabitken** söndü (yaw hızı RMS
    0.439 → 0.006); hızlanma bundan sonra başladı. **Adım 16'nın
    aerodinamik rüzgâr gülü mekanizması böylece doğrudan çürütüldü** —
    Adım 17'nin bağımsız kanıtıyla (MATLAB'da `M_aero(3) ≡ 0` iken adım
    oturuyor) birlikte iki ayrı kanıt. Adım 16'nın *hıza bağlılık ölçümü*
    yine de geçerli; muhtemelen hız, arm geçicisinin ne kadar sürdüğüyle
    dolaylı olarak ilişkiliydi (koşular hızlandıkça geç oluyordu).
35. ~~**(Q) sürekli bir limit cycle DEĞİL, çok zayıf sönümlü bir yaw modu
    (Adım 19); yerleşme ~30-35 s.**~~ **⛔ GERİ ALINDI (Adım 20b/c)** — tek
    uçuşa dayanan aşırı genellemeydi. Üçüncü uçuşta salınım **112 s boyunca
    sönmedi**. Doğru ifade → çıkarım 38.
36. **Trim'i ön-yüklemek δ1'i sınırdan uzak TUTMAZ (Adım 19a).** WLS'in
    amaç fonksiyonunda `du_pref = 0` olduğu için mutlak bir tilt tercihi
    yok; `Fx → 0` hedefi (`Ws_Fx = 0.05`) ortalama tilt'i δ1 tabana değene
    kadar iter. Üç farklı tohum (bias 0/5/10°) **aynı dengeye** yakınsadı
    (δ0=9.00°, δ1=0.00°). Genel ders: **artımlı (incremental) bir
    allocator'da başlangıç koşulu değiştirerek kalıcı bir aktüatör
    konfigürasyonu dayatılamaz** — amaç fonksiyonunu ya da kısıtı
    değiştirmek gerekir.
37. **Kalan asıl nicel boşluk: MATLAB ±30° yaw adımını 3.1-3.7 s'de
    oturtuyor, SITL ~8 s (adım) / ~33 s (arm geçicisi) alıyor.** Bu
    **2-10×'lik sönümleme farkı** hâlâ SITL'e özgü ve açıklanmadı; gölge
    model sapması (p99 ≤ 0.55°) bunu açıklayacak büyüklükte değil.
    Sıradaki iş bu boşluğu kapatmak.

38. **(Q)'nun DOĞRU tarifi (Adım 20, üç uçuş): düşük hızda (<~2 m/s) yaw
    salınımı SÜREKLİ; ~2 m/s civarında MARJİNAL; üstünde kararlı.**
    Üç uçuş: Adım 18 (1.6 m/s, salınım sürüyor), Adım 19 (plato 2.00-2.10
    m/s → ~35 s'de söndü), Adım 20 (plato 1.46-1.87 m/s → **112 s sönmedi**).
    Aynı konfigürasyonda zıt sonuçlar, yani sistem kararlılık sınırında.
    **Bu, hem Adım 16'nın "sönümsüz" hem Adım 18'in "denge kararsızlığı" hem
    de Adım 19'un "~30-35 s'de oturan mod" ifadelerinin yerine geçer.**
    Yöntem dersi: **marjinal kararlı bir sistemde tek uçuştan sönme/sönmeme
    sonucu çıkarmayın** — bu oturumda aynı hata iki kez yapıldı.
39. **`omega_dot` hipotezi ve gölge aktüatör modeli ABLASYONLA elendi
    (Adım 20a).** MATLAB'a enjekte edilen dört SITL kusuru — PX4'ün açık
    çevrim gölge modeli, +8 ms `omega_dot` gecikmesi, 0.02 rad/s² `omega_dot`
    gürültüsü, %25 `dt` jitter'ı — ve dördünün birleşimi, yerleşme süresini
    3.69 s'den **hiç değiştirmedi** (3.64-3.70 s, sıfır kalıcı salınım).
    SITL'de ölçülen gerçek değerler zaten küçüktü: `xyz_derivative` gecikmesi
    4-8 ms (MATLAB'ın filtresi 7.0 ms), yaw HF gürültüsü sinyalin %4'ü.
40. **Yüksek hızın neden stabilize ettiği SDF'den nicel olarak türetildi
    (Adım 20d, ÖLÇÜLMEDİ).** Modelde tek yanal yüzey dikey kuyruk:
    `cp=(-0.74,0,0.12)` (CG'nin 0.74 m arkasında), alan **yalnızca
    0.032 m²**, `alpha_stall = 19.4°`, `cla_stall = -3.85`. Yaw sönümleme
    türevi 2 m/s'de **0.27 Nm/rad**, 11.6 m/s'de **9.1 Nm/rad** (34×).
    Ayrıca salınım genliği ±25-40° iken yan kayma stall açısını aşıyor →
    kuyruk çevrimin çoğunda stall'da, sönümlemesi çöküyor (eğim ters).
    Bu, salınımın **genliğe bağlı** ve kendini sürdürür olmasını açıklar.
41. **Ama aero, düşük hızdaki KARARSIZLIĞI açıklamıyor (Adım 20e).**
    MATLAB'ın plant'inde hiç aerodinamik yaw momenti yok (`M_aero(3) ≡ 0`)
    ve orada sistem rahatça kararlı (ζ≈0.4). Yani **SITL'de MATLAB'da
    bulunmayan bir kararsızlaştırıcı var; aero yalnızca onu maskeliyor.**
    Bu, hâlâ araştırmanın açık merkezi.
42. **Ölçülmemiş boşluk: gölge/gerçek karşılaştırması yalnızca TILT
    kanallarında yapıldı, İTKİ kanalları (T0-T2) hiç ölçülmedi (Adım 18'in
    eksiği).** Yaw torkunun bir bileşeni rotor reaksiyon torkudur (`km·T`),
    dolayısıyla itki farkı doğrudan yaw'a girer. Gazebo'nun gerçek itkisi
    rotor eklem hızından hesaplanabilir (`T = 2e-5·ω²`); `JointStatePublisher`'a
    `rotor_{0,1,2}_joint` eklemek yeterli, rebuild gerekmez.

43. **İTKİ kanalı da elendi (Adım 21a).** Gerçek Gazebo itkisi rotor eklem
    hızından türetilip (`T = 2e-5·(w·20)²`, toplam 49.62 N ≈ ağırlık 49.05 N)
    gölgeyle karşılaştırıldı: salınım rejiminde sapma ort. **0.000-0.001 N**,
    p99 ≤ 0.042 N (~%0.1); yaw reaksiyon torkuna etkisi RMS **0.0020 Nm**
    (otoritenin %0.4'ü). "gz filtreyi ω'ya, PX4 T'ye uyguluyor" yapısal
    şüphesi pratikte önemsiz — küçük salınımda `T = kf·ω²` lineerleşiyor.
    (Arm geçicisinde sapma RMS 0.85 N'e çıkıyor; orada geçiş büyük.)
44. **WLS slew kutusu, döngünün gerçek periyoduyla eşleşmeyen sabit bir
    nominal periyotla (`TS_CTRL = 1/400`) boyutlanıyordu; modül 250 Hz'de
    dönüyor (Adım 21b).** *(Adım 22 düzeltmesi: sabit periyot kullanmak
    **kasıtlıydı** — jitter'a karşı koruma, kod yorumunda yazılı. Gözden
    kaçan, nominal periyodun gerçek periyotla eşleşmesi gerektiğiydi;
    "kod hatası" nitelemem fazla sertti.)*
    `MulticopterIndiTiltrotor.cpp:315-316, 324-325`. Aynı fonksiyon `dt`'yi
    171. satırda doğru hesaplayıp gölge model/LESO/irtifa için kullanıyor.
    Ölçümle doğrulandı: |ddelta| p99 tam **0.00500 rad** = `2.0·(1/400)`,
    tick 4.00 ms, tilt `sat_flag` **%99.4-99.9**, tahsisat yaw verimi
    **%20.6**. **Efektif tilt slew tavanı 1.25 rad/s — hedeflenen 2.0'ın
    %62'si.** Kanat tilt'i yaw'ın tek gerçek aktüatörü olduğundan doğrudan
    yaw otoritesini kısıyor. **Adım 11 ve 12 ile aynı sınıf** (kontrolcünün
    varsayımı ile ortamın gerçeği arasındaki uyumsuzluk) ve **saf MATLAB
    yapısal olarak göremez**, çünkü orada döngü gerçekten `p.Ts_ctrl`
    periyodunda koşar — kutu ile döngü her zaman tutarlıdır.
45. **Bu, Adım 14'ü geriye dönük açıklıyor — ve naif düzeltmenin ZARARLI
    olacağını öngörüyor.** Denenen nominal `TILT_RATE_MAX` değerlerinin
    efektif karşılıkları: 2.0 → **1.25 rad/s** (çalışıyor), 3.0 →
    **1.875 rad/s** (+30° adımda ıraksadı). Kutuyu gerçek `dt` ile
    boyutlamak nominal 2.0'ı **2.0 efektif** yapar — ıraksatan 1.875'ten de
    yüksek. **Yani "hatayı düzelt" hamlesi tek başına Adım 14'ün fiyaskosunu
    tekrarlar.** Doğrusu: kutuyu `dt` ile boyutla **ve** sabiti doğrulanmış
    efektif hızı (1.25 rad/s) koruyacak biçimde yeniden ayarla. Ayrıca
    `TILT_RATE_MAX` şu an iki farklı işi yapıyor (gölge modelin fiziksel
    servo limiti, satır 420; ve tahsisat kutusu) — ayrılmalı.
46. **Ablasyonun bu adayda verdiği null sonuç ELEME DEĞİLDİR (Adım 21c).**
    MATLAB'da tilt slew kutusu hiç bağlamıyor (~2.5°/s hareket, 114°/s
    izin), SITL'de ise %99.4 oranında bağlıyor. **Farklı rejimlerde çalışan
    iki ortamda aynı ablasyon anlamlı değil.** Genel ders: bir ablasyonun
    negatif çıkması, ancak o ablasyonun hedeflediği mekanizma o ortamda
    gerçekten aktifse eleme sayılır — önce mekanizmanın bağladığını
    doğrulayın.

47. **Slew kutusu ayrıştırıldı ve dürüst hale getirildi (Adım 22, KALICI).**
    `TILT_RATE_MAX` (2.0 rad/s) artık **yalnızca gölge modelin fiziksel
    servo limiti**; tahsisat kutusu ayrı: `TILT_SLEW_BOX_RATE = 1.25 rad/s`
    × `TS_BOX = 1/250`. Sonuç tilt kutusunda 1 ULP farkla aynı (0.005 vs
    0.0050000004), yani **davranış-nötr** — SITL'de doğrulandı (`|ddelta|`
    p99 hâlâ tam 0.00500, itki `sat_flag` %0.0, kilitlenme yok,
    \|vz\| ≤ 0.78 m/s, irtifa hata RMS 0.234 m; yaw hâlâ (Q) nedeniyle
    kalıyor, 37.09°). **Kazanç davranışta değil, ölçülebilirlikte:** sabit
    artık gerçek rad/s anlamına geliyor ve taranabilir (1.25 çalışıyor,
    1.875 ıraksıyordu). **Genel ders: sabit-periyotlu bir hız limiti,
    döngünün gerçek periyoduyla eşleşmezse sessizce de-rate eder — periyodu
    ve hızı ayrı ayrı, açıkça yazın.**
48. **İtki kutusunda kendi ilk gerekçem yanlıştı (Adım 22).** "Her iki
    durumda da bağlamıyor" derken yalnızca alt sınıra bakmıştım; üst sınır
    `ROTOR_TMAX/ROTOR_TAU_DOWN·TS·5` eski değerle **22.5 N/tick** ve
    hover'da `abs_hi = 27 N` olduğundan **canlı bir kısıttı**. Yeni değerle
    36 N. Ölçülen itki doyumu önce ve sonra %0.0 olduğundan pratikte
    etkisiz. **Ders: bir kısıtın "bağlamadığını" iddia etmeden önce İKİ
    yönü de hesaplayın.**

49. **(Q)'NUN MEKANİZMASI BULUNDU (Adım 23): düşük hızdaki yaw salınımı bir
    sönümleme eksikliği değil, TAHSİSATIN TİLT SLEW'UNDAN AÇ BIRAKILMASI.**
    Uçuş-içi tarama, iki koşu, ters sıra, her değerde aynı +30° adım
    uyarımı; ayırt edici metrik adımın son 5 s'sindeki yaw hızı RMS:
    **1.25 → 0.583/0.466 (salınıyor), 1.50 → 0.391/0.005 (marjinal),
    1.75 → 0.0037/0.0051 (sakin), 2.00 → 0.0056/0.0055 (sakin).**
    **Hız kesin olarak elendi:** 1.25 hem 0.86 hem 2.20 m/s'de salınıyor,
    1.75/2.00 ise 0.81-3.14 m/s aralığının tamamında sakin. Aşım da
    monotonik (219%→149%→123%→75%). MATLAB çapraz kontrolü tutuyor: onun
    efektif kutusu **3.0 rad/s** (`3.0·(1/400)` @ gerçek 400 Hz), yani
    denenen her değerin üstünde, ve orada aşım %24.1 / yerleşme 3.69 s —
    trend düzgün ekstrapole oluyor. Bu, Adım 21'in ölçtüğü tabloyu
    (tilt `sat_flag` %99.4, tahsisat yaw verimi %20.6) **nedensel** hale
    getiriyor.
50. **Adım 14 çelişmiyor, açıklanıyor (Adım 23).** Adım 14 `TILT_RATE_MAX`'ı
    3.0 yaparken **kutuyu ve gölge modelin fiziksel limitini birlikte**
    oynatmıştı; gölge model gerçek tork-sınırlı Gazebo servosunun
    veremeyeceği bir slew'a inanınca ıraksadı. **Adım 22'nin ayrıştırması,
    kutunun tek başına yükseltilebilmesini sağlayan şeydir** — sabitleri
    ayırmanın somut getirisi bu.
51. **Varsayılan 1.25 → 1.75 (KALICI, Adım 23c).** Fiziksel limite 0.25
    rad/s pay bırakır. `sitl-lockup-check`: kilitlenme ✅ (itki 12.83-19.11 N,
    sat %0.0), dikey hız ✅ (0.816 m/s, irtifa hata RMS 0.279 m), roll/pitch
    ✅ (±0.06°). **Yaw kriteri hâlâ ❌** (tepe 35.80° vs önceki 37.09°) —
    o senaryo aracı hızla hızlandırdığı için düşük-hız penceresinden çıkıyor
    ve kalan aşım arm geçicisinin tek seferlik salınımı. Asıl kazanç
    **kalıcı salınımın yok olması** (yaw hızı RMS ~0.5 → ~0.005, 100×).
52. **Kalan boşluk (Adım 23c):** kutu 2.00'de bile aşım %75 (MATLAB %24) ve
    fiziksel limit 2.0 tavan. Daha ileri gitmek `TILT_RATE_MAX`'ı
    yükseltmeyi gerektirir; doğru yol önce gölge modeli gerçek tork-sınırlı
    servoya sadık kılmaktır (çıkarım 30/45).

53. **Gölge modelin sadakat açığı TAMAMEN Coulomb sürtünme ölü bandıdır,
    2. derece dinamik DEĞİL (Adım 24b).** Çevrimdışı, kayıtlı `u_cmd` ile
    gerçek eklem açısına karşı: mevcut 1. derece RMS 0.287/0.408/0.554°,
    **tam tork-limitli 2. derece model 0.414/0.462/0.553° (DAHA KÖTÜ)**,
    1. derece + ölü bant **0.082/0.051/0.0040° (3.5×/8.0×/139× daha iyi)**.
    Sebep: eklem ataleti J=0.0168 kg·m² ile max ivme 59.4 rad/s², yani
    birkaç ms'de oturuyor — 4 ms'lik tick'in çok içinde. Ölü bant =
    friction/p_gain = **0.573°**, ve bu Adım 18/21'in ölçtüğü kalıcı
    0.52-0.53° ofseti niceliksel olarak açıklıyor.
54. **AMA ölü bant uygulanamaz: kapalı çevrimde KİLİTLENİYOR (Adım 24c,
    DENENDİ-GERİ ALINDI).** `u_cmd = _u_actual + du` komutu gölgeye
    bağlıyor, `du` ise slew kutusuyla **0.40°** ile sınırlı — 0.573°'lik
    ölü banttan küçük. Hiçbir tick sürtünmeyi kıramıyor → gölge donuyor →
    komut donuyor. SITL'de ölçüldü: üç tilt de tüm uçuş boyunca donuk
    (δ0 sabit 9.31°, sıfır varyans), yaw bandı 238°, araç dönüyor.
55. **YÖNTEM DERSİ: açık çevrim replay, kapalı çevrim geri besleme tuzağını
    gösteremez (Adım 24d).** Çevrimdışı doğrulama kendi içinde doğruydu ve
    yine de ölümcül sorunu kaçırdı, çünkü modeli gölgenin *hareket ettiği*
    bir koşudan gelen komut dizisiyle sürdü. **Komut yolunun İÇİNDE duran
    model değişiklikleri kapalı çevrimde doğrulanmalı.**
56. **MİMARİ KISIT: artımlı allocator, gölgenin komuta sürünmesini zorunlu
    kılar (Adım 24d).** Stiction/ölü bant bununla temelden uyumsuz; ancak
    mutlak komut gölgeden bağımsız ayrı bir durum olarak tutulursa mümkün
    olur, o da INDI'nin lineerleştirme noktasını bozar (WLS artışı tanım
    gereği aktüatörün mevcut durumuna görelidir). Çıkarım 30/45'in
    "gölge modeli sadık kıl" önerisi **bu yoldan kapanmıştır.**

57. **"Arm geçicisi" ayrı bir mekanizma DEĞİL (Adım 25).** İki hipotez de
    ölçümle düştü: (1) tilt ön-konumlandırması — gerçek δ0 arm'dan **72 ms**
    sonra trim'in %90'ına ulaşıyor ve +0.20 s'de gölgeyle 0.16° içinde
    örtüşüyor, oysa yaw hızı saniyeler boyunca birikiyor; zaman ölçekleri
    uyuşmuyor. (2) itkiye bağlı trim bozulması — **`τ_tilt` de itkiyle
    ölçekleniyor** (`−Σ py·T·sin δ`, tıpkı `τ_react = −Σ km·T·cos δ` gibi),
    yani ikisi birlikte ölçekleniyor ve trim itki seviyesinden bağımsız
    geçerli kalıyor; tırmanışta itki 49.7 → 68.9 N çıkarken NET tork
    −0.021 → −0.011 N·m. Arm sonrası savrulma **madde (Q)'nun kendisi**:
    zayıf yaw ekseninin 0.03-0.15 N·m'lik artık torklara yanıtı (tahsisat
    yaw otoritesi adım başına ~0.033 N·m, Adım 21). **§4 (Q) yolu (a)
    kapandı.**
58. **Genel ders (Adım 25b): bir dengesizliğin "ölçekle bozulduğunu" iddia
    etmeden önce, onu DENGELEYEN terimin aynı ölçekle büyüyüp büyümediğine
    bakın.** Burada `hover_trim`'in itkiye bağlı olduğu doğruydu ama
    dengelediği terim de aynı şekilde bağlıydı; oran korunuyordu.

59. **Yol (b) NO-OP: `TILT_RATE_MAX` clamp'i hiç bağlamıyor (Adım 26a).**
    Ölçüldü: gölge `ddelta`'nın p99'u **ve** max'ı 0.0467 rad/s, clamp 2.00
    rad/s — **%0.000 örnekte ulaşılıyor, 43× başlık var.** 1.75 seçilirken
    kullandığım "fiziksel limite pay bırak" gerekçesi geçersizdi: iki sabit
    aynı büyüklüğe etki etmiyor.
60. **YÖNETİCİ PARAMETRE: `etkin slew = TILT_SLEW_BOX_RATE·TS_BOX/TILT_TAU`
    (Adım 26b).** `u_cmd = _u_actual + du` olduğu için gölge her tick'te
    `du`'nun yalnızca `dt/TILT_TAU = %2.7`'sini ilerletiyor → nominal kutu
    hızı **38× de-rate**. **Ve MATLAB'ın karşılığı 0.050 rad/s'ye karşı SITL
    1.75'te 0.0467** — yani Adım 23'ün taraması SITL'i MATLAB'ın etkin
    hızının hemen altına getirmişti ve salınım tam orada kesilmişti.
    **İki ortam arasındaki "sönümleme boşluğu" baştan beri bu tek
    büyüklükmüş.** Genel ders: **bir rate limitini artımlı bir komutun
    üstüne UYGULAMAK, o limiti dt/τ ile ölçekler — iki kısıtı seri koymak
    çift sayım yapar.**
61. **Genişletilmiş tarama MATLAB'ı yakalıyor (Adım 26c, iki koşu, ters
    sıra).** Ortalama aşım: kutu 1.75 → %82.5, 2.50 → %38.0, **3.00 →
    %24.8** (MATLAB %24.1), 3.50 → %17.7. Test hızları 0.77-2.65 m/s ve
    **en iyi sonuçlar en düşük hızlarda** — hız artefaktı değil.
    **3.00 önerilir ama kriter koşusu yapılmadan DEPLOY EDİLMEDİ**
    (çıkarım 26d).

## 4. Önerilen sonraki adım

### GÜNCELLEME 2026-07-27 (Adım 16 sonrası) — ★ GÜNCEL ÖNCELİK SIRASI ★

**Durum:** (K) tamamlandı, yaw kriteri geçti ve aktüatör kilitlenmesi
kapandı — **ama Adım 16 gösterdi ki bu doğrulamaların hepsi ~10 m/s ileri
hızda yapılmıştı.** Gerçek, yerinde duran hover'da yaw hâlâ sönümsüz.
Sıradaki iş (Q); geri kalanlar iyileştirme/temizlik niteliğinde:

- **(Q) EN YÜKSEK ÖNCELİK — yaw ekseni GERÇEK HOVER'DA sönümsüz.
  ADIM 17 BUNU YENİDEN YÖNLENDİRDİ.**
  Tek uçuşta kontrollü ölçüm (Adım 16): 11.6 m/s'de +30° adımı monoton
  ve temiz (~%13 aşım), 2.45 m/s'de **±25°'lik sönümsüz salınım**. Bu
  **ölçüm geçerli**. Ama Adım 16'nın önerdiği mekanizma ("yaw'ı sönümleyen
  aerodinamik rüzgâr gülü etkisi, hız düşünce kayboluyor") **Adım 17'de
  çürütüldü**: saf MATLAB plant'inde `M_aero(3) ≡ 0`, yani hiçbir hızda
  aero yaw sönümlemesi yok, buna rağmen ±30° adımı 3.1-3.7 s'de salınımsız
  oturuyor (son 5 s yaw hızı RMS ≤ 0.0001 rad/s). Kontrolcü, aero yardımı
  **olmadan** da bu adımı sönümleyebiliyor → SITL'de **ek, SITL'e özgü bir
  kararsızlaştırıcı** var; aero sönümleme onu yalnızca maskeliyor.

  **ADIM 18 GÜNCELLEMESİ — (Q) yeniden üretildi ve yeniden çerçevelendi.**
  Salınım `yaw_sp = 0` iken oluyor (bant 73.7°, periyot 5.18 s, r RMS
  0.417 rad/s, 1.62 m/s): bu bir *adım yanıtı* kusuru değil, **denge
  kararsızlığı**. Gölge model sapması ölçüldü ve **eleniyor** (p99 ≤ 0.55°,
  salınım bandının iki mertebe altında). Salınımın en güçlü korelatı
  **δ1'in `TILT_MIN=0` sınırında zıplaması** — yani madde (P) ile aynı kök.

  **ADIM 19 GÜNCELLEMESİ — hedef netleşti.** Tek değişkenli A/B
  (`yaw_sp = 0` sabit, yalnızca hız değişken) gösterdi ki **ileri hız
  salınımı bitiren şey değil**: salınım hız 2.00-2.09 m/s'de sabitken
  söndü. Ve salınım aslında **sönümsüz değil, çok zayıf sönümlü** —
  yerleşme ~30-35 s, iki uçuşta tekrarlandı. Trim ön-yükleme fikri de
  MATLAB'da çürütüldü (çıkarım 36). Dolayısıyla:

  **Asıl hedef artık tek bir nicel boşluk:** MATLAB ±30° yaw adımını
  **3.1-3.7 s**'de oturtuyor, SITL ~8 s (adım) / ~33 s (arm geçicisi)
  alıyor — **2-10× daha az sönümleme, sebebi SITL'e özgü ve bilinmiyor.**

  **ADIM 20 GÜNCELLEMESİ — (Q)'nun doğru tarifi ve daralmış aday listesi.**
  Üç uçuşla: **düşük hızda (<~2 m/s) salınım SÜREKLİ, ~2 m/s'de MARJİNAL,
  üstünde kararlı** (çıkarım 38 — Adım 19'un "hız sebep değil" ve "~30-35 s'de
  oturur" çıkarımları geri alındı). Yüksek hızın neden stabilize ettiği SDF'den
  nicel olarak türetildi (çıkarım 40: dikey kuyruk sönümleme türevi 2 m/s'de
  0.27, 11.6 m/s'de 9.1 Nm/rad; ayrıca ±25-40°'lik salınımda kuyruk 19.4°'lik
  stall açısını aşıyor). **Ama aero, düşük hızdaki kararsızlığı açıklamıyor:
  MATLAB'da hiç aero yaw momenti yokken sistem rahatça kararlı (ζ≈0.4).**

  **ADIM 21 GÜNCELLEMESİ — somut bir kod hatası bulundu (çıkarım 44).**
  WLS slew kutusu sabit `TS_CTRL = 1/400` ile boyutlanıyor ama modül 250 Hz'de
  dönüyor → efektif tilt slew tavanı **1.25 rad/s** (hedeflenen 2.0'ın %62'si),
  tilt kanalları zamanın **%99.4'ünde** kutuda, tahsisat yaw verimi **%20.6**.
  Kanat tilt'i yaw'ın tek gerçek aktüatörü. İtki kanalı da elendi (43).

  **ADIM 22 GÜNCELLEMESİ — kutu ayrıştırıldı, nötrlük doğrulandı (çıkarım 47).**
  `TILT_SLEW_BOX_RATE = 1.25 rad/s` × `TS_BOX = 1/250` artık tahsisat kutusu;
  `TILT_RATE_MAX = 2.0` yalnızca gölge modelin fiziksel servo limiti. SITL'de
  davranış-nötr doğrulandı. Sıradaki iş artık **taramak**.

  **ADIM 23 GÜNCELLEMESİ — (Q)'nun MEKANİZMASI BULUNDU (çıkarım 49).**
  Düşük hızdaki yaw salınımı bir sönümleme eksikliği değil, **tahsisatın
  tilt slew'undan aç bırakılması**. Tarama (iki koşu, ters sıra): 1.25
  salınıyor, ≥1.75 sakin; hız 0.81-3.14 m/s aralığında ayırt edici değil.
  Varsayılan **1.75** yapıldı; kalıcı salınım yok oldu (yaw hızı RMS ~0.5 →
  ~0.005), ama arm geçicisinin tepe genliği hâlâ ±30° kriterini aşıyor.

  **ADIM 24 GÜNCELLEMESİ — "gölge modeli sadık kıl" yolu KAPANDI (çıkarım
  53-56).** Sadakat açığının tamamı Coulomb sürtünme ölü bandı (0.573°,
  çevrimdışı 3.5×-139× iyileşme), ama uygulandığında kapalı çevrimde
  kilitleniyor: `du` slew kutusuyla 0.40° ile sınırlı, ölü banttan küçük →
  gölge donuyor → komut donuyor. Geri alındı. Artımlı allocator mimarisi
  gölgenin komuta sürünmesini zorunlu kılıyor.

  **ADIM 25 GÜNCELLEMESİ — yol (a) de kapandı (çıkarım 57).** "Arm geçicisi"
  ayrı bir mekanizma değil, madde (Q)'nun kendisi; kaynağında düzeltilecek bir
  şey yok. İki hipotez de ölçümle düştü (tilt 72 ms'de yerine oturuyor;
  `τ_tilt` de `τ_react` gibi itkiyle ölçekleniyor, trim itkiden bağımsız
  geçerli).

  **Yeni öncelik sırası:**
  1. **YOL (b) — `TILT_RATE_MAX`'ı YALNIZCA yükselt (kutuyu 1.75'te tutarak).**
     Sabitler Adım 22'de ayrıştırıldığından bu artık mümkün ve **hiç
     denenmedi**: Adım 14 ikisini birden 3.0 yapmıştı. Yalnızca fiziksel
     limiti yükseltmek, gölge modelin gerçek servonun yapabildiğine dair
     varsayımını gevşetir; kutu değişmediği için tahsisat davranışı sabit
     kalır. `slewbox` kancasıyla kutu tarafı zaten uçuş-içi taranabiliyor.
     **Adım 20'nin dersi: her değeri en az iki koşuda, hız platosu
     kaydedilerek doğrula.** Rebuild gerekir.
  2. Ölü bandı, mutlak komutu ayrı durum olarak tutan bir mimariyle yeniden
     ele almak — INDI lineerleştirmesini bozma riski var (çıkarım 56),
     en son çare.
  2. **Kuyruğun stall hipotezini ÖLÇ (çıkarım 40 türetilmiş, ölçülmemiş):**
     uçuşta yan kayma açısını (β = atan2(v_y, v_x) gövde ekseninde) ve yaw
     momentini logla; β'nın 19.4°'yi aştığı anlarla salınım genliğinin
     büyümesi örtüşüyor mu?
  3. **Sınırdaki tahsisat hatasını düzeltmek (çıkarım 30):** gölge δ1/δ2
     tam 0 okuduğu için WLS 0.53°'lik gerçek hareket alanını göremiyor.
     Tek başına (Q)'yu çözmez ama gerçek ve sistematik. **Donanıma
     taşınabilir olması için gölge modeli sadık kılmak gerekir — gerçek tilt
     servolarının çoğunda konum geri beslemesi yoktur, yani "gerçek durumu
     oku" yolu taşınabilir değildir.**
  4. Yaw'a sönümleme eklemek (`Kp_rate[2]`) — **en sonda**, çünkü Adım 17
     kontrolcünün MATLAB'da 3.69 s'de sönümleyebildiğini gösterdi (sorun
     kazanç azlığı değil, SITL'de kaybolan sönümleme) ve Adım 10'un roll↔yaw
     limit cycle regresyonu bu yolu riskli kılıyor.
  5. Paralel/alternatif: §4 (N)'i çözüp aracın gerçekten yerinde durmasını
     sağlamak ve kriteri o koşulda yeniden ölçmek.
  6. **Artık aranmaması gerekenler (ölçümle kapandı):** gölge aktüatör
     modelinin TILT kanalı (çıkarım 29, 39), `omega_dot` gecikmesi/gürültüsü
     (39), `dt` jitter'ı (39), trim ön-yükleme (36).

  **Ölçüm altyapısı hazır (Adım 18, rebuild gerektirmez):**
  `sitl/logger_topics_shadow.txt` + `sitl/gz_joint_csv.sh` +
  `sitl/shadow_vs_real.py`; model.sdf'e `JointStatePublisher` eklendi.
  Kullanım `shadow_vs_real.py` docstring'inde.

  **Yeni araç:** `run_yaw_step_test.m` — yaw adım yanıtını saf MATLAB'da
  ölçer (±30°, 25 s, aşım/yerleşme/kalıcı-salınım metrikleri, δ0/δ1 ve
  talep-vs-üretilen Δτ_z grafikleri). `YAW_TEST_TILT_RATE_MAX=2.0` ile
  PX4'ün slew limitinde koşulabilir. **(Q) veya (P) üzerinde çalışan her
  değişiklik önce burada ölçülmeli.**
- **(L) KAPANDI — `TILT_RATE_MAX` 2.0 → 3.0 DENENDİ, GERİ ALINDI**
  (Adım 14). Hover'da iyileştiriyordu (yaw 21 s yerine 6 s'de oturdu) ama
  +30° yaw adımında aracı tam tura sokan bir aşıma yol açtı (iki kez
  tekrarlandı, tepe hız 2.14 rad/s). 2.0'a geri dönüldü ve doğrulandı.
  **Adım 9'un gerekçesi böylece ölçümle doğrulanmış oldu** (§4 (I)'deki
  "gerekçesi zayıfladı" değerlendirmesi yanlıştı).
- **(P) yaw adım yanıtı YÖN-ASİMETRİK — ADIM 17'DE MATLAB'DA YENİDEN
  ÜRETİLDİ, MEKANİZMA KANITLANDI.** `TILT_RATE_MAX=3.0`'da SITL'de +30° adım
  ıraksarken -30° kusursuzdu; 2.0'da bile +yönde ~%13 aşım var, -yönde yok.
  Adım 17 aynı asimetriyi MATLAB'da ölçtü ve **PX4'ün slew limitinde
  keskinleştiğini** gösterdi: +30° aşımı `rate_max` 3.0'da 24.0%, 2.0'da
  **59.2%**; -30° ise 7.7% → 8.0% (yani 7.4× asimetri).
  **Sebep doğrulandı — tek yönlü tilt aralığı.** Hover trim'de **δ1 ≡ 0**,
  yani `p.tilt.min = 0` tabanında oturuyor (δ0 ≈ 8-9°). Geometriden
  `τ_z = -0.25·T0·sin δ0 + 0.25·T1·sin δ1`:
  −yaw yalnızca δ0'ı artırmayı gerektirir → **serbest**;
  +yaw ise δ1'i 0 tabanından kaldırmayı gerektirir → **sürekli sınıra
  vuruyor** (`yaw_step_test*.png` 3. panelde δ1 iki kez ~2°'ye çıkıp 0'a
  çakılıyor). Diğer aday bileşenler (servo tork limiti, pervanenin
  jiroskopik reaksiyonu) MATLAB'da modellenmediği hâlde asimetri ortaya
  çıktığı için **gerekli değiller**; tek yönlü aralık tek başına yeterli
  bir açıklama. Olası çözümler: trim'i her iki kanat rotorü de sıfırdan
  uzakta olacak şekilde ön-yüklemek (δ0=δ1=δ_bias>0, yaw otoritesini iki
  yönde simetrik yapar; bedeli sürekli bir +Fx ve `cos δ` itki kaybı) ya da
  donanımda tilt aralığını negatife açmak (madde (N) ile aynı kök).
  **`TILT_RATE_MAX=3.0` yine de tekrar denenmemeli** — SITL'deki ıraksaması
  (P)'den değil, çıkarım 27'deki gölge-model sapmasından geliyor olabilir.
- **(N) YENİ, Adım 15'te niceliksel doğrulandı — yatay sürüklenme ve tek
  yönlü tilt aralığı.** Tüm tiltler `[0, π/2]`'de olduğu için hover'da
  toplam `Fx ≥ 0`: araç geri kuvvet üretemez, yaw trim'i ise kaçınılmaz
  olarak ~3 N ileri itki doğuruyor (`WS_FX=0.05` bunu cezalandırmıyor;
  kuyruk tilt'i de PY=0 olduğu için yalnızca +Fx verir). Sonuç: 25 s'de
  235 m sürüklenme. Gerçek uçuş için ya bir pozisyon döngüsü ya da tilt
  aralığının negatife açılması (`TILT_MIN < 0`) gerekir. **Gözlem
  koşularında geçici çözüm:** `pitch_sp = +0.061 rad` (≈3.5°) komut
  edin — sürüklenmeyi 9.4 → 1.7 m/s'ye düşürüyor (Adım 15).
- **(O) KAPANDI — agresif adım-alçalma.** Sebep kontrolcü değil komut
  profiliydi: kademeli iniş (`z_sp` 4.5→3.0→1.5→0.6 m) ile aynı koşuda
  sıfır BIG_M ölçüldü. Kalan tek iş, iniş rampasını test sürücülerine
  (`run_hover_gust_test.py` vb.) yerleştirmek.
- **(M) `WS_YAW`** — artık büyük olasılıkla gereksiz; yaw kriteri
  ağırlığa dokunmadan geçti. Yalnızca (L) yetersiz kalırsa denenmeli.
- **(J) Küçük artık irtifa hatası** — Adım 13'te hata RMS 0.456 → 0.211 m
  ve `|vz|` 8.25 → 0.17 m/s'ye düştü; büyük ölçüde kendiliğinden
  iyileşti, düşük öncelik.

---

### GÜNCELLEME 2026-07-27 (Adım 12 sonrası) — öncelik sırası

Adım 12, (H)'yi kapattı ve kalan tek sorunun mekanizmasını **ölçtü**.
Sıradaki adımlar artık hipotez değil, ölçüme dayalı:

- **(K) Yaw hız setpoint'inin kendini bozmasını durdurmak — EN YÜKSEK
  ÖNCELİK, EN UCUZ.** Dış attitude döngüsü, araç dönerken ±3 rad/s
  (`RATE_SP_LIMIT`) arasında salınan bir yaw hız komutu üretiyor (Adım
  12g). Yaw ekseni için ayrı ve çok daha küçük bir hız limiti
  (örn. 0.5-1.0 rad/s; ölçülen gerçek yaw otoritesi ≈ 0.05 N·m/adım ile
  zaten 3 rad/s'lik bir slew hiç gerçekçi değil) hız döngüsünün önce
  **sönümleme** yapmasını sağlar. `RATE_SP_LIMIT` şu an tek skaler —
  eksen bazlı hale getirilmeli. `safe-control-change` uygulanmalı.
- **(L) Yaw otoritesinin slew tavanını yükseltmek — (I) ile birleşti.**
  Adım 12g, `TILT_RATE_MAX`'ın yaw torkunu adım başına ≈0.023 N·m/rotor
  ile doğrudan sınırladığını **ölçtü**. Adım 9'un 3.0→2.0 düşüşünün
  gerekçesi zaten zayıflamıştı (Adım 11); artık bunun yaw'a maliyeti
  ölçülü olduğuna göre 3.0'a dönüş ayrı bir ≥25s koşuyla denenmeli.
- **(M) `WS_YAW` (=3) yeniden değerlendirilmeli.** Adım 7'nin
  "sınırlayıcı faktör değil" sonucu **hem yanlış G matrisi hem yanlış km
  işareti** altında alınmıştı — çifte geçersiz. Ama (K)/(L)'den sonra
  denenmeli: talep zaten üretilemiyorken ağırlığı artırmak anlamsız.
- **(J) Küçük artık irtifa hatası** — değişmedi, düşük öncelik.
- **(A) Y-işareti bulmacası** — Adım 12 aynı dosyadaki km işaretinde
  gerçek bir hata bulduğu için ROTOR_PY notu güncellenmeli, ama PY'nin
  kendisi MATLAB regresyonuyla iki kez doğrulandı; kapalı sayılabilir.

---

**Durum Adım 11 sonrası kökten değişti:** (G) izole edildi, kök neden
(karesel itki eşlemesi) bulundu ve düzeltildi, ayrıca test koşumunu
geçersiz kılan ikinci bir hata (setpoint hiç ulaşmıyor) giderildi.
Aktüatör kilitlenmesi ve dikey hız kontrolsüzlüğü iki bağımsız koşuda
tekrar üretilemiyor. Geriye kalan TEK açık sorun **yaw ekseninin
±60°'lik sınırlı gezinmesi** (kriter: ±30°).

Öncelik sırasına göre:

- **(H) Yaw gezinmesini kapatmak — TEK KALAN AÇIK SORUN, EN OLASI
  SONRAKİ ADIM.** Artık ortam çok daha temiz: itki eşlemesi doğru,
  etkinlik matrisi (G) artık gerçeği yansıtıyor, hiçbir aktüatör
  kilitlenmiyor. Adım 6/7/10'da başarısız olan yaw-otoritesi denemeleri
  **YANLIŞ bir G matrisi altında yapılmıştı** — dolayısıyla o üç
  sonucun hepsi şüpheli hale geldi ve en az biri (özellikle Adım 6,
  `leso_enable_yaw=1`; rebuild gerektirmiyor, `test_sp ... 1 1 1` ile
  anında denenebilir) düzeltilmiş kod üzerinde TEKRAR denenmeli.
  Adım 10'un MATLAB'da yakaladığı roll↔yaw limit cycle regresyonu ise
  MATLAB'a özgüydü ve hâlâ geçerli — o yüzden `Kp` yolunu tekrar
  açmadan önce yine `safe-control-change` disiplini uygulanmalı.
- **(I) Adım 9'u (`TILT_RATE_MAX` 3.0→2.0) yeniden değerlendirmek.**
  Gerekçesi zayıfladı (bkz. Adım 11 sonu): sağladığı kısmi iyileşme
  muhtemelen kök nedeni dolaylı hafifletmesindendi. 3.0'a döndürüp
  ayrı bir ≥25s koşuyla karşılaştırmak, gereksiz bir kısıtı kaldırıp
  kaldıramayacağımızı gösterir.
- **(J) Küçük artık irtifa hatası** — koşu 2'de z hedefin ~0.0-0.16 m
  altında/üstünde salınıyor, `vz` ±0.1-0.2 m/s. Zararsız görünüyor ama
  `ALT_KI_VZ` entegral davranışıyla açıklanmalı.
- **(F-devam) `TILT_TAU`'yu da ayarlamak** (Adım 8/9'un denenmeyen
  yarısı) — artık düşük öncelikli; Adım 8'in ölçtüğü gölge/gerçek tilt
  sapması gerçekti ama kök neden değildi.
- **(A) Y-işareti bulmacası (çok düşük öncelik):** teori Adım 5'te zaten
  zayıflamıştı, Adım 11 kök nedeni başka yerde bulduğu için pratikte
  kapandı sayılabilir.

## 5. Değiştirilen dosyalar (bu oturum, kalıcı — geri alınanlar hariç)

**Adım 39 (2026-08-03) — madde (S): eşik ve fren marjı gövde ileri hızını okuyor:**

| Dosya | Değişiklik |
|---|---|
| `backtrans_loop.m` | imza `(enable, v_h, **v_fwd**, delta_wing_max, state_in, p)`; BRAKE çıkışı ve `brake_pitch` artık `v_fwd` okuyor (marj `max(0, v_fwd)`), RETRACT bilerek `v_h`'de kaldı; madde (S) gerekçesi + kapatılmayan kalıntı belgelendi |
| `tiltrotor_params.m` | `handoff_v` / `brake_v_full` yorumları: DEĞER değil BAKILAN SİNYAL değişti; `POS_ENGAGE_V_MAX` ile aynı SAYI ama artık aynı TEST değil |
| `TiltrotorIndiControl.hpp` | `btBrakePitch(v_fwd)`, `backTransition(..., v_fwd, ...)`; madde (S) notu (mekanizma, ölçüm, iki terimli düzeltme, RETRACT'ın neden `v_h` okuduğu, kalıntı) |
| `TiltrotorIndiParams.hpp` | `BT_HANDOFF_V` / `BT_BRAKE_V_FULL` yorumları aynı ayrımla güncellendi (sayı değişmedi) |
| `MulticopterIndiTiltrotor.cpp` | `bt_v_fwd = vx·cos ψ + vy·sin ψ` hesaplanıp geçiriliyor; durum geçiş log satırı artık İKİ hızı da basıyor; `_bt_handoff_wait` (handoff istendi ama hold devralmadı) sayacı + >0.5 s'de WARN; `status.pos_hold_active` üç yayın noktasında |
| `MulticopterIndiTiltrotor.hpp` | `_bt_handoff_wait` üyesi |
| `msg/TiltrotorIndiStatus.msg` | **`pos_hold_active` YENİ** — `bt_state == HANDOFF` hold'un İSTENDİĞİNİ söyler, DEVRALDIĞINI değil; ölçüt penceresi bunu ayırt etmek zorunda |
| `run_backtrans_sm_test.m` | test 7-9 (madde (S) izi, eski büyüklük mantığının aynı izde takılması, marjın geri giderken sıfırlanması); `simulate` opsiyonel `v_fwd` izi alıyor |
| `sitl/analyze_backtrans.py` | **6. ölçüt: yeniden-hızlanma ≤ 1.0 m/s**, penceresi "fren yasası pitch'e sahip" aralığı; `v_fwd`/`v_lat` teşhis olarak; handoff→hold bekleme süresi raporu; durum tablosuna `v_fwd` sütunu |
| `sitl/probe_lateral_handoff.py` | **YENİ** — madde (S) rejimini deterministik kurar (frenlerken heading +90°), eski eşiğin o anda sağlanmayacağını gerçek uçuş verisinden gösterir |
| `sitl/logger_topics_shadow.txt` | `estimator_status_flags` eklendi (madde (T) ölçümü) |
| `sitl/run_backtrans_test.py` | docstring: güncel eşikler + 6. ölçüt |

**Adım 38 (2026-07-31) — madde (R): RETRACT çıkışı iki terimli:**

| Dosya | Değişiklik |
|---|---|
| `tiltrotor_params.m` | `p.bt.release_v` 8.0 → **10.0**; **`p.bt.floor_dwell = 20.0` YENİ**; `brake_ceil` yorumundaki eskimiş "terminal hız 5.7-7.9 m/s" kaydı düzeltildi (ölçülen: 8.0-9.3) |
| `backtrans_loop.m` | durum vektörü `[state; tilt_ceil]` → **`[state; tilt_ceil; floor_dwell]`**; RETRACT çıkışı iki terimli; sayaç yalnızca tabanda işliyor; madde (R) gerekçesi belgelendi |
| `run_backtrans_sm_test.m` | **YENİ** — plantsız durum makinesi testi (6 kontrol; emniyetsiz mantığın aynı izde takıldığını da gösterir) |
| `sitl/brake_ceiling_margin.py` | **YENİ** — BRAKE'te `BT_BRAKE_CEIL` payının ölçümü (geçiş tick'i artefaktı ayıklanır) |
| `sitl/diag_brake_reversal.py` | **YENİ** — madde (S) teşhisi: büyüklük vs gövde ileri/yanal hız, ve `estimator_status_flags` tazeliği |
| `~/PX4-Autopilot/.../TiltrotorIndiParams.hpp` | `BT_RELEASE_V` 8.0f → **10.0f**; **`BT_FLOOR_DWELL = 20.0f` YENİ**; `BT_BRAKE_CEIL` yorumundaki eskimiş terminal hız kaydı düzeltildi |
| `~/PX4-Autopilot/.../TiltrotorIndiControl.hpp` | `backTransition()` imzasına `float &floor_dwell`; RETRACT çıkışı iki terimli; madde (R) notu |
| `~/PX4-Autopilot/.../MulticopterIndiTiltrotor.{hpp,cpp}` | `_bt_floor_dwell` üyesi + `resetState()` + abort yolu; geçiş log'u hangi terimin tetiklediğini yazıyor (`[via FLOOR_DWELL backstop]` + WARN) |
| `HARDWARE_READINESS_CHECKLIST.md` | madde (R) kapatıldı; **madde (S) 🔴 ve madde (T) 🟠 eklendi; B5 🟠 → 🔴** |
| `dosya_ve_klasor_yapisi.md` | `run_backtrans_sm_test.m` girdisi |

*(Simulink portu **değişmedi** — geri geçiş `.slx`'te hiç yok.)*


**Adım 28 (2026-07-29) — madde (N) ve (P) çözümü:**

| Dosya | Değişiklik |
|---|---|
| `position_loop.m` | **YENİ** — yatay pozisyon dış döngüsü (P→PI→attitude sp) + madde (P)'nin `fx_trim`'i |
| `run_station_keeping_test.m` | **YENİ** — (N) A/B'si + gerçek duruşta ±30° yaw adımı (asimetri ölçümü) |
| `tiltrotor_params.m` | `p.ctrl.fx_trim = 2.9` eklendi; `p.tilt.bias`/`bias_tau` **denendi-geri alındı** olarak kayıtta bırakıldı |
| `indi_attitude_controller.m` | `du_pref = 0`'a geri alındı (bias etkisizdi); Fx trim'in neden burada DEĞİL pozisyon döngüsünde olduğu belgelendi |
| `sf_wls_alloc.m` | aynı not (trim burada uygulanmaz) |
| `~/PX4-Autopilot/msg/TiltrotorIndiSetpoint.msg` | `pos_hold_enable` alanı eklendi |
| `~/PX4-Autopilot/.../TiltrotorIndiControl.hpp` | `positionLoop()` eklendi (`fx_trim` çıkışıyla) |
| `~/PX4-Autopilot/.../TiltrotorIndiParams.hpp` | `POS_*` kazançları + `FX_TRIM = 2.9f` |
| `~/PX4-Autopilot/.../MulticopterIndiTiltrotor.{hpp,cpp}` | pozisyon tutma durumu, yükselen kenarda hedef yakalama, `test_sp`'ye `pos_hold` argümanı |
| `tiltrotor_indi.slx` | yeniden build edildi |

**Adım 27 (2026-07-29) — kutu ayrımının tamamlanması, üç uygulama senkron:**

| Dosya | Değişiklik |
|---|---|
| `~/PX4-Autopilot/.../TiltrotorIndiParams.hpp` | **`TILT_SLEW_BOX_RATE` 1.75f → 3.00f** (Adım 26'nın önerisi dağıtıldı; iki tutuş uçuşuyla doğrulandı) |
| `~/PX4-Autopilot/.../MulticopterIndiTiltrotor.cpp` | `slewbox` yardım metnindeki eskimiş varsayılan (1.25) düzeltildi |
| `tiltrotor_params.m` | **`p.tilt.slew_box_rate = 4.8` eklendi** (yalnızca WLS kutusu); `p.tilt.rate_max = 3.0` artık yalnızca plant clamp'i — PX4 Adım 22 ayrımının MATLAB'a taşınması |
| `indi_attitude_controller.m` | tilt kutusu `p.tilt.rate_max` yerine `p.tilt.slew_box_rate` kullanıyor |
| `sf_wls_alloc.m` | `rate_max_tilt = 3.0` → `slew_box_rate_tilt = 4.8` (codegen-safe literal, `tiltrotor_params.m` ile senkron kalmalı) |
| `run_yaw_step_test.m` | override artık kutuyu hedefliyor (`YAW_TEST_TILT_BOX_RATE`; eski ad geriye dönük kabul ediliyor) |
| `run_box_bind_check.m` | **YENİ** — kutunun MATLAB'da bağlayıp bağlamadığını ölçer; Adım 21(d)'yi çürüten araç |
| `tiltrotor_indi.slx` | yeniden build edildi (`sf_wls_alloc.m` değişikliğiyle) |

**Önceki oturumlar:**

| Dosya | Değişiklik |
|---|---|
| `gain_schedule.m` | `wu_tilt_hover` 8.0→3.0 |
| `sf_wls_alloc.m` | `wu_tilt` sabiti 8.0→3.0, `km` 0.05→0.06 |
| `tiltrotor_params.m` | `p.rotor.km` 0.05→0.06 |
| `sitl/run_transition_test.py` | `WU_TILT_HOVER` 8.0→3.0 (referans sabiti) |
| `tiltrotor_indi.slx` | yeniden build edildi (yukarıdaki `sf_*.m` değişiklikleriyle) |
| `~/PX4-Autopilot/.../TiltrotorIndiParams.hpp` | `WU_TILT_HOVER` 8.0→3.0, `ROTOR_KM` 0.05→0.06, `TILT_RATE_MAX` 3.0→2.0 (Adım 9, PX4'e özgü, MATLAB'a taşınmadı), `T2dbg` tanı logu eklendi; **Adım 11: `ROTOR_WMIN=10.0f` eklendi** (SIM_GZ_EC_MIN, itki eşlemesi tersleme için) |
| `~/PX4-Autopilot/.../TiltrotorIndiControl.hpp` | `T2dbg` tanı logu eklendi (kuyruk/yaw ekseni); **Adım 11: `thrustToNormalized()` eklendi** (karesel Gazebo motor modelinin doğru terslemesi — KÖK NEDEN düzeltmesi), **`WRdbg` eksen-bazlı atıf tanı logu eklendi** (her iki kanat rotorü için, `Hinv` saklanarak) |
| `~/PX4-Autopilot/.../MulticopterIndiTiltrotor.cpp` | **Adım 11: `motors.control[i] = u_cmd(i)/ROTOR_TMAX` → `thrustToNormalized(u_cmd(i))`** (KÖK NEDEN düzeltmesi); **`test_sp`'nin `uORB::Publication`'ı `static` yapıldı** (yoksa yıkıcı `orb_unadvertise()` çağırıp setpoint'in kontrolcüye ulaşmasını tamamen engelliyordu) + publish dönüş değeri kontrol ediliyor |
| `tiltrotor_params.m` | **Adım 12: `p.rotor.km` `[+0.06,-0.06,+0.06]` → `[-0.06,+0.06,-0.06]`** (FRD işaret düzeltmesi, türetme dosyada) |
| `sf_wls_alloc.m` | **Adım 12: `km` literali aynı şekilde düzeltildi** |
| `hover_trim.m` | **Adım 12: trim, uygulanabilir (≥0) tilt veren kanat rotorünü seçiyor** (düzeltme sonrası rotor 1 negatif tilt isterdi) |
| `tiltrotor_indi.slx` | Adım 12: `sf_wls_alloc.m` değişikliğiyle yeniden build edildi |
| `~/PX4-Autopilot/.../TiltrotorIndiParams.hpp` | **Adım 12: `ROTOR_KM` `{+0.06,-0.06,+0.06}` → `{-0.06,+0.06,-0.06}`** (tam türetme + SITL doğrulaması yorumda) |
| `~/PX4-Autopilot/.../TiltrotorIndiControl.hpp` | **Adım 12: `hoverTrim()` uygulanabilir kanat rotorünü seçiyor** |
| `~/PX4-Autopilot/.../MulticopterIndiTiltrotor.cpp` | **Adım 12: `YWdbg` yaw-ekseni atıf tanı logu eklendi** (talep vs üretilen yaw torku, yaw hızı, hız setpoint'i, tilt du'ları) |
| `tiltrotor_params.m` | **Adım 13: `p.ctrl.rate_sp_limit = [3.0; 3.0; 0.5]` eklendi** (eskiden kod içinde skaler 3.0) |
| `indi_attitude_controller.m` | **Adım 13: `omega_sp` doygunu `p.ctrl.rate_sp_limit` ile eksen bazlı** |
| `sf_indi_rate_law.m` | **Adım 13: aynı limit literal olarak `[3.0; 3.0; 0.5]`** (codegen-safe kopya) |
| `~/PX4-Autopilot/.../TiltrotorIndiParams.hpp` | **Adım 13: `RATE_SP_LIMIT` skaler → `RATE_SP_LIMIT[3] = {3.0, 3.0, 0.5}`** |
| `~/PX4-Autopilot/.../MulticopterIndiTiltrotor.cpp` | **Adım 13: `constrain(..., RATE_SP_LIMIT[i], ...)`** |
| `~/PX4-Autopilot/.../TiltrotorIndiParams.hpp` | Adım 14: `TILT_RATE_MAX` 3.0 DENENDİ → **2.0'a GERİ ALINDI** (gerekçe ve ölçüm yorumda) |
| `~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf` | **Adım 15: `CameraTracking` eklentisine `<follow_pgain>1.0</follow_pgain>`** (varsayılan 0.01 hızlı hedefe yetişemiyor; yalnızca GUI gözlemi etkiler, fiziği/kontrolü etkilemez) |
| `sitl/gz_follow.sh` | **Adım 12: yeni** — Gazebo GUI kamerasını modele kilitler (`/gui/follow` + `/gui/follow/offset` servisleri). GUI'li her koşuda zorunlu: yatay konum döngüsü olmadığı için araç görüş alanından çıkıyor (~170 m sürüklenme ölçüldü). |
| `.claude/skills/sitl-lockup-check/SKILL.md` | Adım 12: GUI'li koşu + kamera kilidi adımı, "yaw'ı açıdan değil hızdan ölç" uyarısı, "irtifada disarm etme" uyarısı eklendi |
| `README.md`, `sitl/RUNBOOK.md` | bulgular ve durum belgelendi (RUNBOOK'a §1a "Gazebo arayüzüyle izleme + kamera kilidi" eklendi) |
| `run_yaw_step_test.m` | **Adım 17: YENİ** — yaw adım yanıtı testi (saf MATLAB, ±30°, 25 s). Aşım / ±2° yerleşme / kalıcı-salınım (son 5 s yaw hızı RMS) metrikleri + δ0/δ1 ve talep-vs-üretilen Δτ_z grafikleri. `YAW_TEST_TILT_RATE_MAX` ile tilt slew limiti geçersiz kılınabilir (PX4 ile aynı 2.0'da koşmak için). Kontrol sabiti DEĞİŞTİRMEZ. |
| `yaw_step_test.png`, `yaw_step_test_rate2.0.png` | Adım 17: yukarıdaki testin A/B çıktıları |
| `~/PX4-Autopilot/Tools/.../models/tiltrotor_indi/model.sdf` | **Adım 18: `JointStatePublisher` eklentisi eklendi** (`motor_{0,1,2}_joint`) — gerçek tilt eklem açılarını 250 Hz yayınlar. Yalnızca gözlem; fiziği/kontrolü etkilemez (Adım 15'in `follow_pgain`'i ile aynı sınıf). |
| `sitl/shadow_vs_real.py` | **Adım 18: YENİ** — gölge `_u_actual` ile gerçek Gazebo eklem açısını hizalayıp karşılaştırır (istatistik + grafik). Yinelenen zaman damgalarını eler, p99 basar. |
| `sitl/gz_joint_csv.sh` | **Adım 18: YENİ** — gz `joint_state` topic'ini gerçek zamanlı CSV'ye indirger |
| `sitl/logger_topics_shadow.txt` | **Adım 18: YENİ** — PX4 logger'a `tiltrotor_indi_status` dahil 10 topic ekleyen, rebuild gerektirmeyen konfigürasyon. **Kullanınca `rootfs/etc/logging/logger_topics.txt`'e kopyalayın, ölçüm bitince SİLİN** (varsayılan log profilini tamamen değiştirir). |
| `sitl/shadow_vs_real_tilt.png`, `sitl/gz_joint_step18.csv.gz` | Adım 18: ölçüm çıktısı ve ham gerçek-eklem kaydı |
| `sitl/yaw_airspeed_ab.png` | **Adım 19: YENİ** — tek değişkenli A/B çıktısı (yaw_sp=0 sabit, yalnızca hız değişken). *Not: bu koşudan çıkarılan "hız sebep değil" sonucu Adım 20c'de geri alındı.* |
| `run_yaw_ablation.m` | **Adım 20: YENİ** — SITL kusurlarını (gölge aktüatör modeli, `omega_dot` gecikme/gürültü, `dt` jitter) MATLAB'a tek tek enjekte eden ablasyon. Kontrol sabiti DEĞİŞTİRMEZ. Çıktı `yaw_ablation.png`. |
| `sitl/yaw_flight_variability.png` | **Adım 20: YENİ** — aynı konfigürasyonla iki uçuşun zıt sonucu (söndü / 112 s sönmedi); (Q)'nun marjinal kararlılığının kanıtı. |
| `~/PX4-Autopilot/Tools/.../models/tiltrotor_indi/model.sdf` | **Adım 21: `JointStatePublisher`'a `rotor_{0,1,2}_joint` eklendi** — gerçek itkiyi eklem hızından türetmek için (`T = 2e-5·(w·20)²`). Yalnızca gözlem. |
| `sitl/gz_joint_csv.sh` | **Adım 21: rotor eklem HIZLARI da yazılıyor** (7 sütun); mesaj sınırından emit eden, 3 ve 6 eklemle de çalışan awk sürümü |
| `sitl/gz_joint_step21.csv.gz` | Adım 21: itki ölçümünün ham kaydı |
| `~/PX4-Autopilot/.../TiltrotorIndiParams.hpp` | **Adım 22 (KALICI): `TS_CTRL` kaldırıldı → `TS_BOX = 1/250` (ölçülen döngü hızı) + `TILT_SLEW_BOX_RATE = 1.25f` (tahsisat kutusu, rad/s). `TILT_RATE_MAX = 2.0f` artık YALNIZCA gölge modelin fiziksel servo limiti.** Tam gerekçe ve ölçümler yorumda. |
| `~/PX4-Autopilot/.../MulticopterIndiTiltrotor.cpp` | **Adım 22 (KALICI): WLS kutusu `TILT_SLEW_BOX_RATE * TS_BOX` ve `... * TS_BOX` kullanıyor** (eski: `TILT_RATE_MAX * TS_CTRL`, `... * TS_CTRL`). Tilt kutusu 1 ULP farkla aynı; itki üst sınırı 22.5 → 36 N/tick (ölçülen doyum %0.0, aktif değil). |
| `~/PX4-Autopilot/.../MulticopterIndiTiltrotor.cpp` | **Adım 23 (KALICI): `slewbox <rad/s>` custom command + `_slew_box_rate_mrs` (`px4::atomic<int32_t>`, millirad/s)** — kutu hızını UÇUŞ İÇİNDE değiştirmeye izin veren test kancası. Varsayılan `TILT_SLEW_BOX_RATE`, yani komut verilmezse davranış değişmez. Aralık kontrolü [0.1, `TILT_RATE_MAX`]. Aktif değer log'dan `\|du(3)\| p99.5 / TS_BOX` ile geri okunabilir. |
| `~/PX4-Autopilot/.../TiltrotorIndiParams.hpp` | **Adım 23 (KALICI): `TILT_SLEW_BOX_RATE` 1.25f → 1.75f** — tarama tablosu ve gerekçe yorumda. |
| `~/PX4-Autopilot/.../TiltrotorIndiParams.hpp` | **Adım 24: `TILT_STICTION_BAND = 0.01f` eklendi (rad, = friction/p_gain)** — gerçek servonun Coulomb ölü bandı. **Şu an KULLANILMIYOR** (aşağıya bkz.), yeniden denenmek istenirse diye türetmesiyle birlikte bırakıldı. |
| `~/PX4-Autopilot/.../MulticopterIndiTiltrotor.cpp` | **Adım 24: stiction ölü bandı gölge modele eklendi → KİLİTLENDİ → GERİ ALINDI.** Kod baseline'a döndü; neden kilitlendiği ve çevrimdışı doğrulamanın bunu neden kaçırdığı uzun yorum olarak bırakıldı (uyarı niteliğinde). |
| `sitl/servo_model.py` | **Adım 24: YENİ** — üç gölge tilt modelini (1. derece / tam 2. derece / 1. derece+ölü bant) kayıtlı `u_cmd` ile sürüp gerçek eklem açısına karşı karşılaştırır. **Uyarı: açık çevrim replay'dir, kapalı çevrim tuzaklarını gösteremez.** |
| `sitl/sweep_step.py` | Adım 23: `slewbox` taramasının adım-yanıtı metriklerini çıkarır |
| `sitl/gz_joint_step24_stiction.csv.gz` | Adım 24: kilitlenen koşunun ham gerçek-eklem kaydı |
| `.claude/skills/sitl-lockup-check/SKILL.md` | yeni — bkz. §6 |
| `.claude/skills/safe-control-change/SKILL.md` | yeni — bkz. §6 |
| `.claude/skills/flight-risk-status/SKILL.md` | yeni — bkz. §6 |

**Denenip GERİ ALINAN #1:** `ROTOR_PY` `[0.25,-0.25,0]`→`[-0.25,0.25,0]`
(`TiltrotorIndiParams.hpp`, `tiltrotor_params.m`, `sf_wls_alloc.m`) —
Adım 4, MATLAB'da regresyona neden oldu, üç dosyada da eski değerine
döndürüldü, PX4'e hiç deploy edilmedi.

**Denenip GERİ ALINAN #2:** `WS_YAW`/`Ws_yaw` `3.0`→`6.0`
(`TiltrotorIndiParams.hpp`, `indi_attitude_controller.m`,
`sf_wls_alloc.m`) — Adım 7, SITL'de yaw'ı hiç hızlandırmadı ve Fx/roll'u
kötüleştirdi, üç dosyada da 3.0'a döndürüldü, MATLAB testleriyle
doğrulandı, `slx` ve PX4 modülü baseline'a (Wu_tilt=3.0, km=0.06,
ROTOR_PY orijinal, Ws_yaw=3.0, T0dbg+T2dbg tanı logları) yeniden build
edildi — mevcut derlenmiş PX4 ikilisi şu an bu baseline'ı yansıtıyor.

**Denenip KALICI YAPILMADI (nötr/test edilebilir durumda bırakıldı):**
`leso_enable_yaw` — `test_sp` argümanıyla `1` denendi (Adım 6, kısmen
olumlu: roll/pitch iyileşti, yaw yakınsamadı), ama kod içindeki
varsayılan (`Run()`: `{true,true,false}`, `custom_command`: `false`)
DEĞİŞTİRİLMEDİ. İstenirse `test_sp ... 1 1 1` ile tekrar elle
denenebilir, rebuild gerekmez.

---

## 6. Proje skill'leri (2026-07-26)

Bu oturumdaki tekrarlayan hatalar (sandbox/arka-plan/pkill tuzakları;
MATLAB'da test etmeden SITL'e sabit değişikliği gönderme) sistematik
hale getirilmek üzere `.claude/skills/` altında üç proje skill'i olarak
belgelendi:

| Skill | Amaç |
|---|---|
| `sitl-lockup-check` | §4/RUNBOOK repro senaryosunu güvenli, standart bir prosedürle (sandbox/arka-plan gotchaları çözülmüş, ≥25s izleme zorunlu) çalıştırır. |
| `safe-control-change` | Herhangi bir kontrol sabiti (Ws/Wu/Kp/ROTOR_*) değiştirilmeden önce MATLAB-önce doğrulama disiplinini zorunlu kılar — Aday çözüm 4 ve Adım 7 fiyaskolarından çıkarılan ders. |
| `flight-risk-status` | §1a'daki GO/NO-GO değerlendirmesini güncel duruma göre hızlıca raporlar — kritiklik analizinin her seferinde sıfırdan türetilmesini önler. |

Bu skill'ler ileride bu projeyle çalışan herhangi bir oturumun (bu
oturum dahil, gelecekte) aynı hataları tekrarlamadan devam edebilmesi
için yazıldı.
