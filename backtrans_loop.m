function [tilt_ceil, pitch_sp, req_pos_hold, state_out] = backtrans_loop( ...
        enable, v_h, v_fwd, delta_wing_max, state_in, p)
%BACKTRANS_LOOP  Seyirden hover'a geri gecis durum makinesi (madde B5).
%
% Adim 31 (2026-07-29). Bu, Adim 30'da DENENIP GERI ALINAN decel_loop.m'in
% yerini alir -- ve ondan farki bir ayar farki degil, mekanizma farkidir.
%
% NEDEN decel_loop BASARISIZ OLDU, VE BURADA NE DEGISTI
% ----------------------------------------------------
% decel_loop hiza orantili burun yukari komut ediyordu ve `fx_sp = 0` vererek
% tilt'lerin geri cekilmesini UMUYORDU. Arac hic yavaslamadi. Adim 31 / Faz 0
% sebebi tahsisati her ornek icin CEVRIMDISI YENIDEN COZEREK olctu
% (sitl/analyze_backtrans_probe.py): tilt'i ileri suren sey Fx degil IRTIFA
% (Fz) kanalidir. 15 m/s'de kanat yukun bir kismini tasir, irtifa dongusu
% "tasimayi azalt" der (nu_des(Fz) = +2.9 N), ve dFz/ddelta = T*sin(delta)
% buyuk, dFz/dT = -cos(delta) ise tilt buyudukce sifira gittigi icin tahsisat
% icin taşımayı bosaltmanin EN UCUZ yolu tilt'i ILERI almaktir. Kendini
% besleyen dongu: tilt ileri -> hiz artar -> kanat tasimasi artar -> talep
% buyur. Fz talebini sifirlamak surukleneyi TERSINE ceviriyor (+0.64 ->
% -0.45 deg/s), Fx talebini sifirlamak hicbir sey degistirmiyor.
%
% Sonuc: Fx amac terimini agirliklandirmak (Ws_Fx) bu isi YAPAMAZ -- Ws_Fz=20
% ile Ws_Fx=0.05 arasinda amac fonksiyonunda 160.000x fark var, ve Ws_Fx'i o
% mertebeye cikarmak Adim 7'de yaw'i bozmustu. Agirliklarin tartisamayacagi
% tek sey KUTU KISITIDIR. Bu yuzden tilt bir TAVANLA (kanat kanallarinin
% abs_hi'si) surulur, bir tercihle degil. (Adim 19'un dersi: artimli bir
% tahsisatta bir aktuator konfigurasyonunu tercihle dayatamazsin.)
%
% VE EN ONEMLI DERS -- TAVAN ZAMANINDA BIRAKILMALIDIR
% ---------------------------------------------------
% Tavan takili tutulursa delta0 tavana, delta1 `p.tilt.min`e cakilir ve
% diferansiyel tilt yalnizca kucuk degil DEGISTIRILEMEZ olur. Diferansiyel
% tilt yaw'in tek gercek aktuatoru oldugu icin yaw'in kontrol otoritesi
% SIFIRA iner; elde yalnizca sabit bir trim kalir. Seyirde bunu aero
% weathervane sonumu maskeler (Adim 20: 11.6 m/s'de 9.1 Nm/rad, 2 m/s'de
% 0.27), yavaslayinca maske kalkar. SITL'de olculdu: tavan takiliyken
% frenleme sirasinda arac yaw'da KACTI (pitch +4'te 981 deg, +6'da 2117 deg
% donus). Tavan birakilinca ayni manevrada donus faz basina +-2 deg.
%
% Ve bu bir odun DEGIL: tavanin gerekcesi de yavaska yok olur -- taban
% fazlarinda olculen nu_des(Fz) ~ 0.00, cunku kanat artik tasimiyor. Yani
% tavan yalnizca hizliyken gereklidir.
%
% *** GENEL DERS: bir kisit, cozdugu problem gectikten sonra yururlukte
% kalirsa saf zarara doner. Kisiti eklerken onu ne zaman KALDIRACAGINI da
% tasarla. ***
%
% MADDE (R) -- RETRACT'IN CIKISI DA BIR DENGENIN SINIRINDAYDI (Adim 38)
% --------------------------------------------------------------------
% Yukaridaki dersin devami, ve adim 37'nin BRAKE'te bulduguyla AYNI hastalik
% bir evre once: bir kisiti kaldirmayi tasarlamak yetmiyor, kaldirma KOSULUNUN
% gercekten saglanabildigini de gostermek gerekiyor.
%
% Taban tavanda (9 deg) pitch = 0 iken arac durmaz, bir TERMINAL HIZA oturur:
% delta1/delta2 p.tilt.min'de cakili oldugu icin yaw trimi delta0'i tabanda
% tutar ve kalan ~2.4 N ileri itki surukleneyi dengeler. Adim 37 bu hizi
% olctu: 8.0-9.3 m/s. Eski cikis kosulu ise v_h < 8.0 istiyordu -- yani esik
% dengenin TAM SINIRINDA. Olculen sonuc: gecis bazen 22 s'de oluyor, bazen
% HIC olmuyor (bir ucusta RETRACT 200.3 s boyunca cikmadi, arac 8.97 m/s'nin
% altina inmedi). Tamamlanan ucuslar esigi ilk yavaslama gecicisinde gecmisti.
%
% Tehlikeli olan gecikme degil, O SIRADA TUTULAN KONFIGURASYON: tavan tam trim
% degerinde otururken yaw'in modulasyon otoritesi sifirdir (yukaridaki paragraf)
% -- 200 s'lik ucusta yaw +117.7 deg surukleendi.
%
% CIKIS BU YUZDEN IKI TERIME AYRISTIRILDI:
%   1) v_h < p.bt.release_v  -- esik artik dengenin DISINDA (8.0 -> 10.0).
%      Normal yolu hizli tutar. Ama bu sayi SITL aerodinamigine ait.
%   2) tabanda gecen sure >= p.bt.floor_dwell -- hiza HIC bakmaz, yani
%      terminal hiz ne olursa olsun gecerlidir. Donanimda (1) yanlis
%      olcaklenmis olsa bile yaw'i ac birakan konfigurasyonda gecirilen sure
%      SINIRLI kalir. Sinirsiz bir arizayi sinirli bir gecikmeye cevirir.
% Ikincisi IPTAL degil ILERLEME'dir: bekleme, RETRACT'in verebilecegi EN DUSUK
% hizda dolar (asimptot), ve BRAKE tavani yukselterek yaw otoritesini zaten
% geri verir. Yani "kurtarma" yonu manevrayi terk etmek degil, surdurmektir.
%
% MADDE (S) -- ESIK, YASANIN KONTROL ETTIGI EKSENI OKUMALIDIR (Adim 39)
% ---------------------------------------------------------------------
% Ayni hastaligin DORDUNCU gorulusu, ve bu sefer sebep ne otoritenin sonmesi
% (adim 37) ne esigin dengeye konmasi (madde R): esik DOGRU SINYALE BAKMIYORDU.
%
% BRAKE -> HANDOFF kosulu `v_h < p.bt.handoff_v` idi ve v_h bir BUYUKLUK
% (hypot(vx,vy)). Fren yasasi ise yalnizca burun yukari pitch komut ediyor,
% yani YALNIZCA GOVDE ILERI eksenini kontrol ediyor. Yanal eksende RETRACT ve
% BRAKE boyunca hicbir kontrol yok (pozisyon dongusu ancak HANDOFF'ta devreye
% girer), yani buyuklugu esigin ustunde tutan bilesen manevranin KALDIRAMAYACAGI
% bir bilesen olabilir.
%
% Olculdu (2026-07-31, sitl/diag_brake_reversal.py, log 14_13_11): BRAKE
% penceresinde min|v_h| = 3.08 m/s (esik 3.0) iken govde ileri hizi
% u = -0.51 m/s, yanal hiz v = +3.04 m/s. Yani arac ILERI yonde COKTAN durmustu;
% esigi tutan sey yanal surukleneydi. Handoff hic istenmedi, sonmeyen fren
% pitch'i (3.39 deg trim + 4.0 deg marj = 6.3 N, yenmesi gereken 3.1-4.1 N'un
% USTUNDE) itmeye devam etti ve arac GERI yonde 12.8 m/s'ye kacti -- tek ucusta
% bes kez. Buyukluk esigi bu kacisi gormez, hatta arac geri hizlandikca buyukluk
% yeniden BUYUR, yani esik gitgide daha ulasilmaz olur: pozitif geri besleme.
%
% DUZELTME IKI YERDE, IKISI DE AYNI ILKEDEN:
%   1) Cikis kosulu isaretli govde ileri hizini okur: `v_fwd < handoff_v`.
%      Isaretli olmasi onemli -- u negatifse (arac geri gidiyorsa) BRAKE'in
%      yapacak isi ZATEN bitmistir, mutlak deger almak ayni hatayi tekrarlardi.
%   2) Frenleme MARJI da v_fwd ile soner (max(0, v_fwd)), yani arac geri
%      gitmeye basladiginda marj SIFIRLANIR ve geriye yalnizca durus trimi
%      kalir. Kacisi fiilen durduran terim budur; (1) tek basina durumu
%      degistirir ama itmeyi kesmez.
% Duruş trimi terimi SONMEZ (adim 37): yendigi ileri kuvvet hizdan bagimsiz.
%
% RETRACT NEDEN HALA v_h OKUYOR (bilincli, ayni gerekceden): oradaki soru
% "hala frenlemem gereken ileri hizim var mi" degil, "kanat hala tasiyor mu,
% yani Fz kaynakli tilt kacisi mumkun mu" -- ve o aerodinamik bir sorudur,
% cevabi hava hizinin BUYUKLUGUDUR. Iki evre ayni sinyale bakmak zorunda degil;
% her esik KENDI yasasinin sorusunu sormalidir.
%
% KALAN ACIK NOKTA (kapatilmadi, olculecek): pos_hold'un devreye girme kapisi
% (POS_ENGAGE_V_MAX) hala bir BUYUKLUK kapisidir, yani v_fwd dustugu halde
% yanal surukleme 3 m/s'nin ustundeyse HANDOFF istenir ama hold REDDEDER ve
% istek her tick yeniden denenir. O beklemede pitch = trim (marj sifir), tavan
% birakilmistir ve yanal eksen hala kontrolsuzdur -- eski davranistan kesin
% olarak iyi, ama tam bir teslim degil. Sureyi SITL'de olcun; uzunsa dogru
% cozum yanal ekseni erken kapatmak (BRAKE'te yanal sonumleme) ya da o kapiyi
% da yon-duyarli yapmaktir. Ikisi de kendi olcumunu hak eder, tahminle degil.
%
% DURUMLAR
% --------
%   0 IDLE     kapali. tilt_ceil = p.tilt.max (kisit yok), pitch_sp = 0.
%   1 RETRACT  tavan mevcut kanat tilt'inden p.bt.retract_rate ile
%              p.bt.ceil_floor'a iner. pitch_sp = 0 (burun yukari BU HIZDA
%              tehlikeli -- Adim 29: 14.5 m/s'de pitch doydu, arac 35 s
%              tirmandi). Frenleme tilt'i geri cekmekten gelir: surukleme
%              isini yapar, cunku tilt bir FREN degil GAZ KOLUDUR.
%              Cikisi IKI TERIMLIDIR, bkz. asagidaki madde (R) notu.
%   2 BRAKE    tavan BIRAKILIR (yaw aktuatoru geri verilir) ve sinirli burun
%              yukari devreye girer. Cikisi GOVDE ILERI hizina bakar
%              (madde (S)), buyukluge degil.
%   3 HANDOFF  pos_hold istenir. pitch yasasi ayni kalir; pos_hold devreye
%              girdiginde zaten roll/pitch'i devralir (bkz. Run()).
%
% Durumlar TEK YONLUDUR (geri donus yok): +6 deg asiri komutta arac durduktan
% sonra GERIYE hizlanip v_h'yi 0.63 -> 5.77 m/s'ye cikardi, yani v_h'nin
% yeniden yukselmesi RETRACT'a donmek icin gecerli bir sebep degil.
%
% enable dusurulurse tavan p.tilt.max'e BIRAKILIR (kisiti kaldirmak guvenli
% yondur; takili birakmak yaw'i oldurur).
%
% Girisler:
%   enable          (logical) geri gecis istegi
%   v_h             (m/s)  yatay hiz BUYUKLUGU -- yalnizca RETRACT kullanir
%                   (aerodinamik soru: kanat hala tasiyor mu)
%   v_fwd           (m/s)  ISARETLI govde ileri hizi (yatay hizin bas yonune
%                   izdusumu: vx*cos(psi) + vy*sin(psi)). BRAKE/HANDOFF bunu
%                   kullanir -- fren yasasinin kontrol ettigi eksen budur
%                   (madde (S)). Negatif = arac GERI gidiyor.
%   delta_wing_max  (rad)  max(delta0, delta1), tavanin baslangic degeri
%   state_in        [state; tilt_ceil; floor_dwell]  (floor_dwell: s, tavanin
%                   tabanda gecirdigi sure -- madde (R) emniyeti)
%   p               tiltrotor_params()
% Ciktilar:
%   tilt_ceil     (rad)     kanat tilt kutusu ust siniri (kuyruga UYGULANMAZ)
%   pitch_sp      (rad)     burun yukari attitude setpoint'i (>= 0)
%   req_pos_hold  (logical) yatay pozisyon dongusu istegi
%   state_out     [state; tilt_ceil; floor_dwell]
%
% !! MATLAB BU MANEVRAYI DOGRULAYAMAZ (Adim 30'da olculdu): MATLAB plant'i tek
% bir boylamsal yuzey kullanir (p.aero.area = 0.5 m^2), 12 m/s / 15 deg'de
% tasima ~25 N -- 49 N agirligi kaldiramaz, yani ne kacis tirmanisi ne de
% Fz kaynakli tilt kacisi burada olusur. Gazebo modelinde bes lift-drag yuzeyi
% var. Bu dosyanin buradaki varligi SENKRON ve referans icindir; dogrulama
% yalnizca SITL'de yapilir (Adim 21d: bir ortam ancak hedeflenen mekanizma
% orada AKTIFSE bir sey kanitlar).

BT_IDLE = 0; BT_RETRACT = 1; BT_BRAKE = 2; BT_HANDOFF = 3;

state       = state_in(1);
tilt_ceil   = state_in(2);
floor_dwell = state_in(3);

req_pos_hold = false;
pitch_sp     = 0.0;

if ~enable
    state       = BT_IDLE;
    tilt_ceil   = p.tilt.max;
    floor_dwell = 0.0;
    state_out   = [state; tilt_ceil; floor_dwell];
    return;
end

switch state
    case BT_IDLE
        % Yukselen kenar: tavani MEVCUT tilt'ten baslat ki hemen baglayici olsun.
        state       = BT_RETRACT;
        tilt_ceil   = min(p.tilt.max, delta_wing_max);
        floor_dwell = 0.0;

    case BT_RETRACT
        tilt_ceil = max(p.bt.ceil_floor, tilt_ceil - p.bt.retract_rate * p.Ts_pos);
        at_floor  = tilt_ceil <= p.bt.ceil_floor + 1e-6;

        % Sayac YALNIZCA tabanda islesin: olctugu sey "inis ne kadar surdu"
        % degil, "tabanda ne kadar bekledik". Tavan hala inerken gecen sure
        % giris hizina bagli ve dengeyle ilgisi yok.
        if at_floor
            floor_dwell = floor_dwell + p.Ts_pos;
        else
            floor_dwell = 0.0;
        end

        % Birakma icin tavan HER HALUKARDA tabana varmis olmali; ikinci kosul
        % iki terimlidir (madde (R), Adim 38):
        %   - hiz dengenin disindaki esigin altina indi (normal, hizli yol), YA DA
        %   - tabanda p.bt.floor_dwell kadar beklendi (aero-bagimsiz emniyet;
        %     terminal hiz esigin ustunde kalirsa cikis yine de olur).
        if at_floor && (v_h < p.bt.release_v || floor_dwell >= p.bt.floor_dwell)
            state = BT_BRAKE;
        end

    case BT_BRAKE
        % Tavan KALDIRILMAZ, YUKSELTILIR. Trim'in ustune cikarmak yaw'a
        % modulasyonunu geri verir (delta1 p.tilt.min'de oldugu icin
        % diferansiyel = delta0, artik [0, brake_ceil] arasinda serbest);
        % bir tavanin VAR olmaya devam etmesi ise Fz kacisinin yeniden
        % baslamasini engeller -- bu hizda tamamen birakmak manevranin kendini
        % bozmasina yol acti. Bkz. p.bt.brake_ceil.
        tilt_ceil = p.bt.brake_ceil;
        pitch_sp  = brake_pitch(v_fwd, p);
        % Madde (S): ISARETLI govde ileri hizi. Buyukluk esigi, manevranin
        % kaldiramadigi yanal bir bilesen yuzunden sonsuza kadar saglanmayabilir
        % -- olculdu ve arac geri kacti. u < 0 de gecerli bir cikistir: BRAKE'in
        % isi ileri hizi bitirmekti, bitti.
        if v_fwd < p.bt.handoff_v
            state = BT_HANDOFF;
        end

    case BT_HANDOFF
        % Kisit ARTIK gercekten kalkabilir: handoff_v altinda kacisi surecek
        % kanat tasimasi kalmadi, ve cozdugu problem gectikten sonra yururlukte
        % kalan bir kisit tam olarak bu dosyanin basinda uyardigi hatadir.
        tilt_ceil    = p.tilt.max;
        pitch_sp     = brake_pitch(v_fwd, p);
        req_pos_hold = true;

    otherwise
        state       = BT_IDLE;
        tilt_ceil   = p.tilt.max;
        floor_dwell = 0.0;
end

state_out = [state; tilt_ceil; floor_dwell];

end


function pitch_sp = brake_pitch(v_fwd, p)
%BRAKE_PITCH  Duruş trimi + ILERI hizla orantili frenleme payi.
%
% AYRISTIRILDI 2026-07-30 (Adim 37). Onceki yasa yalnizca
% `p.bt.pitch_max * v_h/brake_v_full` idi, yani hiz sifira giderken burun
% yukari acisi da SIFIRA gidiyordu. Bu yanlisti: aracin yenmesi gereken ILERI
% kuvvet hizla azalmiyor, SABIT. delta1 ve delta2 p.tilt.min'de cakili
% oldugundan (madde (P), tek yonlu tilt araligi) yaw trimi delta0'i 10-15 deg'de
% tutuyor ve bu surekli ~3.1-4.1 N ileri itki uretiyor. Yani "yerinde durmak"
% bile bir burun yukari acisi gerektiriyor: asin(fx_trim/(m*g)) = 3.39 deg.
%
% Sonuc olarak eski yasa iki bagimsiz SITL ucusunda manevrayi devir hizinin
% ustunde takti (3.2-3.5 m/s'de 90+ s kararli denge), ve pitch_max = 4 deg
% (3.42 N) ile duzeltildikten sonra bile ucuslardan biri 4.9 m/s'de takildi:
% o ucusta tilt 14.86 deg'e oturmus, ileri kuvvet 4.11 N idi -- yani frenleme
% otoritesi yapisal itkinin ta kendisiyle AYNI mertebede.
%
% Dogru ayristirma fizigin kendisi: toplam = (yerinde durmak icin gereken) +
% (frenlemek icin fazladan). Ilk terim sonmez, ikincisi soner. Boylece
% v_h -> 0'da yasa 3.39 deg'de kaliyor (yerinde durma), 0'a dusup araci yeniden
% ileri birakmiyor; ve "durmus araca +6 deg GERIYE hizlandirir" olcumuyle de
% celismiyor, cunku bu yasa durmus araca 6 deg vermiyor.
% Devir hizinin altinda pitch'in sahibi zaten pos_hold oluyor (madde (N)).
%
% Not: ilk terim p.ctrl.fx_trim'e bagli, yani fx_trim gercek arac uzerinde
% yeniden olculdugunde (donanim kontrol listesi) bu yasa kendiliginden duzelir.
%
% MADDE (S) DUZELTMESI (Adim 39): MARJ ARTIK BUYUKLUKLE DEGIL ISARETLI GOVDE
% ILERI HIZIYLA soner. Eski hali v_h (buyukluk) okuyordu, yani arac ileri yonde
% durup GERI hizlanmaya basladiginda bile marj yeniden BUYUYOR ve aracı daha da
% geri itiyordu -- olculdu: 12.8 m/s'ye geri kacis, tek ucusta bes kez.
% max(0, .) sayesinde v_fwd <= 0 iken marj tam olarak sifirdir ve geriye
% yalnizca durus trimi kalir: yani "geri gidiyorsan frenleme kuvveti uygulama",
% isaret degisiminde sert bir kesme olmadan.
pitch_trim = asin(min(1.0, p.ctrl.fx_trim / (p.m * p.g)));
pitch_sp = pitch_trim + p.bt.pitch_max * min(1.0, max(0.0, v_fwd) / p.bt.brake_v_full);
end
