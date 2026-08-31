function [fx_cmd, pitch_sp, release_hold, req_abort, state_out, warn_code] = forwardtrans_loop( ...
        enable, v_h, z, sat_thrust, state_in, p)
%FORWARDTRANS_LOOP  Hover'dan seyre ILERI gecis durum makinesi (madde V).
%
% Adim 42 (2026-08-03). backtrans_loop.m'in AYNADAKI karsiligi.
%
% NEDEN BU DOSYA BUGUNE KADAR YOKTU -- VE NEDEN BU BIR KUSURDU
% ------------------------------------------------------------
% Geri gecisin dort durumlu bir makinesi var (backtrans_loop.m), ileri yonun
% hicbir seyi yoktu: seyre cikmanin tek yolu `fx_sp`'yi disaridan rampalamakti,
% yani bir hata ayiklama konsolu. Asimetri kasitli degil TARIHSELDI -- geri
% gecis bir ARIZA olarak (engelleyici B5) ortaya cikti ve o yuzden yazildi;
% ileri yon "zaten calisiyor" sayildi cunku test betikleri onu elle suruyordu.
%
% Gereksinim 2026-08-03'te netlesti: gorev TAM OTONOM olmali (kalkis
% multikopter, seyir sabit kanat/tilt motor, inis multikopter). Pilot bu
% profilin surucusu degil, yalnizca mudahale yolu. Dolayisiyla "elle rampalanan
% bir girdi" bir yetenek degildir. Madde (V).
%
% TILT KOMUT EDILMEZ -- TILT BIR SONUCTUR
% ---------------------------------------
% Bu makine tilt acisi komut ETMEZ. `fx_cmd`'yi buyutur ve tilti WLS tahsisati
% kendi secer (olculdu: 15 m/s'de kanat tiltleri 45-54 deg). Bu, geri gecisin
% tam TERSI bir mekanizmadir ve sebebi ayni olcumdur: adim 31/faz 0, tilti
% suren seyin Fz kanali oldugunu gosterdi. Ileri yonde Fx TALEBI zaten
% tiltlenmeyi UCUZLASTIRIYOR (kanat tasidikca dFz/dT kuculuyor), yani burada
% tahsisatla savasmak gerekmiyor -- geri yonde gerekiyordu, cunku orada
% tahsisatin tercihi manevranin TERSI yondeydi ve yalnizca bir KUTU KISITI
% (tavan) onu degistirebiliyordu.
% *** GENEL: ayni fiziksel sonucu (tilt) elde etmenin dogru araci, o anda
% tahsisatin kendi tercihinin hangi yone baktigina baglidir. ***
%
% PITCH NEDEN 0 TUTULUYOR
% -----------------------
% Adim 29'un olcumu: bu araçta ~5-6 m/s ustunde burun yukari bir FRENLEME degil
% bir TIRMANMA komutudur (kanat tasimasi), ve irtifa dongusu buna karsi koyamaz
% -- 14.5 m/s'de pos_hold devreye girince arac 35 s boyunca 1.1 m/s tirmandi.
% Yani hizlanirken pitch'e dokunmak en kotu zamanda en tehlikeli koldur.
% Irtifa yalnizca irtifa dongusune (Fz) birakilir; RETRACT'in pitch = 0
% tercihiyle ayni gerekce.
%
% pos_hold BIRAKILMALI
% --------------------
% Aktifken pozisyon dongusu roll/pitch'in SAHIBIDIR ve fx_trim'i o saglar
% (madde N/P), yani fx komutunu o ezer. Bu yuzden makine `release_hold`
% bayragini kaldirir. Olculdu (adim 28, gecis testi): birakma handoff'u TEMIZ,
% birakma aninda hiz aktivitesi DUSUYOR (p RMS 0.2123 -> 0.1503), yani birakma
% bir basamak uretmiyor.
%
% VE ONEMLI OLAN: BU MANEVRA TEK YONLU BIR KAPIDIR
% ------------------------------------------------
% v_h > POS_ENGAGE_V_MAX (3 m/s) olur olmaz pos_hold ARTIK KABUL ETMEZ
% (adim 29'un kapisi). Yani ileri gecis basladiktan sonra hover'a donusun tek
% yolu GERI GECIStir. Bu yuzden buradaki "iptal" fx'i sifirlamak DEGILDIR:
% adim 30 tam olarak bunu denedi ve arac yavaslamadi, cunku `fx_sp = 0` tiltleri
% geri cekmez (Fx cok zayif bir amac terimi). Iptal, GERI GECISI ISTEMEKTIR.
% `req_abort` bunun icin var.
%
% DURUMLAR
% --------
%   0 IDLE   kapali. fx_cmd = 0, hold birakilmaz.
%   1 RAMP   hold birakilir, fx_cmd 0'dan p.ft.fx_cruise'a p.ft.fx_rate ile
%            cikar. pitch_sp = 0. Cikis: fx tavana vardi VE v_h >= cruise_v.
%   2 CRUISE fx_cmd = p.ft.fx_cruise sabit.
%
% EMNIYET, IKI TERIMLI (adim 38'in dersi: her esik bir ORTAMIN olcumudur;
% yanina ortamdan bagimsiz, tek yonlu bir emniyet koy):
%   1) IRTIFA BANDI -- |z - z_giris| > p.ft.alt_band. Bu, adim 29'un kacis
%      tirmanisinin (35 s'de 44 m) dogrudan dedektorudur ve aeroya baglidir.
%   2) SURE -- RAMP p.ft.timeout_s icinde seyir hizina ulasamazsa. Hiza HIC
%      bakmaz, yani gercek kanat farkli tasirsa bile gecerlidir: sinirsiz bir
%      "hizlanamiyorum" durumunu sinirli bir gecikmeye cevirir.
%
% *** IPTAL VARSAYILAN OLARAK KAPALIDIR (p.ft.allow_abort = false, 2026-08-04).
% GEREKSINIM: gorev profili tek parcadir -- kalkis multikopter, ucus SABIT
% KANAT, gecisler tilt motorla -- ve ILERI GECIS IPTAL OLMAZ. Yukaridaki iki
% terim SILINMEDI; dedektor olarak kalir ve `warn_code` ile disari verilir
% (loglanir), yalnizca EYLEMLERI kalkar.
%
% Bunun BEDELI acikca yazilmalidir: adim 29'un kacis tirmanisi artik otomatik
% bir tepki uretmez, yalnizca rapor edilir. Buna karsilik KAZANCI da olculmus
% bir seydir -- iptalin kendi kacis yolu garantili degildi: iptal geri gecisi
% ISTEMEK demekti (adim 30: `fx_sp = 0` tiltleri geri cekmiyor, arac
% yavaslamiyor), ve geri gecis BT_MIN_ALT = 15 m altinda BASLAMAYI REDDEDER.
% Sabitler bunu sifir marjla birakiyordu: FT_MIN_ALT(20) - FT_ALT_BAND(5) = 15
% = BT_MIN_ALT. Yani ALCALARAK iptal eden bir gecis, tam olarak kacis yolunun
% reddedildigi irtifada iptal ediyordu; o kosede iptal etmek, etmemekten
% KOTUYDU (arac seyir hizinda, tiltler onde, kimsenin sahiplenmedigi bir
% durumda kalirdi). Iptal geri getirilirse ONCE o marj kapatilmalidir.
%
% warn_code: 0 = yok, 1 = irtifa bandi, 2 = sure. allow_abort true ise ayni
% kosullar `req_abort` da verir ve eski davranis birebir geri gelir.
%
% Girisler:
%   enable      (logical) ileri gecis istegi
%   v_h         (m/s) yatay hiz buyuklugu -- burada BUYUKLUK dogru sinyaldir:
%               soru "kanat tasiyacak hava hizim var mi", aerodinamik bir soru
%               (madde (S)'nin ayrimi; bkz. backtrans_loop.m)
%   z           (m, NED) mevcut irtifa; giris degeri bandin referansi olur
%   sat_thrust  (logical) itki kanallarindan biri kutu sinirinda mi
%   state_in    [state; fx_cmd; z_entry; t_ramp]
%   p           tiltrotor_params()
% Ciktilar:
%   fx_cmd       (N)       govde-x kuvvet komutu (WLS tilti buradan turetir)
%   pitch_sp     (rad)     her zaman 0 (yukaridaki gerekce)
%   release_hold (logical) pos_hold BIRAKILMALI
%   req_abort    (logical) manevra basarisiz -- GERI GECIS istenmeli
%   state_out    [state; fx_cmd; z_entry; t_ramp]
%
% !! MATLAB BU MANEVRANIN AEROSUNU DOGRULAYAMAZ (bkz. backtrans_loop.m'in ayni
% uyarisi): tek boylamsal yuzey, 12 m/s'de ~25 N tasima. Buradaki varlik SENKRON
% ve MANTIK testi icindir (run_forwardtrans_sm_test.m); sayilarin dogrulanmasi
% SITL'de yapilir.

FT_IDLE = 0; FT_RAMP = 1; FT_CRUISE = 2;

state   = state_in(1);
fx_cmd  = state_in(2);
z_entry = state_in(3);
t_ramp  = state_in(4);

pitch_sp     = 0.0;      % adim 29: hizlanirken burun yukari = tirmanma
release_hold = false;
req_abort    = false;
warn_code    = 0;

if ~enable
    state_out = [FT_IDLE; 0.0; 0.0; 0.0];
    fx_cmd    = 0.0;
    return;
end

switch state
    case FT_IDLE
        % Yukselen kenar: irtifa bandinin referansini BURADA dondur.
        state   = FT_RAMP;
        fx_cmd  = 0.0;
        z_entry = z;
        t_ramp  = 0.0;
        release_hold = true;

    case FT_RAMP
        release_hold = true;
        t_ramp = t_ramp + p.Ts_pos;

        % Itki doygunsa DAHA FAZLA isteme. Bu bir iptal sebebi degil, bir
        % bekleme sebebi: tahsisat zaten kutuda, fx'i buyutmek yalnizca
        % cozulemeyen bir talep yaratir (adim 11'in dersi).
        if ~sat_thrust
            fx_cmd = min(p.ft.fx_cruise, fx_cmd + p.ft.fx_rate * p.Ts_pos);
        end

        % Dedektorler her zaman calisir; EYLEM p.ft.allow_abort'a baglidir.
        if abs(z - z_entry) > p.ft.alt_band
            warn_code = 1;                 % emniyet 1: kacis tirmanisi/dalisi
        elseif t_ramp >= p.ft.timeout_s && v_h < p.ft.cruise_v
            warn_code = 2;                 % emniyet 2: aero-bagimsiz sure
        end

        if warn_code ~= 0 && p.ft.allow_abort
            req_abort = true;
        elseif warn_code == 0 && fx_cmd >= p.ft.fx_cruise - 1e-6 && v_h >= p.ft.cruise_v
            state = FT_CRUISE;
        elseif warn_code ~= 0 && fx_cmd >= p.ft.fx_cruise - 1e-6 && v_h >= p.ft.cruise_v
            % Iptal kapaliyken uyari, CRUISE'a gecisi engellemez: manevra
            % tamamlanir, uyari yalnizca rapor edilir.
            state = FT_CRUISE;
        end

    case FT_CRUISE
        release_hold = true;
        fx_cmd = p.ft.fx_cruise;

        if abs(z - z_entry) > p.ft.alt_band
            warn_code = 1;                 % seyirde de gecerli
            if p.ft.allow_abort
                req_abort = true;
            end
        end

    otherwise
        state   = FT_IDLE;
        fx_cmd  = 0.0;
        z_entry = 0.0;
        t_ramp  = 0.0;
end

state_out = [state; fx_cmd; z_entry; t_ramp];

end
