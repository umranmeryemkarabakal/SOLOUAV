function [u_cmd, sat_flag, wls_state_out] = sf_wls_alloc(dtau, F_sp, u_actual, ...
                                                         agl, roll, pdot, wls_state_in)
%SF_WLS_ALLOC  MATLAB Function blok icerigi: anlik etkinlik Jacobian'i +
%agirlikli en kucuk kareler (WLS) kontrol tahsisi, kutu-kisitli, + INIS UCLUSU.
%#codegen
%
% effectiveness_matrix.m ile AYNI geometrik model, wls_allocate.m ile ayni
% amac fonksiyonu — ama MATLAB Function blogu icin `find`/degisken-boyutlu
% dizi kullanmadan, sabit-boyutlu "buyuk-M ceza" yontemiyle yeniden yazildi:
% kutu kisitini ihlal eden bir aktuator, kumeden CIKARILMAK yerine agirligi
% asiri buyutulerek (Wu->1e6) o sinira "yapistirilir" — active-set ile ayni
% sonuca yakinsar, degisken boyut gerektirmez.
%
% Girisler:
%   dtau      (3x1 Nm)  INDI moment artisi
%   F_sp      (2x1 N)   istenen govde-eksen [Fx;Fz]
%   u_actual  (6x1)     [T0;T1;T2;d0;d1;d2] aktuator GERCEK durumu
%   agl       (1x1 m)   YERDEN yukseklik -- ham -z DEGIL, bkz. asagidaki not
%   roll      (1x1 rad) olculen roll acisi (temas dalinin ilk olcutu)
%   pdot      (1x1 rad/s^2) olculen roll acisal ivmesi (ikinci temas olcutu)
%   wls_state_in (5x1)  [temas_bekleme_s; temas_mandali(0/1); prev_du_tilt(3)]
% Ciktilar:
%   u_cmd     (6x1)     yeni aktuator komutu (mutlak, [T;delta])
%   sat_flag  (6x1)     0/1, doyuma ulasan aktuatorler
%   wls_state_out (5x1) guncellenmis durum (Unit Delay ile geri baglanir)
%
% YUZEYLER BILEREK YOK (Adim 125): PX4'un UCAN yapilandirmasinda
% SURF_ENABLE = false, yani yuzeyler WLS tahsisatina HIC girmiyor. Iki deneme
% de SITL'de basarisiz oldu ve kayit olarak duruyor (TiltrotorIndiParams.hpp
% SURF_ENABLE basligi: 1. deneme uc rotoru 45 N'e cakti ve seyir tiltini
% 44 -> 13.3 dereceye dusurdu; 2. deneme araci TERS DONDURDU). MATLAB
% referansinin sanal-esleme yaklasimi (Gv = Gp*Mv) PX4'e hic tasinmadi, yani
% SITL'de hic denenmedi. Bu dosya UCAN yapilandirmayi hedefler.

% NOTE (2026-07-26): a Y sign-flip was tried here to match the SDF's
% "right/left" comments and REVERTED -- it caused a ~100-1000x RMS
% regression in the pure-MATLAB reference test. See tiltrotor_params.m note.
% SYNCED with tiltrotor_params.m p.rotor.pos (wing rotors moved above the
% wing, Z 0.06 -> -0.11, 2026-08-16 friend's fix).
% Kuyruk Z -0.07 -> -0.16 (Adim 132, geometrik cakisma duzeltmesi).
% tiltrotor_params.m p.rotor.pos ile SENKRON KALMALI.
rpos = [0.27   0.27  -0.55;
        0.35  -0.35   0.00;
       -0.11  -0.11  -0.07];
% SIGN FIX (2026-07-27, step 12): was [+0.06,-0.06,+0.06]. The magnitude
% matches the SDF <momentConstant> and the (+,-,+) pattern matches the
% ccw/cw/ccw turningDirections, but the OVERALL sign in FRD was inverted:
% gz applies tau_z(FLU) = -turningDirection*T*km, which is +km*T in FRD,
% while this model's m_i = km_i*T_i*dir_i (dir=(0,0,-1)) gives -km*T.
% See tiltrotor_params.m for the full derivation + SITL confirmation.
km = [-0.06, 0.06, -0.06];

T  = u_actual(1:3);
de = u_actual(4:6);

G   = zeros(5,6);
nu0 = zeros(5,1);
for i = 1:3
    s_i = sin(de(i)); c_i = cos(de(i));
    dir_i  = [s_i; 0; -c_i];
    ddir_i = [c_i; 0;  s_i];
    ri = rpos(:,i);

    f_i = T(i)*dir_i;
    m_i = km(i)*T(i)*dir_i;
    tau_i = cross(ri, f_i) + m_i;

    dtau_dT     = cross(ri, dir_i) + km(i)*dir_i;
    dtau_ddelta = (cross(ri, ddir_i) + km(i)*ddir_i) * T(i);

    nu0(1:3) = nu0(1:3) + tau_i;
    nu0(4) = nu0(4) + f_i(1);
    nu0(5) = nu0(5) + f_i(3);

    G(1:3, i)   = dtau_dT;
    G(1:3, 3+i) = dtau_ddelta;
    G(4, i)   = dir_i(1);
    G(4, 3+i) = T(i)*ddir_i(1);
    G(5, i)   = dir_i(3);
    G(5, 3+i) = T(i)*ddir_i(3);
end

%% --- Gain-scheduled WLS agirliklari ---
delta_bar = mean(de);
s = max(0, min(1, delta_bar/(pi/2)));
w = 3*s^2 - 2*s^3;

% NOT (madde (P), Adim 28): Fx trim BURADA uygulanmaz — position_loop.m'de
% uretilip F_sp(1) ile gelir, cunku bedeli olan kalici +Fx'i ancak yatay
% pozisyon dongusu tasiyabilir. Gerekce: indi_attitude_controller.m ayni yer.
dF = F_sp - nu0(4:5);
nu_des = [dtau; dF];

%% --- INIS UCLUSU, 1/3: TEMASTA MOMENT ARTIMI KESILIR (Adim 119) ---
% indi_attitude_controller.m'deki ayni blogun birebir karsiligi.
% Adim 118 olctu ki tek tek CIKIS kanallarini kirpmak sarmayi tahsisatin
% icinde KOVALIYOR: kanat itki farki sifirlaninca ayni sarma tilt kanalina
% gociyor (T0=T1 tam esitken d0/d1 ~4.6 deg/s rampa, tilt farki 7.6 -> 38.1
% deg, yaw kacti). Kusur cikista degil ARTIMDA: yerde omega_dot ~ 0 kaldigi
% icin dtau hic sonmuyor ve WLS onu hangi aktuator ucuzsa oraya biriktiriyor.
%
% YALNIZCA MOMENT: temasta arac agirlik merkezi etrafinda degil TEMAS NOKTASI
% etrafinda doner, yani etkinlik matrisinin moment satirlari o rejimde
% GECERSIZDIR. Kuvvet satirlari gecerlidir; Fz'yi de kesmek irtifa dongusunun
% yerde itkiyi azaltmasini engellerdi.
%
% Mandal BIR ONCEKI tikten gelir (land_state_in), yani degerlendirme bir tik
% gecikmelidir -- referansta da oyle.
land_dwell   = wls_state_in(1);
land_latch   = wls_state_in(2) > 0.5;
prev_du_tilt = wls_state_in(3:5);
if land_latch
    nu_des(1:3) = 0;
end
% wu_tilt_hover=3.0 (was 8.0): keeps tilt cheaper than the ~3.665 threshold
% where WLS starts preferring thrust drain over tilt for roll correction
% -- see gain_schedule.m for the full derivation. NOTE: SITL validation
% (2026-07-26) showed this alone does not fix the T0/T1 lockup, only
% relocates it -- see sitl/RUNBOOK.md "Aday cozum 2".
wu_tilt = 3.0 + (1.5 - 3.0)*w;

% Fx onceligi (4. satir) dusuk: yaw'i duzelten diferansiyel tilt ayni zamanda
% onemli bir Fx uretir; Fx=0'i roll/pitch mertebesinde agirliklandirmak WLS'i
% "sapmayi durdur" ile "Fx=0" arasinda sonlanmayan bir cekismeye sokuyor ve
% delta1 rate-limit sinirinda +/- salinip attitude asla referansa donmuyor.
% Deneysel olarak Ws_Fx ~0.1 altina inince bu cekisme tamamen kalkiyor (bkz.
% indi_attitude_controller.m ayni notu).
% NOTE (2026-07-26): Ws_yaw=6 was tried and REVERTED -- see sitl/RUNBOOK.md
% "Adim 7". It did not help yaw (nu_des(2) barely moved for ~14s, same as
% Ws_yaw=3) and made things worse elsewhere (Fx demand grew to -28N, roll
% grew larger, T0 still collapsed to 0 by the end).
Ws_v = [200; 200; 3; 0.05; 20];
Wu_v = [1; 1; 1; wu_tilt; wu_tilt; wu_tilt*3];

%% --- Kutu kisitlari (mutlak + hiz limiti) ---
Tmin = 0; Tmax = 45; dmin = 0; dmax = pi/2;
tau_up = 0.0125; tau_down = 0.025;
Ts_ctrl = 0.0025;
% AYRISTIRILDI 3.0 -> 4.8 (2026-07-29, Adim 27). Bu sabit artik fiziksel servo
% limiti (p.tilt.rate_max = 3.0, plant clamp'i) DEGIL, yalnizca WLS tahsisat
% kutusudur (p.tilt.slew_box_rate). Codegen-safe olmasi icin literal tutuluyor —
% tiltrotor_params.m'deki p.tilt.slew_box_rate ile SENKRON KALMALI.
% Deger PX4'un tick-basi kutusuna esitlendi: 3.00*(1/250) = 0.012 rad/tick,
% MATLAB 400 Hz'de 0.012/0.0025 = 4.8 rad/s. Gerekce: tiltrotor_params.m.
slew_box_rate_tilt = 4.8;

abs_lo = [Tmin;Tmin;Tmin;dmin;dmin;dmin] - u_actual;
% Kuyruk tilt tavani AYRI (Adim 133): dmax_tail = 20 deg. FIZIKSEL kisit --
% 0.10 m disk 90 derecede kuyruk cubugunun icinden geciyor. Motoru yukseltmek
% denendi ve geri alindi (geri gecişte BIG_M 0 -> 3843). tiltrotor_params.m
% p.tilt.max_tail ve model.sdf motor_2_joint <upper> ile SENKRON.
dmax_tail = 20.0*pi/180;
abs_hi = [Tmax;Tmax;Tmax;dmax;dmax;dmax_tail] - u_actual;
rate_lo = [-Tmax/tau_up*Ts_ctrl*5*ones(3,1);   -slew_box_rate_tilt*Ts_ctrl*ones(3,1)];
rate_hi = [ Tmax/tau_down*Ts_ctrl*5*ones(3,1);   slew_box_rate_tilt*Ts_ctrl*ones(3,1)];
du_min = max(abs_lo, rate_lo);
du_max = min(abs_hi, rate_hi);

%% --- TILTJERK (Adim 95/96b) ---
% Slew KUTUSUNUN kendisini KUCULTMEZ -- o, madde (Q)'nun tahsisat acligini
% yeniden acardi (Adim 23/27'de 1.25 -> 3.00 ile kapatilmisti). Bunun yerine
% BIR TUREV USTUNDE calisir: bu tikin du'sunun BIR ONCEKI tikin du'sundan ne
% kadar uzaklasabilecegini sinirlar, yani hiz kutunun bir ucundan digerine tek
% tikte siciramaz.
%
% NEDEN VAR (Adim 94): motor+rotor kutlesini hizla dondurmenin D'Alembert tepki
% torku (I_tilt = 0.01684 kg*m^2) hicbir modelde yok ama Gazebo'nun tam
% cok-cisim fizigi onu otomatik uretiyor -- olculen buyuklugu aktif momentle
% AYNI mertebede (%115). Kontrolcu bunu gormedigi icin kendi kendini besleyen
% bir pitch salinimi olusuyordu.
%
% DEGER: 0.3 rad/s. MATLAB'in ince taramasi 0.25'i onermisti ama GERCEK SITL
% olcumu 0.30'u her metrikte daha iyi buldu (yaw bandi [-4.61,+5.20] vs
% [-7.45,+6.58]; pitch p2p 12.13 vs 15.76 deg) ve karar OLCUME gore verildi
% (Adim 96b). PX4'te de bu deger kalici varsayilandir
% (MulticopterIndiTiltrotor.cpp: _tilt_jerk_limit_mrs = 0.3).
%
% ⚠ REFERANSTAN AYRISIR, BILEREK: indi_attitude_controller.m'de tilt_jerk_limit
% varsayilani Inf'tir (KAPALI) ve yalnizca run_poshold_climb_jerk_sweep.m onu
% veriyor. Bu dosya UCAN yapilandirmayi hedefler (Adim 125), dolayisiyla PX4'un
% varsayilanini alir. Parite testi referansa ayni degeri GECIRIR.
tilt_jerk_limit = 0.45;                     % rad/s (adim 134: canli
%                                            supurme, pitch t-t 11.34 -> 3.75 deg)
jerk_max = tilt_jerk_limit * Ts_ctrl;
du_min(4:6) = max(du_min(4:6), prev_du_tilt - jerk_max);
du_max(4:6) = min(du_max(4:6), prev_du_tilt + jerk_max);

du_min = min(du_min, du_max);

%% --- Buyuk-M cezali WLS (sabit boyutlu, degisken indeksleme yok) ---
du_pref = zeros(6,1);
Wu_eff  = Wu_v;
big_M   = 1e6;

du = zeros(6,1);
for it = 1:6
    Hm  = G' * (Ws_v.^2 .* G) + diag(Wu_eff.^2);
    rhs = G' * (Ws_v.^2 .* nu_des) + Wu_eff.^2 .* du_pref;
    du  = Hm \ rhs;

    viol_hi = du > du_max;
    viol_lo = du < du_min;
    if ~any(viol_hi) && ~any(viol_lo)
        break;
    end
    du_pref(viol_hi) = du_max(viol_hi);
    du_pref(viol_lo) = du_min(viol_lo);
    Wu_eff(viol_hi | viol_lo) = big_M;
end
du = max(min(du, du_max), du_min);

u_cmd = u_actual + du;
u_cmd(1:3) = max(min(u_cmd(1:3), Tmax), Tmin);

%% --- INIS UCLUSU, 2/3 ve 3/3: KANAT ITKI FARKI SINIRI + TEMAS MANDALI ---
% (Adim 112 sinir, Adim 117 kapinin AGL'si, Adim 118 ikinci temas olcutu.)
% indi_attitude_controller.m ile ayni sirada, ayni sabitlerle.
%
% SABITLER -- tiltrotor_params.m p.ctrl.* ile SENKRON KALMALI (codegen-safe
% olmasi icin literal):
land_diff_max     = 10.0;          % N,   p.ctrl.land_diff_max
land_diff_alt     = 2.0;           % m,   p.ctrl.land_diff_alt
land_contact_roll = 8.0*pi/180;    % rad, p.ctrl.land_contact_roll
land_contact_diff = 6.0;           % N,   p.ctrl.land_contact_diff
land_contact_acc  = 0.05;          % rad/s^2, p.ctrl.land_contact_acc
land_contact_dwl  = 0.20;          % s,   p.ctrl.land_contact_dwell
land_tz_max       = 2.0;           % x|Fz_sp|, TiltrotorIndiParams.hpp LAND_TZ_MAX
land_tail_floor_frac = 0.5;        % TiltrotorIndiParams.hpp LAND_TAIL_FLOOR_FRAC

% KAPI: `agl` YERDEN yukseklik olmak ZORUNDA. Kestirimcinin yerel orijininden
% yukseklik (-z) DEGILDIR -- 23 SITL kosumunda olculen datum ofseti
% -0.67 .. +1.77 m, yani asagidaki 2.0 m'lik esikle AYNI MERTEBEDE. PX4 portu
% Adim 117'ye kadar tam bu hatayi yapiyordu: arac 0.64 m'de yere degdi, sinyal
% 2.41 m dedi, mekanizma hic armanmadi ve fark 45 N'e (tam olcek) gitti.
% tiltrotor_indi_build.m bu porta kalkis datumuna gore duzeltilmis sinyali
% baglar; HAM -z BAGLANMAMALIDIR.
% NaN/Inf korumasi: gecersiz irtifayla kapiyi acmak mekanizmayi ait olmadigi
% bir rejimde devreye sokar; gecersizse KAPALI kalir (PX4'teki alt_ok ile ayni
% guvenli taraf).
if isfinite(agl) && agl < land_diff_alt
    T_mean = 0.5*(u_cmd(1) + u_cmd(2));
    T_diff = 0.5*(u_cmd(1) - u_cmd(2));
    d_lim  = 0.5*land_diff_max;

    % TEMAS DALI 1 -- roll esigi. Olculen TAKLA olayindan (-19.78 deg) turedi;
    % DUZ inisi yakalamaz (orada roll 0.18-0.54 derecede donuyor).
    if abs(roll) > land_contact_roll
        d_lim = 0.0;
    end
    % TEMAS DALI 2 -- mandal (Adim 118). Duz inisi yakalayan dal budur.
    if land_latch
        d_lim = 0.0;
    end

    T_diff = max(min(T_diff, d_lim), -d_lim);
    u_cmd(1) = T_mean + T_diff;
    u_cmd(2) = T_mean - T_diff;
    % Kutuya geri kirp: ortalama kutu icindeyse toplam da icindedir, ama
    % ortalama raya yakinken fark onu disari itebilir.
    u_cmd(1:2) = max(min(u_cmd(1:2), Tmax), Tmin);

    % MANDAL GUNCELLEMESI: "buyuk diferansiyel KOMUT, ama acisal ivme YOK".
    % Serbest ucusta imkansiz -- 6 N, 0.25 m kolda ~1.5 Nm eder ve govdeyi
    % ivmelendirmek zorundadir; yerde zemin momenti karsilar. 26 tam gorev
    % logunda olculen ayrim: AGL > 2 m'de olcut en fazla 0.01 s kesintisiz
    % surdu, AGL < 1.5 m'de 3.28 s'ye kadar. Bekleme suresi o yanlis-pozitifin
    % 20 kati. Mandal SART: fark sifirlaninca olcut kendi kendini bozar ve
    % mandalsiz mekanizma acilip kapanarak salinir.
    if (abs(u_cmd(1) - u_cmd(2)) > land_contact_diff) && (abs(pdot) < land_contact_acc)
        land_dwell = land_dwell + Ts_ctrl;
    else
        land_dwell = 0;
    end
    if land_dwell >= land_contact_dwl
        land_latch = true;
    end

    % DIKEY ITKI TAVANI (Adim 145). Tahsisat momentleri kuvvetten 200 kat agir
    % tartiyor, yani yerde olusan SAHTE bir moment talebi icin kuvvet komutunu
    % cignemekten cekinmez. Olculen ariza (ULog 11_28_57): temastan sonra dikey
    % itki toplami |Fz_sp|'nin 3.18 KATINA cikti ve arac 0.29 -> 1.18 m geri
    % kalkti. Yukaridaki kisma bunu GORMEZ: o yalniz kanat FARKINI (roll)
    % kisar, ariza ise on-vs-kuyruk (pitch) kanalinda.
    % Pitch farkini dogrudan kismak ISE YARAMAZ (olculdu): kapi altinda pitch
    % sapmasi saglikli inislerde de buyuk, cunku kanadin AERODINAMIK pitch
    % momenti de dengeleniyor. Bu olcut kontak tespitine hic bagli degil.
    % Uc rotor BIRLIKTE olceklenir -> moment ORANLARI korunur, net kaldirma kisilir.
    % KUYRUK ITKI TABANI (Adim 160, PX4'ten tasindi -- Adim 157'de olculdu).
    % Kuyruk rotoru 0'a cokunce yaw dengesi bozulur: ROTOR_KM = {-.06,+.06,-.06}
    % oldugu icin iki kanat rotoru birbirini GOTURUR (esitken), kuyruk
    % GOTURULMEZ. Kuyrugun torku her zaman kanat TILT FARKIYLA dengelenir:
    %     sin(d0) - sin(d1) = 0.171 * T2 / Tw
    % Trimde 0.171*16/17 = 0.161 -> 9.3 deg; SITL'de olculen 9.4 deg.
    % Kuyruk 0.15 N'e dusunce gereken fark 0.1 dereceye iner AMA TILT HIZ
    % SINIRLIDIR (tiltjerk), itki ise aninda duser: olculen fark hala 9.2 deg
    % ve arada -0.64 Nm dengelenmemis tork kalir. Arac YERINDE doner
    % (olculdu: yaw -29.9 deg, tepe 37.8 deg/s, yatay kayma yalnizca 0.27 m).
    % Alcalma icin gereken azaltma KANATLARDAN alinir; simetrik dusurmek yaw
    % dengesini bozmaz. Toplam dikey itki korunur, yani alcalma etkilenmez.
    % SITL olcumu: BIG_M 932 -> 0, doyum %6.9 -> %0, LAND yaw -29.9 -> +0.8 deg.
    arm_ratio  = 2*rpos(1,1) / abs(rpos(1,3));
    tail_share = arm_ratio / (2 + arm_ratio);
    ctz0 = u_cmd(1)*cos(u_cmd(4)) + u_cmd(2)*cos(u_cmd(5)) + u_cmd(3)*cos(u_cmd(6));
    tail_floor = land_tail_floor_frac * tail_share * max(ctz0, 0);
    if isfinite(tail_floor) && (u_cmd(3) < tail_floor)
        add = tail_floor - u_cmd(3);
        u_cmd(3) = tail_floor;
        take = 0.5*add;
        u_cmd(1) = max(min(u_cmd(1) - take, Tmax), Tmin);
        u_cmd(2) = max(min(u_cmd(2) - take, Tmax), Tmin);
    end

    ctz = u_cmd(1)*cos(u_cmd(4)) + u_cmd(2)*cos(u_cmd(5)) + u_cmd(3)*cos(u_cmd(6));
    tz_cap = land_tz_max * abs(F_sp(2));
    if isfinite(ctz) && isfinite(tz_cap) && (ctz > tz_cap) && (ctz > 1e-3)
        u_cmd(1:3) = max(min(u_cmd(1:3) * (tz_cap/ctz), Tmax), Tmin);
    end
else
    % Kapi kapali (ucus, veya gecersiz irtifa): mandal ve sayac temizlenir --
    % arac 2 m'nin ustune cikinca ucusa donus otomatiktir.
    land_dwell = 0;
    land_latch = false;
end

wls_state_out = [land_dwell; double(land_latch); du(4:6)];

% AYRISMA KAPANDI (Adim 125). Adim 115'ten Adim 124'e kadar bu dosya yere
% yakin itki farki sinirini ICERMIYORDU ve not buraya yazilmisti. Uc mekanizma
% da (Adim 112 sinir, Adim 117 datuma gore AGL, Adim 118 temas mandali,
% Adim 119 artim kesme) yukariya tasindi.
%
% GERIYE KALAN AYRISMALAR (Adim 123 envanteri, bu dosya icin):
%   - fx_trim (Adim 28): F_sp(1) uzerinden gelir, position_loop gerektirir
%   - tiltjerk (Adim 95/96b, 0.3 rad/s): tilt du'suna jerk limiti
%   - ileri/geri gecis, seyir, sabit kanat (Adim 59-103)
% Guncel envanter icin: run_codegen_parity_check
u_cmd(4:5) = max(min(u_cmd(4:5), dmax), dmin);
u_cmd(6)   = max(min(u_cmd(6), dmax_tail), dmin);

sat_flag = double(Wu_eff >= big_M);

end
