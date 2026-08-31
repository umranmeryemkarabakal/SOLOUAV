%RUN_LAND_DIFF_CHECK  p.ctrl.land_diff_max mekanizmasinin birim kapisi.
%
% Uc sey dogrular (2026-08-29):
%   1. Esik USTUNDE cikti, mekanizma hic yokmus gibi BIREBIR aynidir
%      -- yani ucus yollari (hover/gecis/seyir/sabit kanat) etkilenmez.
%   2. Esik ALTINDA |T0-T1| land_diff_max ile sinirlanir.
%   3. T0+T1 TOPLAMI degismez -- dikey kanal (Fz) ortalamadan geldigi icin
%      irtifa dongusu bu mekanizmadan hic etkilenmez.
%
% Neden gerekli: hover_gust/transition regresyonlari INIS ICERMEZ, dolayisiyla
% bu mekanizmanin CALISTIGINI gosteremezler; yalnizca bozmadigini gosterirler.
% Bu dosya eksik olan pozitif testtir.

p = tiltrotor_params();
cs = init_ctrl_state();
% Buyuk fark iceren bir aktuator durumu kur (arizanin imzasi: T0 yuksek, T1 sifir)
u_act = [34.0; 0.0; 5.0; 0.17; 0.0; 0.0];
att_sp=[0;0;0]; att=[0.003;0;0]; om=[0;0;0]; omd=[0;0;0]; Fsp=[0;-50];
run_one = @(agl) indi_attitude_controller(att_sp, att, om, omd, Fsp, u_act, cs, p, ...
                                          [1 1 0], pi/2, [], [], agl);
[T_hi,~,~,~] = run_one(10.0);   % esik USTU -> mekanizma kapali
[T_lo,~,~,~] = run_one(0.30);   % esik ALTI  -> devrede
[T_no,~,~,~] = indi_attitude_controller(att_sp, att, om, omd, Fsp, u_act, cs, p, ...
                                        [1 1 0], pi/2, [], []);  % agl HIC yok
fprintf('agl yok    : T0=%7.3f T1=%7.3f  fark=%7.3f  toplam=%7.3f\n', T_no(1),T_no(2),abs(T_no(1)-T_no(2)),sum(T_no));
fprintf('agl=10.0 m : T0=%7.3f T1=%7.3f  fark=%7.3f  toplam=%7.3f\n', T_hi(1),T_hi(2),abs(T_hi(1)-T_hi(2)),sum(T_hi));
fprintf('agl= 0.3 m : T0=%7.3f T1=%7.3f  fark=%7.3f  toplam=%7.3f\n', T_lo(1),T_lo(2),abs(T_lo(1)-T_lo(2)),sum(T_lo));
fprintf('\nKONTROL 1 (esik ustu = agl yok ile ayni): %s\n', string(isequal(T_hi,T_no)));
fprintf('KONTROL 2 (esik alti fark <= %.1f N)     : %s\n', p.ctrl.land_diff_max, string(abs(T_lo(1)-T_lo(2)) <= p.ctrl.land_diff_max+1e-9));
fprintf('KONTROL 3 (ORTALAMA korundu, |d|<1e-9)  : %s  [%.3e]\n', ...
        string(abs((T_lo(1)+T_lo(2))-(T_hi(1)+T_hi(2)))<1e-9), abs((T_lo(1)+T_lo(2))-(T_hi(1)+T_hi(2))));

% --- 4) TEMAS DALI: roll esigi asilinca fark TAMAMEN silinmeli ---
att_tilted = [12*pi/180; 0; 0];   % 12 deg > land_contact_roll (8 deg)
[T_ct,~,~,~] = indi_attitude_controller(att_sp, att_tilted, om, omd, Fsp, u_act, cs, p, ...
                                        [1 1 0], pi/2, [], [], 0.30);
[T_ct_hi,~,~,~] = indi_attitude_controller(att_sp, att_tilted, om, omd, Fsp, u_act, cs, p, ...
                                           [1 1 0], pi/2, [], [], 10.0);
fprintf('\ntemas (roll 12 deg, agl 0.3 m): T0=%7.3f T1=%7.3f  fark=%7.3f  toplam=%7.3f\n', ...
        T_ct(1),T_ct(2),abs(T_ct(1)-T_ct(2)),sum(T_ct));
fprintf('KONTROL 4 (temasta fark ~ 0)             : %s  [%.3e]\n', ...
        string(abs(T_ct(1)-T_ct(2))<1e-9), abs(T_ct(1)-T_ct(2)));
fprintf('KONTROL 5 (ayni roll ama 10 m: fark VAR) : %s  [%.3f N]\n', ...
        string(abs(T_ct_hi(1)-T_ct_hi(2))>1.0), abs(T_ct_hi(1)-T_ct_hi(2)));
fprintf('KONTROL 6 (temasta da TOPLAM korundu)    : %s  [%.3e]\n', ...
        string(abs(sum(T_ct(1:2))-sum(T_ct_hi(1:2)))<1e-9), abs(sum(T_ct(1:2))-sum(T_ct_hi(1:2))));

% --- 7-9) IKINCI TEMAS OLCUTU: "buyuk fark, ivme yok" (Adim 118) ---
% Bu, roll esiginin kacirdigi DUZ inisi yakalayan daldir; roll her yerde ~0.
n_tick = ceil(p.ctrl.land_contact_dwell / p.Ts_ctrl) + 2;

% (7) pdot = 0  -> mandal ATESLENMELI, fark sifirlanmali
cs7 = init_ctrl_state();  T7 = [];
for k = 1:n_tick
    [T7,~,cs7,~] = indi_attitude_controller(att_sp, att, om, [0;0;0], Fsp, u_act, cs7, p, ...
                                            [1 1 0], pi/2, [], [], 0.30);
end
fprintf('\nKONTROL 7 (pdot=0, %d tick sonra mandal)   : %s  [fark %.3f N]\n', ...
        n_tick, string(cs7.land_contact_latch && abs(T7(1)-T7(2)) < 1e-9), abs(T7(1)-T7(2)));

% (8) pdot BUYUK (arac tepki veriyor) -> mandal ATESLENMEMELI
cs8 = init_ctrl_state();  T8 = [];
for k = 1:n_tick
    [T8,~,cs8,~] = indi_attitude_controller(att_sp, att, om, [2.0;0;0], Fsp, u_act, cs8, p, ...
                                            [1 1 0], pi/2, [], [], 0.30);
end
fprintf('KONTROL 8 (pdot=2.0 -> mandal YOK)         : %s  [fark %.3f N]\n', ...
        string(~cs8.land_contact_latch && abs(T8(1)-T8(2)) > 1.0), abs(T8(1)-T8(2)));

% (9) Mandal ateslendikten sonra ucusa donus (agl 10 m) -> mandal TEMIZLENMELI
[~,~,cs9,~] = indi_attitude_controller(att_sp, att, om, [0;0;0], Fsp, u_act, cs7, p, ...
                                       [1 1 0], pi/2, [], [], 10.0);
fprintf('KONTROL 9 (agl 10 m -> mandal temizlendi)  : %s\n', string(~cs9.land_contact_latch));

% --- 10) ARTIM ANTI-WINDUP: temasta tilt kanali da RAMPA YAPMAMALI (Adim 119) ---
% Adim 118'in bulgusu: itki farki sifirlaninca sarma TILT kanalina gociyordu
% (SITL'de d0/d1 ~4.6 deg/s ramp, tilt farki 7.6 -> 38.1 deg). Bu test
% aktuator durumunu GERI BESLEYEREK o birikimi yeniden uretebilir hale getirir.
% A/B: TEK degisken, mandalin ateslenip ateslenmemesi. "Kapali" kolu, bekleme
% suresini erisilmez yaparak kurulur -- baska hicbir sey degismez.
p_off = p;  p_off.ctrl.land_contact_dwell = 1e6;   % mandal asla ateslenmez
g_on  = tilt_growth(p,     att_sp, att, om, Fsp, u_act);
g_off = tilt_growth(p_off, att_sp, att, om, Fsp, u_act);
% ESIK NEDEN 1.0 DEG: bu kosum 300 tick = 0.75 s (Ts_ctrl = 1/400). SITL'de
% olculen 38 deg'lik buyume ~15 s'ye yayiliyordu, yani ~2.5-4.6 deg/s. Buradaki
% 1.82 deg / 0.75 s = 2.4 deg/s AYNI mertebedir; esigi SITL'in toplam
% genliginden kopyalamak testi anlamsiz kilardi.
fprintf('KONTROL 10 (artim anti-windup tilt rampasini kesiyor): %s\n', ...
        string(g_on < 0.10 && g_off > 1.0));
fprintf('           mandal ACIK  tilt buyumesi: %7.2f deg  (< 0.10 olmali)\n', g_on);
fprintf('           mandal KAPALI tilt buyumesi: %7.2f deg  (>  1.0 olmali)\n', g_off);

% MATLAB kurali: yerel fonksiyonlar script'in EN SONUNDA tanimlanir.
function dgrow = tilt_growth(p, att_sp, att, om, Fsp, u0)
    u_fb = u0(:);  cs = init_ctrl_state();  d0 = NaN;  dmax = -Inf;
    for k = 1:300
        [T, d, cs, ~] = indi_attitude_controller(att_sp, att, om, [0;0;0], Fsp, u_fb, cs, p, ...
                                                 [1 1 0], pi/2, [], [], 0.30);
        u_fb = [T(:); d(:)];
        dd = abs(d(1) - d(2));
        if isnan(d0), d0 = dd; end
        dmax = max(dmax, dd);
    end
    dgrow = (dmax - d0) * 180/pi;   % deg, BASLANGICA gore BUYUME
end
