function [dtau, d_hat_out, state_out] = sf_indi_rate_law(att_sp, att, omega, omega_dot_raw, u_actual, leso_enable, state_in)
%SF_INDI_RATE_LAW  MATLAB Function blok icerigi: disaridaki attitude P +
%icerideki INDI rate kanunu + LESO bozucu telafisi + basit gain-scheduling.
%#codegen
%
% SAF fonksiyon (persistent YOK) — durum disaridan state_in(13x1) olarak
% girer, state_out(13x1) olarak cikar ve Simulink diyagraminda bir Unit
% Delay ile kendine geri baglanir (bkz. tiltrotor_indi_build.m). Bu, MATLAB
% Function bloklarinin "persistent + surekli ornekleme zamani" kisitini
% (bkz. dosya notu asagida) tamamen ortadan kaldirir.
%
% state_in/out duzeni (13x1): [z1(3); z2(3); prev_u_leso(3); leso_accum(1); omega_dot_filt(3)]
%
% Sabitler tiltrotor_params.m / gain_schedule.m / indi_attitude_controller.m
% ile AYNI degerlerdir (derleme-zamaninda gomulu).

z1            = state_in(1:3);
z2            = state_in(4:6);
prev_u_leso   = state_in(7:9);
leso_accum    = state_in(10);
omega_dot_filt= state_in(11:13);

Ts_ctrl = 0.0025;   % s, 400 Hz — tiltrotor_params.m: p.Ts_ctrl
Ts_leso = 0.005;    % s, 200 Hz — tiltrotor_params.m: p.Ts_leso
I_diag  = [0.2; 0.25; 0.25];

%% --- "Sensor" tarafi: raw turev + tek kutuplu filtre (alpha=0.3) ---
omega_dot_filt = omega_dot_filt + 0.3*(omega_dot_raw - omega_dot_filt);

%% --- Gain-scheduling (ortalama tilt acisina gore, smoothstep) ---
delta_bar = mean(u_actual(4:6));
s = max(0, min(1, delta_bar/(pi/2)));
w = 3*s^2 - 2*s^3;

Kp_att_hover  = [3.0; 3.0; 1.5];
Kp_att_cruise = [2.0; 2.0; 1.0];
Kp_att = Kp_att_hover + (Kp_att_cruise - Kp_att_hover)*w;

Kp_rate_hover  = [4.0; 4.0; 2.0];   % Adim 144: 10.0 DENENDI, GERI ALINDI
Kp_rate_cruise = [3.0; 3.0; 1.5];
Kp_rate = Kp_rate_hover + (Kp_rate_cruise - Kp_rate_hover)*w;

%% --- Disaridaki attitude P dongusu ---
e_att = att_sp - att;
e_att(3) = atan2(sin(e_att(3)), cos(e_att(3)));   % yaw hata sarmalama
% Eksen bazli rate setpoint doygunu (2026-07-27, Adim 13; eskiden skaler 3.0).
% Simulink codegen-safe rewrite oldugu icin p.* yerine literal tutulur --
% tiltrotor_params.m'deki p.ctrl.rate_sp_limit ile AYNI kalmali.
rate_sp_limit = [3.0; 3.0; 0.5];
omega_sp = max(min(Kp_att.*e_att, rate_sp_limit), -rate_sp_limit);

%% --- Ic INDI rate dongusu: istenen acisal ivme ---
e_omega = omega_sp - omega;
omega_dot_des = Kp_rate .* e_omega;

%% --- LESO guncelleme (decimasyonlu, sadece etkin eksenler) ---
d_hat = z2;
leso_accum = leso_accum + Ts_ctrl;
if leso_accum >= Ts_leso - 1e-12
    wo = 15;                 % rad/s, gozlemci bant genisligi
    beta1 = 2*wo;
    beta2 = wo^2;
    for ax = 1:3
        if leso_enable(ax) > 0.5
            e = omega(ax) - z1(ax);
            z1(ax) = z1(ax) + Ts_leso*(z2(ax) + prev_u_leso(ax) + beta1*e);
            z2(ax) = z2(ax) + Ts_leso*(beta2*e);
        end
    end
    d_hat = z2;
    leso_accum = leso_accum - Ts_leso;
end

d_hat_active = d_hat .* leso_enable;
omega_dot_des_adj = omega_dot_des - d_hat_active;

% ESO'nun "girisi": bozucu-telafili (GERCEKTEN uygulanmasi istenen) ivme —
% telafi ONCESI deger kullanilirsa ESO kendi telafisini gormemis gibi
% davranir ve z2 sinirsizca surukler.
prev_u_leso = omega_dot_des_adj;

%% --- INDI artimli kontrol kanunu ---
domega_dot = omega_dot_des_adj - omega_dot_filt;
dtau = I_diag .* domega_dot;

d_hat_out = d_hat;
state_out = [z1; z2; prev_u_leso; leso_accum; omega_dot_filt];

end
