%% RUN_FULL_TRANSITION_GLIDE_TEST
% Kullanicinin tam senaryosu: mevcut kontrolcuyle hiz+irtifa kazan -> TUM
% MOTORLARI KAPAT -> suzulme sirasinda BASIT bir yuzey-only P kontrolle
% (tam INDI/WLS DEGIL) roll/pitch'i sinirla, AYNI ANDA kanat rotorlerini
% 90 dereceye tilt et -> motorlari yeniden calistir (artik iki ileri-itki
% pervane + kuyruk kapali/dik).
%
% Basit yuzey P yasasi (INDI/LESO/WLS'in aksine, T=0 rejimi icin ozel):
%   a_ail = -Kp_roll  * phi    (aileron, differential elevon)
%   a_ele = -Kp_pitch * theta  (elevator)
%   a_rud = -Kp_yaw   * psi    (rudder, kucuk)
% Isaretler effectiveness_matrix.m'deki qbar katsayilarindan turetildi
% (tau_x = -1.2*qbar*a_ail, tau_y = +0.806*qbar*a_ele, tau_z = -0.142*qbar*a_rud).

clear; clc;
addpath(fileparts(mfilename('fullpath')));

p = tiltrotor_params();
u_trim = hover_trim(p);

x0 = zeros(24,1);
x0(3)     = -80;
x0(7:10)  = quat_from_euler(0,0,0);
x0(14:19) = u_trim;

V_TRIGGER = 12.0;
TILT_RAMP_RATE = deg2rad(20.0);   % rad/s, motor kapaliyken HIZLI tilt (bedava, kuvvet yok)
Kp_roll  = 0.15;
Kp_pitch = 0.15;
Kp_yaw   = 0.05;

att_sp = [0;0;0];
z_sp   = x0(3);
Fx_final = 12;
t_ramp   = 12;
Tsim = 60;   % tum kontrolcuyu bastan sona test icin uzatildi (once 30s, relight sonrasi sadece ~15s cruise vardi)
dt_ctrl = p.Ts_ctrl;
n_sub = 5;
dt_phys = dt_ctrl/n_sub;
N = round(Tsim/dt_ctrl);

leso_axes = [true;true;false];

x = x0;
ctrl_state = init_ctrl_state();
omega_dot_filt = zeros(3,1);
alt_state = 0;
alt_accum = 0;
Fz_sp = -p.m*p.g;
a_virt = zeros(3,1);
gliding = false;
t_glide_start = NaN;
delta_wing = NaN;
relit = false;
fw_state = [0;0];
V_CRUISE_SP = 16.0;   % fixedwing_control_law.m'in trim'ini bulduğu hız (find_fixedwing_trim.m)
t_relit_start = NaN;
ROLL_CMD_DELAY = 5.0;  % relight'ten bu kadar sn sonra roll komutu (kontrolcunun once irtifa/hizi oturtmasina izin ver)
ROLL_CMD_DUR   = 6.0;
ROLL_CMD_DEG   = 10.0;

log.t = zeros(N,1);
log.att = zeros(3,N);
log.omega = zeros(3,N);
log.T = zeros(3,N);
log.delta = zeros(3,N);
log.surf = zeros(5,N);
log.vel_body = zeros(3,N);
log.pos = zeros(3,N);
log.phase = zeros(N,1);

for k = 1:N
    t = (k-1)*dt_ctrl;
    Fx_sp = Fx_final * min(1, t/t_ramp);

    wind_ned = [0;0;0];
    ext_m = [0;0;0];

    att = quat_to_euler(x(7:10));
    omega = x(11:13);
    T_act = x(14:16);
    delta_act = x(17:19);
    vel_b = x(4:6);
    V = norm(vel_b);
    qbar = 0.5 * p.rho * V^2;

    if ~gliding && vel_b(1) >= V_TRIGGER
        gliding = true;
        t_glide_start = t;
        delta_wing = mean(delta_act(1:2));
        fprintf('t=%.2f s: v_fwd=%.2f m/s -- TUM MOTORLAR KAPANDI, kanat tilt hizli rampalaniyor\n', t, vel_b(1));
    end
    if gliding
        delta_wing = min(delta_wing + TILT_RAMP_RATE*dt_ctrl, p.tilt.max);
    end
    tilt_done = gliding && delta_wing >= p.tilt.max - 1e-3;
    if tilt_done && ~relit
        relit = true;
        t_relit_start = t;
        fprintf('t=%.2f s: kanat tilt tamamlandi (90 deg) -- MOTORLAR YENIDEN CALISIYOR\n', t);
    end

    if ~gliding
        u_actual9 = [T_act; delta_act; a_virt];
        xdot_now = tiltrotor_plant_deriv(x, T_act, delta_act, p, wind_ned, ext_m, zeros(p.surf.n,1));
        omega_dot_raw = xdot_now(11:13);
        omega_dot_filt = omega_dot_filt + 0.3*(omega_dot_raw - omega_dot_filt);

        alt_accum = alt_accum + dt_ctrl;
        if alt_accum >= p.Ts_pos - 1e-12
            vz_ned = xdot_now(3);
            [Fz_sp, alt_state] = altitude_loop(z_sp, x(3), vz_ned, alt_state, p);
            alt_accum = alt_accum - p.Ts_pos;
        end
        F_sp = [Fx_sp; Fz_sp];

        [T_cmd, delta_cmd, ctrl_state, diagn, surf_cmd] = indi_attitude_controller( ...
            att_sp, att, omega, omega_dot_filt, F_sp, u_actual9, ctrl_state, p, leso_axes, [], qbar);
        a_virt = diagn.a_cmd;
    elseif ~relit
        % SUZULME: motorlar kapali, BASIT yuzey P kontrolu
        T_cmd = [0;0;0];
        delta_cmd = [delta_wing; delta_wing; delta_act(3)];  % kuyruk oldugu yerde (dik, guc yok)
        % ISARET DUZELTMESI: tau_x = -1.2*qbar*a_ail (effectiveness_matrix.m),
        % yani phi>0'i duzeltmek (tau_x<0 istiyoruz) icin a_ail>0 gerekir --
        % onceki -Kp*phi TERS isaretliydi (pozitif geri besleme).
        % MIMARI DUZELTMESI (Adim 75): rudder ARTIK heading hatasina dogrudan
        % tepki vermiyor -- sadece yaw-rate sonumu. roll_sp=0 (bu kisa
        % surzulme fazinda sabit kanatlarla kalmak yeterli, heading takibi
        % ACTIVE'e birakiliyor).
        a_ail = +Kp_roll  * att(1);
        a_ele = -Kp_pitch * att(2);
        a_rud = 0;   % Adim 77: rudder kalici olarak kapali (fixedwing_control_law.m ile tutarli)
        a_virt = [a_ail; a_ele; a_rud];
        Mv = surf_virtual_map(p);
        surf_cmd = Mv * a_virt + surf_trim_offset(p, qbar);
        surf_cmd = max(min(surf_cmd, p.surf.max(:)), -p.surf.max(:));
    else
        % SABIT KANAT MODU (Adim 59): INDI/WLS'e DONMEK YERINE ozel,
        % yuzey-merkezli fixedwing_control_law.m -- rotor-tilt otoritesi
        % olmayan bu rejimde eski relight denemesi cokuyordu (pitch 84,
        % roll 180), standalone testte ayni yasa irtifayi 0.15m sapmayla
        % tutuyordu.
        % Adim 75: MIMARI DUZELTMESI -- artik dogrudan roll_sp komut
        % edilmiyor, bir HEADING DEGISIKLIGI komut ediliyor (bank-to-turn).
        t_since_relit = t - t_relit_start;
        if t_since_relit >= ROLL_CMD_DELAY
            hdg_sp = deg2rad(ROLL_CMD_DEG * 2);  % onceki "roll komutu" testinin yerini alan heading komutu
        else
            hdg_sp = 0;
        end
        [T_wing, surf_cmd, fw_state] = fixedwing_control_law( ...
            hdg_sp, z_sp, V_CRUISE_SP, att, omega, x(3), x(6), vel_b(1), qbar, fw_state, p);
        T_cmd = [T_wing; T_wing; 0];
        delta_cmd = [p.tilt.max; p.tilt.max; delta_act(3)];
        a_virt = zeros(3,1);
    end

    for s = 1:n_sub
        x = rk4_step(@(xx) tiltrotor_plant_deriv(xx, T_cmd, delta_cmd, p, wind_ned, ext_m, surf_cmd), x, dt_phys);
    end

    log.t(k) = t;
    log.att(:,k) = att;
    log.omega(:,k) = omega;
    log.T(:,k) = T_act;
    log.delta(:,k) = delta_act;
    log.surf(:,k) = x(20:24);
    log.vel_body(:,k) = vel_b;
    log.pos(:,k) = x(1:3);
    log.phase(k) = 1 + gliding + relit;
end

idx_glide = log.phase == 2;
idx_relit = log.phase == 3;
fprintf('\nSuzulme baslangici: t=%.2f s\n', t_glide_start);
fprintf('Suzulme SIRASINDA max |pitch|: %.2f deg, max |roll|: %.2f deg\n', ...
    max(abs(rad2deg(log.att(2,idx_glide)))), max(abs(rad2deg(log.att(1,idx_glide)))));
if any(idx_relit)
    fprintf('Yeniden calistirma SONRASI max |pitch|: %.2f deg, max |roll|: %.2f deg\n', ...
        max(abs(rad2deg(log.att(2,idx_relit)))), max(abs(rad2deg(log.att(1,idx_relit)))));
    idx_roll_cmd = idx_relit & (log.t >= t_relit_start+ROLL_CMD_DELAY+ROLL_CMD_DUR-0.2) & (log.t < t_relit_start+ROLL_CMD_DELAY+ROLL_CMD_DUR);
    if any(idx_roll_cmd)
        fprintf('Roll komutu (%.0f deg) sonundaki roll: %.2f deg\n', ROLL_CMD_DEG, rad2deg(log.att(1,find(idx_roll_cmd,1,'last'))));
    end
    idx_cruise_settled = idx_relit & (log.t < t_relit_start+ROLL_CMD_DELAY);
    fprintf('Duz cruise (roll komutundan once) irtifa sapma araligi: [%.2f, %.2f] m\n', ...
        min(-log.pos(3,idx_cruise_settled)-80), max(-log.pos(3,idx_cruise_settled)-80));
end
fprintf('Genel max |omega| = %.4f rad/s\n', max(vecnorm(log.omega)));
fprintf('Son durum: ileri hiz=%.2f m/s, irtifa sapmasi=%.2f m\n', log.vel_body(1,end), -log.pos(3,end)-80);

fig = figure('Position',[100 100 1100 900]);
subplot(3,2,1); hold on; grid on;
plot(log.t, rad2deg(log.att(1,:)), 'DisplayName','\phi'); plot(log.t, rad2deg(log.att(2,:)), 'DisplayName','\theta');
xline(t_glide_start,'r--'); ylabel('deg'); legend('Location','best'); title('Attitude');

subplot(3,2,2); hold on; grid on;
plot(log.t, log.T(1,:)); plot(log.t, log.T(2,:)); plot(log.t, log.T(3,:));
xline(t_glide_start,'r--'); ylabel('N'); title('Rotor itkileri');

subplot(3,2,3); hold on; grid on;
plot(log.t, rad2deg(log.delta(1,:))); plot(log.t, rad2deg(log.delta(2,:))); plot(log.t, rad2deg(log.delta(3,:)));
xline(t_glide_start,'r--'); ylabel('deg'); title('Tilt acilari');

subplot(3,2,4); hold on; grid on;
plot(log.t, rad2deg(log.surf(1,:)), 'DisplayName','elevon'); plot(log.t, rad2deg(log.surf(3,:)), 'DisplayName','elevator');
plot(log.t, rad2deg(log.surf(5,:)), 'DisplayName','rudder');
xline(t_glide_start,'r--'); ylabel('deg'); legend('Location','best'); title('Yuzey sapmalari');

subplot(3,2,5); hold on; grid on;
plot(log.t, log.vel_body(1,:)); xline(t_glide_start,'r--'); ylabel('m/s'); xlabel('t (s)'); title('Ileri hiz');

subplot(3,2,6); hold on; grid on;
plot(log.t, -log.pos(3,:)-80); xline(t_glide_start,'r--'); ylabel('irtifa sapmasi (m)'); xlabel('t (s)'); title('Irtifa');

saveas(fig, fullfile(fileparts(mfilename('fullpath')), 'full_transition_glide_FWLAW_test.png'));
fprintf('\nGrafik kaydedildi: full_transition_glide_FWLAW_test.png\n');
