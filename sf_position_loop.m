function [att_sp_xy, fx_trim, state_out] = sf_position_loop(pos_sp, pos, vel_ned, ...
                                                            psi, delta_bar, state_in)
%SF_POSITION_LOOP  MATLAB Function blok icerigi: yatay pozisyon dis dongusu
%(P konum -> PI hiz -> attitude setpoint) + Fx trim.
%#codegen
%
% position_loop.m'in codegen-guvenli yeniden yazimi (Adim 127). AYNI matematik,
% ayni sabitler -- tek fark struct (`p`) yerine literal sabitler ve `state_in`
% ile disaridan gelen durum (MATLAB Function blogu `persistent` kullanamaz).
%
% Girisler:
%   pos_sp   (2x1 m, NED)    istenen yatay konum
%   pos      (2x1 m, NED)    olculen konum
%   vel_ned  (2x1 m/s, NED)  olculen yatay hiz
%   psi      (1x1 rad)       olculen yaw (NED ivmeyi govde-heading'e dondurmek icin)
%   delta_bar(1x1 rad)       ortalama tilt (fx_trim sonumlemesi icin)
%   state_in (2x1 m/s^2)     hiz PI integrali [int_vx; int_vy]
% Ciktilar:
%   att_sp_xy (2x1 rad)      [phi_sp; theta_sp]
%   fx_trim   (1x1 N)        F_sp(1) olarak tahsisata verilecek trim
%   state_out (2x1)          guncellenmis integral
%
% !! BU DONGU YALNIZCA HOVER ICINDIR (Adim 29'da SITL'de aracin dusmesiyle
% ogrenildi). Asagidaki theta_sp = -atan2(ax_b, g) klasik multikopter
% varsayimidir: "yatay kuvvetin tek yolu itki vektorunu egmektir". BU ARACIN
% KANADI VAR. ~5-6 m/s ustunde kanat baskin olur ve burun yukari komutu bir
% FRENLEME degil ENERJI/TIRMANIS komutuna doner. 14.5 m/s'de devreye
% alindiginda olculen: pitch orneklerin %94'unde doydu, hiz 9.6 m/s'de takildi
% ve arac 35 s boyunca ~1.1 m/s tirmandi (z -9.9 -> -54.0 m).
% Dogrulanmis zarf: DURUSTAN devreye alma, <= 3 m/s. PX4 bunu bir kapiyla
% zorlar (POS_ENGAGE_V_MAX); bu blokta kapi YOKTUR, cunku Simulink modeli
% hover senaryosu kosar. Model bir gecis senaryosuna genisletilirse kapi
% EKLENMELIDIR.

% --- SABITLER: position_loop.m ve TiltrotorIndiParams.hpp ile SENKRON KALMALI ---
v_max     = 3.0;            % m/s
Kp_p      = 0.80;           % 1/s
Kp_v      = 2.00;           % (m/s^2)/(m/s)
Ki_v      = 0.40;           % (m/s^2)/(m/s*s)
int_max   = 2.0;            % m/s^2
a_max     = 3.0;            % m/s^2
tilt_max  = 15.0*pi/180;    % rad, govde egim limiti
Ts_pos    = 1/50;           % s,  p.Ts_pos (irtifa dongusuyle ayni decimasyon)
g         = 9.81;           % m/s^2
FX_TRIM   = 2.9;            % N,  p.ctrl.fx_trim / PX4 FX_TRIM
TILT_MAX  = pi/2;           % rad, p.tilt.max (fx_trim sonumlemesinin olcegi)

int_v = state_in(:);

% --- P: konum hatasi -> hiz setpoint'i (NED) ---
err_p = pos_sp(:) - pos(:);
v_sp  = Kp_p * err_p;
nv    = sqrt(v_sp(1)^2 + v_sp(2)^2);    % norm() yerine acik: codegen-guvenli
if nv > v_max
    v_sp = v_sp * (v_max / nv);         % yonu koru, buyuklugu kirp
end

% --- PI: hiz hatasi -> ivme setpoint'i (NED) ---
err_v = v_sp - vel_ned(:);
int_v = int_v + err_v * Ts_pos;
int_v = max(min(int_v, int_max), -int_max);

a_ned = Kp_v * err_v + Ki_v * int_v;
na    = sqrt(a_ned(1)^2 + a_ned(2)^2);
if na > a_max
    a_ned = a_ned * (a_max / na);
end

% --- NED ivme -> govde-heading cercevesi ---
c = cos(psi); s = sin(psi);
ax_b =  a_ned(1)*c + a_ned(2)*s;
ay_b = -a_ned(1)*s + a_ned(2)*c;

% --- ivme -> attitude setpoint ---
theta_sp = -atan2(ax_b, g);
phi_sp   =  atan2(ay_b, g);

theta_sp = max(min(theta_sp, tilt_max), -tilt_max);
phi_sp   = max(min(phi_sp,   tilt_max), -tilt_max);

att_sp_xy = [phi_sp; theta_sp];

% --- Fx TRIM (madde (P), Adim 28) ---
% NEDEN BURADA, tahsisatta DEGIL: tek yonlu tilt araligi ([0, pi/2]) yuzunden
% hover'da Fx >= 0'dir ve yaw trim'i ~2.9 N kalici ileri kuvvet uretir.
% Tahsisattan Fx = 0 istemek, aracin YAPISAL OLARAK iptal edemeyecegi bir
% kuvveti iptal etmesini istemektir.
% Trim dogrudan tahsisata konulursa NE OLUR, olculdu: hover_gust regresyonunda
% q RMS 0.0004 -> 0.0013 (4.3x) kotulesti, cunku o test pozisyon dongusu
% KULLANMIYOR ve olusan ileri kuvveti tasiyacak hicbir sey yok. Bagimlilik
% bu yuzden YAPIYA gomulu: (P) yalnizca (N) aktifken uygulanir. sf_wls_alloc.m
% bu yuzden fx_trim'i KENDI hesaplamaz -- F_sp(1) ile buradan alir.
%
% Gecis boyunca sondurulur (gain_schedule.m ile ayni smoothstep): cruise'da
% tilt zaten buyuk ve Fx kasitli olarak yuksek, orada trim anlamsiz.
s_sched  = max(0, min(1, delta_bar/TILT_MAX));
w_sched  = 3*s_sched^2 - 2*s_sched^3;
fx_trim  = FX_TRIM * (1 - w_sched);

state_out = int_v;

end
