function [att_sp_xy, fx_trim, state_out] = position_loop(pos_sp, pos, vel_ned, psi, delta_bar, state_in, p)
%POSITION_LOOP  Yatay pozisyon dis dongusu: P (pozisyon) -> PI (hiz) -> attitude sp.
%
% Madde (N)'nin cozumu (2026-07-29, Adim 28). Bu kontrolcude yatay pozisyon
% dongusu YOKTU; sonucu rapor §4 (N)'de niceliksel: hover'da yaw trim'i
% kacinilmaz olarak ~3 N ileri itki doguruyor ve arac 25 s'de 235 m
% surukleniyordu. Her "hover" dogrulamasi aslinda ~10 m/s seyir testiydi
% (Adim 16) -- yani yaw ekseninin EN KOTU kosulu olan gercek durus hicbir
% zaman test edilmemisti.
%
% NEDEN Fx_sp DEGIL DE ATTITUDE URETIYOR:
% Tiltrotor'un "dogal" yatay kanali govde-x kuvveti (Fx) uretmek icin rotorlari
% egmektir, ve kontrolcu zaten F_sp = [Fx_sp; Fz_sp] aliyor. AMA tilt araligi
% TEK YONLU ([0, pi/2], p.tilt.min = 0), yani arac yapisal olarak Fx >= 0
% uretebilir -- FRENLEYEMEZ. Bu yuzden yatay dongu klasik multikopter yolunu
% kullanir: govdeyi egerek (pitch/roll) itki vektorunu yonlendirir. Bu her iki
% yonde de calisir ve donanima dogrudan aktarilabilir. Adim 15'in elle
% "pitch_sp = +0.061 rad" gecici cozumu tam olarak bunun manuel halidir;
% bu fonksiyon onu kapali cevrime alir.
%
% Girisler:
%   pos_sp   [x_sp; y_sp]   (m, NED)  istenen yatay konum
%   pos      [x; y]         (m, NED)  olculen konum
%   vel_ned  [vx; vy]       (m/s,NED) olculen yatay hiz
%   psi      (rad)                    olculen yaw (NED ivmeyi govde-heading
%                                     cercevesine dondurmek icin)
%   delta_bar (rad)                   ortalama tilt (fx_trim'in hover->cruise
%                                     sonumlemesi icin)
%   state_in [int_vx; int_vy] (m/s^2) hiz PI integrali
%   p        tiltrotor_params()
%
% Ciktilar:
%   att_sp_xy [phi_sp; theta_sp] (rad)  attitude referansinin roll/pitch parcasi
%   fx_trim   (N)                       F_sp(1) olarak verilecek Fx trim'i
%   state_out [int_vx; int_vy]
%
% NEDEN fx_trim BURADA URETILIYOR (madde (P), Adim 28):
% Tek yonlu tilt araligi yuzunden hover'da Fx >= 0'dir ve yaw trim'i ~2.9 N
% kalici ileri kuvvet uretir. Tahsisattan Fx = 0 istemek, aracin YAPISAL OLARAK
% iptal edemeyecegi bir kuvveti iptal etmesini istemektir; olculdu ki bu
% durumda kisitsiz cozum ucu tilt icin de negatif ve delta1/delta2 TILT_MIN
% tabanina kalici cakiliyor -- madde (P)'nin yon asimetrisi tam olarak budur.
% Trim'i talebe eklemek asimetriyi 1.35x -> 1.05x'e indiriyor.
% AMA bedeli kalici bir +Fx'tir ve onu ancak BU dongu tasiyabilir. Bu yuzden
% trim inner controller'a degil buraya konuldu: boylece (P)'nin cozumu
% yapisal olarak (N)'in cozumune bagli. (Once inner controller'a konulmustu;
% pozisyon dongusu kullanmayan hover_gust regresyonunu q RMS 0.0004 -> 0.0013
% ile bozdu.)
%
% ISARET KURALI (dikkat -- bir kez yanlis yapildi):
% +theta = burun YUKARI = GERI kuvvet. Yani ileri ivme (ax_b > 0) NEGATIF
% pitch ister: theta_sp = -atan2(ax_b, g). Adim 15'in gozlemi bunu dogruluyor:
% +0.061 rad (burun yukari) ileri suruklenmeyi 9.4 -> 1.7 m/s'ye DUSURUYORDU.
% +phi = sag kanat asagi = +y (saga) kuvvet: phi_sp = +atan2(ay_b, g).
%
% Ts_pos ile (50 Hz) cagrilmasi beklenir; altitude_loop.m ile ayni decimasyon.
%
% !! BU DONGU YALNIZCA HOVER ICINDIR (2026-07-29, Adim 29'da SITL'de aracin
% dusmesiyle ogrenildi). Asagidaki theta_sp = -atan2(ax_b, g) bagintisi klasik
% multikopter varsayimidir: "yatay kuvvetin tek yolu itki vektorunu egmektir".
% BU ARACIN KANADI VAR. ~5-6 m/s ustunde kanat baskin hale gelir ve burun
% yukari komutu bir FRENLEME degil, bir ENERJI/TIRMANIS komutuna doner.
% 14.5 m/s'de devreye alindiginda olculen: pitch orneklerin %94'unde
% POS_TILT_MAX'te doydu, hiz 14.5 -> 7.6 dustukten sonra 9.6 m/s'de takildi ve
% arac 35 s boyunca ~1.1 m/s tirmandi (z -9.9 -> -54.0 m). Irtifa dongusu kanat
% tasimasini yenemez -- rotor itkisini en fazla sifira indirebilir.
% Dogrulanmis zarf: DURUSTAN devreye alma, <= 3 m/s. PX4 tarafinda bu bir
% kapiyla zorlanir (POS_ENGAGE_V_MAX); MATLAB testleri zaten hover'dan
% basladigi icin bu sinira hic dayanmadi.

v_max     = 3.0;        % m/s, yatay hiz limiti
Kp_p      = 0.80;       % 1/s          konum -> hiz
Kp_v      = 2.00;       % (m/s^2)/(m/s)
Ki_v      = 0.40;       % (m/s^2)/(m/s*s)
int_max   = 2.0;        % m/s^2, anti-windup clamp
a_max     = 3.0;        % m/s^2, yatay ivme limiti
tilt_max  = deg2rad(15);% rad, govde egim limiti (att_sp saturasyonu)

int_v = state_in(:);

% --- P: konum hatasi -> hiz setpoint'i (NED) ---
err_p = pos_sp(:) - pos(:);
v_sp  = Kp_p * err_p;
nv    = norm(v_sp);
if nv > v_max
    v_sp = v_sp * (v_max / nv);     % yonu koru, buyuklugu kirp
end

% --- PI: hiz hatasi -> ivme setpoint'i (NED) ---
err_v = v_sp - vel_ned(:);
int_v = int_v + err_v * p.Ts_pos;
int_v = max(min(int_v, int_max), -int_max);

a_ned = Kp_v * err_v + Ki_v * int_v;
na    = norm(a_ned);
if na > a_max
    a_ned = a_ned * (a_max / na);
end

% --- NED ivme -> govde-heading cercevesi ---
c = cos(psi); s = sin(psi);
ax_b =  a_ned(1)*c + a_ned(2)*s;
ay_b = -a_ned(1)*s + a_ned(2)*c;

% --- ivme -> attitude setpoint ---
theta_sp = -atan2(ax_b, p.g);
phi_sp   =  atan2(ay_b, p.g);

theta_sp = max(min(theta_sp, tilt_max), -tilt_max);
phi_sp   = max(min(phi_sp,   tilt_max), -tilt_max);

att_sp_xy = [phi_sp; theta_sp];

% Fx trim, gecis boyunca sonduruluyor (gain_schedule.m ile ayni smoothstep):
% cruise'da tilt zaten buyuk ve Fx kasitli olarak yuksek, orada trim anlamsiz.
s_sched  = max(0, min(1, delta_bar/p.tilt.max));
w_sched  = 3*s_sched.^2 - 2*s_sched.^3;
fx_trim  = p.ctrl.fx_trim * (1 - w_sched);

state_out = int_v;

end
