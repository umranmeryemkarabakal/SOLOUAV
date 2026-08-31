%% MATLAB_PX4_OFFBOARD
% MATLAB (ROS Toolbox) uzerinden PX4 SITL'e (gz_tiltrotor_tailplane, Gazebo)
% MAVROS koprusu ile offboard attitude+thrust setpoint gonderir.
%
% ONEMLI KAPSAM NOTU: Bu, tiltrotor_indi_wls projesindeki WLS/tilt-allocation
% mantigini TEST ETMEZ — PX4'un kendi control_allocator'i (CA_ROTORi_TILT
% parametreleriyle, airframe dosyasinda tanimli) rotor+tilt mixing'i kendi
% yapar. Buradan gonderilen sadece bir attitude+thrust HEDEFİ; PX4'un
% mc_att_control + mc_rate_control + control_allocator zinciri bunu
% gerceklestirir. Yani bu script CUSTOM WLS allocation'i degil, ustteki
% guidance/attitude-hedef mantigini (gain-scheduled P + istege bagli LESO
% bozucu telafisi) PX4'un SITL fizigine karsi dener.
%
% ON KOSULLAR (ayri terminallerde):
%   1) cd ~/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_tiltrotor_tailplane
%   2) source /opt/ros/humble/setup.bash
%      ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
%
% Kullanim: bu dosyayi MATLAB'da calistir (Ctrl+C ile durdurulana kadar
% surekli setpoint gonderir — PX4 OFFBOARD modda >2Hz setpoint bekler,
% yoksa moddan cikar).

clear; clc;

%% --- ROS2 baglantisi ---
setenv('ROS_DOMAIN_ID','0');
node = ros2node('/matlab_offboard_ctrl');
pause(1);   % discovery icin kisa bekleme

stateSub = ros2subscriber(node, '/mavros/state', 'mavros_msgs/State');
poseSub  = ros2subscriber(node, '/mavros/local_position/pose', 'geometry_msgs/PoseStamped');

attPub = ros2publisher(node, '/mavros/setpoint_raw/attitude', 'mavros_msgs/AttitudeTarget');

armClient    = ros2svcclient(node, '/mavros/cmd/arming', 'mavros_msgs/CommandBool');
modeClient   = ros2svcclient(node, '/mavros/set_mode',   'mavros_msgs/SetMode');

fprintf('MAVROS baglantisi bekleniyor...\n');
tStart = tic;
while toc(tStart) < 15
    st = stateSub.LatestMessage;
    if ~isempty(st) && st.connected
        fprintf('Baglandi. mode=%s armed=%d\n', st.mode, st.armed);
        break;
    end
    pause(0.2);
end
if isempty(stateSub.LatestMessage) || ~stateSub.LatestMessage.connected
    error('MAVROS/PX4 baglantisi kurulamadi — SITL ve mavros calisiyor mu kontrol edin.');
end

%% --- Hedef: sabit irtifada hover (attitude=level, thrust=hover-trim civari) ---
% mavros_msgs/AttitudeTarget.type_mask bitleri:
%   IGNORE_ROLL_RATE=1, IGNORE_PITCH_RATE=2, IGNORE_YAW_RATE=4, IGNORE_THRUST=64,
%   IGNORE_ATTITUDE=128. 7=1+2+4: sadece attitude+thrust kullan, PX4'un KENDI
%   rate loop'u (mc_rate_control) calisir — body_rate alani yok sayilir.
IGNORE_RATE_ALL = uint8(7);

% NOT (ONEMLI): MAVROS local_position topic'leri ENU kullanir (Z pozitif YUKARI).
% tiltrotor_indi_wls projesindeki (bu depo) tum kodlar NED (Z pozitif ASAGI)
% kullaniyordu — buradaki isaretler KASITLI olarak farkli, karistirmayin.

target_alt = 10.0;   % m, MIS_TAKEOFF_ALT ile uyumlu (airframe dosyasi notu)
hover_thrust = 0.5;  % [0..1] normalize itki — once dusuk baslayip QGC/log ile
                      % gercek hover thrust'i (PX4 log'undan) dogrulayin

rate = ros2rate(node, 20);   % 20 Hz setpoint (PX4 >2Hz ister, pay birakildi)
reset(rate);

fprintf('OFFBOARD icin setpoint akisi basliyor (mod gecisinden ONCE akmali)...\n');
for i = 1:40   % ~2s'lik on-akis, PX4 offboard'a gecmeden once setpoint gormeli
    msg = ros2message(attPub);
    msg.type_mask = IGNORE_RATE_ALL;
    msg.orientation.w = 1; msg.orientation.x = 0; msg.orientation.y = 0; msg.orientation.z = 0;
    msg.thrust = single(hover_thrust);
    send(attPub, msg);
    waitfor(rate);
end

%% --- OFFBOARD moda gec + arm ---
modeReq = ros2message(modeClient);
modeReq.custom_mode = 'OFFBOARD';
call(modeClient, modeReq, 5);

armReq = ros2message(armClient);
armReq.value = true;
call(armClient, armReq, 5);

fprintf('OFFBOARD + arm istendi. Surekli setpoint gonderiliyor (Ctrl+C ile durdur)...\n');

%% --- Ana dongu: basit irtifa P + sabit attitude=level ---
% Bu, altitude_loop.m'deki AYNI basit P+PI felsefesinin PX4 offboard
% uzerinden bir versiyonu — burada Fz yerine dogrudan normalize thrust
% komut ediliyor (PX4'un kendi itki/kutle donusumunu kullanir).
Kp_alt = 0.08;   % thrust/m, kaba baslangic kazanci — SITL'de once dusuk tutun
int_alt = 0;
Ki_alt = 0.01;

while true
    poseMsg = poseSub.LatestMessage;
    if ~isempty(poseMsg)
        z_err = target_alt - poseMsg.pose.position.z;
        int_alt = max(min(int_alt + z_err*(1/20), 5), -5);   % anti-windup
        thrust_cmd = hover_thrust + Kp_alt*z_err + Ki_alt*int_alt;
        thrust_cmd = max(min(thrust_cmd, 0.9), 0.1);
    else
        thrust_cmd = hover_thrust;
    end

    msg = ros2message(attPub);
    msg.type_mask = IGNORE_RATE_ALL;
    msg.orientation.w = 1; msg.orientation.x = 0; msg.orientation.y = 0; msg.orientation.z = 0;
    msg.thrust = single(thrust_cmd);
    send(attPub, msg);

    waitfor(rate);
end
