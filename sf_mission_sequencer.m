function [req, z_sp, yaw_sp, state_out] = sf_mission_sequencer(enable, agl, v_h, ...
        ft, bt, fw, land, pos_x, pos_y, yaw_now, z_now, z_datum, state_in, dt)
%SF_MISSION_SEQUENCER  MATLAB Function blok icerigi: tam gorev dizicisi.
%#codegen
%
% TiltrotorIndiControl.hpp missionSequencer()'in codegen-guvenli portu
% (Adim 160). Tek bir `enable` ile butun gorev yurur:
%   CLIMB -> HOVER -> FWD -> CRUISE -> FW -> FW_CRUISE -> BACK -> RETURN
%         -> SETTLE -> LAND -> DONE
%
% NEDEN CODEGEN'DE DE GEREKLI: HITL uretilen kodu kosar. Adim 154'te dizici
% yalnizca PX4 C++'a eklenmisti; o haliyle HITL'de otonom gorev olmazdi --
% bayraklari yine disaridan biri kaldirmak zorunda kalirdi (madde B0).
%
% Dizici mevcut durum makinelerinin USTUNDE durur; hicbirinin isini yapmaz,
% yalnizca "hangi bayrak ne zaman" der. Gecisler OLAY tabanlidir
% (ft==CRUISE, fw==ACTIVE, bt==HANDOFF, land==TOUCHDOWN); yalnizca seyir
% sureleri ve hover oturmasi zamanlidir.
%
% Cikislar:
%   req    (5x1 bool-benzeri) [pos_hold; ft; bt; fw; land]
%   z_sp   (m,NED) irtifa hedefi
%   yaw_sp (rad)   yon hedefi
%   state_out (5x1) [durum; zamanlayici; ev_x; ev_y; yaw_tut]
%
% DURUMLAR: 0=IDLE 1=CLIMB 2=HOVER 3=FWD 4=CRUISE 5=FW 6=FW_CRUISE
%           7=BACK 8=RETURN 9=SETTLE 10=LAND 11=DONE

% --- SABITLER: TiltrotorIndiParams.hpp MSN_* ile SENKRON KALMALI ---
climb_alt   = 40.0;    % m,  MSN_CLIMB_ALT
climb_tol   = 2.0;     % m,  MSN_CLIMB_TOL
settle_s    = 12.0;    % s,  MSN_SETTLE_S
cruise_s    = 8.0;     % s,  MSN_CRUISE_S
fw_cruise_s = 10.0;    % s,  MSN_FW_CRUISE_S
land_vh     = 1.0;     % m/s,MSN_LAND_VH
timeout_s   = 60.0;    % s,  MSN_PHASE_TIMEOUT_S
home_r      = 8.0;     % m,  MSN_HOME_R
ret_timeout = 400.0;   % s,  MSN_RETURN_TIMEOUT_S
fw_phase    = true;    %     MSN_FW_PHASE

st     = state_in(1);
timer  = state_in(2);
home_x = state_in(3);
home_y = state_in(4);
yawh   = state_in(5);

if ~enable
    req = [0;0;0;0;0];
    z_sp = z_now;
    yaw_sp = yaw_now;
    state_out = [0; 0; home_x; home_y; yaw_now];
    return;
end

timer = timer + dt;

% VARSAYILANLAR. pos_hold KAPALI, ve bu bilerek: gecis evrelerinde acik
% birakmak pozisyon dongusunu ileri gecisle guresTIRIR -- olculdu, ft_state
% CRUISE'a hic ulasmadi ve arac 39 m'de 7-15 m/s ile ucup gitti.
req = [0;0;0;0;0];

% IRTIFA HEDEFI: ASLA ALCALMA KOMUT ETME (Adim 159). NED'de min DAHA YUKSEK
% irtifayi secer. Sabit hedef kullanildiginda arac sabit kanatta 47.8 m'ye
% tirmaniyor ve geri gecise 7.8 m'lik alcalma talebiyle giriyordu: olculen
% doyum %44.2 (kuyruk), BIG_M 2519. Kural betikten geliyor: min(z_sp, z_now).
z_sp = min(z_datum - climb_alt, z_now);
yaw_sp = yawh;

timed_out = timer > timeout_s;

if st == 0                                   % IDLE
    home_x = pos_x;  home_y = pos_y;  yawh = yaw_now;  yaw_sp = yaw_now;
    st = 1;  timer = 0;

elseif st == 1                               % CLIMB
    req(1) = 1;
    if abs(agl - climb_alt) < climb_tol
        st = 2;  timer = 0;
    end

elseif st == 2                               % HOVER
    req(1) = 1;
    if timer >= settle_s
        st = 3;  timer = 0;
    end

elseif st == 3                               % FWD
    req(2) = 1;
    if ft >= 1.5                              % FtState::CRUISE
        st = 4;  timer = 0;
    elseif timed_out
        st = 7;  timer = 0;
    end

elseif st == 4                               % CRUISE
    req(2) = 1;
    if timer >= cruise_s
        if fw_phase; st = 5; else; st = 7; end
        timer = 0;
    end

elseif st == 5                               % FW
    % ft BAYRAGI KALKIK KALMALI: sabit kanat giris kapisi ft==CRUISE istiyor.
    req(2) = 1;  req(4) = 1;
    if fw >= 1.5                              % FwState::ACTIVE
        st = 6;  timer = 0;
    elseif timed_out
        st = 7;  timer = 0;
    end

elseif st == 6                               % FW_CRUISE
    req(2) = 1;  req(4) = 1;
    if timer >= fw_cruise_s
        st = 7;  timer = 0;
    end

elseif st == 7                               % BACK
    req(3) = 1;
    if (bt >= 2.5) || timed_out               % BtState::HANDOFF
        st = 8;  timer = 0;
    end

elseif st == 8                               % RETURN
    % EVE DON. Olculdu: donus olmadan arac 683 m uzaga gidip ORAYA iniyordu.
    req(1) = 1;
    dx = pos_x - home_x;  dy = pos_y - home_y;
    dist = sqrt(dx*dx + dy*dy);
    % BURNU EVE CEVIR: pozisyon dongusu araci hedefe goturur ama burnunu
    % CEVIRMEZ -- olculdu, arac 683 m'yi GERI GERI geldi (ileri hiz
    % -2.93 m/s, burun evden 163 deg sapik). Yakinda kerteriz gurultulendigi
    % icin 2*home_r altinda yon komutu DONDURULUR.
    if dist > 2*home_r
        yaw_sp = atan2(home_y - pos_y, home_x - pos_x);
    else
        yaw_sp = yaw_now;
    end
    if (dist < home_r) || (timer > ret_timeout)
        yawh = yaw_now;                       % YAW'I DONDUR (Adim 156)
        st = 9;  timer = 0;
    end

elseif st == 9                               % SETTLE
    req(1) = 1;  yaw_sp = yawh;
    if (v_h < land_vh) || timed_out
        st = 10;  timer = 0;
    end

elseif st == 10                              % LAND
    req(1) = 1;  req(5) = 1;  yaw_sp = yawh;  z_sp = z_now;
    if land >= 2.5                            % LandState::TOUCHDOWN
        st = 11;  timer = 0;
    end

else                                         % DONE
    req(1) = 1;  req(5) = 1;  yaw_sp = yawh;  z_sp = z_now;
end

state_out = [st; timer; home_x; home_y; yawh];
end
