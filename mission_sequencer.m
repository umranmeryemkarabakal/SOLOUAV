function [req, z_sp, yaw_sp, state_out] = mission_sequencer(enable, agl, v_h, ...
        ft, bt, fw, land, pos_x, pos_y, yaw_now, z_now, z_datum, state_in, p)
%MISSION_SEQUENCER  Tam gorev dizicisi (MATLAB REFERANSI).
%
% Uc uygulamanin referans ayagi. Digerleri:
%   codegen : sf_mission_sequencer.m
%   PX4 C++ : TiltrotorIndiControl.hpp missionSequencer()
%
% Tek `enable` ile butun gorev yurur:
%   CLIMB -> HOVER -> FWD -> CRUISE -> FW -> FW_CRUISE -> BACK -> RETURN
%         -> SETTLE -> LAND -> DONE
% Gecisler OLAY tabanli; yalnizca seyir sureleri ve hover oturmasi zamanli.
% Gerekcelerin tamami sf_mission_sequencer.m ve TiltrotorIndiParams.hpp'de.

c = p.ctrl;
st=state_in(1); timer=state_in(2); hx=state_in(3); hy=state_in(4); yawh=state_in(5);

if ~enable
    req=[0;0;0;0;0]; z_sp=z_now; yaw_sp=yaw_now;
    state_out=[0;0;hx;hy;yaw_now]; return;
end

timer = timer + p.Ts_ctrl;
req = [0;0;0;0;0];                      % pos_hold gecislerde KAPALI (olculdu)
z_sp = min(z_datum - c.msn_climb_alt, z_now);   % ASLA ALCALMA (Adim 159)
yaw_sp = yawh;
timed_out = timer > c.msn_timeout_s;

switch st
    case 0
        hx=pos_x; hy=pos_y; yawh=yaw_now; yaw_sp=yaw_now; st=1; timer=0;
    case 1
        req(1)=1;
        if abs(agl - c.msn_climb_alt) < c.msn_climb_tol; st=2; timer=0; end
    case 2
        req(1)=1;
        if timer >= c.msn_settle_s; st=3; timer=0; end
    case 3
        req(2)=1;
        if ft >= 1.5; st=4; timer=0; elseif timed_out; st=7; timer=0; end
    case 4
        req(2)=1;
        if timer >= c.msn_cruise_s
            if c.msn_fw_phase; st=5; else; st=7; end
            timer=0;
        end
    case 5
        req(2)=1; req(4)=1;               % ft kalkik KALMALI (giris kapisi)
        if fw >= 1.5; st=6; timer=0; elseif timed_out; st=7; timer=0; end
    case 6
        req(2)=1; req(4)=1;
        if timer >= c.msn_fw_cruise_s; st=7; timer=0; end
    case 7
        req(3)=1;
        if (bt >= 2.5) || timed_out; st=8; timer=0; end
    case 8
        req(1)=1;
        dx=pos_x-hx; dy=pos_y-hy; dist=sqrt(dx*dx+dy*dy);
        if dist > 2*c.msn_home_r
            yaw_sp = atan2(hy - pos_y, hx - pos_x);   % BURNU EVE CEVIR
        else
            yaw_sp = yaw_now;
        end
        if (dist < c.msn_home_r) || (timer > c.msn_return_timeout_s)
            yawh = yaw_now; st=9; timer=0;            % YAW'I DONDUR
        end
    case 9
        req(1)=1; yaw_sp=yawh;
        if (v_h < c.msn_land_vh) || timed_out; st=10; timer=0; end
    case 10
        req(1)=1; req(5)=1; yaw_sp=yawh; z_sp=z_now;
        if land >= 2.5; st=11; timer=0; end
    otherwise
        req(1)=1; req(5)=1; yaw_sp=yawh; z_sp=z_now;
end

state_out = [st; timer; hx; hy; yawh];
end
