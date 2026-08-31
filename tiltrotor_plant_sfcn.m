function tiltrotor_plant_sfcn(block)
%TILTROTOR_PLANT_SFCN  Level-2 S-Function - 3 tilt-rotorlu VTOL 6-DOF plant.
%
% tiltrotor_plant_deriv.m'i (tam nonlineer model) dogrudan cagiran ince bir
% Simulink sarmalayicisi. Level-2 MATLAB S-Function normal (hizlandirilmamis)
% simulasyonda YORUMLANARAK calisir, bu yuzden struct/hucre gibi genel MATLAB
% yapilarini serbestce kullanabilir (MATLAB Function bloklarinin aksine).
%
% Baglantilar:
%   Giris  u_cmd(1:6)      : [T_cmd(3); delta_cmd(3)]  (WLS allocator cikisi)
%   Giris  wind_ned(1:3)   : ruzgar hizi (NED, m/s)
%   Giris  ext_moment(1:3) : ek govde-eksen dis moment (Nm)
%   Cikis  y(1:19)         : x — bkz. tiltrotor_plant_deriv.m basligi
%   Cikis  y(20:38)        : xdot — anlik durum turevi; INDI'nin "olculen"
%                            acisal ivme geri beslemesi (omega_dot) buradan
%                            (30:32) secilir. Gercek ucuste bu bir IMU
%                            turevidir, burada modelden dogrudan okunuyor
%                            (basitlestirme). TEK cikis portunda birlesik
%                            tutuluyor — S-Function'in derleme sirasinda
%                            cok-portlu boyut cozumlemesi Simulink build
%                            script'inde kararsizliga yol actigi icin.
%
% S-Function parametreleri: p (struct, tiltrotor_params.m), x0 (19x1 baslangic)

setup(block);

%% ================================================================
function setup(block)

block.NumDialogPrms = 2;   % p, x0

block.NumInputPorts  = 3;
block.NumOutputPorts = 1;

block.InputPort(1).Dimensions = 6;
block.InputPort(1).DirectFeedthrough = true;
block.InputPort(2).Dimensions = 3;
block.InputPort(2).DirectFeedthrough = true;
block.InputPort(3).Dimensions = 3;
block.InputPort(3).DirectFeedthrough = true;

block.OutputPort(1).Dimensions = 38;

block.NumContStates = 19;
block.SampleTimes = [0 0];

block.RegBlockMethod('InitializeConditions', @InitCond);
block.RegBlockMethod('Outputs',              @Outputs);
block.RegBlockMethod('Derivatives',          @Derivatives);
block.RegBlockMethod('Terminate',            @Terminate);

%% ================================================================
function InitCond(block)
x0 = block.DialogPrm(2).Data;
block.ContStates.Data = x0;

%% ================================================================
function Outputs(block)
p = block.DialogPrm(1).Data;
x = block.ContStates.Data;
u_cmd      = block.InputPort(1).Data;
wind_ned   = block.InputPort(2).Data;
ext_moment = block.InputPort(3).Data;

xdot = tiltrotor_plant_deriv(x, u_cmd(1:3), u_cmd(4:6), p, wind_ned, ext_moment);

block.OutputPort(1).Data = [x; xdot];

%% ================================================================
function Derivatives(block)
p = block.DialogPrm(1).Data;
x = block.ContStates.Data;
u_cmd      = block.InputPort(1).Data;
wind_ned   = block.InputPort(2).Data;
ext_moment = block.InputPort(3).Data;

block.Derivatives.Data = tiltrotor_plant_deriv(x, u_cmd(1:3), u_cmd(4:6), p, wind_ned, ext_moment);

%% ================================================================
function Terminate(block) %#ok<INUSD>
