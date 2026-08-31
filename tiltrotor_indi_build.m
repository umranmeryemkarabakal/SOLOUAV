function tiltrotor_indi_build()
%TILTROTOR_INDI_BUILD  3 tilt-rotorlu VTOL icin INDI + WLS + gain-scheduling +
%LESO kontrolcusunu ve plant'ini iceren tiltrotor_indi.slx modelini kurar.
%
% f450_indi_build.m / f450_hybrid_build.m ile ayni insa deseni: bloklari
% programatik ekle, MATLAB Function bloklarinin icerigini ayri .m
% dosyalarindan yukle, hatlari bagla, kaydet.
%
% Kullanim:
%   1. Bu dosyayi tum .m dosyalariyla ayni klasore koy
%   2. O klasoru MATLAB Current Folder yap
%   3. tiltrotor_indi_build komutunu calistir
%   4. sim('tiltrotor_indi') ile calistir (ya da Simulink'te Run'a bas)

mdl = 'tiltrotor_indi';
if bdIsLoaded(mdl); close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'); delete([mdl '.slx']); end

% S-Function parametreleri ('p','x0_tilt') Simulink tarafindan derleme
% sirasinda BASE workspace'te aranir — InitFcn yalnizca sonraki sim()
% cagrilarinda calisir, bu yuzden insa sirasinda da ayni degiskenleri
% base workspace'e koyuyoruz.
p = tiltrotor_params(); %#ok<NASGU>
x0_tilt = zeros(19,1);
x0_tilt(3) = -50;
x0_tilt(7:10) = quat_from_euler(0,0,0);
x0_tilt(14:19) = hover_trim(p);
assignin('base', 'p', p);
assignin('base', 'x0_tilt', x0_tilt);

new_system(mdl);
open_system(mdl);

%% ================================================================
%  COZUCU AYARLARI — sabit adim, LESO/kontrol ic mantigi bunu varsayiyor
%% ================================================================
set_param(mdl, ...
    'SolverType',       'Fixed-step', ...
    'Solver',           'ode4', ...
    'FixedStep',        '0.0025', ...   % = Ts_ctrl (400 Hz)
    'StopTime',         '12', ...
    'SaveFormat',       'Dataset');

set_param(mdl, 'InitFcn', [ ...
    'p = tiltrotor_params(); ' ...
    'x0_tilt = zeros(19,1); x0_tilt(3) = -50; ' ...
    'x0_tilt(7:10) = quat_from_euler(0,0,0); ' ...
    'x0_tilt(14:19) = hover_trim(p);']);

%% ================================================================
%  BLOK POZISYONLARI
%% ================================================================
pos = struct();
pos.att_sp      = [40   40  100  70];
pos.Fx_sp       = [40  100  100 130];
pos.z_sp        = [40  220  100 250];
pos.leso_enable = [40  160  100 190];
pos.wind_ned    = [40  480  100 510];
pos.ext_moment  = [40  540  100 570];

pos.u_cmd_delay  = [820  330 860   360];
pos.leso_state_delay = [1360 60 1420 90];
pos.alt_state_delay  = [820 200 860 230];
pos.plant        = [900  300 1060  430];
pos.sel_quat     = [1100 250 1150  280];
pos.sel_omega    = [1100 300 1150  330];
pos.sel_uactual  = [1100 350 1150  380];
pos.sel_omegadot = [1100 400 1150  430];
pos.sel_z        = [700  230 750  260];
pos.sel_vz       = [700  270 750  300];
pos.altitude_loop = [780 230 900 280];
pos.F_sp_mux      = [1000 150 1020 200];

pos.quat2euler   = [1190 250 1300  280];
pos.indi_rate    = [1360 190 1540  350];
pos.wls_alloc    = [1620 260 1800  390];

% INIS UCLUSU icin eklenen bloklar (Adim 125)
pos.wls_state_delay = [1620 130 1680 160];
pos.sel_roll     = [1360 130 1410 160];
pos.sel_pdot     = [1360  90 1410 120];
pos.agl_gain     = [1470 170 1510 200];

% POZISYON DONGUSU icin eklenen bloklar (Adim 127)
pos.position_loop  = [780  60  920 140];
pos.pos_state_delay= [780  10  840  40];
pos.pos_sp         = [620  60  680  90];
pos.sel_pos        = [700  100 750 130];
pos.sel_vel        = [700  140 750 170];
pos.sel_psi        = [1330 250 1350 270];
pos.att_sp_mux     = [960   40 980  110];
pos.deltabar_mean  = [700   20 750   50];
pos.sel_tilt       = [620   20 670   50];

pos.scope_att    = [1360 400 1420 440];
pos.scope_omega  = [1360 450 1420 490];
pos.scope_dhat   = [1620 420 1680 460];
pos.scope_satflag= [1620 470 1680 510];
pos.scope_act    = [1190 440 1250 480];

%% ================================================================
%  KAYNAK BLOKLARI
%% ================================================================
% att_sp artik SABIT DEGIL (Adim 127): roll/pitch pozisyon dongusunden gelir,
% yaw referansi sabit kalir. Eskiden [0;0;0] sabitti ve yatay kanal ACIKTI --
% madde (N): hover'da yaw trim'i ~3 N ileri itki doguruyor ve arac 25 s'de
% 235 m surukleniyordu, yani her "hover" testi aslinda seyir testiydi.
add_block('simulink/Sources/Constant', [mdl '/yaw_sp'], ...
    'Value', '0', 'Position', pos.att_sp);
add_block('simulink/Signal Routing/Mux', [mdl '/att_sp_mux'], ...
    'Inputs', '2', 'Position', pos.att_sp_mux);
add_block('simulink/Sources/Constant', [mdl '/pos_sp'], ...
    'Value', '[0;0]', 'Position', pos.pos_sp);
% Fx_sp SABITI KALDIRILDI (Adim 127): artik position_loop'un fx_trim ciktisi.
% BILEREK boyle -- fx_trim dogrudan tahsisata konuldugunda hover_gust q RMS'i
% 0.0004 -> 0.0013 (4.3x) kotulesmisti (Adim 28), cunku olusan kalici ileri
% kuvveti tasiyacak bir yatay dongu yoktu. Bagimlilik yapiya gomulu:
% (P) yalnizca (N) aktifken uygulanir.
add_block('simulink/Sources/Constant', [mdl '/z_sp'], ...
    'Value', 'x0_tilt(3)', 'Position', pos.z_sp);
add_block('simulink/Sources/Constant', [mdl '/leso_enable'], ...
    'Value', '[1;1;0]', 'Position', pos.leso_enable);
add_block('simulink/Sources/Constant', [mdl '/wind_ned'], ...
    'Value', '[0;0;0]', 'Position', pos.wind_ned);
add_block('simulink/Sources/Constant', [mdl '/ext_moment'], ...
    'Value', '[0;0;0]', 'Position', pos.ext_moment);

%% ================================================================
%  GERI BESLEME GECIKMESI — Plant'in girisi (u_cmd) dogrudan-besleme
%  (DirectFeedthrough) ile ciktisina (xdot) baglanirken, ayni zamanda
%  WLS uzerinden Plant ciktisina geri baglanmasi cebirsel bir dongu
%  (algebraic loop) olusturuyor. Unit Delay, bunu bir kontrol adimlik
%  (Ts_ctrl) gerceki bir hesaplama gecikmesiyle kirar — hem derlemeyi
%  hem de gercek donanimdaki komut gecikmesini dogru temsil eder.
%% ================================================================
add_block('simulink/Discrete/Unit Delay', [mdl '/u_cmd_delay'], ...
    'X0', 'x0_tilt(14:19)', 'Position', pos.u_cmd_delay);

% indi_rate_law'in LESO/filtre durumu (13x1) — MATLAB Function blogu icinde
% `persistent` kullanmak surekli ornekleme zamaniyla celisir; bunun yerine
% durum disari cikarilip bir Unit Delay ile kendine geri baglaniyor.
add_block('simulink/Discrete/Unit Delay', [mdl '/leso_state_delay'], ...
    'X0', 'zeros(13,1)', 'Position', pos.leso_state_delay);

% Irtifa dis dongusunun (P+PI) integral durumu — ayni desen, tek skaler.
add_block('simulink/Discrete/Unit Delay', [mdl '/alt_state_delay'], ...
    'X0', '0', 'Position', pos.alt_state_delay);

% wls_alloc'un durumu (5x1: [temas_bekleme_s; mandal; prev_du_tilt(3)]) — ayni
% desen. prev_du_tilt tiltjerk icindir (Adim 126); temas alanlari Adim 125.
% Mandal bir tik gecikmeli degerlendirilir; referans (indi_attitude_controller.m)
% de ayni sirayi kullanir, yani bu gecikme bir yaklasiklik DEGIL, paritenin
% parcasidir.
add_block('simulink/Discrete/Unit Delay', [mdl '/wls_state_delay'], ...
    'X0', 'zeros(5,1)', 'Position', pos.wls_state_delay);

%% ================================================================
%  PLANT (S-Function)
%% ================================================================
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', [mdl '/Plant'], ...
    'FunctionName', 'tiltrotor_plant_sfcn', ...
    'Parameters',   'p, x0_tilt', ...
    'Position',     pos.plant);

%% ================================================================
%  SELECTOR'LAR (Plant/1 = x(19), Plant/2 = xdot(19))
%% ================================================================
% Plant TEK cikis portunda [x(19); xdot(19)] = 38 genislik veriyor.
add_block('simulink/Signal Routing/Selector', [mdl '/sel_quat'], ...
    'Indices', '7:10', 'InputPortWidth', '38', 'Position', pos.sel_quat);
add_block('simulink/Signal Routing/Selector', [mdl '/sel_omega'], ...
    'Indices', '11:13', 'InputPortWidth', '38', 'Position', pos.sel_omega);
add_block('simulink/Signal Routing/Selector', [mdl '/sel_uactual'], ...
    'Indices', '14:19', 'InputPortWidth', '38', 'Position', pos.sel_uactual);
add_block('simulink/Signal Routing/Selector', [mdl '/sel_omegadot'], ...
    'Indices', '30:32', 'InputPortWidth', '38', 'Position', pos.sel_omegadot);
add_block('simulink/Signal Routing/Selector', [mdl '/sel_z'], ...
    'Indices', '3', 'InputPortWidth', '38', 'Position', pos.sel_z);
add_block('simulink/Signal Routing/Selector', [mdl '/sel_vz'], ...
    'Indices', '22', 'InputPortWidth', '38', 'Position', pos.sel_vz);   % xdot(3) = 19+3

% INIS UCLUSU girisleri (Adim 125).
% pdot: xdot(11) = 19+11 = 30, yani roll acisal ivmesi. sel_omegadot zaten
% 30:32 aliyor; temas olcutu YALNIZCA roll eksenini kullanir (gerekce
% tiltrotor_params.m p.ctrl.land_contact_acc notunda).
add_block('simulink/Signal Routing/Selector', [mdl '/sel_pdot'], ...
    'Indices', '30', 'InputPortWidth', '38', 'Position', pos.sel_pdot);

% AGL: bu PLANT'te zemin z = 0 datumundadir ve kestirimci YOKTUR, dolayisiyla
% agl = -z TAM DOGRUDUR.
% ⚠ GERCEK DONANIMDA BOYLE DEGIL (Adim 116-117): orada -z kestirimcinin yerel
% orijininden yuksekliktir ve olculen datum ofseti -0.67 .. +1.77 m'ye cikar --
% kapinin 2.0 m'lik esigiyle AYNI MERTEBEDE. PX4 portu bu hatayi yapiyordu ve
% mekanizma rastgele armaniyordu; duzeltmesi captureGroundDatum(). Uretilen kod
% gercek bir araca konursa buraya kalkis datumuna gore duzeltilmis AGL
% baglanmalidir, HAM -z DEGIL.
add_block('simulink/Math Operations/Gain', [mdl '/agl_gain'], ...
    'Gain', '-1', 'Position', pos.agl_gain);

% roll: quat2euler ciktisinin (3x1) ilk elemani. Temas dalinin BIRINCI olcutu
% (LAND_CONTACT_ROLL, olculen TAKLA olayindan turedi); duz inisi mandal
% yakalar, bu esik degil.
add_block('simulink/Signal Routing/Selector', [mdl '/sel_roll'], ...
    'Indices', '1', 'InputPortWidth', '3', 'Position', pos.sel_roll);

%% ================================================================
%  MATLAB FUNCTION BLOKLARI
%% ================================================================
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/quat2euler'], 'Position', pos.quat2euler);
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/indi_rate_law'], 'Position', pos.indi_rate);
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/wls_alloc'], 'Position', pos.wls_alloc);
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/altitude_loop'], 'Position', pos.altitude_loop);

add_block('simulink/Signal Routing/Mux', [mdl '/F_sp_mux'], ...
    'Inputs', '2', 'Position', pos.F_sp_mux);

% POZISYON DONGUSU (Adim 127, madde (N)+(P))
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/position_loop'], 'Position', pos.position_loop);
add_block('simulink/Discrete/Unit Delay', [mdl '/pos_state_delay'], ...
    'X0', 'zeros(2,1)', 'Position', pos.pos_state_delay);
% x(1:2) = kuzey/dogu konum, xdot(1:2) = 19+1:19+2 = 20:21 = kuzey/dogu hiz
add_block('simulink/Signal Routing/Selector', [mdl '/sel_pos'], ...
    'Indices', '1:2', 'InputPortWidth', '38', 'Position', pos.sel_pos);
add_block('simulink/Signal Routing/Selector', [mdl '/sel_vel'], ...
    'Indices', '20:21', 'InputPortWidth', '38', 'Position', pos.sel_vel);
% psi: quat2euler ciktisinin (3x1) ucuncu elemani
add_block('simulink/Signal Routing/Selector', [mdl '/sel_psi'], ...
    'Indices', '3', 'InputPortWidth', '3', 'Position', pos.sel_psi);
% delta_bar: u_actual'in tilt uclusunun ortalamasi (fx_trim sonumlemesi icin)
add_block('simulink/Signal Routing/Selector', [mdl '/sel_tilt'], ...
    'Indices', '17:19', 'InputPortWidth', '38', 'Position', pos.sel_tilt);
add_block('simulink/Math Operations/Sum of Elements', [mdl '/tilt_sum'], ...
    'Position', pos.deltabar_mean);
add_block('simulink/Math Operations/Gain', [mdl '/deltabar'], ...
    'Gain', '1/3', 'Position', [pos.deltabar_mean(1)+60 pos.deltabar_mean(2) ...
                                pos.deltabar_mean(3)+60 pos.deltabar_mean(4)]);


%% ================================================================
%  SCOPE'LAR
%% ================================================================
add_block('simulink/Sinks/Scope', [mdl '/scope_att'],     'Position', pos.scope_att);
add_block('simulink/Sinks/Scope', [mdl '/scope_omega'],   'Position', pos.scope_omega);
add_block('simulink/Sinks/Scope', [mdl '/scope_dhat'],    'Position', pos.scope_dhat);
add_block('simulink/Sinks/Scope', [mdl '/scope_satflag'], 'Position', pos.scope_satflag);
add_block('simulink/Sinks/Scope', [mdl '/scope_act'],     'Position', pos.scope_act);
set_param([mdl '/scope_att'],   'NumInputPorts', '1');
set_param([mdl '/scope_omega'], 'NumInputPorts', '1');

%% ================================================================
%  MATLAB FUNCTION ICERIKLERINI YUKLE
%% ================================================================
set_mfcn(mdl, 'quat2euler',    'sf_quat_to_euler.m');
set_mfcn(mdl, 'indi_rate_law', 'sf_indi_rate_law.m');
set_mfcn(mdl, 'wls_alloc',     'sf_wls_alloc.m');
set_mfcn(mdl, 'altitude_loop', 'sf_altitude_loop.m');
set_mfcn(mdl, 'position_loop', 'sf_position_loop.m');

%% ================================================================
%  BAGLANTILAR
%% ================================================================

% Plant -> Selector'lar (tek 38-genislikli cikis portu, bkz. tiltrotor_plant_sfcn.m)
add_line(mdl, 'Plant/1', 'sel_quat/1',     'autorouting','smart');
add_line(mdl, 'Plant/1', 'sel_omega/1',    'autorouting','smart');
add_line(mdl, 'Plant/1', 'sel_uactual/1',  'autorouting','smart');
add_line(mdl, 'Plant/1', 'sel_omegadot/1', 'autorouting','smart');

% quat -> euler
add_line(mdl, 'sel_quat/1', 'quat2euler/1', 'autorouting','smart');

% indi_rate_law girisleri: att_sp(1) att(2) omega(3) omega_dot_raw(4)
% u_actual(5) leso_enable(6) state_in(7)
add_line(mdl, 'att_sp_mux/1',    'indi_rate_law/1', 'autorouting','smart');
add_line(mdl, 'quat2euler/1',    'indi_rate_law/2', 'autorouting','smart');
add_line(mdl, 'sel_omega/1',     'indi_rate_law/3', 'autorouting','smart');
add_line(mdl, 'sel_omegadot/1',  'indi_rate_law/4', 'autorouting','smart');
add_line(mdl, 'sel_uactual/1',   'indi_rate_law/5', 'autorouting','smart');
add_line(mdl, 'leso_enable/1',   'indi_rate_law/6', 'autorouting','smart');
add_line(mdl, 'leso_state_delay/1', 'indi_rate_law/7', 'autorouting','smart');

% indi_rate_law/3 (state_out) -> leso_state_delay -> indi_rate_law/7 (state_in)
add_line(mdl, 'indi_rate_law/3', 'leso_state_delay/1', 'autorouting','smart');

% Irtifa dis dongusu: z_sp(1) z(2) vz(3) state_in(4) -> Fz_sp(1) state_out(2)
add_line(mdl, 'Plant/1', 'sel_z/1',  'autorouting','smart');
add_line(mdl, 'Plant/1', 'sel_vz/1', 'autorouting','smart');
add_line(mdl, 'z_sp/1',            'altitude_loop/1', 'autorouting','smart');
add_line(mdl, 'sel_z/1',           'altitude_loop/2', 'autorouting','smart');
add_line(mdl, 'sel_vz/1',          'altitude_loop/3', 'autorouting','smart');
add_line(mdl, 'alt_state_delay/1', 'altitude_loop/4', 'autorouting','smart');
add_line(mdl, 'altitude_loop/2',   'alt_state_delay/1', 'autorouting','smart');

% F_sp = [fx_trim; Fz_sp] -- Fx artik pozisyon dongusunden (Adim 127)
add_line(mdl, 'position_loop/2', 'F_sp_mux/1', 'autorouting','smart');

% --- POZISYON DONGUSU baglantilari (Adim 127) ---
% girisler: pos_sp(1) pos(2) vel_ned(3) psi(4) delta_bar(5) state(6)
add_line(mdl, 'pos_sp/1',           'position_loop/1', 'autorouting','smart');
add_line(mdl, 'Plant/1',            'sel_pos/1',       'autorouting','smart');
add_line(mdl, 'sel_pos/1',          'position_loop/2', 'autorouting','smart');
add_line(mdl, 'Plant/1',            'sel_vel/1',       'autorouting','smart');
add_line(mdl, 'sel_vel/1',          'position_loop/3', 'autorouting','smart');
add_line(mdl, 'quat2euler/1',       'sel_psi/1',       'autorouting','smart');
add_line(mdl, 'sel_psi/1',          'position_loop/4', 'autorouting','smart');
add_line(mdl, 'Plant/1',            'sel_tilt/1',      'autorouting','smart');
add_line(mdl, 'sel_tilt/1',         'tilt_sum/1',      'autorouting','smart');
add_line(mdl, 'tilt_sum/1',         'deltabar/1',      'autorouting','smart');
add_line(mdl, 'deltabar/1',         'position_loop/5', 'autorouting','smart');
add_line(mdl, 'pos_state_delay/1',  'position_loop/6', 'autorouting','smart');
add_line(mdl, 'position_loop/3',    'pos_state_delay/1','autorouting','smart');
% att_sp = [roll_sp; pitch_sp (dongudan)] ++ [yaw_sp (sabit)]
add_line(mdl, 'position_loop/1',    'att_sp_mux/1',    'autorouting','smart');
add_line(mdl, 'yaw_sp/1',           'att_sp_mux/2',    'autorouting','smart');
add_line(mdl, 'altitude_loop/1', 'F_sp_mux/2', 'autorouting','smart');

% wls_alloc girisleri: dtau(1) F_sp(2) u_actual(3) agl(4) roll(5) pdot(6) land_state(7)
add_line(mdl, 'indi_rate_law/1', 'wls_alloc/1', 'autorouting','smart');
add_line(mdl, 'F_sp_mux/1',      'wls_alloc/2', 'autorouting','smart');
add_line(mdl, 'sel_uactual/1',   'wls_alloc/3', 'autorouting','smart');
% INIS UCLUSU girisleri (Adim 125)
add_line(mdl, 'sel_z/1',              'agl_gain/1',   'autorouting','smart');
add_line(mdl, 'agl_gain/1',           'wls_alloc/4',  'autorouting','smart');
add_line(mdl, 'quat2euler/1',         'sel_roll/1',   'autorouting','smart');
add_line(mdl, 'sel_roll/1',           'wls_alloc/5',  'autorouting','smart');
add_line(mdl, 'Plant/1',              'sel_pdot/1',   'autorouting','smart');
add_line(mdl, 'sel_pdot/1',           'wls_alloc/6',  'autorouting','smart');
add_line(mdl, 'wls_state_delay/1',    'wls_alloc/7',  'autorouting','smart');
% ...ve durumun geri beslemesi (3. cikis -> Unit Delay)
add_line(mdl, 'wls_alloc/3',          'wls_state_delay/1', 'autorouting','smart');

% wls_alloc -> Unit Delay -> Plant (kapali cevre, cebirsel donguyu kirar) + bozucu girisleri
add_line(mdl, 'wls_alloc/1',   'u_cmd_delay/1', 'autorouting','smart');
add_line(mdl, 'u_cmd_delay/1', 'Plant/1',       'autorouting','smart');
add_line(mdl, 'wind_ned/1',    'Plant/2', 'autorouting','smart');
add_line(mdl, 'ext_moment/1',  'Plant/3', 'autorouting','smart');

% Scope'lar
add_line(mdl, 'quat2euler/1',    'scope_att/1',     'autorouting','smart');
add_line(mdl, 'sel_omega/1',     'scope_omega/1',   'autorouting','smart');
add_line(mdl, 'indi_rate_law/2', 'scope_dhat/1',    'autorouting','smart');
add_line(mdl, 'wls_alloc/2',     'scope_satflag/1', 'autorouting','smart');
add_line(mdl, 'sel_uactual/1',   'scope_act/1',     'autorouting','smart');

%% ================================================================
%  KAYDET
%% ================================================================
save_system(mdl, [mdl '.slx']);
fprintf('\n[OK] %s.slx olusturuldu.\n', mdl);
fprintf('     Calistirmak icin: sim(''%s'')\n', mdl);

end % tiltrotor_indi_build

%% ====================================================================
function set_mfcn(mdl, blk_name, filename)
% .m dosyasini okuyup MATLAB Function Block'a yazar (f450 projesindeki
% ayni yardimci ile ayni desen).
fid = fopen(filename, 'r');
if fid == -1
    warning('Dosya bulunamadi: %s — blogu elle doldur.', filename);
    return;
end
code = fread(fid, '*char')';
fclose(fid);

blk_path = [mdl '/' blk_name];
rt = sfroot();
m  = rt.find('-isa', 'Simulink.BlockDiagram', 'Name', mdl);
if isempty(m)
    warning('Model bulunamadi: %s', mdl);
    return;
end
sf_blk = m.find('-isa', 'Stateflow.EMChart', 'Path', blk_path);
if ~isempty(sf_blk)
    sf_blk.Script = code;
    fprintf('  [OK] %s icerigi yuklendi\n', blk_name);
else
    warning('  [WARN] %s blogu bulunamadi — elle doldur', blk_name);
end
end
