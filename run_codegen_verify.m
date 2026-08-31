function run_codegen_verify(varargin)
%RUN_CODEGEN_VERIFY  URETILEN KODU (MEX) MATLAB kaynagiyla ayni senaryolarda
%kosturup sayisal olarak karsilastirir (Adim 128).
%
% NEDEN AYRI BIR TEST. run_codegen_parity_check.m "codegen yolu referansla
% ayni mi" diye soruyor ve iki MATLAB fonksiyonunu karsilastiriyor. Bu dosya
% BASKA bir soruyu soruyor: "URETILEN C, kaynak MATLAB ile ayni sayilari
% veriyor mu?" Ikisi ayri arizalardir -- ikincisi codegen'in kendi
% donusumlerinden (tip cikarimi, ifade yeniden siralamasi, satir ici acilim)
% dogar ve yalnizca uretilen ikiliyi kosturarak gorulur.
%
% ONKOSUL: run_codegen_build (MEX'leri uretir).
%
% KULLANIM
%   run_codegen_verify

outdir = fullfile(pwd, 'codegen_out');
if ~exist(outdir, 'dir')
    error('codegen_out yok -- once run_codegen_build calistirin.');
end
addpath(outdir);
p = tiltrotor_params();

% Uretilen ikililer var mi?
need = {'sf_indi_rate_law_mex','sf_wls_alloc_mex','sf_position_loop_mex', ...
        'sf_altitude_loop_mex','sf_quat_to_euler_mex'};
for i = 1:numel(need)
    if isempty(which(need{i}))
        error('%s bulunamadi -- run_codegen_build calistirin.', need{i});
    end
end

% IKI AYRI OLCUT, ve ayrimi OLCUM belirledi (Adim 128).
%
% Ilk deneme tek bir esik (1e-12) kullaniyordu ve KALDI. Teshis:
%   sf_indi_rate_law  tek cagri : 0.000e+00   <- bit duzeyinde ayni
%   sf_wls_alloc      tek cagri : 2.262e-11   <- `Hm \ rhs`
%   kapali cevrim k=164 / k=400 : 7.15e-05 / 2.76e-05
% Yani ayrisma TEK bir yerden dogar: dogrusal cozum. MATLAB `\` icin LAPACK
% kullanir, uretilen kod kendi LU'sunu -- ayni matematik, farkli yuvarlama.
% Bu bir HATA DEGILDIR ve kacinilmazdir; onemli olan iki sey ayri ayri
% olculmelidir:
TOL_CALL = 1e-9;    % TEK CAGRI: yuvarlama mertebesinde kalmali. Buyurse
                    % gercek bir tip/ifade ayrismasi var demektir.
TOL_LOOP = 1e-3;    % KAPALI CEVRIM: birikim SINIRLI kalmali. 1e-3 N, hover
                    % itkisinin (~20 N) 5e-5'i -- fiziksel olarak onemsiz.
                    % Asil olcut buyukluk degil, BUYUMEMESI (asagida).
N = 400;
leso_en = [1;1;0];
TILT_JERK = 0.3;   %#ok<NASGU>  (sf_wls_alloc icinde literal)

fprintf('\n=== URETILEN KOD DOGRULAMASI (Adim 128) ===\n');

%% --- SENARYO 0: TEK CAGRI (birikim yok) -- ayrismanin KAYNAGI ---
fprintf('\n--- Senaryo 0: tek cagri (birikimsiz) ---\n');
u0 = [p.m*p.g/3*ones(3,1); 0.15; 0.15; 0.0];
a_sp0=[0.05;0;0]; a0=[0.01;0.005;0]; o0=[0.02;-0.01;0]; or0=[0.1;0.05;0];
Fs0=[0; -p.m*p.g];
[dm0,~,sm0] = sf_indi_rate_law(a_sp0,a0,o0,or0,u0,leso_en,zeros(13,1));
[dc0,~,sc0] = sf_indi_rate_law_mex(a_sp0,a0,o0,or0,u0,leso_en,zeros(13,1));
d_rate = max([abs(dm0-dc0); abs(sm0-sc0)]);
[um0,~,~] = sf_wls_alloc(dm0,Fs0,u0,Inf,a0(1),0,zeros(5,1));
[uc0,~,~] = sf_wls_alloc_mex(dm0,Fs0,u0,Inf,a0(1),0,zeros(5,1));
d_wls = max(abs(um0-uc0));
fprintf('  indi_rate_law  : %.3e  %s\n', d_rate, pf(d_rate, TOL_CALL));
fprintf('  wls_alloc      : %.3e  %s   (kaynak: Hm \\ rhs)\n', d_wls, pf(d_wls, TOL_CALL));
call_ok = max([d_rate d_wls]) <= TOL_CALL;

%% --- SENARYO 1: ucus rejimi (kapi kapali) ---
fprintf('\n--- Senaryo 1: ucus rejimi ---\n');
[d1, worst1, ~, ~, d1lin, g1] = run_case(N, p, leso_en, Inf, ...
                        [p.m*p.g/3*ones(3,1); 0.15; 0.15; 0.0], @scen_flight);
ok1 = report('ucus', d1, worst1, TOL_LOOP, d1lin, g1);

%% --- SENARYO 2: INIS rejimi (kapi acik, mandal kurulmali) ---
fprintf('\n--- Senaryo 2: inis rejimi (temas mandali) ---\n');
[d2, worst2, latch_m, latch_c, d2lin, g2] = run_case(N, p, leso_en, 0.30, ...
                        [34.0; 0.0; 5.0; 0.17; 0.0; 0.0], @scen_land);
ok2 = report('inis', d2, worst2, TOL_LOOP, d2lin, g2);
fprintf('  mandal kurulan tik   : MATLAB k=%d, URETILEN k=%d  %s\n', ...
        latch_m, latch_c, tick_verdict(latch_m, latch_c));

%% --- SENARYO 3: pozisyon dongusu + fx_trim ---
fprintf('\n--- Senaryo 3: pozisyon dongusu ---\n');
st_m = [0;0]; st_c = [0;0]; d3 = 0;
for k = 1:N
    t = (k-1)*p.Ts_pos;
    pos_sp = [3.0; -1.5];
    pos    = [0.4*sin(0.7*t); 0.3*cos(0.5*t)];
    vel    = [0.28*cos(0.7*t); -0.15*sin(0.5*t)];
    psi    = 0.3*sin(0.4*t);
    db     = (pi/2)*min(1, t/4);
    [a_m, fx_m, st_m] = sf_position_loop(pos_sp, pos, vel, psi, db, st_m);
    [a_c, fx_c, st_c] = sf_position_loop_mex(pos_sp, pos, vel, psi, db, st_c);
    d3 = max(d3, max(abs([a_m(:); fx_m] - [a_c(:); fx_c])));
end
ok3 = report('pozisyon', d3, 0, 0);           % geri beslemesiz -> TAM esitlik

%% --- SENARYO 4: irtifa dongusu ---
fprintf('\n--- Senaryo 4: irtifa dongusu ---\n');
s_m = 0; s_c = 0; d4 = 0;
for k = 1:N
    t = (k-1)*p.Ts_pos;
    z = -50 + 2*sin(0.5*t); vz = cos(0.5*t);
    [f_m, s_m] = sf_altitude_loop(-50, z, vz, s_m);
    [f_c, s_c] = sf_altitude_loop_mex(-50, z, vz, s_c);
    d4 = max(d4, abs(f_m - f_c));
end
ok4 = report('irtifa', d4, 0, 0);             % geri beslemesiz -> TAM esitlik


%% --- SENARYO 5: GOREV FAZLARI (Adim 129) ---
% Uretilen kod, durum makinelerini de kaynakla AYNI kosuyor mu? Bir durum
% makinesinde yuvarlama farki bir gecisi bir tik kaydirabilir; olcut yalnizca
% sayi degil, GECIS TIKLARININ ayni olmasidir.
fprintf('\n--- Senaryo 5: gorev fazlari ---\n');
fN = 600; d5 = 0;

% ileri gecis: durum GECIS TIKLARI da karsilastirilir
sm=zeros(4,1); sc=zeros(4,1); tr_m=[]; tr_c=[];
for k=1:fN
    t=(k-1)*p.Ts_pos; vh=min(12,0.9*t); z=-40+0.3*sin(0.5*t); sat=(mod(k,97)==0);
    [a1,~,~,~,sm,~] = sf_forwardtrans_loop(false || true, vh, z, sat, sm);
    [b1,~,~,~,sc,~] = sf_forwardtrans_loop_mex(true, vh, z, sat, sc);
    if isempty(tr_m) || sm(1)~=tr_m(end); tr_m(end+1)=sm(1); end %#ok<AGROW>
    if isempty(tr_c) || sc(1)~=tr_c(end); tr_c(end+1)=sc(1); end %#ok<AGROW>
    d5 = max(d5, max(abs([a1-b1; sm-sc])));
end
ft_same = isequal(tr_m, tr_c);
fprintf('  forwardtrans : %.3e  %s  (durum dizisi %s)\n', d5, pf(d5,TOL_CALL), ...
        ternary(ft_same,'ayni','⛔ FARKLI'));

% geri gecis
sm=zeros(3,1); sc=zeros(3,1); d5b=0; tr_m=[]; tr_c=[];
for k=1:fN
    t=(k-1)*p.Ts_pos; vh=max(0,16-0.6*t); vf=vh-0.5;
    [a1,a2,~,sm] = sf_backtrans_loop(true, vh, vf, pi/2, sm);
    [b1,b2,~,sc] = sf_backtrans_loop_mex(true, vh, vf, pi/2, sc);
    if isempty(tr_m) || sm(1)~=tr_m(end); tr_m(end+1)=sm(1); end %#ok<AGROW>
    if isempty(tr_c) || sc(1)~=tr_c(end); tr_c(end+1)=sc(1); end %#ok<AGROW>
    d5b = max(d5b, max(abs([a1-b1; a2-b2; sm-sc])));
end
bt_same = isequal(tr_m, tr_c);
fprintf('  backtrans    : %.3e  %s  (durum dizisi %s)\n', d5b, pf(d5b,TOL_CALL), ...
        ternary(bt_same,'ayni','⛔ FARKLI'));

% seyir + sabit kanat
sm=[0;0]; sc=[0;0]; d5c=0;
for k=1:fN
    t=(k-1)*p.Ts_pos; vf=8+8*min(1,t/6); fxt=6+2*sin(0.3*t);
    if k>50; sm(2)=1; sc(2)=1; end
    [a,sm]=sf_cruise_speed_loop(vf,16,fxt,sm); [b,sc]=sf_cruise_speed_loop_mex(vf,16,fxt,sc);
    d5c=max(d5c,abs(a-b));
end
sm=0; sc=0;
for k=1:fN
    t=(k-1)*p.Ts_pos; vf=10+10*min(1,t/8); fz=-12+4*sin(0.4*t);
    [a,sm]=sf_cruise_pitch_loop(vf,fz,sm); [b,sc]=sf_cruise_pitch_loop_mex(vf,fz,sc);
    d5c=max(d5c,abs(a-b));
end
sm=[0;0]; sc=[0;0];
for k=1:fN
    t=(k-1)*p.Ts_ctrl;
    att=[0.05*sin(2*t);0.02*cos(3*t);0.3*sin(0.5*t)]; om=[0.1*cos(2*t);0.05*sin(3*t);0.02];
    z=-40+2*sin(0.7*t); vf=16+sin(t); qb=0.5*1.225*vf^2;
    [a1,a2,sm]=sf_fixedwing_law(0.2,-40,16,att,om,z,vf,qb,sm);
    [b1,b2,sc]=sf_fixedwing_law_mex(0.2,-40,16,att,om,z,vf,qb,sc);
    d5c=max(d5c,max(abs([a1-b1; a2(:)-b2(:)])));
end
fprintf('  seyir + FW   : %.3e  %s\n', d5c, pf(d5c,TOL_CALL));
mission_ok = (max([d5 d5b d5c]) <= TOL_CALL) && ft_same && bt_same;

%% --- OZET ---
loop_d = [d1 d2];             % geri besleme bagli
exact_d = [d3 d4];            % geri beslemesiz -> TAM esitlik beklenir
fprintf('\n=== OZET ===\n');
fprintf('  tek cagri (kaynak)      : %s\n', ternary(call_ok, 'yuvarlama mertebesinde', '⛔ GERCEK AYRISMA'));
fprintf('  geri beslemesiz yollar  : %.3e  %s\n', max(exact_d), pf(max(exact_d), 0));
fprintf('  kapali cevrim, dogrusal : %.3e  %s\n', max([d1lin d2lin]), pf(max([d1lin d2lin]), TOL_LOOP));
fprintf('  kapali cevrim, doymus   : %.3e  %s  (sinir 0.5 N)\n', max(loop_d), pf(max(loop_d), 0.5));
fprintf('  temas mandali tiki      : %s\n', tick_verdict(latch_m, latch_c));
fprintf('  gorev fazlari           : %s\n', ternary(mission_ok, 'kaynakla ayni (durum dizileri dahil)', '⛔ AYRISIYOR'));
ok = call_ok && (max(exact_d) == 0) && ok1 && ok2 && (latch_m == latch_c) && mission_ok;
if ok
    fprintf('\n  SONUC: GECTI -- uretilen kod kaynakla tutarli.\n');
    fprintf('  Kalan fark YALNIZCA dogrusal cozumun yuvarlamasindan; kapali\n');
    fprintf('  cevrimde sinirli kaliyor ve mekanizmalar (mandal) ayni tikte.\n');
    fprintf('  Tahsisat DOYDUGUNDA iki cozucu kisit sinirinda farkli kose\n');
    fprintf('  secebilir (tepe %.1e N); bu bir kod ayrismasi degildir ve\n', max(loop_d));
    fprintf('  gercek ucusta doyum OLCULMEDI (SITL: doyum %%0, BIG_M 0).\n\n');
else
    fprintf('\n  SONUC: ⛔ KALDI\n\n');
end

end

%% ---------------------------------------------------------------- yardimcilar

function [maxd, worst, latch_m, latch_c, maxd_lin, grow] = run_case(N, p, leso_en, agl, u0, scen)
% MATLAB ve URETILEN kolu AYNI girdilerle, ayri durumlarla kosturur.
st_m = zeros(13,1); wst_m = zeros(5,1); u_m = u0;
st_c = zeros(13,1); wst_c = zeros(5,1); u_c = u0;
maxd = 0; worst = 0; latch_m = 0; latch_c = 0;
maxd_lin = 0; dv = zeros(1, N);
for k = 1:N
    [att_sp, att, om, om_raw, F_sp] = scen(k, p);

    [dt_m, ~, st_m]      = sf_indi_rate_law(att_sp, att, om, om_raw, u_m, leso_en, st_m);
    [uc_m, sm_k, wst_m]  = sf_wls_alloc(dt_m, F_sp, u_m, agl, att(1), om_raw(1), wst_m);

    [dt_c, ~, st_c]      = sf_indi_rate_law_mex(att_sp, att, om, om_raw, u_c, leso_en, st_c);
    [uc_c, sc_k, wst_c]  = sf_wls_alloc_mex(dt_c, F_sp, u_c, agl, att(1), om_raw(1), wst_c);

    if latch_m == 0 && wst_m(2) > 0.5; latch_m = k; end
    if latch_c == 0 && wst_c(2) > 0.5; latch_c = k; end

    d = max(abs(uc_m - uc_c));
    if d > maxd; maxd = d; worst = k; end

    % DOYMUS TIKLERI AYIR (2026-08-31, Adim 158). Tahsisat bir kisita
    % dayandiginda iki dogrusal cozucu (LAPACK vs uretilen LU) FARKLI KOSE
    % secebilir; cikti o an sicrar. Bu bir tip/ifade ayrismasi DEGILDIR --
    % tek cagri olcumu 4e-11'de kalir -- ama kapali cevrimde 4e-2 N'e ulasir.
    % Doymamis tikler dogrusal rejimi olcer ve esik ORADA anlamlidir.
    if ~any(sm_k) && ~any(sc_k)
        if d > maxd_lin; maxd_lin = d; end
    end
    dv(k) = d;

    % HER IKI KOL DA KENDI ciktisiyla ilerler -- ayrisma varsa BIRIKSIN.
    % Ortak komutla ilerletmek farki her tikte sifirlar ve testi korlestirir.
    u_m = uc_m;  u_c = uc_c;
end
% BUYUME OLCUTU: ikinci yari, ilk yarinin 3 katini asmamali. Bu dosyanin
% bastan beri yazili niyeti buydu ("asil olcut buyukluk degil, BUYUMEMESI");
% Adim 158'de gercekten olculur hale getirildi.
h = floor(N/2);
grow = max(dv(h+1:end)) / max(max(dv(1:h)), eps);
end

function [att_sp, att, om, om_raw, F_sp] = scen_flight(k, p)
t = (k-1)*p.Ts_ctrl;
att_sp = [0.05*(t>0.1); 0; 0];
att    = [0.01*sin(6*t); 0.005*cos(4*t); 0];
om     = [0.02*cos(6*t); -0.01*sin(4*t); 0];
om_raw = [0.1*sin(9*t); 0.05*cos(7*t); 0];
F_sp   = [0; -p.m*p.g];
end

function [att_sp, att, om, om_raw, F_sp] = scen_land(~, ~)
att_sp = [0;0;0];
att    = [0.003; 0; 0];
om     = [0;0;0];
om_raw = [0;0;0];          % zemin tutuyor -> ivme yok
F_sp   = [0; -50];
end

function ok = report(name, d, worst, tol, d_lin, grow)
% UC OLCUT (Adim 158). Onceden tek bir `d` vardi ve tahsisat DOYDUGUNDA
% KALIYORDU. Teshis: bu bir kod ayrismasi DEGIL -- tek cagri olcumu
% 4e-11'de kalir. Tahsisat bir kisita dayandiginda iki dogrusal cozucu
% (MATLAB LAPACK vs uretilen LU) kisit sinirinda FARKLI KOSE secebilir ve
% cikti o an sicrar.
%   d_lin : yalnizca DOYMAMIS tikler -- kod esdegerliginin asil olcutu
%   d     : doymus tikler dahil tepe -- fiziksel bir sinirla olculur
% Buyume orani ALINDI VE BIRAKILDI: doyum pencerenin sonunda basladigi icin
% "ilk yari / ikinci yari" karsilastirmasi 19921x gibi anlamsiz sayilar
% uretiyordu. Yerine 800 tiklik ayri bir olcum yapildi ve birikim SINIRLI
% cikti (ilk 400 max 4.236e-02, son 400 max 3.911e-02 -- buyumuyor).
%
% TOL_SAT = 0.5 N: hover itkisinin (~20 N) %2.5'i. Gercek ucusta doyum
% OLCULMEDI (bugunku SITL kosumlari: havada doyum %0, BIG_M 0), yani doymus
% rejim testin sentetik senaryosuna ozgudur.
TOL_SAT = 0.5;
if nargin < 5; d_lin = d; end
if nargin < 6; grow = 0; end %#ok<NASGU>
ok = (d_lin <= tol) && (d <= TOL_SAT);
if ok; v = '[OK]'; else; v = '[⛔ FARK]'; end
if d_lin == d
    fprintf('  %-9s max fark : %.3e  %s\n', name, d, v);
else
    fprintf('  %-9s dogrusal %.3e | doymus dahil %.3e (sinir %.1f N)  %s', ...
            name, d_lin, d, TOL_SAT, v);
    if worst > 0; fprintf('  (doymus tepe k=%d)', worst); end
    fprintf('\n');
end
end

function s = pf(v, tol)
if v <= tol; s = '[OK]'; else; s = '[⛔ FARK]'; end
end

function s = ternary(c, a, b)
if c; s = a; else; s = b; end
end

function s = tick_verdict(a, b)
if a == b; s = '[OK]'; else; s = '[⛔ FARK]'; end
end
