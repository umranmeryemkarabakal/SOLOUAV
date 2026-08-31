function run_codegen_sitl_replay(ulog_path)
%RUN_CODEGEN_SITL_REPLAY  URETILEN KODU gercek SITL ucus verisiyle sinar
%(Adim 130).
%
% NEDEN BU TEST. Bugune kadarki dogrulamalar UYDURMA senaryolar kullandi
% (sinuslar, rampalar). Bu dosya baska bir soru soruyor: PX4'un GERCEK bir
% gorevde gordugu girdiler uretilen koda verilirse, PX4'un fiilen komut
% ettigi seyi mi uretir?
%
% Gorev dizisi (SITL'de dogrulandi, 2026-08-30 05_47_06.ulg):
%   kalkis -> ileri gecis -> SABIT KANAT (uc rotor de 0 N, tilt 90 deg kilitli,
%   kanatla ucuyor) -> geri gecis -> hover -> inis
%
% YONTEM: her tikte log'dan u_actual ve nu_des okunur; nu0 ayni geometriyle
% YENIDEN hesaplanip F_sp = nu_des(4:5) + nu0(4:5) ile geri cozulur (nu0
% loglanmiyor). Sonra sf_wls_alloc_mex cagrilir ve URETILEN du, PX4'un
% LOGLADIGI du ile karsilastirilir.
%
% ⚠ BU BIR PARITE TESTI DEGIL, BIR KARSILASTIRMADIR. Iki uygulama BILINEN
% bicimlerde ayrisir (Adim 125/128):
%   - PX4 11 aktuatorle cozer (yuzeyler SURF_ENABLE=false ile sifir sutun),
%     codegen 6 ile;
%   - PX4 250 Hz, codegen sabitleri 400 Hz varsayar (tilt kutusu tik basina
%     ESITLENMIS ama itki kutusu degil);
%   - sabit kanat fazinda PX4 tahsisati HIC kullanmaz (FW yasasi dogrudan
%     yazar), codegen'de o yol ayri bir fonksiyondur.
% Beklenen sonuc "sifir fark" DEGIL; beklenen, farkin HOVER/INIS fazlarinda
% kucuk ve FW fazinda ACIKLANABILIR olmasidir.

if nargin < 1
    d = dir(fullfile(getenv('HOME'), 'PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg'));
    [~,i] = max([d.datenum]);
    ulog_path = fullfile(d(i).folder, d(i).name);
end
addpath(fullfile(pwd,'codegen_out'));
fprintf('\n=== SITL TEKRAR OYNATMA (Adim 130) ===\nlog: %s\n', ulog_path);

ul = ulogreader(ulog_path);
st = readTopicMsgs(ul, 'TopicNames', {'tiltrotor_indi_status'});
S  = st.TopicMessages{1};

n = height(S);
ua  = double(S.u_actual);      % n x 11
du  = double(S.du);            % n x 11
nud = double(S.nu_des);        % n x 5
fw  = double(S.fw_state);
ftS = double(S.ft_state);
btS = double(S.bt_state);

fprintf('ornek sayisi: %d\n', n);

% Faz maskeleri -- her fazi AYRI raporla, cunku beklentiler farkli.
ph_hover = (fw==0) & (ftS==0) & (btS==0);
ph_ft    = (ftS>0) & (fw==0);
ph_fw    = (fw>0);
ph_bt    = (btS>0) & (fw==0);

wst = zeros(5,1);
d_du = nan(n,1);
for k = 1:n
    u6 = ua(k,1:6).';
    % nu0'i ayni geometriyle yeniden kur (loglanmiyor).
    nu0 = local_nu0(u6);
    F_sp = nud(k,4:5).' + nu0(4:5);
    dtau = nud(k,1:3).';
    % Inis kapisi: bu tekrar oynatmada KAPALI tutuluyor. Sebebi durust olsun --
    % agl, roll ve pdot bu topic'te yok; onlari baska topic'lerden zaman
    % hizalayarak getirmek ayri bir is. Inis fazinin kendisi TEST D'de
    % (parite) ve Senaryo 2'de (uretilen kod) zaten sinandi.
    [uc, ~, wst] = sf_wls_alloc_mex(dtau, F_sp, u6, Inf, 0, 0, wst);
    d_du(k) = max(abs((uc - u6) - du(k,1:6).'));
end

report('hover/durus', d_du(ph_hover));
report('ileri gecis', d_du(ph_ft));
report('SABIT KANAT', d_du(ph_fw));
report('geri gecis ', d_du(ph_bt));
report('TUMU       ', d_du);

fprintf(['\nNOT: sabit kanat fazinda buyuk fark BEKLENIR ve bir kusur degildir --\n' ...
         'PX4 o fazda WLS tahsisatini HIC kullanmaz, FW yasasi aktuatorleri\n' ...
         'dogrudan yazar (fw_surf_active). Codegen tarafinda bunun karsiligi\n' ...
         'sf_fixedwing_law''dir, ayri bir fonksiyon.\n\n']);

end

function report(name, v)
v = v(~isnan(v));
if isempty(v); fprintf('  %s : ornek yok\n', name); return; end
fprintf('  %s : n=%5d  ort %.3e  p95 %.3e  max %.3e  (N)\n', ...
        name, numel(v), mean(v), prctile(v,95), max(v));
end

function nu0 = local_nu0(u)
% sf_wls_alloc.m'deki nu0 hesabinin AYNISI -- ayni sabitler, ayni sira.
rpos = [0.27 0.27 -0.55; 0.35 -0.35 0.00; -0.11 -0.11 -0.07];
km   = [-0.06, 0.06, -0.06];
T = u(1:3); de = u(4:6);
nu0 = zeros(5,1);
for i = 1:3
    s_i = sin(de(i)); c_i = cos(de(i));
    dir_i = [s_i; 0; -c_i];
    f_i = T(i)*dir_i;
    m_i = km(i)*T(i)*dir_i;
    nu0(1:3) = nu0(1:3) + cross(rpos(:,i), f_i) + m_i;
    nu0(4) = nu0(4) + f_i(1);
    nu0(5) = nu0(5) + f_i(3);
end
end
