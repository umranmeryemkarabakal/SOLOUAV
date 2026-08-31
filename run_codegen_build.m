function run_codegen_build(varargin)
%RUN_CODEGEN_BUILD  sf_* kontrolcu fonksiyonlarindan C kodu ve MEX uretir
%(Adim 128).
%
% NEDEN IKI HEDEF:
%   lib : gomulu C kutuphanesi -- HITL/donanimda derlenecek olan sey. Ciktinin
%         KENDISI urun; burada uretilebildigini ve temiz derlendigini gosteririz.
%   mex : ayni koddan MATLAB'a baglanabilen ikili. UretiLEN kodu MATLAB
%         kaynagiyla AYNI senaryoda kosturup sayisal olarak karsilastirmanin
%         tek pratik yolu budur (run_codegen_verify.m).
%
% Plant (tiltrotor_plant_sfcn) BILEREK haric: Level-2 MATLAB S-Function
% codegen'e girmez, girmesi de gerekmez -- karta giden sey kontrolcudur.
%
% KULLANIM
%   run_codegen_build          % lib + mex
%   run_codegen_build('-mex')  % yalnizca mex (hizli)

only_mex = any(strcmp(varargin, '-mex'));

% --- ornek girdiler: boyut ve tip sozlesmesini belirler ---
z3 = zeros(3,1); z2 = zeros(2,1);
specs = {
%   fonksiyon             ornek girdiler
    'sf_indi_rate_law',   {z3, z3, z3, z3, zeros(6,1), z3, zeros(13,1)}
    'sf_wls_alloc',       {z3, z2, zeros(6,1), 0, 0, 0, zeros(5,1)}
    'sf_position_loop',   {z2, z2, z2, 0, 0, z2}
    'sf_altitude_loop',   {0, 0, 0, 0}
    'sf_quat_to_euler',   {zeros(4,1)}
    % --- GOREV FAZLARI (Adim 129) ---
    % Gercek gorev: kalkis -> ileri gecis -> sabit kanat -> geri gecis ->
    % hover -> inis. Asagidakiler o dizinin ORTA kismidir.
    'sf_forwardtrans_loop', {false, 0, 0, false, zeros(4,1)}
    'sf_backtrans_loop',    {false, 0, 0, 0, zeros(3,1)}
    'sf_cruise_speed_loop', {0, 0, 0, z2}
    'sf_cruise_pitch_loop', {0, 0, 0}
    'sf_fixedwing_law',     {0, 0, 0, z3, z3, 0, 0, 0, z2}
    % --- INIS DIZISI ve GOREV DIZICISI (Adim 160) ---
    % Adim 153/154'te yalnizca PX4 C++'a eklenmislerdi; o haliyle HITL'de
    % (uretilen kod) otonom gorev OLMAZDI -- bayraklari yine disaridan biri
    % kaldirmak zorunda kalirdi (madde B0).
    'sf_landing_sequence',  {false, 0, 0, 0, zeros(4,1), 0}
    'sf_mission_sequencer', {false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, zeros(5,1), 0}
};

outdir = fullfile(pwd, 'codegen_out');
if ~exist(outdir, 'dir'); mkdir(outdir); end

fprintf('\n=== KOD URETIMI (Adim 128) ===\n');

for i = 1:size(specs,1)
    fname = specs{i,1};
    args  = specs{i,2};

    % --- MEX: dogrulama icin ---
    cfg_mex = coder.config('mex');
    cfg_mex.IntegrityChecks   = false;   % uretilen kodun kendisini olcuyoruz
    cfg_mex.ResponsivenessChecks = false;
    try
        codegen('-config', cfg_mex, fname, '-args', args, ...
                '-o', fullfile(outdir, [fname '_mex']), '-report');
        fprintf('  [MEX] %-22s OK\n', fname);
    catch ME
        fprintf('  [MEX] %-22s ⛔ %s\n', fname, ME.message);
        continue;
    end

    if only_mex; continue; end

    % --- LIB: karta gidecek gomulu C ---
    % ert (Embedded Coder) hedefi: donanim icin uretilen kod bu.
    cfg_lib = coder.config('lib', 'ecoder', true);
    cfg_lib.GenerateReport   = true;
    cfg_lib.GenCodeOnly      = true;      % derleyici zinciri gerekmesin
    cfg_lib.TargetLang       = 'C';
    % Dinamik bellek YOK: gomulu hedefte malloc kabul edilemez. Bu ayar,
    % fonksiyonlardan biri sabit-boyutlu olmaktan cikarsa uretimi DURDURUR --
    % yani sessizce heap'e kacan bir degisiklik burada yakalanir.
    cfg_lib.DynamicMemoryAllocation = 'Off';
    % HEDEF DONANIM -- bu ayar OLMADAN uretilen kod HOST icindir ve
    % rtwtypes.h `tmwtypes.h`'i (MATLAB kurulum basligi) include eder;
    % ARM capraz derleyicide "No such file or directory" ile durur.
    % OLCULDU (Adim 128): ayarsiz 27 dosyanin 27'si derlenmedi.
    % Cube Orange = STM32H753, Cortex-M7 + cift duyarlikli FPU.
    cfg_lib.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-M';
    cfg_lib.HardwareImplementation.ProdLongLongMode = true;
    % Tek dosyalik cikti: karta tasimak icin daha az parca.
    cfg_lib.FilePartitionMethod = 'SingleFile';
    % `tmwtypes.h` bagimliligini kes -- tipler rtwtypes.h icinde tanimlansin.
    cfg_lib.PurelyIntegerCode = false;
    cfg_lib.SupportNonFinite  = false;   % rtGetInf/rtGetNaN dosyalarini da kaldirir
    try
        codegen('-config', cfg_lib, fname, '-args', args, ...
                '-d', fullfile(outdir, [fname '_lib']));
        fprintf('  [LIB] %-22s OK\n', fname);
    catch ME
        fprintf('  [LIB] %-22s ⛔ %s\n', fname, ME.message);
    end
end

fprintf('\nCikti: %s\n', outdir);
fprintf('Dogrulama icin: run_codegen_verify\n\n');

end
