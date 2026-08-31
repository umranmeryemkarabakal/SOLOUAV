function [fx_cmd, pitch_sp, release_hold, req_abort, state_out, warn_code] = ...
        sf_forwardtrans_loop(enable, v_h, z, sat_thrust, state_in)
%SF_FORWARDTRANS_LOOP  MATLAB Function blok icerigi: hover'dan seyre ILERI
%gecis durum makinesi (madde V).
%#codegen
%
% forwardtrans_loop.m'in codegen-guvenli yeniden yazimi (Adim 129).
% `switch`/`case` codegen-guvenlidir; durum sabitleri derleme-zamani
% literalleri oldugu icin `enum` gerekmez.
%
% Girisler:
%   enable     (logical) gecis istegi
%   v_h        (m/s) yatay hiz buyuklugu
%   z          (m, NED) irtifa (band dedektoru icin)
%   sat_thrust (logical) tahsisat itki kutusunda doygun mu
%   state_in   [state; fx_cmd; z_entry; t_ramp]
% Ciktilar:
%   fx_cmd (N), pitch_sp (rad), release_hold, req_abort,
%   state_out, warn_code (0=yok, 1=irtifa bandi, 2=zaman asimi)
%
% !! MATLAB BU MANEVRANIN AEROSUNU DOGRULAYAMAZ (backtrans_loop.m'in ayni
% uyarisi): tek boylamsal yuzey, 12 m/s'de ~25 N tasima. Buradaki varlik
% SENKRON ve MANTIK testi icindir; sayilarin dogrulanmasi SITL'de yapilir.

% --- SABITLER: forwardtrans_loop.m / tiltrotor_params.m ile SENKRON ---
FT_FX_CRUISE = 12.0;         % p.ft.fx_cruise   (N)
FT_FX_RATE   = 10.0/12.0;    % p.ft.fx_rate  (N/s, olculen 0->10 N / 12 s)
FT_CRUISE_V  = 8.0;          % p.ft.cruise_v    (m/s)
FT_ALT_BAND  = 5.0;          % p.ft.alt_band    (m)
FT_TIMEOUT_S = 30.0;         % p.ft.timeout_s
FT_ALLOW_ABORT = false;      % p.ft.allow_abort
TS_POS       = 1/50;         % p.Ts_pos

FT_IDLE = 0; FT_RAMP = 1; FT_CRUISE = 2;

state   = state_in(1);
fx_cmd  = state_in(2);
z_entry = state_in(3);
t_ramp  = state_in(4);

pitch_sp     = 0.0;      % Adim 29: hizlanirken burun yukari = TIRMANMA
release_hold = false;
req_abort    = false;
warn_code    = 0;

if ~enable
    state_out = [FT_IDLE; 0.0; 0.0; 0.0];
    fx_cmd    = 0.0;
    return;
end

switch state
    case FT_IDLE
        % Yukselen kenar: irtifa bandinin referansini BURADA dondur.
        state   = FT_RAMP;
        fx_cmd  = 0.0;
        z_entry = z;
        t_ramp  = 0.0;
        release_hold = true;

    case FT_RAMP
        release_hold = true;
        t_ramp = t_ramp + TS_POS;

        % Itki doygunsa DAHA FAZLA isteme. Bu bir iptal sebebi degil, bir
        % BEKLEME sebebi: tahsisat zaten kutuda, fx'i buyutmek yalnizca
        % cozulemeyen bir talep yaratir (Adim 11'in dersi).
        if ~sat_thrust
            fx_cmd = min(FT_FX_CRUISE, fx_cmd + FT_FX_RATE * TS_POS);
        end

        % Dedektorler HER ZAMAN calisir; EYLEM allow_abort'a baglidir.
        if abs(z - z_entry) > FT_ALT_BAND
            warn_code = 1;                 % emniyet 1: kacis tirmanisi/dalisi
        elseif t_ramp >= FT_TIMEOUT_S && v_h < FT_CRUISE_V
            warn_code = 2;                 % emniyet 2: aero-bagimsiz sure
        end

        if warn_code ~= 0 && FT_ALLOW_ABORT
            req_abort = true;
        elseif warn_code == 0 && fx_cmd >= FT_FX_CRUISE - 1e-6 && v_h >= FT_CRUISE_V
            state = FT_CRUISE;
        elseif warn_code ~= 0 && fx_cmd >= FT_FX_CRUISE - 1e-6 && v_h >= FT_CRUISE_V
            % Iptal kapaliyken uyari, CRUISE'a gecisi ENGELLEMEZ: manevra
            % tamamlanir, uyari yalnizca rapor edilir.
            state = FT_CRUISE;
        end

    case FT_CRUISE
        release_hold = true;
        fx_cmd = FT_FX_CRUISE;

        if abs(z - z_entry) > FT_ALT_BAND
            warn_code = 1;                 % seyirde de gecerli
            if FT_ALLOW_ABORT
                req_abort = true;
            end
        end

    otherwise
        state   = FT_IDLE;
        fx_cmd  = 0.0;
        z_entry = 0.0;
        t_ramp  = 0.0;
end

state_out = [state; fx_cmd; z_entry; t_ramp];

end
