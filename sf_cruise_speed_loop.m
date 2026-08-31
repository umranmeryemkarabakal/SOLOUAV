function [fx_cmd, state_out] = sf_cruise_speed_loop(v_fwd, v_sp, fx_track, state_in)
%SF_CRUISE_SPEED_LOOP  MATLAB Function blok icerigi: seyir hiz dongusu (PI).
%#codegen
%
% cruise_speed_loop.m'in codegen-guvenli yeniden yazimi (Adim 129). AYNI
% matematik, ayni sabitler -- fark yalnizca struct (`p`) yerine literaller.
%
% Girisler:
%   v_fwd    (m/s) GOVDE ileri hizi (yatay hiz buyuklugu DEGIL -- yanal
%                  suruklenme bir hiz hatasi gibi gorunmemeli)
%   v_sp     (m/s) hedef seyir hizi
%   fx_track (N)   dongu KAPALIYKEN izlenecek deger (bumpless devralma)
%   state_in [I; active]
% Ciktilar:
%   fx_cmd   (N)
%   state_out [I; active]

% --- SABITLER: cruise_speed_loop.m / tiltrotor_params.m ile SENKRON ---
TECS_ENABLE = true;      % p.tecs.enable
TECS_KP     = 1.0;       % p.tecs.kp
TECS_KI     = 0.2;       % p.tecs.ki
FX_MAX      = 13.0;      % p.tecs.fx_max  (N)
TS_POS      = 1/50;      % p.Ts_pos

I      = state_in(1);
active = state_in(2) > 0.5;

if ~TECS_ENABLE
    % Ozellik kapali: davranis Adim 42/46'daki gibi BIREBIR acik dongu.
    fx_cmd    = fx_track;
    state_out = [fx_track; 0];
    return;
end

if ~active
    % TAKIP MODU: dongu henuz devrede degil. Integrator uygulanan fx'i izler,
    % boylece devralma tam olarak basamaksiz olur.
    I         = min(max(fx_track, 0.0), FX_MAX);
    fx_cmd    = fx_track;
    state_out = [I; 0];
    return;
end

e = v_sp - v_fwd;
P = TECS_KP * e;

% Integral: once aday guncelleme, sonra ANTI-WINDUP.
I_cand  = I + TECS_KI * e * TS_POS;
I_cand  = min(max(I_cand, 0.0), FX_MAX);
fx_cand = I_cand + P;

if fx_cand > FX_MAX || fx_cand < 0.0
    % Cikis doygun: integratoru DAHA DA ayni yone iten guncellemeyi geri al.
    % (Kosullu integrasyon; integratoru dondurmak, doygunluk bittiginde
    % birikmis bir hatanin geri tepmesini onler.)
    pushing_out = (fx_cand > FX_MAX && e > 0) || (fx_cand < 0.0 && e < 0);
    if pushing_out
        I_cand = I;
    end
end

I      = I_cand;
fx_cmd = min(max(I + P, 0.0), FX_MAX);

state_out = [I; 1];

end
