function [F_body, M_body, dg] = aero_panels(v_rel_b, omega, surf, p)
%AERO_PANELS  gz-sim LiftDrag eklentisinin BIREBIR FRD portu (5 panel).
%
%   v_rel_b : 3x1, govde eksenli hava-goreli hiz (FRD, m/s)
%   omega   : 3x1, govde acisal hizi (rad/s) -- cp noktasindaki hiz icin
%   surf    : 5x1, kontrol yuzeyi eklem acilari (rad) -- gz'de JointPositionController
%             hedefi; [sol elevon; sag elevon; sol elevator; sag elevator; rudder]
%   F_body  : 3x1, toplam aero kuvveti (FRD, N)
%   M_body  : 3x1, CG etrafinda toplam aero momenti (FRD, N*m)
%
% NEDEN BU DOSYA VAR (Adim 46, 2026-08-03)
% ----------------------------------------
% Eski plant TEK bir 0.5 m^2 boylamsal panel kullaniyordu ve BES kontrol
% yuzeyinin hicbirini modellemiyordu. Ikisi de plant'i madde (V) icin
% KULLANILAMAZ kiliyordu: bir yuzey sapmasi plant'te tam olarak sifir etki
% uretiyordu, yani "yuzeyleri tahsisata sok" degisikligini MATLAB kapisi
% yapisal olarak GOREMEZDI (safe-control-change adim 2 anlamsiz olurdu).
%
% *** VE OLCULDU KI ESKI PANEL ISARETI TERSTI. *** 15 m/s'de burun YUKARI,
% eski yasada 43-67 N ASAGI kuvvet uretiyordu (dogrusu: yukari). Zincir:
%   alpha_eski = atan2(-w, u) = -alpha_std   ve   Cl = cla*(alpha_eski - a0)
%   => Cl = -cla*(alpha_std + a0) = dogru degerin TAM NEGATIFI.
% Kuvvet DONDURME matrisi de ayni ters isaretle yazildigi icin hata kendi
% icinde "tutarli" gorunuyordu; disari yalnizca tasimanin isareti olarak
% sizmisti. gz'nin gercek yasasi asagida turetildi ve bu dosya onu birebir
% uygular, yani plant artik SITL'in gordugu fizigi gorur.
%
% gz-sim 8 (Harmonic) LiftDrag.cc yasasi, FRD'ye cevrilmis haliyle:
%   span     = fwd x up                      (birim)
%   v_cp     = v_rel + omega x cp            (cp noktasinin hava-goreli hizi)
%   vLD      = v_cp - (v_cp . span) span     (lift-drag duzlemine izdusum)
%   liftDir  = normalize(span x vLD)
%   dragDir  = -vLD/|vLD|
%   alpha    = a0 +/- acos(liftDir . up)     (isaret: liftDir . fwd >= 0 ise +)
%   cl       = cla*alpha  (stall disinda) + rad_to_cl*delta
%   cd       = |cda*alpha|
%   F        = q*area*(cl*liftDir + cd*dragDir),   q = 0.5*rho*|vLD|^2
% Kritik ayrinti: gz'de cd, SABIT degil alpha ile ORANTILIDIR (cd = |cda*alpha|).
% Eski plant cd'yi sabit 0.6417 aliyordu; seyir hucum acisinda (~0.15 rad)
% gercek deger 0.096, yani eski model ~7 kat fazla surukleme uyguluyordu.
%
% a0 ISARETI: gz'de a0 alpha'ya EKLENIR (kanat oturma acisi), yani theta = 0'da
% kanat zaten Cl = cla*a0 = +0.284 uretir. Eski plant onu CIKARIYORDU.

n = p.aero.n;
F_body = zeros(3,1);
M_body = zeros(3,1);
dg.cl    = zeros(n,1);
dg.alpha = zeros(n,1);
dg.Fz    = zeros(n,1);

fwd = [1;0;0];

for j = 1:n
    up   = p.aero.up(:,j);
    cp   = p.aero.cp(:,j);
    span = cross(fwd, up);
    span = span / norm(span);

    v_cp = v_rel_b + cross(omega, cp);

    % gz erken cikisi: |v| <= 0.01 ise panel hic kuvvet uretmez.
    if norm(v_cp) <= 0.01
        continue;
    end

    vLD = v_cp - dot(v_cp, span)*span;
    sp  = norm(vLD);
    if sp <= 1e-6
        continue;
    end

    dragDir = -vLD / sp;
    liftDir = cross(span, vLD);
    nl = norm(liftDir);
    if nl <= 1e-9
        continue;
    end
    liftDir = liftDir / nl;

    cosAlpha = max(-1, min(1, dot(liftDir, up)));
    if dot(liftDir, fwd) >= 0
        alpha = p.aero.a0(j) + acos(cosAlpha);
    else
        alpha = p.aero.a0(j) - acos(cosAlpha);
    end
    % gz: +/-90 dereceye sar
    while abs(alpha) > 0.5*pi
        if alpha > 0
            alpha = alpha - pi;
        else
            alpha = alpha + pi;
        end
    end

    as = p.aero.alpha_stall;
    if alpha > as
        cl = p.aero.cla*as + p.aero.cla_stall*(alpha - as);
        cl = max(0, cl);
    elseif alpha < -as
        cl = -p.aero.cla*as + p.aero.cla_stall*(alpha + as);
        cl = min(0, cl);
    else
        cl = p.aero.cla * alpha;
    end
    % kontrol yuzeyi katkisi (gz: cl += controlJointRadToCL * controlAngle)
    cl = cl + p.aero.rad_to_cl(j) * surf(j);

    if alpha > as
        cd = p.aero.cda*as + p.aero.cda_stall*(alpha - as);
    elseif alpha < -as
        cd = -p.aero.cda*as + p.aero.cda_stall*(alpha + as);
    else
        cd = p.aero.cda * alpha;
    end
    cd = abs(cd);

    q  = 0.5 * p.aero.rho * sp^2;
    Fj = q * p.aero.area(j) * (cl*liftDir + cd*dragDir);

    F_body = F_body + Fj;
    M_body = M_body + cross(cp, Fj);

    dg.cl(j)    = cl;
    dg.alpha(j) = alpha;
    dg.Fz(j)    = Fj(3);
end

end
