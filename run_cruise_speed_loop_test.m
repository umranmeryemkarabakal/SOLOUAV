function run_cruise_speed_loop_test()
%RUN_CRUISE_SPEED_LOOP_TEST  cruise_speed_loop.m'in PLANT'SIZ mantik testi.
%
% Adim 47 (2026-08-03). Adim 38'in dersi: bir durum/dongu yasasinin mantigi,
% plant'ten BAGIMSIZ olarak da sinanabilmeli -- aksi halde bir mantik hatasi
% ancak aerodinamik bir kilikta ortaya cikar ve tesbiti pahali olur.
%
% Denetlenenler:
%   A) KAPALI   -- p.tecs.enable = false iken cikis BIREBIR fx_track (eski davranis)
%   B) TAKIP    -- dongu pasifken cikis fx_track'i izler ve integrator onu tutar
%   C) BUMPLESS -- devralma aninda cikis basamak URETMEZ
%   D) REGULASYON -- sabit bir "plant"te hiz hedefe oturur, kalici hata ~0
%   E) ANTI-WINDUP -- ulasilamaz bir hedefte integrator sismez ve hedef
%      erisilebilir hale gelince GECIKMELI bir asim yapmaz
%   F) TAVAN/TABAN -- cikis [0, fx_max] disina cikmaz

p  = tiltrotor_params();
ok = true;
fprintf('=== seyir hiz dongusu mantik testi (Adim 47) ===\n');
fprintf('v_sp = %.1f m/s, kp = %.2f, ki = %.2f, fx_max = %.1f N\n\n', ...
        p.tecs.v_sp, p.tecs.kp, p.tecs.ki, p.tecs.fx_max);

% --- A) KAPALI --------------------------------------------------------------
poff = p; poff.tecs.enable = false;
st = [0;1];
[fx, st] = cruise_speed_loop(5.0, 16.0, 7.25, st, poff);
ok = check('A1) enable=false iken cikis TAM fx_track', fx == 7.25, ok, ...
           sprintf('%.4f', fx));

% --- B) TAKIP MODU ----------------------------------------------------------
st = [0;0];
for fxt = [0 2 5 9]
    [fx, st] = cruise_speed_loop(3.0, 16.0, fxt, st, p);
end
ok = check('B1) pasifken cikis fx_track', fx == 9.0, ok, sprintf('%.4f', fx));
ok = check('B2) pasifken integrator fx_track''i tutuyor', abs(st(1) - 9.0) < 1e-12, ok, ...
           sprintf('I = %.4f', st(1)));

% --- C) BUMPLESS DEVRALMA ---------------------------------------------------
% Devralma aninda hiz hedefe ESIT olsun ki P terimi sifir olsun; o zaman cikis
% tam olarak izlenen degere esit kalmali. Bu, "basamak yok"un en sert halidir.
st = [0;0];
[fx_before, st] = cruise_speed_loop(p.tecs.v_sp, p.tecs.v_sp, 9.0, st, p);
st(2) = 1;                                    % dongu ACILIYOR
[fx_after, ~]  = cruise_speed_loop(p.tecs.v_sp, p.tecs.v_sp, 9.0, st, p);
ok = check('C1) devralmada basamak YOK (hata=0 iken)', ...
           abs(fx_after - fx_before) < 1e-9, ok, ...
           sprintf('%.6f -> %.6f', fx_before, fx_after));

% --- D) REGULASYON ----------------------------------------------------------
% Oyuncak hiz plant'i:  m*vdot = fx - D(v),  D(v) = D0 + c*(v - v0)
% DIKKAT (bu testin ilk yaziminda YAPILAN HATA): c = 2.3 N/(m/s) bir YEREL
% EGIMDIR (dD/dv), toplam surukleme degil. D(v) = c*v yazmak 16 m/s'de 36.8 N
% surukleme demek olurdu; gercekte olculen ~10 N. Yanlis oyuncak plant, dongu
% dogru calisirken testi dusuruyordu -- yani hata testteydi, yasada degil.
% Dogru lineerlestirme: 16.0 m/s'de D0 = 10 N (Adim 46'da fx = 10 N o hizda
% dengeleniyordu), egim c = 2.3.
m = p.m; c = 2.3; D0 = 10.0; v0 = 16.0;
Dfun = @(v) D0 + c*(v - v0);
v = 10.0; st = [8;1]; fx = 8;
Ts = p.Ts_pos;
for k = 1:round(120/Ts)
    [fx, st] = cruise_speed_loop(v, p.tecs.v_sp, fx, st, p);
    v = v + Ts*(fx - Dfun(v))/m;
end
ok = check('D1) hiz hedefe oturdu (|hata| < 0.05 m/s)', ...
           abs(v - p.tecs.v_sp) < 0.05, ok, sprintf('v = %.4f m/s', v));
ok = check('D2) denge fx''i suruklemeye esit (D(v_sp))', ...
           abs(fx - Dfun(p.tecs.v_sp)) < 0.3, ok, ...
           sprintf('fx = %.2f N, D(v_sp) = %.2f N', fx, Dfun(p.tecs.v_sp)));

% --- E) ANTI-WINDUP ---------------------------------------------------------
% Ulasilamaz hedef: fx_max yetmiyor. Integrator sismemeli. Sonra hedef
% erisilebilir hale getirilince asim KUCUK kalmali.
pw2 = p; pw2.tecs.fx_max = 5.0;      % D(v_sp) = 10 N, yani 5 N ile ULASILAMAZ
v = 2.0; st = [0;1]; fx = 0;
for k = 1:round(60/Ts)
    [fx, st] = cruise_speed_loop(v, pw2.tecs.v_sp, fx, st, pw2);
    v = v + Ts*(fx - Dfun(v))/m;
end
I_windup = st(1);
ok = check('E1) doygunlukta integrator fx_max''i asmiyor', I_windup <= pw2.tecs.fx_max + 1e-9, ok, ...
           sprintf('I = %.3f (fx_max = %.1f)', I_windup, pw2.tecs.fx_max));
% Simdi tavan acilir; asim olculur.
v_peak = v;
for k = 1:round(120/Ts)
    [fx, st] = cruise_speed_loop(v, p.tecs.v_sp, fx, st, p);
    v = v + Ts*(fx - Dfun(v))/m;
    v_peak = max(v_peak, v);
end
overshoot = (v_peak - p.tecs.v_sp) / p.tecs.v_sp * 100;
ok = check('E2) tavan acilinca asim < %5', overshoot < 5, ok, ...
           sprintf('asim = %.2f %%, son v = %.3f', overshoot, v));

% --- F) TAVAN / TABAN -------------------------------------------------------
st = [0;1]; fx_lo = inf; fx_hi = -inf;
for v_test = [0 5 10 16 25 40]
    [fx, st] = cruise_speed_loop(v_test, p.tecs.v_sp, 0, st, p);
    fx_lo = min(fx_lo, fx); fx_hi = max(fx_hi, fx);
end
ok = check('F1) cikis [0, fx_max] icinde', ...
           fx_lo >= -1e-12 && fx_hi <= p.tecs.fx_max + 1e-12, ok, ...
           sprintf('[%.3f, %.3f]', fx_lo, fx_hi));

fprintf('\n%s\n', repmat('-', 1, 62));
if ok
    fprintf('SONUC: TUM DENETIMLER GECTI\n');
else
    fprintf('SONUC: ***BASARISIZ***\n');
end

end

function ok = check(name, cond, ok, detail)
if cond
    fprintf('  [ok]   %s   %s\n', name, detail);
else
    fprintf('  [HATA] %s   %s\n', name, detail);
    ok = false;
end
end
