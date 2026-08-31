%% FIND_FIXEDWING_TRIM
% Belirli bir hizda (V_SP), hangi hucum acisi (alpha=theta, ucus yolu
% acisi=0 varsayimiyla) L=W dengesini sagliyor, ve o noktada surtunmeyi
% (drag) dengeleyen itki ne kadar -- aero_panels.m'i DOGRUDAN sorgulayarak.

clear; clc;
addpath(fileparts(mfilename('fullpath')));
p = tiltrotor_params();

V_SP = 16.0;
alphas = deg2rad(-2:0.25:20);
Fz = zeros(size(alphas));
Fx_aero = zeros(size(alphas));

surf0 = zeros(p.surf.n,1);
omega0 = [0;0;0];

for i = 1:numel(alphas)
    a = alphas(i);
    v_rel = V_SP * [cos(a); 0; sin(a)];   % govde eksen, FRD
    [F_aero, ~] = aero_panels(v_rel, omega0, surf0, p);
    Fz(i) = F_aero(3);       % govde-Z (asagi pozitif, FRD)
    Fx_aero(i) = F_aero(1);  % govde-X (surukleme, ileri pozitif -- genelde negatif=drag)
end

W = p.m*p.g;
% Fz(aero) + gövde-eksen agirlik bilesenii = 0 dengede (kucuk theta icin
% agirlik govde-Z'ye yaklasik +W*cos(theta) katkisi yapar -- basitce W
% kullanalim, kucuk aci varsayimi)
[~, idx] = min(abs(Fz + W));   % Fz_aero = -W (aero yukari, FRD'de -Z=yukari yonu telafi eder)
alpha_trim = alphas(idx);
drag_trim = -Fx_aero(idx);   % pozitif surukleme

fprintf('V_SP=%.1f m/s icin:\n', V_SP);
fprintf('  Denge hucum acisi (theta_trim): %.2f deg\n', rad2deg(alpha_trim));
fprintf('  O noktada Fz(aero) = %.2f N (hedef -W=%.2f N)\n', Fz(idx), -W);
fprintf('  O noktada surukleme: %.2f N -- iki kanat rotoru esit paylasirsa her biri %.2f N\n', ...
    drag_trim, drag_trim/2);

fig = figure;
subplot(2,1,1); plot(rad2deg(alphas), Fz, 'LineWidth',1.3); hold on; grid on;
yline(-W, 'r--', 'DisplayName','-W (hedef)'); xline(rad2deg(alpha_trim),'k:');
ylabel('Fz aero (N)'); title('Hucum acisina gore dikey aero kuvvet');
subplot(2,1,2); plot(rad2deg(alphas), -Fx_aero, 'LineWidth',1.3); hold on; grid on;
xline(rad2deg(alpha_trim),'k:');
ylabel('Surukleme (N)'); xlabel('alpha (deg)'); title('Hucum acisina gore surukleme');
saveas(fig, fullfile(fileparts(mfilename('fullpath')), 'fixedwing_trim.png'));
