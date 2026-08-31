function u_trim = hover_trim(p)
%HOVER_TRIM  Yaklasik-analitik hover dengesi (trim) aktuator durumu.
%
% Once Fz ve pitch dengesini (2*T_front*0.22 = T_rear*0.65, tail-mount
% dokumaninda tarif edildigi gibi) delta=0 icin kapali-form cozer. Bu
% konfigurasyonda km isaretleri (2x CCW + 1x CW) delta=0'da net bir yaw
% moment birakir (rotor reaksiyon torklarindan); bu, sol kanat rotorunun
% tilt'i (iyi kosullanmis tek-eksenli bir duzeltme, bkz. proje notlari)
% ile sifirlanir. Kalan kucuk artik roll/pitch, kapali-cevrim INDI/WLS
% tarafindan calisma anında duzeltilir.

Tw = p.m*p.g / (2 + 2*p.rotor.pos(1,1)/abs(p.rotor.pos(1,3)));
Tt = (2*p.rotor.pos(1,1)/abs(p.rotor.pos(1,3))) * Tw;

u0 = [Tw; Tw; Tt; 0; 0; 0];
[G0, nu0] = effectiveness_matrix(u0, p);

% Adim 12 (2026-07-27) sonrasi: km isaret duzeltmesiyle net yaw reaksiyon
% torku isaret degistirdi, dolayisiyla rotor 1 tilt'i ile sifirlama artik
% NEGATIF tilt isterdi -- tilt fiziksel olarak [0, pi/2] ile sinirli oldugu
% icin uygulanamaz. Karsi kanat rotorunun (PY isareti zit) POZITIF tilt'i
% birebir esdeger duzeltmeyi verir; bu yuzden trim, iki kanat rotorundan
% hangisi uygulanabilir (>=0) duzeltme veriyorsa onu secer.
d0_trim = -nu0(3) / G0(3,4);   % rotor 0 tilt'i ile tek-eksenli duzeltme
d1_trim = -nu0(3) / G0(3,5);   % rotor 1 tilt'i ile (zit isaretli)

u_trim = [Tw; Tw; Tt; 0; 0; 0];

if d0_trim >= 0
    u_trim(4) = min(d0_trim, p.tilt.max);
else
    u_trim(5) = min(d1_trim, p.tilt.max);
end

end
