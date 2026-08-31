function sched = gain_schedule(delta_bar, p)
%GAIN_SCHEDULE  Ortalama tilt acisina (hover..cruise) gore basit kazanc/agirlik
%interpolasyonu.
%
%   delta_bar : rad, 3 rotorun ortalama tilt acisi (0 = hover, pi/2 = cruise)
%
% Sohbette tarif edilen yaklasim: WLS zaten anlik G(u) Jacobian'i ile kendini
% ayarliyor (bkz. effectiveness_matrix), ama INDI rate-loop kazanclari ve WLS
% aktuator tercih agirliklari icin ayrica basit bir "duz cizgi" (smoothstep)
% interpolasyon uygulaniyor — ayri bir gain-scheduler / durum makinesi degil.

s = max(0, min(1, delta_bar / (pi/2)));
w = 3*s.^2 - 2*s.^3;   % smoothstep(0..1), transition boyunca sicrama olmasin diye

%% --- INDI attitude (outer, P) ve rate (inner, INDI) kazanclari ---
% NOT: Bu kazanclar kapali-cevrim simulasyonda ampirik olarak dogrulanmistir
% (bkz. run_hover_gust_test.m). Bu airframe'de yaw, kanat rotorlerinin ayni
% aktuatorunu (diferansiyel tilt) roll ile PAYLASTIGI ve itki-farki yoluyla
% yaw kontrolu neredeyse tekil (near-singular) oldugu icin, yaw kazanci
% roll/pitch'e gore kasitli olarak daha dusuk tutulur — aksi halde WLS'in
% roll/yaw arasinda "cekismesi" sonucu sonumsuz bir limit cycle olusuyor.
%
% DENENDI VE GERI ALINDI (2026-07-26, Adim 10, bkz. sitl/RUNBOOK.md):
% yaw kazanclari (3. bilesen) 1.5/2.0'dan 2.5/3.5'e (hover) yukseltilip
% run_hover_gust_test ile kontrol edildi -- yukaridaki yorumun uyardigi
% roll<->yaw limit cycle GERCEKTEN tetiklendi: RMS p/q 0.0065/0.0015'ten
% 0.046/0.155'e firladi (~10-100x). PX4'e hic tasinmadi (MATLAB'da
% yakalandi). Bu, Ws_yaw artirmanin da (Adim 7) basarisiz olmasiyla
% birlikte, "yaw'a daha fazla otorite ver" ailesindeki UCUNCU basarisiz
% deneme -- roll<->yaw paylasimli-aktuator kisitlamasi bu airframe'de
% gercekten sert bir duvar gibi gorunuyor.
Kp_att_hover   = [3.0; 3.0; 1.5];
Kp_att_cruise  = [2.0; 2.0; 1.0];
sched.Kp_att   = Kp_att_hover + (Kp_att_cruise - Kp_att_hover)*w;

Kp_rate_hover  = [4.0; 4.0; 2.0];   % Adim 144: 10.0 DENENDI, GERI ALINDI
Kp_rate_cruise = [3.0; 3.0; 1.5];
sched.Kp_rate  = Kp_rate_hover + (Kp_rate_cruise - Kp_rate_hover)*w;

%% --- WLS aktuator tercih agirliklari (Wu kosegeni) ---
% Hover'da tilt sapmasi cos(delta) uzerinden dikey itkiyi hizla dusurur, o
% yuzden tilt kullanimi pahali tutulur; cruise'a yaklastikca bu hassasiyet
% azalir ve tilt daha "ucuz" hale gelir. Itki (thrust) tercihi sabit tutulur.
%
% wu_tilt_hover ust siniri (2026-07-26 SITL T0-kilitlenme incelemesi):
% hover trim itkisinde (Tw~=18.32 N, bkz. hover_trim.m) roll ekseni icin
% |dtau_ddelta0| = Tw*0.05 ~= 0.916, |dtau_dT0| = 0.25 sabit -- yani tilt,
% roll uretmede thrust'tan ~3.665x daha etkili. WLS'nin bir actuator'i
% digerine tercih etme kosulu "birim tork basina ceza" oranidir,
% Wu_i/|G_i|; tilt'in (wu_thrust=1 sabitken) thrust'a tercih edilmesi icin
% wu_tilt/0.916 < 1.0/0.25 = 4.0, yani wu_tilt < 4.0*0.916 = ~3.665
% olmalidir. Eski deger (8.0) bu esigin USTUNDEYDI: WLS, roll hatasini
% kapatmak icin (tilt fiziksel olarak daha etkili olmasina ragmen) T0'i
% sifira kadar tuketmeyi tercih ediyor, bu da SITL'de gozlenen kalici
% aktuator kilitlenmesine katkida bulunuyordu (bkz. sitl/RUNBOOK.md S4).
% 3.0, esigin ~%18 altinda tutularak tilt'i roll/pitch duzeltmesi icin
% acikca ucuz kilar. NOT (2026-07-26 SITL dogrulamasi): bu tek basina
% kilitlenmeyi COZMEDI -- hangi rotorun kilitlendigini degistirdi (T0
% yerine T1). Kok neden hala acik; bkz. RUNBOOK.md "Aday cozum 2" notu.
wu_tilt_hover  = 3.0;
wu_tilt_cruise = 1.5;
wu_tilt = wu_tilt_hover + (wu_tilt_cruise - wu_tilt_hover)*w;

sched.wu_thrust = [1.0; 1.0; 1.0];
% Kuyruk rotoru (3.) merkez hatta oldugu icin roll/yaw'a tilt yoluyla katki
% yapmiyor (p.rotor.tilt_yaw_effective(3)=false) — WLS'in onu bu eksenler
% icin gereksiz yere kullanmasini onlemek icin ekstra agirlik.
tail_penalty = 3.0;
sched.wu_tilt = [wu_tilt; wu_tilt; wu_tilt*tail_penalty];

sched.smooth = w;

end
