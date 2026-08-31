function [fx_cmd, state_out] = cruise_speed_loop(v_fwd, v_sp, fx_track, state_in, p)
%CRUISE_SPEED_LOOP  Seyirde hiz denetimi: fx'i acik dongu olmaktan cikarir.
%
% Adim 47 (2026-08-03), madde (V). ONCEKI ADIMIN OLCUMUNUN DOGRUDAN SONUCU.
%
% NEDEN VAR -- BIR TERCIH DEGIL, OLCULMUS BIR ENGELIN CEVABI
% ----------------------------------------------------------
% Adim 46, kontrol yuzeylerini tahsisata sokmayi MATLAB'da calistirdi (kanat yuk
% payi %98.9 -> %106.4, tilt 52.1 -> 70.6 deg) ama iki engelden biri suydu:
%
%   fx >= 12 N'de rejim YUZEYLER KAPALIYKEN BILE marjinal. Tahsisat doygunlugu
%   %13.5-16.1 ve kanat rotorlarinin tilt'i 89.5 derecede, yani 90 derecelik
%   MEKANIK DURAKTA cakili. Aracin hizini sinirlayan sey bir kontrol yasasi
%   DEGIL, o duraktir. Yuzeyler acilinca bu KAZARA koruma ortadan kalkiyor,
%   arac hizlanmaya devam ediyor ve trimleyemedigi bir rejime giriyor
%   (t = 29-37 s'de iraksama).
%
% Yani `fx` bugune kadar bir HIZ komutu degil, bir IVME komutuydu: sabit bir
% kuvvet, ve hizin nerede duracagini yalnizca surukleme ile bir mekanik durak
% belirliyordu. Bu, kontrol yuzeylerinin ON KOSULUDUR -- yuzey ayariyla
% cozulemez, cunku sorun yuzeylerde degil, eksik bir donguded.
%
% NEDEN "TAM TECS" DEGIL
% ----------------------
% Klasik TECS iki kanali (itki + pitch) iki enerjiye (toplam enerji + enerji
% dagilimi) baglar, cunku sabit kanatta itki ve pitch'in IKISI DE hem hizi hem
% irtifayi etkiler. Bu araçta parametrelestirme zaten daha ayrik: dis dongu
% GOVDE EKSENLI KUVVET komut eder (fx, Fz) ve hangi aktuatorun bunu uretecegine
% WLS karar verir. Irtifa zaten Fz'ye kapali (altitude_loop.m). Eksik olan tek
% serbestlik derecesi hizdi. Onu kapatmak, TECS'in bu mimarideki karsiligidir;
% pitch'e DOKUNULMAZ (adim 29: ~5-6 m/s ustunde burun yukari bir TIRMANMA
% komutudur ve irtifa dongusu buna karsi koyamaz -- forwardtrans_loop.m'in
% pitch = 0 tercihiyle ayni gerekce).
%
% KAZANCLARIN TURETILMESI (olculen zarftan, uydurulmadi)
% ------------------------------------------------------
% Adim 46'nin fx taramasi denge hizlarini verdi: fx = 8 N -> 16.01 m/s,
% fx = 10 N -> 16.89 m/s. Yani surukleme egimi
%     c = dD/dv ~= (10-8)/(16.89-16.01) = 2.3 N/(m/s)
% Acik dongu hiz zaman sabiti zaten kisa: tau = m/c = 5/2.3 = 2.2 s. Demek ki
% AGRESIF bir kazanc gereksiz ve zararli olur (hiz dongusu, duruş dongusunun
% cok altinda kalmali). Kp ile:
%     tau_kapali = m/(c + Kp)
% Kp = 1.0 -> tau = 5/3.3 = 1.5 s. Yeterince canli, duruş dongusunden (0.25 s
% mertebesi) hala ~6 kat yavas.
% Ki, kalici hatayi kapatir; integral zaman sabiti ~5 s secildi:
%     Ki = Kp/5 = 0.2 N/(m/s)/s
% !! Bu iki sayi SITL'de olculmeden dogrulanmis sayilmaz.
%
% BUMPLESS (basamaksiz) DEVRALMA
% ------------------------------
% Dongu devreye girdigi anda cikisi, o an uygulanan fx ile AYNI olmalidir;
% aksi halde devralma bir basamak uretir ve bu araçta bir fx basamagi dogrudan
% bir tilt gecicisidir. Bu yuzden dongu kapali degilken integrator `fx_track`i
% IZLER (takip modu) ve acildiginda tam oradan devam eder.
%
% Girisler:
%   v_fwd    (m/s) GOVDE ILERI hizi -- BUYUKLUK degil, ISARETLI. Madde (S)'nin
%            dersi: yasa ileri ekseni denetliyorsa esigi de o eksen okumali;
%            yanal suruklenme bir hiz hatasi gibi gorunmemeli.
%   v_sp     (m/s) hedef seyir hizi
%   fx_track (N)   dongu KAPALIYKEN izlenecek deger (bumpless devralma)
%   state_in [I; active]  I = integrator (N), active = onceki tick'te acik miydi
%   p        tiltrotor_params()
% Ciktilar:
%   fx_cmd    (N) govde-x kuvvet komutu
%   state_out [I; active]

I      = state_in(1);
active = state_in(2) > 0.5;

if ~p.tecs.enable
    % Ozellik kapali: davranis Adim 42/46'daki gibi BIREBIR acik dongu.
    fx_cmd    = fx_track;
    state_out = [fx_track; 0];
    return;
end

if ~active
    % TAKIP MODU: dongu henuz devrede degil. Integrator uygulanan fx'i izler,
    % boylece devralma tam olarak basamaksiz olur.
    I         = min(max(fx_track, 0.0), p.tecs.fx_max);
    fx_cmd    = fx_track;
    state_out = [I; 0];
    return;
end

e = v_sp - v_fwd;

% Oransal terim
P = p.tecs.kp * e;

% Integral: once aday guncelleme, sonra ANTI-WINDUP.
I_cand  = I + p.tecs.ki * e * p.Ts_pos;
I_cand  = min(max(I_cand, 0.0), p.tecs.fx_max);
fx_cand = I_cand + P;

if fx_cand > p.tecs.fx_max || fx_cand < 0.0
    % Cikis doygun: integratoru DAHA DA ayni yone iten guncellemeyi geri al.
    % (Kosullu integrasyon; integratoru dondurmak, doygunluk bittiginde
    % birikmis bir hatanin geri tepmesini onler.)
    pushing_out = (fx_cand > p.tecs.fx_max && e > 0) || (fx_cand < 0.0 && e < 0);
    if pushing_out
        I_cand = I;
    end
end

I      = I_cand;
fx_cmd = min(max(I + P, 0.0), p.tecs.fx_max);

state_out = [I; 1];

end
