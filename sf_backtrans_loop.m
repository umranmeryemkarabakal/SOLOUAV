function [tilt_ceil, pitch_sp, req_pos_hold, state_out] = sf_backtrans_loop( ...
        enable, v_h, v_fwd, delta_wing_max, state_in)
%SF_BACKTRANS_LOOP  MATLAB Function blok icerigi: seyirden hover'a GERI gecis
%durum makinesi (madde B5).
%#codegen
%
% backtrans_loop.m'in codegen-guvenli yeniden yazimi (Adim 129). Yerel
% `brake_pitch` alt fonksiyonu korundu -- codegen yerel fonksiyonlari destekler
% ve gerekcesi (Adim 37/39) o fonksiyonun basliginda yasiyor.
%
% Girisler:
%   enable         (logical) geri gecis istegi
%   v_h            (m/s) yatay hiz BUYUKLUGU
%   v_fwd          (m/s) ISARETLI govde ileri hizi (madde (S) icin sart)
%   delta_wing_max (rad) kanat rotorlerinin mevcut en buyuk tilt'i
%   state_in       [state; tilt_ceil; floor_dwell]
% Ciktilar:
%   tilt_ceil (rad), pitch_sp (rad), req_pos_hold (logical), state_out
%
% !! MATLAB BU MANEVRAYI DOGRULAYAMAZ (Adim 30'da olculdu): MATLAB plant'i tek
% bir boylamsal yuzey kullanir (area = 0.5 m^2), 12 m/s / 15 deg'de tasima
% ~25 N -- 49 N agirligi kaldiramaz, yani ne kacis tirmanisi ne de Fz kaynakli
% tilt kacisi burada olusur. Gazebo modelinde bes lift-drag yuzeyi var. Bu
% dosyanin varligi SENKRON ve mantik testi icindir; dogrulama YALNIZCA SITL'de
% yapilir (Adim 21d: bir ortam ancak hedeflenen mekanizma orada AKTIFSE bir sey
% kanitlar).

% --- SABITLER: backtrans_loop.m / tiltrotor_params.m ile SENKRON ---
TILT_MAX       = pi/2;        % p.tilt.max
BT_CEIL_FLOOR  = 9.0*pi/180;  % p.bt.ceil_floor   = deg2rad(9)
BT_RETRACT_RATE= 2.0*pi/180;  % p.bt.retract_rate = deg2rad(2)
BT_FLOOR_DWELL = 20.0;        % p.bt.floor_dwell  (s)
BT_RELEASE_V   = 10.0;        % p.bt.release_v    (m/s)
BT_BRAKE_CEIL  = 20.0*pi/180; % p.bt.brake_ceil   = deg2rad(20)
BT_HANDOFF_V   = 3.0;         % p.bt.handoff_v    (m/s)
TS_POS         = 1/50;        % p.Ts_pos

BT_IDLE = 0; BT_RETRACT = 1; BT_BRAKE = 2; BT_HANDOFF = 3;

state       = state_in(1);
tilt_ceil   = state_in(2);
floor_dwell = state_in(3);

req_pos_hold = false;
pitch_sp     = 0.0;

if ~enable
    state       = BT_IDLE;
    tilt_ceil   = TILT_MAX;
    floor_dwell = 0.0;
    state_out   = [state; tilt_ceil; floor_dwell];
    return;
end

switch state
    case BT_IDLE
        % Yukselen kenar: tavani MEVCUT tilt'ten baslat ki hemen baglayici olsun.
        state       = BT_RETRACT;
        tilt_ceil   = min(TILT_MAX, delta_wing_max);
        floor_dwell = 0.0;

    case BT_RETRACT
        tilt_ceil = max(BT_CEIL_FLOOR, tilt_ceil - BT_RETRACT_RATE * TS_POS);
        at_floor  = tilt_ceil <= BT_CEIL_FLOOR + 1e-6;

        % Sayac YALNIZCA tabanda islesin: olctugu sey "inis ne kadar surdu"
        % degil, "tabanda ne kadar bekledik". Tavan hala inerken gecen sure
        % giris hizina bagli ve dengeyle ilgisi yok.
        if at_floor
            floor_dwell = floor_dwell + TS_POS;
        else
            floor_dwell = 0.0;
        end

        % Birakma icin tavan HER HALUKARDA tabana varmis olmali; ikinci kosul
        % iki terimlidir (madde (R), Adim 38):
        %   - hiz dengenin disindaki esigin altina indi (normal, hizli yol), YA DA
        %   - tabanda floor_dwell kadar beklendi (aero-bagimsiz emniyet;
        %     terminal hiz esigin ustunde kalirsa cikis yine de olur).
        if at_floor && (v_h < BT_RELEASE_V || floor_dwell >= BT_FLOOR_DWELL)
            state = BT_BRAKE;
        end

    case BT_BRAKE
        % Tavan KALDIRILMAZ, YUKSELTILIR. Trim'in ustune cikarmak yaw'a
        % modulasyonunu geri verir (delta1 tilt.min'de oldugu icin diferansiyel
        % = delta0, artik [0, brake_ceil] arasinda serbest); bir tavanin VAR
        % olmaya devam etmesi ise Fz kacisinin yeniden baslamasini engeller --
        % bu hizda tamamen birakmak manevranin kendini bozmasina yol acti.
        tilt_ceil = BT_BRAKE_CEIL;
        pitch_sp  = brake_pitch(v_fwd);
        % Madde (S): ISARETLI govde ileri hizi. Buyukluk esigi, manevranin
        % kaldiramadigi yanal bir bilesen yuzunden sonsuza kadar saglanmayabilir
        % -- olculdu ve arac GERI kacti. u < 0 de gecerli bir cikistir:
        % BRAKE'in isi ileri hizi bitirmekti, bitti.
        if v_fwd < BT_HANDOFF_V
            state = BT_HANDOFF;
        end

    case BT_HANDOFF
        % Kisit ARTIK gercekten kalkabilir: handoff_v altinda kacisi surecek
        % kanat tasimasi kalmadi, ve cozdugu problem gectikten sonra yururlukte
        % kalan bir kisit tam olarak bu dosyanin uyardigi hatadir.
        tilt_ceil    = TILT_MAX;
        pitch_sp     = brake_pitch(v_fwd);
        req_pos_hold = true;

    otherwise
        state       = BT_IDLE;
        tilt_ceil   = TILT_MAX;
        floor_dwell = 0.0;
end

state_out = [state; tilt_ceil; floor_dwell];

end


function pitch_sp = brake_pitch(v_fwd)
%BRAKE_PITCH  Durus trimi + ILERI hizla orantili frenleme payi.
%
% AYRISTIRILDI (Adim 37). Onceki yasa yalnizca `pitch_max * v_h/brake_v_full`
% idi, yani hiz sifira giderken burun yukari acisi da SIFIRA gidiyordu. Bu
% yanlisti: aracin yenmesi gereken ILERI kuvvet hizla azalmiyor, SABIT. delta1
% ve delta2 tilt.min'de cakili oldugundan (madde (P), tek yonlu tilt araligi)
% yaw trimi delta0'i 10-15 deg'de tutuyor ve bu surekli ~3.1-4.1 N ileri itki
% uretiyor. Yani "yerinde durmak" bile bir burun yukari acisi gerektiriyor:
% asin(fx_trim/(m*g)) = 3.39 deg.
%
% Eski yasa iki bagimsiz SITL ucusunda manevrayi devir hizinin ustunde takti
% (3.2-3.5 m/s'de 90+ s kararli denge); pitch_max = 4 deg ile duzeltildikten
% sonra bile bir ucus 4.9 m/s'de takildi -- orada tilt 14.86 deg'e oturmus,
% ileri kuvvet 4.11 N idi, yani frenleme otoritesi yapisal itkinin ta
% kendisiyle AYNI mertebede.
%
% MADDE (S) DUZELTMESI (Adim 39): marj artik BUYUKLUKLE degil ISARETLI govde
% ileri hiziyla soner. Eski hali v_h okuyordu, yani arac ileri yonde durup GERI
% hizlanmaya basladiginda marj yeniden BUYUYOR ve araci daha da geri itiyordu
% -- olculdu: 12.8 m/s'ye geri kacis, tek ucusta bes kez. max(0,.) sayesinde
% v_fwd <= 0 iken marj tam sifirdir ve geriye yalnizca durus trimi kalir.
FX_TRIM       = 2.9;          % p.ctrl.fx_trim  (N)
MASS          = 5.0;          % p.m
G             = 9.81;         % p.g
BT_PITCH_MAX  = 4.0*pi/180;   % p.bt.pitch_max     = deg2rad(4)
BT_BRAKE_V_FULL = 3.0;        % p.bt.brake_v_full  (m/s)

pitch_trim = asin(min(1.0, FX_TRIM / (MASS * G)));
pitch_sp   = pitch_trim + BT_PITCH_MAX * min(1.0, max(0.0, v_fwd) / BT_BRAKE_V_FULL);
end
