function Mv = surf_virtual_map(p)
%SURF_VIRTUAL_MAP  Sanal yuzey aktuatorlerinden fiziksel servo sapmalarina.
%
%   surf_phys(5x1) = Mv * a_virt(3x1)
%
%   a_virt = [a_ail; a_ele; a_rud]   (rad)
%   surf   = [s0 sol elevon; s1 sag elevon; s2 sol elevator;
%             s3 sag elevator; s4 rudder]  (rad)
%
% NEDEN SANAL AKTUATOR (Adim 46, madde V) -- tam gerekce effectiveness_matrix.m
% basliginda. Ozet: Adim 31 Faz 0 olctu ki TILT'I SUREN SEY Fz TALEBIDIR, yani
% tahsisata ucuz bir Fz aktuatoru verilirse tilt mekanizmasi olur. Adim 45'in
% 1. denemesi bes yuzeyi BAGIMSIZ aktuator yapti; WLS elevonlari SIMETRIK
% surerek (0.05 m kollu, yani neredeyse saf bir tasima cihazi) Fz talebini
% karsiladi ve tilt 44 -> 13.3 dereceye coktu.
%
% Bu esleme o serbestligi YAPISAL olarak kaldirir: simetrik elevon (flap)
% Mv'nin goruntu uzayinda DEGILDIR, yani WLS onu hicbir agirlikta secemez.
% Kalan uc yon ise ya tam kuvvet-serbest (aileron, rudder) ya da durust bir
% 0.70 m kuyruk kolu (elevator).
%
% ISARET: iki elevon eklem ekseni de SDF'de (0,1,0) yonunde, yani ayni komut
% ayni fiziksel donusu verir; roll uretmek icin ZIT surulmeleri gerekir. SDF'in
% kendi yorumu: "The x8 elevons act as ailerons here (roll only) - pitch is the
% tailplane's job."

Mv = [ +1   0   0;     % s0  sol elevon    = +a_ail
       -1   0   0;     % s1  sag elevon    = -a_ail
        0  +1   0;     % s2  sol elevator  = a_ele
        0  +1   0;     % s3  sag elevator  = a_ele
        0   0  +1 ];   % s4  rudder        = a_rud

if nargin > 0 && size(Mv,1) ~= p.surf.n
    error('surf_virtual_map:boyut', ...
          'Mv %d satirli ama p.surf.n = %d', size(Mv,1), p.surf.n);
end

% ETKINLESTIRME MASKESI (p.wls.surf_enable, 3x1 logical). Kapatilan bir eksen
% tahsisattan TAMAMEN CIKARILIR -- agirligi buyutulerek "bastirilmaz". Fark
% onemli: cok buyuk bir Wu, WLS Hessian'ini tekillestirir (olculdu: RCOND
% 1e-18) ve cozum sayisal cope doner; sutunu kaldirmak ise tam olarak "o
% aktuator yok" demektir. Ablasyon olcumleri ve PX4'te kademeli devreye alma
% bu maskeyi kullanir.
if nargin > 0 && isfield(p, 'wls') && isfield(p.wls, 'surf_enable')
    Mv = Mv(:, logical(p.wls.surf_enable));
end

end
