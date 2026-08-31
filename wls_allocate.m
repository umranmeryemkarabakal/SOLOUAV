function [du, sat_flag, n_iter] = wls_allocate(G, nu_des, du_min, du_max, Ws, Wu, du_pref, max_iter)
%WLS_ALLOCATE  Agirlikli en kucuk kareler kontrol tahsisi (active-set).
%
%   min_{du}  || Ws*(G*du - nu_des) ||^2 + || Wu*(du - du_pref) ||^2
%   s.t.      du_min <= du <= du_max
%
% Bodson (2002) "sequential least squares" / PX4 control_allocator WLS
% algoritmasinin kompakt bir uygulamasi: doygunlasan aktuatorler sirayla
% "sabit" kumeye alinir, kalan serbest aktuatorler icin problem yeniden
% cozulur ve hedef, sabitlenenlerin katkisi kadar duzeltilir.
%
% Girisler:
%   G        (m x n)  anlik etkinlik Jacobian'i (bkz. effectiveness_matrix)
%   nu_des   (m x 1)  istenen sanal kontrol artisi
%   du_min/max (n x 1) aktuator artis kutu kisitlari (hiz*Ts ve/veya mutlak sinir)
%   Ws       (m x m)  sanal kontrol onceligi (buyuk = sikica izlenir)
%   Wu       (n x n)  aktuator kullanim cezasi / tercih agirligi
%   du_pref  (n x 1)  tercih edilen artis (genelde 0 = minimum efor)
%
% Ciktilar:
%   du        (n x 1) hesaplanan aktuator artisi (kisitlar icinde)
%   sat_flag  (n x 1) logical, doyuma ulasan aktuatorler
%   n_iter    kac active-set iterasyonu calisti

if nargin < 8 || isempty(max_iter)
    max_iter = 2*numel(du_pref) + 2;
end

n = numel(du_pref);
free = true(n,1);
du   = du_pref(:);

WsWs = Ws'*Ws;
WuWu = Wu'*Wu;

n_iter = 0;
for it = 1:max_iter
    n_iter = it;
    idx_f = find(free);
    idx_s = find(~free);

    if isempty(idx_f)
        break;
    end

    nu_resid = nu_des - G(:,idx_s)*du(idx_s);

    Gf = G(:,idx_f);
    H  = Gf' * WsWs * Gf + WuWu(idx_f, idx_f);
    rhs = Gf' * WsWs * nu_resid + WuWu(idx_f, idx_f) * du_pref(idx_f);

    % Kotu kosullanmaya karsi kucuk regularizasyon (Jacobian sutun-yetersiz
    % olabilir, orn. bir rotor itkisi ~0 iken tilt hassasiyeti de ~0 olur).
    H = H + 1e-9*eye(numel(idx_f));

    du_free = H \ rhs;
    du(idx_f) = du_free;

    viol_hi = idx_f(du(idx_f) > du_max(idx_f));
    viol_lo = idx_f(du(idx_f) < du_min(idx_f));

    if isempty(viol_hi) && isempty(viol_lo)
        break;
    end

    du(viol_hi) = du_max(viol_hi);
    du(viol_lo) = du_min(viol_lo);
    free(viol_hi) = false;
    free(viol_lo) = false;
end

sat_flag = ~free;

end
