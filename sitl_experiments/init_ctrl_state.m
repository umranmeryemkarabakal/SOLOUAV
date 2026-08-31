function ctrl_state = init_ctrl_state()
%INIT_CTRL_STATE  indi_attitude_controller icin baslangic durumu.
ctrl_state.z1          = zeros(3,1);   % LESO: omega tahmini [p;q;r]
ctrl_state.z2          = zeros(3,1);   % LESO: bozucu acisal ivme tahmini d_hat
ctrl_state.d_hat       = zeros(3,1);   % son yayinlanan d_hat (LESO adimlari arasi tutulur)
ctrl_state.prev_u_leso = zeros(3,1);   % ESO'nun "girisi" (bir onceki omega_dot_des)
ctrl_state.leso_accum  = 0;            % s, LESO decimation birikici
ctrl_state.prev_du_tilt = zeros(3,1);  % tiltjerk (Adim 95/96) icin bir onceki tick'in tilt du'su
end
