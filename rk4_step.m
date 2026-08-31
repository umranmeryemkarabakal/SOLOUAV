function x_next = rk4_step(deriv_fun, x, dt)
%RK4_STEP  Sabit adimli 4. derece Runge-Kutta entegrasyonu (tek adim).
%   deriv_fun: @(x) -> xdot   (kontrol/bozucu girisleri kapatma icinde sabit)
k1 = deriv_fun(x);
k2 = deriv_fun(x + dt/2*k1);
k3 = deriv_fun(x + dt/2*k2);
k4 = deriv_fun(x + dt*k3);
x_next = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end
