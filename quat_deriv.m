function qdot = quat_deriv(q, omega)
%QUAT_DERIV  Kuaterniyon kinematigi: qdot = 0.5*q(x)[0;omega] (govde hizlari).
q0=q(1); q1=q(2); q2=q(3); q3=q(4);
p=omega(1); qq=omega(2); r=omega(3);
qdot = 0.5*[ -q1*p - q2*qq - q3*r;
              q0*p + q2*r  - q3*qq;
              q0*qq - q1*r + q3*p;
              q0*r  + q1*qq - q2*p ];
end
