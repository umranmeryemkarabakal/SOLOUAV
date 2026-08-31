function att = sf_quat_to_euler(q)
%SF_QUAT_TO_EULER  MATLAB Function blok icerigi: kuaterniyon -> ZYX Euler.
%#codegen
q0=q(1); q1=q(2); q2=q(3); q3=q(4);
phi   = atan2(2*(q0*q1+q2*q3), 1-2*(q1^2+q2^2));
theta = asin(max(-1,min(1, 2*(q0*q2-q3*q1))));
psi   = atan2(2*(q0*q3+q1*q2), 1-2*(q2^2+q3^2));
att = [phi;theta;psi];
end
