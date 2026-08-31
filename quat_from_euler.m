function q = quat_from_euler(phi, theta, psi)
%QUAT_FROM_EULER  ZYX Euler -> kuaterniyon [q0;q1;q2;q3] (Hamilton, body->earth).
cy=cos(psi/2); sy=sin(psi/2);
cp=cos(theta/2); sp=sin(theta/2);
cr=cos(phi/2); sr=sin(phi/2);
q0 = cr*cp*cy + sr*sp*sy;
q1 = sr*cp*cy - cr*sp*sy;
q2 = cr*sp*cy + sr*cp*sy;
q3 = cr*cp*sy - sr*sp*cy;
q = [q0;q1;q2;q3];
end
