# Modeling and Control of a Tilt tri-rotor Airplane

| | |
|---|---|
| **Yazarlar** | Duc Anh Ta, Isabelle Fantoni, Rogelio Lozano |
| **Yayın** | American Control Conference (ACC 2012), Montreal, Kanada, Haziran 2012, s. 131–136 |
| **Arşiv** | HAL Id: [hal-00767341](https://hal.science/hal-00767341v1) (19 Aralık 2012) |
| **Kurum** | Heudiasyc Laboratuvarı, Université de Technologie de Compiègne / CNRS |
| **Kaynak dosya** | `avion_x4_vf.pdf` (7 sayfa; 1. sayfa HAL kapağı) |

> **Not:** Bu dosya PDF'in markdown'a çevrilmiş tam metnidir. Denklemler LaTeX olarak
> yazıldı ve hem sayfa görüntüsünden hem `pdftotext` çıkarımından çapraz doğrulandı.
> Şekiller (Fig. 1–12) markdown'a taşınmadı; yerleri ve açıklamaları korundu, görselleri
> için PDF'e bakın.

---

## Abstract

A helicopter offers the capability of hover, slow forward displacement, vertical
take-off and landing while a conventional airplane has the performance of fast forward
movement, long reach and superior endurance. The aim of this paper is to present the
modelling and control of a tilt tri-rotor UAV's configuration that combines the
advantages of both rotary wing and fixed wing vehicle.

---

## I. INTRODUCTION

Nowadays, Unmanned Aerial Vehicle (UAV) possess several applications in the military
and civil domains: surveillance, detection, reconnaissance, search and rescue
operations... The objective of this work is to design a miniature aircraft which has the
performance of an airplane for the forward flight and the capacity of a helicopter for
the hover flight. In the past, this configuration has motivated lots of researchers
because this type of aircraft does not require a runway and the ability of hover makes it
very useful for aerial surveillance missions.

In [4], ongoing works on the development of a quadrotor aerial vehicle having a tilt-wing
mechanism using an LQR and a sliding mode controller for the stabilization of the
attitude and the altitude are presented via vertical take-off and landing simulations.
Other simulation results of the transition to level flight of a tilt-wing aircraft which
uses a gain-scheduled, multi-variable $H_\infty$ control law are obtained in [7]. More
longitudinal dynamic simulation results are found in [6], in which non-linear approaches
are used for the autonomous transition of two vertical/short take-off and landing
aircraft (a fixed-wing fighter equipped with a vectored thrust and lift fan and a tilt
rotor). In [1], we proposed a convertible tail-sitter UAV which has the capability of
transition between the vertical and horizontal flight. However, in order to make a
transition, the whole fuselage of the convertible airplane must rotate to the horizontal.

In this paper, we propose a configuration of the tilt-rotor mini aircraft capable of
performing the hovering, transition, vertical take off and landing. In the phases of
transition, the fuselage still remains in the horizontal, then the transition is more
stable than the convertible airplane. The detailed mathematical model is obtained by the
equations of Newton-Euler and the aerodynamic formulas. In terms of control, we propose a
simple nonlinear PID control using Neural Network to stabilize the attitude and the
altitude and a motion profile generator to realize the phases of take-off, landing and
transition.

The paper is organized in five sections. The second section is devoted to the
presentation of the platform. The mechanical structure and the aerodynamic model of the
aircraft will be presented in the third section. The fourth section contains our
contributions, namely the presentation of a control strategy to achieve a longitudinal
flight path. Simulation results are presented in the fifth section. Finally, the paper
ends with some conclusions.

*Fig. 1. Prototype of the Tilt tri-rotor Airplane*

---

## II. TILT TRI-ROTOR AIRPLANE

### 1) Platform

The tilt tri-rotor airplane has been built and completely developed at Heudiasyc
laboratory (see Figure 1). It has a total weight of about **1250 g** and uses one
**2200 mAh LiPo battery**.

In the vertical flight (Figure 2), the altitude is controlled by collective thrust of the
three brushless electric DC Motors. The roll motion is achieved with the difference of
thrust between the two front motors: the left rotates in the clockwise direction while
the right in counterclockwise direction. The rear motor compensates the force generated
by two front motors to stabilize the pitch angle of the airplane.

After take-off, the two front motors tilt forward thanks to an electromechanical system.
The rear motor reduces its rotational speed according to the tilt angle $\delta$ of the
two font motors. When $\delta$ reaches zero degree, i.e. the axis of both front motors
coincides with the axis of the fuselage, the rear motor will stop completely. The
airplane then flies horizontally like a conventional aircraft (Figure 3) with a cruise
speed to the target. In horizontal flight, the roll motion is controlled by moving the
ailerons differentially, the pitch motion is controlled by moving the elevator and the
yaw motion is controlled by moving the rudder. When the vehicle reaches the target, the
two front motors tilt to the vertical and the vehicle returns to the helicopter mode and
lands to the ground vertically. **Note that the rotational speeds of both front motors
are equal in the phases of transition and horizontal flight.**

### 2) Embedded Control System

An *Inertial Measurement Unit* (Microstrain, *3DM-GX3-25*) is used to obtain the
attitude. It weighs just 11.5 g and is composed of three triaxial accelerometers and
angular rate gyros as well as three orthogonal magnetometers. A *Digital Signal
Controller* (Microchip, *dsPIC33FJ256GP710*) was selected to implement the Embedded
Control System. The measurement used for the altitude is obtained from the low cost
ultrasonic sensor (*SRF10*) which has a range of 10 cm to 3 m.

*Fig. 2. Vertical flight (helicopter mode)* — *Fig. 3. Horizontal flight (airplane mode)*

---

## III. MATHEMATICAL MODEL

From the design model of the tilt rotor airplane described above, the *Newton-Euler*
equations of motion describing the six degrees of freedom of the system can be separated
into translational motion ($\Sigma_P$) and rotational motion ($\Sigma_A$):

$$
\Sigma_P : \begin{cases}
\dot{\vec{p}}^{\,f} = R\,\vec{v}^{\,b} \\[4pt]
\dot{\vec{v}}^{\,b} = R^{T}\vec{g}^{\,f} + \dfrac{1}{\bar{m}}\vec{F}^{\,b}_{A,T} - \vec{\omega}\times\vec{v}^{\,b}
\end{cases}
\tag{1}
$$

$$
\Sigma_A : \begin{cases}
\dot{\Theta} = J(\Theta)\,\vec{\omega} \\[4pt]
I\dot{\vec{\omega}} = -\vec{\omega}\times I\vec{\omega} + \vec{\Gamma}_{A,T}
\end{cases}
\tag{2}
$$

where: $R$ is the rotation matrix from $E_b$ to $E_f$; $\vec{p}^{\,f}$ represents the
position of the airplane in $E_f$; $\vec{v}^{\,b}$ is the linear velocity vector
presented in $E_b$; $\bar{m} = diag(m) \in \mathbb{R}^{3\times3}$ where
$m\ (\simeq 1250\,g)$ is the weight of the vehicle; $\vec{g}^{\,f}$ is the vector of the
gravity acceleration in $E_f$; $\vec{\omega}$ is the angular velocity vector;
$\Theta = \begin{bmatrix}\phi & \theta & \psi\end{bmatrix}$ is the vector of the Euler
angles; $I \in \mathbb{R}^{3\times3}$ represents the inertial matrix of the airplane;
$\vec{F}^{\,b}_{A,T} = \vec{F}_A + \vec{F}^{\,b}_T$ is the vector of the external
aerodynamic forces and the thrusts expressed in the coordinate $E_b$;
$\vec{\Gamma}_{A,T} = \vec{\Gamma}_A + \vec{\Gamma}_T$ is the external torque vector;
$J(\Theta)$ is the Jacobian matrix defined as:

$$
J(\Theta) = \begin{pmatrix}
1 & \tan\theta\sin\phi & \tan\theta\cos\phi \\
0 & \cos\phi & -\sin\phi \\
0 & \dfrac{\sin\phi}{\cos\theta} & \dfrac{\cos\phi}{\cos\theta}
\end{pmatrix}
\tag{3}
$$

Note that the singularity of the attitude representation by Euler angles can be
eliminated if the airplane's orientation in space is limited. In our case, since the
fuselage is always in horizontal ($-45^\circ \leqslant \theta \leqslant 45^\circ$), the
Euler angles are used to represent the attitude due to their simplicity.

### Actuator forces and torques

At first, we analyse the forces $\vec{F}^{\,b}_T$ and the torques $\vec{\Gamma}_T$
generated by the actuators. Each motor produces a force $T_i$ parallel to its axis of
rotation, and a reactive torque $Q_i$ opposite to the direction of rotation. Since the
rotor speed reaches very high values (more than *200 rad/sec*), we can approximate the
forces $T_i$ and the reactive torques $Q_i$ generated by the motors:

$$
T_i = b_i s_i^2 \ ;\qquad Q_i = k_i s_i^2
\tag{4}
$$

where $s_i$ is the rotational speed of rotor; $b_i$ and $k_i$ are two positive parameters
depending on the density of air, the radius, the shape, the pitch angle of the blade and
other factors [11]. The combination of the forces $T_i$ and the reactive torques $Q_i$ is
given by:

$$
\begin{cases}
\vec{F}^{\,b}_T = \begin{bmatrix} (T_1+T_2)\cos\delta & 0 & (T_1+T_2)\sin\delta + T_3 \end{bmatrix}^{T} \\[8pt]
\vec{\Gamma}_T = \begin{bmatrix}
(T_2-T_1)\,l_m\sin\delta + (Q_2-Q_1)\cos\delta \\
T_3 l_2 - (T_1+T_2)\,l_1\sin\delta \\
(T_1-T_2)\,l_m\cos\delta + (Q_2-Q_1)\sin\delta + Q_3
\end{bmatrix}
\end{cases}
\tag{5}
$$

In the **vertical mode** (tilt angle $\delta = 90^\circ$, Figure 2), the equation (5)
becomes:

$$
\begin{cases}
\vec{F}^{\,b}_T = \begin{bmatrix} 0 & 0 & T_1+T_2+T_3 \end{bmatrix}^{T} \\[8pt]
\vec{\Gamma}_T = \begin{bmatrix}
(T_2-T_1)\,l_m \\
T_3 l_2 - (T_1+T_2)\,l_1 \\
(Q_2-Q_1) + Q_3
\end{bmatrix}
\end{cases}
\tag{6}
$$

In the **horizontal mode** ($\delta = 0^\circ$, $T_3 = Q_3 = 0$, Figure 3), the equation
(5) becomes:

$$
\begin{cases}
\vec{F}^{\,b}_T = \begin{bmatrix} (T_1+T_2) & 0 & 0 \end{bmatrix}^{T} \\[6pt]
\vec{\Gamma}_T = \begin{bmatrix} (Q_2-Q_1) & 0 & (T_1-T_2)\,l_m \end{bmatrix}^{T}
\end{cases}
\tag{7}
$$

### Aerodynamic forces and torques

The aerodynamic forces $\vec{F}^{\,b}_A$ and torques $\vec{\Gamma}_A$ depend on the
working mode of the airplane. In all phases of flight, we assume that there is no wind in
the environment. So when the airplane flies vertically, there are no aerodynamic forces
and torques because the speed of the vehicle is minor. However in transition and
horizontal phases, the air velocity will be equal to the relative velocity of the UAV but
in the opposite direction: $\vec{V}_{air} = -\vec{V}$. The vector of the air velocity
creates with the airplane an angle of attack $\alpha$ (Figure 3). In these modes, the
aerodynamic forces and torques are given by:

$$
\begin{cases}
\vec{F}^{\,b}_A = \begin{bmatrix}
-\left(T^b_w + T^s_{ar} + T^s_{al} + T^b_e + T^s_r\right) \\
P^s_r \\
P^b_w - P^s_{ar} + P^s_{al} + P^b_e + P^s_e
\end{bmatrix} \\[18pt]
\Gamma_A = \begin{bmatrix}
P^s_{ar}l_m + P^s_{al}l_m &
\left(P^b_e + P^s_e\right)l_e - P^b_w l_w &
P^s_r l_r
\end{bmatrix}^{T}
\end{cases}
\tag{8}
$$

Note that in order to balance in the longitudinal dynamics, the distances $l_e$ and $l_w$
must satisfy this condition [10]:

$$
P^b_e l_e = P^b_w l_w \ \rightarrow\ \frac{l_e}{l_w} = \frac{P^b_w}{P^b_e} \approx \frac{P_w}{P_e} \approx \frac{S_w}{S_e}
\tag{9}
$$

Then:

$$
(8) \rightarrow
\begin{cases}
\vec{F}^{\,b}_A \approx \begin{bmatrix} -\left(T^b_w + T^b_e\right) & 0 & P^b_w + P^b_e \end{bmatrix}^{T} \\[6pt]
\Gamma_A \approx \begin{bmatrix} P^s_{ar}l_m + P^s_{al}l_m & P^s_e l_e & P^s_r l_r \end{bmatrix}^{T}
\end{cases}
\tag{10}
$$

where $P^s_i = \tfrac{1}{2}\rho v_{in}^2 S^s_i C^{P_s}_i$ and
$T^s_i = \tfrac{1}{2}\rho v_{in}^2 S^s_i C^{T_s}_i$ are respectively the lift force and
the drag force created by slipstream with $i = al, ar, e, r$ (left aileron, right
aileron, elevator and rudder); $\rho$ is the density of air;
$C^{P_s}_i = C^{P_s}_{i_\delta}\delta_i$; $C^{T_s}_i = C^{T_s}_{i_\delta}\delta_i$;
$\delta_i$ and $S_i$ are the deflection angle and the area of control surface;

$$
v_{in} = \sqrt{\frac{2T_i}{\rho A^s_M} + V^2}
$$

is the air velocity created by the propeller (inflow or induced velocity).

We have used the same analysis as in [1] and aerodynamics formulas in [9].

Then, in the vertical mode ($\delta = 90^\circ$) the external torque vector
$\Gamma_{A,T}$ is given by:

$$
\vec{\Gamma}_{A,T_V} = \vec{\Gamma}_T = \begin{bmatrix}
b\left(s_2^2 - s_1^2\right)l_m \\
b s_3^2 l_2 - b\left(s_1^2 + s_2^2\right)l_1 \\
k\left(s_2^2 - s_1^2\right) + k s_3^2
\end{bmatrix}
\tag{11}
$$

In the transition mode, the external torque and external force depend on several
aerodynamic parameters. Due to space limitations, the expressions are given in [2].

---

## IV. CONTROL STRATEGY

In the longitudinal dynamics, the tilt-rotor airplane performs the following phases of
flight: vertical take-off, hover mode, transition to the horizontal, horizontal flight,
transition to the vertical and vertical landing.

### A. Control Law

#### 1) Altitude Control

The control of the altitude is used when the airplane flies vertically but it is not used
in the transition and horizontal flight. Extracting the translational motion on the $z$
axis of $E_f$ from equation (1) yields:

$$
m\ddot{z} = T\cos\theta\cos\phi - mg
\tag{12}
$$

where $T$ is the total thrust force of the three motors. A simple PD control can guarantee
the convergence of the altitude to the desired altitude $z_d$:

$$
T = T_1 + T_2 + T_3 = \frac{k^z_p(z_d - z) + k^z_d(\dot{z}_d - \dot{z}) + mg}{\cos\theta\cos\phi}
\tag{13}
$$

but for security reasons and in order to improve the performance, a PID control with a
saturation is used:

$$
T = \frac{sat_T\left(k^z_p(z_d - z) + k^z_d(\dot{z}_d - \dot{z}) + k^z_i\int (z_d - z)\right) + mg}{\cos\theta\cos\phi}
\tag{14}
$$

$$
T = \frac{sat_T\left(k^z_p e^z_p + k^z_i e^z_i + k^z_d e^z_d\right) + mg}{\cos\theta\cos\phi}
\tag{15}
$$

where $sat_M(x)$ is a nonlinear saturated sigmoid function:

$$
\mathrm{sat}_M(x) = \frac{M\left(1 - e^{-2x/M}\right)}{\left(1 + e^{-2x/M}\right)}
\tag{16}
$$

Note that $\lim_{x\to\pm\infty}(\mathrm{sat}_M(x)) = M\,sign(x)$, where $sign(x)$ is the
signum function defined as $sign(x) = \{1 \text{ if } x \geqslant 0;\ -1 \text{ if } x < 0\}$,
then $M$ is the boundary saturation. The sigmoid function $\mathrm{sat}_M(x)$ becomes
linear when $M$ goes to infinity.

#### 2) Attitude Control

The attitude control is used to stabilize the UAV when it takes off and lands vertically.
However in the transition phase, two front motors must tilt forward with the same
rotational speed. So only the pitching movement is stabilized in the transition phases,
the rolling and yawing motions are controlled manually. The bounded attitude control PID
is used:

$$
\Gamma_j = sat_{\Gamma_j}\left(k^j_p e^j_p + k^j_i e^j_i + k^j_d e^j_d\right)
\tag{17}
$$

with $j = \phi, \theta, \psi$ and $\Gamma_j$ are the boundary of the control torques
around the axes $x, y, z$.

#### 3) Adaptative Control Using Neural Network

A modification of control parameters for PID control must be adjusted to realize more
accurate control. It is obvious that the PID control method is limited because the
parameters $k_p, k_d, k_i$ are constant. This means that the control output is not
adaptable and optimal in any case. An intelligent adaptative control, which has the
adaptability of control parameters to minimize the position error, is used. With the
capacity of learning and adaptability of neural network, the controller can solve these
problems. The parameters PID will be tuned adaptively and optimally in order to minimize
the position error with respect to external perturbations.

Figure 4 shows the structure of the PID control using neural network for one degree of
freedom. Here we take the pitch angle for example, the control method will be applied
similarly for other degrees of freedom (altitude, roll and yaw). The block diagram of
neural network is shown in Figure 5. Here, $k_p, k_d, k_i, e_p, e_i$ and $e_d$ are
proportional, integral and derivative gains, the system error between desired and actual
output, the integral of the system error and the difference of the system error,
respectively. Neural networks are trained by the conventional back propagation algorithm
to minimize the system error between the desired and actual output. In Figure 5, the
input signal of the sigmoid function in the output layer becomes:

$$
x(k) = k^\theta_p(k)e^\theta_p(k) + k^\theta_i(k)e^\theta_i(k) + k^\theta_d(k)e^\theta_d(k)
\tag{18}
$$

where,

$$
\begin{cases}
e^\theta_p(k) = \theta_d(k) - \theta(k) \\[6pt]
e^\theta_i(k) = \displaystyle\sum_{n=1}^{k} e^\theta_p(n)\,\Delta T \\[10pt]
e^\theta_d(k) = \dfrac{e^\theta_p(k) - e^\theta_p(k-1)}{\Delta T}
\end{cases}
\tag{19}
$$

$\Delta T$ is the sampling time, $k$ is the discrete sequence; $\theta_d(k)$ and
$\theta(k)$ are the desired and actual pitch angle. To tune the gains of the PID
controller $k_p, k_i, k_d$, the Gradient Descent method using the following equation was
applied:

$$
k^\theta_j(k+1) = k^\theta_j(k) - \eta^\theta_j \frac{\partial E(k)}{\partial k^\theta_j}
\tag{20}
$$

for $j = p, i, d$ where $\eta^\theta_p, \eta^\theta_i, \eta^\theta_d$ are learning rates
determining the convergence speed, and $E(k)$ is a cost function. The PID parameters are
updated at the step $k$ in order to decrease the cost function. Since the control purpose
is to decrease the error $e^\theta_p(k)$ therefore we choose the cost function
$E(k) = \tfrac{1}{2}\left(\theta_d(k) - \theta(k)\right)^2$. According to [3], the
parameters are updated for $j = p, i, d$:

$$
k^\theta_j(k+1) = k^\theta_j(k) + \eta^\theta_j e^\theta_p(k) e^\theta_j(k) \frac{\partial \Gamma_\theta(k)}{\partial x}
\tag{21}
$$

where:

$$
\frac{\partial \Gamma_\theta(k)}{\partial x}
= \frac{\partial\left(\mathrm{sat}_{\Gamma_\theta} x(k)\right)}{\partial x}
= \frac{\partial}{\partial x}\left(\frac{\Gamma_\theta\left(1 - e^{-2x(k)/\Gamma_\theta}\right)}{\left(1 + e^{-2x(k)/\Gamma_\theta}\right)}\right)
= \frac{4e^{-2x(k)/\Gamma_\theta}}{\left(1 + e^{-2x(k)/\Gamma_\theta}\right)^2}
\tag{22}
$$

Then the PID parameters are tuned at every loop.

We can notice that when the learning rates $\eta_j$ (with $j = p, i, d$) are minor, the
convergence speed will be slow, which will decrease the adaptation of system. But if a
high $\eta_j$ is used, the Neural Network adapts more quickly but the cost function will
overpass more easily the minimum value and the gains cannot converge to the needed values.
In order to improve the Gradient Descent algorithm, the learning rates $\eta_j$ are also
updated adaptively as follow:

$$
\eta_j(k+1) = \left(1 + \varsigma_j\, sign\left(E(k+1) - E(k)\right)\right)\eta_j(k)
\tag{23}
$$

where $\varsigma_j$ are positive parameters.

*Fig. 4. Structure of the nonlinear PID controller using neural network.*
*Fig. 5. Block diagram of neural network.*

### B. Trapezoidal Trajectory of Velocity - Servo Control

In the case when the vehicle autonomously takes off, lands and makes a transition, the
controller requires the next desired altitude and the next desired tilt angle. Then a
trajectory generation algorithm must be used for optimum motion control. We have used the
same motion profile algorithm that controls the speed and acceleration as in [1].

The trajectory generator will produce trapezoidal shaped velocity curves for a long
movement and triangular curves for a short movement where maximum velocity was not
reached.

The control law of the altitude will force the vehicle to track the new desired altitude
created by the trajectory generator in order to realize the process of take-off and
landing. Figure 6 shows the rotor's tilt angle generated by the motion profile algorithm
and its adaptation when the airplane makes transition to the horizontal flight.

*Fig. 6. Desired tilt angle generated by the motion profile algorithm and its adaptation*

During the transition to the level flight when both front rotors start tilting forward,
the force of motors on the $x$ axis appears and then the aircraft speed increases. The
aerodynamic lift force appears, however we can not measure or estimate this force. So the
control law of the pitching movement must be used. From (5) the pitching moment equation
is:

$$
\Gamma_{C_\theta} = T_3 l_2 - (T_1 + T_2)\,l_1 \sin\delta
\tag{24}
$$

where $\Gamma_{C_\theta}$ is the pitching torque deduced from the attitude control law to
stabilize the aircraft, $T_1 = T_2$ are the thrusts of two front motors which are given by
the operator via R/C receiver. By using this method, the operator can adjust the UAV's
speed during the transition. Knowing that the tilt angle $\delta$ decreases from
$90^\circ$ to $0^\circ$, so the third motor's speed must vary according to the equation:

$$
T_3 = \frac{\Gamma_{C_\theta} + (T_1 + T_2)\,l_1\sin\delta}{l_2}
\tag{25}
$$

While the force generated by the motors in the $z$ axis decreases ($T_3$ and
$(T_1+T_2)\sin\delta$), the force in the $x$ axis will increase. Therefore the aerodynamic
force $F^z_A$ at the moment will increase in order to compensate the reduction of $F^z_T$
thanks to the high velocity of the vehicle.

---

## V. SIMULATION RESULTS

### A. Longitudinal Flight Simulation

In this section, simulation results from *Matlab-Simulink* and *MSC. Visual Nastran 4D*
simulator are presented to demonstrate the performance of the control strategy. In this
simulation, we assume that the airplane is stable in lateral dynamics and the reaction of
the tilt angle's servo motor is much faster than the vehicle's dynamic. It means that we
exam here only the longitudinal dynamics and the tilt angle's servo adapts immediately
with respect to the desired tilt angle.

According to the logics of transition (1 = transition to level flight, 0 = transition to
hover) and of vertical flight (1 = take off, 0 = landing) (Figure 8), we can divide the
phases of flight as follows:

**(1). Take-off** *(from the beginning to second 3)*: Initially, the vehicle takes off
automatically up to 2 m, the altitude velocity has a trapezoidal form, it accelerates and
reduces the speed when the vehicle reaches 2 m.

**(2). Transition to the level flight** *(from second 3.5 to 8.5)*: the airplane makes the
transition in second 3.5, the tilt angle varies from $90^\circ$ to $0^\circ$ and the pitch
angle inclines $2^\circ$ in order to have a high angle of attack. This pitch angle can be
increased more but $2^\circ$ is sufficient to prove the performance of the control
strategy. The vehicle loses the altitude at the beginning of the transition because its
low speed cannot permit to create the sufficient lift force. In fact, we can increase the
rotational speed of both front rotors so that the airplane can avoid losing the altitude
during the transition but here they are still kept at the value (349 rad/s) at which they
contribute to the total thrust force equal to the airplane's weight in the vertical
flight (Figure 10). During the transition, the rotation speed of the rear motor reduces to
0 rad/s according to the equation (25) and the airplane accelerates thanks to the increase
of thrust force $F^x_T$ (see Figure 9). The transition terminates at second 8.3 when the
tilt angle reaches $0^\circ$.

**(3). Horizontal flight** *(from second 8.5 to 12.5)*: After the transition, the velocity
still increases to a certain speed although the rotational speed of the both front motors
has been reduced to 270 rad/s (Figure 10). The velocity then converges to a constant value
due to the friction drag. It means that at the moment the longitudinal acceleration of the
airplane is zero. In Figure 11, we can notice that the angle of attack also converges to a
constant value. Despite this small value (approximately $1.5^\circ$), it is enough to
create the lift force thanks to the high speed of the aircraft (approximately 15 m/s)
(note that the nominal airplane's speed is about 13 m/s).

**(4). Transition to the hover flight** *(from second 12.5 to 14.5)*: During the
transition, the rear rotor increases the speed from zero to a necessary value in order to
maintain the pitch angle at $-2^\circ$ while the speed of both front rotors still remains
at 270 rad/s. The velocity of the airplane decreases because the force $F^x_T$ reduces
rapidly (Figure 9). The airplane still has the lift force $F^z_A$ but this lift force also
reduces to zero according to the velocity. When the velocity reaches zero, the desired
pitch angle is zero to stabilize the airplane and after that, the airplane lands to the
ground at the second 34.

**(5) and (6). Vertical flight and landing** *(from second 14.5 to 43)*: Without the
$F^x_T$, the aircraft still flies with cruise speed because it still has the momentum in
horizontal flight. However its velocity reduces because of the drag force and the pitch
angle $-2^\circ$. Therefore the aerodynamic lift force $F^z_A$ also decreases while the
total thrust force of motors $F^z_T$ increases.

Note that during the period of flight, the sum of the aerodynamic lift force $F^z_A$ and
the total thrust force $F^z_T$ is always approximate to the weight of the vehicle
($mg \approx 12.26\,N$) (Figure 9). It guarantees that the airplane cannot fall down. The
control law of the altitude is inactivated during the horizontal flight and the
transition, the profile of the altitude velocity is trapezoidal in the periods of take-off
and landing and there is no overshooting.

*Fig. 7. Longitudinal flight path* — *Fig. 8. State variables of the airplane during the
period of flight* — *Fig. 9. Forces generated in $E_b$ axis* — *Fig. 10. Speed of each
rotor* — *Fig. 11. Angle of attack*

### B. Control performance improved by Neural Network

The Figure 12 shows that the performance of the control was improved by Neural Network
when the vehicle realizes 3 times of take-off and landing from *0 m* to *2 m*.

| Eğri | Koşul |
|---|---|
| (1) | Yörünge üretecinin verdiği istenen konum |
| (2) | Kazançlar **doğru** seçildiğinde irtifa kontrolünün adaptasyonu |
| (3) | Kazançlar **yanlış** seçildiğinde adaptasyon — yörünge üreteci sayesinde kabul edilebilire yakın, ancak başlangıçta statik hatalar var; bunlar integral kontrolle hızla yok ediliyor |
| (4) | Denklem (15)'teki irtifa kontrolü ofseti $mg$ **0.6 mg** olarak değiştirildi (sistem parametrelerinin yanlış kestirimi) ve kazançlar (3)'teki gibi hatalı bırakıldı |
| (5) | (4) ile aynı, ancak irtifa ölçümüne ve $F^z$'ye **sensör gürültüsü ve dış bozucular** eklendi (sıfır ortalamalı, sınırlı gürültü süreçleri; $\sigma_z = 0.03\,m$ ve $\sigma_{F^z} = 4\,N$) |
| (6) | (5) ile aynı koşullar (yanlış kazançlar, kontrol parametreleri, giriş ve çıkış bozucuları) ama kazançlar **Neural Network ile adaptif olarak düzeltildi** |

*Fig. 12. Control improved by Neural Network*

---

## VI. CONCLUSIONS AND FUTURE WORKS

In this work, we have presented the dynamic model of a tilt tri-rotor airplane and a
simple control strategy to achieve a longitudinal flight path. The obtained simulation
results demonstrate that the proposed control strategy could be implemented in reality.
The control law is simple and suitable for embedded applications, it does not require a
high computational cost for control loop. The proposed approach is currently implemented
on the platform and the experimental tests will be presented in a longer version of the
paper.

---

## VII. ACKNOWLEDGMENTS

This work has been supported by the French Armament Procurement Agency (DGA) and the
French National Center for Scientific Research (CNRS).

---

## REFERENCES

1. D. A. Ta, I. Fantoni, R. Lozano, *Modeling and Control of a Convertible Mini-UAV*, in
   18th World Congress of the International Federation of Automatic Control, Italy, 2011.
2. D. A. Ta, *Avion convertible à décollage et atterrissage vertical*, PhD thesis,
   University of Technology of Compiègne, France, December 2011.
3. T.D.C. Thanh and K.K. Ahn, *Techical note: Nonlinear PID control to improve the control
   performance of 2 axes pneumatic artificial muscle manipulator using neural network*, In
   Journal of Mechatronics, 2006.
4. K. T. Oner, E. Cetinsoy, E. Sirimoglu, C. Hancer, T. Ayken and M. Unel, *LQR and SMC
   Stabilization of a New Unmanned Aerial Vehicle*, in World Academy of Science,
   Engineering and Technology, 2009.
5. F. Kendoul, I. Fantoni, R. Lozano, *Modeling and control of a small autonomous aircraft
   having two tilting rotors*, in Proceedings of the 44th IEEE Conference on Decision and
   Control, and the European Control Conference, Spain, 2005.
6. Y. Xili, F. Yong and Z. Jihong, *Transition Flight Control of Two Vertical/Short
   Takeoff and Landing Aircraft*, in Journal of Guidance, Control and Dynamics, vol. 31,
   no. 2, 2008.
7. J. Dickeson, D. Miles, O. Cifdaloz, V. Wells and A. A. Rodriguez, *Robust LPV
   $H_\infty$ Gain-Scheduled Hover-to-Cruise Conversion for a Tilt-Wing Rotorcraft in the
   Presence of CG Variations*, in Proceedings of the 2007 American Control Conference, USA,
   2007.
8. R. Lozano, editor. *Unmanned Aerial Vehicles*. John Wiley, 2010.
9. B. W. Mc Cormick, *Aerodynamics Aeronautics and Flight Mechanics*, John Wiley and Sons
   Inc, 1995.
10. Michael V.Cook, *Flight Dynamics Principles: A Linear Systems Approach to Aircraft
    Stability and Control*, Elsevier, 2007.
11. G. Fay, *Derivation of the aerodynamic forces for the mesicopter simulation*, Technical
    Report, Stanford University, USA, 2001.
12. A. R. Teel, *Global stabilization and restricted tracking for multiple integrators with
    bounded controls*, Systems and Control Letters, 18:165-171, 1992.
13. T.D.C. Thanh and K.K. Ahn, *Intelligent Phase Plane Switching Control of Pneumatic
    Artificial Muscle Manipulators with Magneto Rheological Brake*, Journal of Mechatronics,
    vol. 16, no. 2, 2006.
