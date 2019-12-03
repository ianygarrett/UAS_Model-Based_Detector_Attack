function dx = StandardSimDynamics(~,x,command,wind,param,psi0)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Flight dynamic model for the Senior Telemaster fixed-wing UAS
% Written by Devaprakash Muniraj, Micah Fry, and Mazen Farhood
%
% Inputs: x (UAS state vector of size 23x1): x(1:3)   -> Roll/Pitch/Yaw rates in rad/s
%											 x(4:6)   -> Body-axis linear velocities in m/s
%											 x(7:9)   -> Roll/Pitch/yaw Euler angles in rad 
% 											 x(10:12) -> X/Y/H inertial positions in m  
%											 x(13:18) -> Actuator states 
%											 x(19:23) -> States pertaining to Dryden turbulence model 
%
%		  command (UAS control input vector of size 4x1): command(1) -> Elevator command (varies between -0.4 to 0.4) 
% 														  command(2) -> Aileron command (varies between -0.4 to 0.4)
% 														  command(3) -> Rudder command (varies between -0.4 to 0.4)		
% 													      command(4) -> Throttle command (varies between 0 to 0.9)											
% 											              For elevator, aileron, and rudder
% 														  -0.4: PWM value of 1100 microseconds, 0: PWM value of 1500 microseconds, 0.4: PWM value of 1900 microseconds
%														  For throttle 
% 														  0: PWM value of 1100 microseconds, 0.9: PWM value of 1900 microseconds
% 		  wind (vector of length 7x1): wind(1:3) -> random values drawn from a standard normal distribution 
% 									   wind(4)   -> 1: Light turbulence, 2: Moderate turbulence, 3: Heavy turbulence 
% 									   wind(5:7) -> Steady wind in the N/E/D directions (typically wind(7) is set to zero)
% 	      param(vector of size 31x1): parameters used in the aerodynamic model
% 		  psi0 -> Initial heading angle (typically set to zero)	
%
% Outputs: dx(vector of size 23x1): derivative of the state vector 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin<6
    psi0 = 0;
end

% STATES:
p = x(1); q = x(2); r = x(3);         %roll/pitch/yaw rate
u = x(4); v = x(5); w = x(6);         %u/v/w velocity components
phi = x(7); theta = x(8); psi = x(9); %attitude
xF = x(10); tF = x(11); h = x(12);    %#ok<NASGU>
xe1 = x(13); xe2 = x(14);             %elevator dynamics
xa1 = x(15); xa2 = x(16);             %aileron dynamics
xr1 = x(17); xr2 = x(18);             %rudder dynamics
dryU = x(19);                         %Dryden states
dryV = [x(20); x(21)];
dryW = [x(22); x(23)];

% INPUTS:
dEc = command(1); %elevator command (pwm)
dAc = command(2); %aileron command (pwm)
dRc = command(3); %rudder command (pwm)
dT = command(4); %thrust command (pwm)
dE = xe1; dA = xa1; dR = xr1; % actuator deflections
windU = wind(1);   %random variable to drive u disturbance
windV = wind(2);   %random variable to drive v disturbance
windW = wind(3);   %random variable to drive w disturbance
magCase = wind(4); %gust magnitude identifier
windN = wind(5);   %north wind component
windE = wind(6);   %east wind component
windH = wind(7);   %vertical wind component

% ROTATION (earth to body)
sthe = sin(theta); cthe = cos(theta);
sphi = sin(phi); cphi = cos(phi);
spsi = sin(psi); cpsi = cos(psi);
R1 = [cpsi spsi 0;-spsi cpsi 0;0 0 1];
R2 = [cthe 0 -sthe;0 1 0;sthe 0 cthe];
R3 = [1 0 0;0 cphi sphi;0 -sphi cphi];
REarthToBody = R3*R2*R1;

%added this
H0 = 1;

% DRYDEN CALCULATIONS:
Vt1 = sqrt(u^2+v^2+w^2);
if isnan(Vt1)
    error('Airspeed is NaN');
end
[Au,Av,Aw,Bu,Bv,Bw,Cu,Cv,Cw,Du,Dv,Dw] = DrydenParams(h+H0,Vt1,magCase);
dryUdot = Au*dryU + Bu*windU;
dryVdot = Av*dryV + Bv*windV;
dryWdot = Aw*dryW + Bw*windW;
fts2ms = 0.3048;
uWind = fts2ms*(Cu*dryU + Du*windU);
vWind = fts2ms*(Cv*dryV + Dv*windV);
wWind = fts2ms*(Cw*dryW + Dw*windW);
windBody = [uWind; vWind; wWind] + REarthToBody*[windN; windE; windH];
dryDyn = [dryUdot; dryVdot; dryWdot];

% V/AOA/AOS:
ua = u - windBody(1);
va = v - windBody(2);
wa = w - windBody(3);
Va = sqrt(ua^2+va^2+wa^2);

alpha = atan2(wa,ua);
beta = asin(va/Va);

% SATURATE INPUTS:
if abs(dE) > 0.4
    dE = sign(dE)*0.4;
end
if abs(dA) > 0.4
    dA = sign(dA)*0.4;
end
if abs(dR) > 0.4
    dR = sign(dR)*0.4;
end
if dT<0
    dT = 0;
end

if dT>0.9
    dT = 0.9;
end

if abs(dEc) > 0.4
    dEc = sign(dEc)*0.4;
end
if abs(dAc) > 0.4
    dAc = sign(dAc)*0.4;
end
if abs(dRc) > 0.4
    dRc = sign(dRc)*0.4;
end

% DEFINE AIRCRAFT GEOMETRY AND MASS PARAMETERS:
cbar = 0.3594;    %mean chord (meters) (14.149 in.)
bSpan = 2.3876;   %span (meters) (94 in.)
Sref = cbar*bSpan;   %reference area (meters^2) (1330 sq.in.)
mass = 5.7096;    %mass (kg)
Ix      =   1.3168;
Iy      =   1.5699;
Iz      =   1.8704;

Rho0 = 1.225;
T0   = 288.15;
R0   = 8.31432;
M0   = 0.0289644;
g0   = 9.80665;
L0   = -0.0065;
Th   = T0+(h+H0)*L0;
Rho = Rho0*(Th/T0)^(g0*M0/R0/L0-1);

CX0     =   param(1);
CXalp   =   param(2);
CZ0     =   param(3);
CZalp   =   param(4);
CZdE    =   param(5);
CM0     =   param(6);
CMalp   =   param(7);
CMq     =   param(8);
CMdE    =   param(9);
CXct    =   param(10);
CZct    =   param(11);
CZq     =   param(12);
CXdT    =   param(13);

CY0     =   param(14);
CYbeta  =   param(15);
CYp     =   param(16);
CYr     =   param(17);
CYdR    =   param(18);
CYdA    =   param(19);
CL0     =   param(20);
CLbeta  =   param(21);
CLp     =   param(22);
CLr     =   param(23);
CLdA    =   param(24);
CLdR    =   param(25);
CN0     =   param(26);
CNbeta  =   param(27);
CNp     =   param(28);
CNr     =   param(29);
CNdA    =   param(30);
CNdR    =   param(31);

Thrust = throttle_model_new(Va,(1.1+dT)*1000);

CT      =   Thrust/(1/2*Rho*Sref*Va^2);

CX      =   CX0 + CXalp*alpha + CXct*CT + CXdT*dT;
CZ      =   CZ0 + CZalp*alpha + CZdE*dE + CZct*CT + CZq*q*cbar./(2*Va); 
CM      =   CM0 + CMalp*alpha + CMq*q*cbar./(2*Va) + CMdE*dE;
CY      =   CY0 + CYbeta*beta + CYp*p*bSpan./(2*Va) + CYr*r*bSpan./(2*Va) + CYdA*dA + CYdR*dR;
CL      =   CL0 + CLbeta*beta + CLp*p*bSpan./(2*Va) + CLr*r*bSpan./(2*Va) + CLdA*dA + CLdR*dR;
CN      =   CN0 + CNbeta*beta + CNp*p*bSpan./(2*Va) + CNr*r*bSpan./(2*Va) + CNdA*dA + CNdR*dR;

% CALCULATE BODY FORCES AND MOMENTS
Fx = CX*1/2*Rho*Va.^2*Sref + Thrust;
Fy = CY*1/2*Rho*Va.^2*Sref;
Fz = CZ*1/2*Rho*Va.^2*Sref;
L = CL*1/2*Rho*Va.^2*Sref*bSpan;
M = CM*1/2*Rho*Va.^2*Sref*cbar;
N = CN*1/2*Rho*Va.^2*Sref*bSpan;

% EQUATIONS OF MOTION
pdot = 1/Ix*(L + (Iy-Iz)*q*r);
qdot = 1/Iy*(M + (Iz-Ix)*p*r);
rdot = 1/Iz*(N + (Ix-Iy)*p*q);

udot = -q*w + r*v + 1/mass*Fx - g0*sin(theta);
vdot = -r*u + p*w + 1/mass*Fy + g0*cos(theta)*sin(phi);
wdot = -p*v + q*u + 1/mass*Fz + g0*cos(theta)*cos(phi);

phidot = p + (q.*sin(phi)+r.*cos(phi)).*tan(theta);
thetadot = q.*cos(phi) - r.*sin(phi);
psidot = (q.*sin(phi) + r.*cos(phi)).*sec(theta);

xFdot = u.*cos(theta).*cos(psi-psi0) + ...
    v.*(sin(phi).*sin(theta).*cos(psi-psi0) - cos(phi).*sin(psi-psi0)) + ...
    w.*(cos(phi).*sin(theta).*cos(psi-psi0) + sin(phi).*sin(psi-psi0));
yFdot = u.*cos(theta).*sin(psi-psi0) + ...
    v.*(sin(phi).*sin(theta).*sin(psi-psi0) + cos(phi).*cos(psi-psi0)) + ...
    w.*(cos(phi).*sin(theta).*sin(psi-psi0) - sin(phi).*cos(psi-psi0));
hdot = u.*sin(theta) - v.*sin(phi).*cos(theta) - w.*cos(phi).*cos(theta);

% ACTUATOR DYNAMICS:
wn   = 13.7;
z    = 0.67;
xe1dot = xe2;
xe2dot = -wn^2*xe1-2*z*wn*xe2+wn^2*dEc;
xa1dot = xa2;
xa2dot = -wn^2*xa1-2*z*wn*xa2+wn^2*dAc;
xr1dot = xr2;
xr2dot = -wn^2*xr1-2*z*wn*xr2+wn^2*dRc;
actdot = [xe1dot; xe2dot; xa1dot; xa2dot; xr1dot; xr2dot];

dx = [pdot;qdot;rdot;udot;vdot;wdot;phidot;thetadot;psidot;xFdot;yFdot;hdot;actdot;dryDyn];