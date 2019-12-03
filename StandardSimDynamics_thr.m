
function dx = StandardSimDynamics_thr(~,x,command,wind,param,psi0)

global ThrustMod
global H0
global ax ay az

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
xt1 = x(19); xt2 = x(20);             %throttle dynamics
dryU = x(21);                         %Dryden states
dryV = [x(22); x(23)];
dryW = [x(24); x(25)];

% INPUTS:
dEc = command(1); %elevator command (pwm)
dAc = command(2); %aileron command (pwm)
dRc = command(3); %rudder command (pwm)
dTc = command(4); %thrust command (pwm)
dE = xe1; dA = xa1; dR = xr1; % actuator deflections
dT = xt1;
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

% if abs(dA) > 0.3
%     dA = sign(dA)*0.3;
% end
% if abs(dR) > 0.3
%     dR = sign(dR)*0.3;
% end

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

if dTc<0
    dTc = 0;
end

if dTc>0.9
    dTc = 0.9;
end

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

% DEFINE AIRCRAFT GEOMETRY AND MASS PARAMETERS:
cbar = 0.3594;    %mean chord (meters) (14.149 in.)
bSpan = 2.3876;   %span (meters) (94 in.)
S = cbar*bSpan;   %reference area (meters^2) (1330 sq.in.)
mass = 5.7096;    %mass (kg)
g = 9.81;      %acceleration due to gravity (m/s)
Ix      =   1.3168;
Iy      =   1.5699;
Iz      =   1.8704;

% mass = mass*1.2;
% Ix = Ix*1.2;
% Iy = Iy*1.2;
% Iz = Iz*1.2;

Rho0 = 1.225;
T0   = 288.15;
R0   = 8.31432;
M0   = 0.0289644;
g0   = 9.80665;
L0   = -0.0065;
Th   = T0+(h+H0)*L0;
rho = Rho0*(Th/T0)^(g0*M0/R0/L0-1);

Thrust = throttle_model_new(Va,(1.1+dT)*1000);
CT      = Thrust/(1/2*rho*S*Va^2);

CX      =   CX0 + CXalp*alpha + CXct*CT + CXdT*dT;
CZ      =   CZ0 + CZalp*alpha + CZdE*dE + CZct*CT + CZq*q*cbar./(2*Va);
CM      =   CM0 + CMalp*alpha + CMq*q*cbar./(2*Va) + CMdE*dE;
CY      =   CY0 + CYbeta*beta + CYp*p*bSpan./(2*Va) + CYr*r*bSpan./(2*Va) + CYdA*dA + CYdR*dR;
CL      =   CL0 + CLbeta*beta + CLp*p*bSpan./(2*Va) + CLr*r*bSpan./(2*Va) + CLdA*dA + CLdR*dR;
CN      =   CN0 + CNbeta*beta + CNp*p*bSpan./(2*Va) + CNr*r*bSpan./(2*Va) + CNdA*dA + CNdR*dR;

% CALCULATE BODY FORCES AND MOMENTS
Fx = CX*1/2*rho*Va.^2*S + Thrust;
Fy = CY*1/2*rho*Va.^2*S;
Fz = CZ*1/2*rho*Va.^2*S;
L = CL*1/2*rho*Va.^2*S*bSpan;
M = CM*1/2*rho*Va.^2*S*cbar;
N = CN*1/2*rho*Va.^2*S*bSpan;

ax = Fx/mass;
ay = Fy/mass;
az = Fz/mass;

% EQUATIONS OF MOTION
pdot = 1/Ix*(L + (Iy-Iz)*q*r);
qdot = 1/Iy*(M + (Iz-Ix)*p*r);
rdot = 1/Iz*(N + (Ix-Iy)*p*q);

udot = -q*w + r*v + 1/mass*Fx - g*sin(theta);
vdot = -r*u + p*w + 1/mass*Fy + g*cos(theta)*sin(phi);
wdot = -p*v + q*u + 1/mass*Fz + g*cos(theta)*cos(phi);

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

% if(xt2>=0)
%     wnt = 13.7;
%     zt = 0.85;
% else
%     wnt = 2.2;
%     zt = 1.6;
% end
    
if(xt2>=0)
    wnt   = 5.7; 
    zt    = 1.3; 
else
    wnt = 4.5;
    zt = 1.6;
end

% wnt = 18.7;
% zt = 0.85;

xe1dot = xe2;
xe2dot = -wn^2*xe1-2*z*wn*xe2+wn^2*dEc;
xa1dot = xa2;
xa2dot = -wn^2*xa1-2*z*wn*xa2+wn^2*dAc;
xr1dot = xr2;
xr2dot = -wn^2*xr1-2*z*wn*xr2+wn^2*dRc;

xt1dot = xt2;
xt2dot = -wnt^2*xt1-2*zt*wnt*xt2+wnt^2*dTc;

actdot = [xe1dot; xe2dot; xa1dot; xa2dot; xr1dot; xr2dot; xt1dot; xt2dot];

dx = [pdot;qdot;rdot;udot;vdot;wdot;phidot;thetadot;psidot;xFdot;yFdot;hdot;actdot;dryDyn];
