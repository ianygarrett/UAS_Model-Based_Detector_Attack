function y = StandardSimObservations(x,wind,noise)
% STATES:
p = x(1); q = x(2); r = x(3);               %roll/pitch/yaw rate
u = x(4); v = x(5); w = x(6);               %u/v/w velocity components
phi = x(7); theta = x(8); psi = x(9);       %euler angles
xF = x(10); yF = x(11); h = x(12);          %positions
dryU = x(19);                               %dryden states
dryV = [x(20); x(21)];
dryW = [x(22); x(23)];

% INPUTS:
windU = wind(1);   %random variable to drive u disturbance
windV = wind(2);   %random variable to drive v disturbance
windW = wind(3);   %random variable to drive w disturbance
magCase = wind(4); %gust magnitude identifier
windN = wind(5);   %north wind component
windE = wind(6);   %east wind component
windH = wind(7);   %vertical wind component

% SENSOR NOISE
pn = noise(1);   qn = noise(2);   rn = noise(3);
Vtn = noise(4);  phin = noise(5); thetan = noise(6);
psin = noise(7); xFn = noise(8); yFn = noise(9); hn = noise(10);
noise = [pn; qn; rn; Vtn; phin; thetan; psin; xFn; yFn; hn];

% ROTATION (earth to body)
sthe = sin(theta); cthe = cos(theta);
sphi = sin(phi); cphi = cos(phi);
spsi = sin(psi); cpsi = cos(psi);
R1 = [cpsi spsi 0;-spsi cpsi 0;0 0 1];
R2 = [cthe 0 -sthe;0 1 0;sthe 0 cthe];
R3 = [1 0 0;0 cphi sphi;0 -sphi cphi];
REarthToBody = R3*R2*R1;

% DRYDEN OBSERVATION CALCULATIONS
Vt = sqrt(u^2+v^2+w^2);
[~,~,~,~,~,~,Cu,Cv,Cw,Du,Dv,Dw] = DrydenParams(h,Vt,magCase);
fts2ms = 0.3048;
uWind = fts2ms*(Cu*dryU + Du*windU);
vWind = fts2ms*(Cv*dryV + Dv*windV);
wWind = fts2ms*(Cw*dryW + Dw*windW);
windBody = [uWind; vWind; wWind] + REarthToBody*[windN; windE; windH];

% V/AOA/AOS:
ua = u - windBody(1);
va = v - windBody(2);
wa = w - windBody(3);
Va = sqrt(ua^2+va^2+wa^2);

AOA = atan2(wa,ua);
AOS = asin(va/Va);

% MEASUREMENTS
y = [p; q; r; Va; phi; theta; psi; xF; yF; h] + noise;

% y = [y(1:4);AOA;AOS;y(5:10)];

