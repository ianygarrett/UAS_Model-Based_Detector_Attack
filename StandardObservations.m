function y = StandardObservations(x,command,dist)
% STATES:
p = x(1); q = x(2); r = x(3);               %roll/pitch/yaw rate
u = x(4); v = x(5); w = x(6);               %u/v/w velocity components
phi = x(7); theta = x(8); psi = x(9);    %euler angles
xF = x(10); yF = x(11); h = x(12);

% INPUTS:
windU = dist(1); %North gust component
windV = dist(2); %East gust component
windW = dist(3); %Down gust component

noise = dist(4:end);
% SENSOR NOISE
pn = noise(1);   qn = noise(2);   rn = noise(3);
Vtn = noise(4);  phin = noise(5); thetan = noise(6);
psin = noise(7); xFn = noise(8); yFn = noise(9); hn = noise(10);
noise = [pn; qn; rn; Vtn; phin; thetan; psin; xFn; yFn; hn];

% V/AOA/AOS:
u = u - windU;
v = v - windV;
w = w - windW;
Vt = sqrt(u^2+v^2+w^2);

% MEASUREMENTS
y = [p; q; r; Vt; phi; theta; psi; xF; yF; h] + noise;

