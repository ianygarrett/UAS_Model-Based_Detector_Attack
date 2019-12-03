%Uncomment for High-Fidelity data
%SteadyTurnSimulation_pathFollowing

load Model_circle
load circleTrajLPV_2

tau = 1;
yhat = 0;

dx = xOut(:,1);
dx = dx(1:18);
y = ones(10,1); 
du = uOut(1);

yarray = zeros(10,1);
dxdotarray = zeros(18,1);
dxarray = zeros(18,1);
residual_array = zeros(18,500);
attackres_array = zeros(18,500);
attack_state_array = zeros(18,1);

%CUSUM threshold
cst = 0;

k = 0;
ind = 0;
total_tau = zeros(1,18);
attack_total_tau = zeros(1,18);

%Sample Rate Variables
sr_hz = 50; %50hz / 25hz / 13hz
if sr_hz == 25
   sr = 476;
   inc = 0.04;
   runs = 19;
elseif sr_hz == 50
    sr = 451;
    inc = 0.02;
    runs = 9;
else %TODO: Find actual number for sr
    sr = 476;
    inc = 0.07;
    runs = 34;
end

for i = inc:inc:runs     
    k = k+4;
    ind = ind+1;
    du = uOut(:,k);
    dx = xOut(:,k);
    dy = yOut(:,k);
    dx = dx(1:18);
    dy = dy(1:18);
    dxdot = A*dx + B2*du;
    dydot    = C2*dx + D22*du;
    dydot = [dydot(1:3); zeros(3,1);dydot(5:10);zeros(6,1)];
    
    %Kalman Filter
    dxdot = dxdot + (dy - dydot);
  %  yarray = [yarray,dy];
  
    %Attack Height State
    dxdot(12) = dxdot(12)+50;
    dxdotarray = [dxdotarray,dxdot];
    
   % residual = abs(xplot(k+1)-dxdot(10));
   dx =xOut(:,k+1);
   dx = dx(1:18);
   dxarray = [dxarray,dx];
   attackdx = dx*2;
   attackdx(12) = attackdx(12) + 50;
   dx(12) = dx(12)+50;
   attack_state_array = [attack_state_array,attackdx];
   residual = abs(dx-dxdot);
   attackres = abs(attackdx-dxdot);
   residual_array(:,ind) = residual;
   attackres_array(:,ind) = attackres;

   disp(k)
end
