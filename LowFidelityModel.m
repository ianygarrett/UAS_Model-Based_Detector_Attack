%Uncomment for High-Fidelity data
%SteadyTurnSimulation_pathFollowing

load Model_circle
load circleTrajLPV_2
load H

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
karray = zeros(18,1);

%CUSUM threshold
cst = 0;

k = 0;
ind = 0;




%Sample Rate Variables
sr_hz = 25; %50hz / 25hz / 13hz
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

%TODO: inital estimated state
%fix this so that it's random since
%we wouldn't know the actual

%Initial State Estimate
dxdot = xOut(:,1);
dxdot = dxdot(1:18);

%Inital Covariance
P = 0.1*eye(18);

%Q = Process noise co-variance
Q = 0.01;

%R = measurement noise covariance; small = accurate; large = inaccurate
R = 1;

%I = unit matrix
I = ones(18);

for i = inc:inc:runs     
    k = k+4;
    ind = ind+1;
    du = uOut(:,k);
    dx = xOut(:,k);
    dy = yOut(:,k);
    dx = dx(1:18);
    dy = dy(1:18);
    
%%%%%Kalman Filter%%%%%
    
    %Measurement/Observation Matrix
    %18x18 H variable loaded in the beginning
    %H = C2;

    %State Estimate 
    dxdot = A*dxdot + B2*du;
    
    %Covariance Estimate 
    P = A*P*A' + Q;
    
    %Measurement Estimate 
    dydot    = H*dxdot ;% + D22*du;
   % dydot = [dydot(1:3); zeros(3,1);dydot(5:10);zeros(6,1)];
    
    %Innovation Covariance
    S = H*P*H'+ R;
    
    %Kalman Gain
    K = P*H'/S;
    karray = [karray,K];
    
    dxdot = dxdot + K*(dy-dydot);
    
    %Update the error covariance
    % I = unit matrix
    P = (I - (K*H))*P;
%%%%%END Kalman Filter%%%%%
 
  dxdot(12) = dxdot(12)+50;
  dxdotarray = [dxdotarray,dxdot];
  
  %Attack Height State
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

%plot(yRef,xRef,'r','LineWidth',2);grid on; xlabel('EAST/WEST (m)');ylabel('NORTH/SOUTH (m)'); hold on;
plot(karray(12,1:sr));
%plot(karray);
