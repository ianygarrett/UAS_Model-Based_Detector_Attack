%Uncomment for High-Fidelity Circle data
%SteadyTurnSimulation_pathFollowing

load Model_circle
load circleTrajLPV_2
load CircleTrim
load H

 toggleModel = 1;
 if toggleModel == 1
     for x = 1:1:18
        for y = 1:1:18
            A(x,y) =0.9*A(x,y);
     
        end
        for y = 1:1:4
            B2(x,y) = 0.9 *B2(x,y);
        end
     end
   % A = random('normal',0,1,size(A)).*A;
   % B2 = random('normal',0,1,size(B2)).*B2;
 end


tau = 1;
yhat = 0;

dx = xOut(:,1);
dx = dx(1:18)-xTrim2;
y = ones(10,1); 
du = uOut(1)-uTrim;

yarray = zeros(10,1);
dxdotarray = zeros(18,1);
dxdotarraykf = zeros(18,1);
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
sr_hz = 13; %50hz / 25hz / 13hz
if sr_hz == 25
   sr = 476;
   inc = 0.04;
   runs = 19;
elseif sr_hz == 50
    sr = 451;
    inc = 0.02;
    runs = 9;
else
    sr = 485;
    inc = 0.07;
    runs = 34;
end
meas_noise_array = zeros(10,1999);
%Create measurement noise array with mean and std dev
%std dev.*randn(10,1) + mean
for i = 1:1:1999
    meas_noise_array(:,i)= (0*(randn(10,1)-2.0752));
end
toggle = 0; %toggles wind and noise
wind = constWind * toggle; %constWind comes from SteadTurnSimulation_pathFollowing
%Initial State Estimate
dxdot = xOut(:,1);

%Inital Covariance
P = eye(18);

%Q = Process noise co-variance
Q = 0.00005 *eye(18);
% Q = [ 0.01 0 0 0 0.1 0.01 0.01 0.01 0.01 0.01 0.01 0 0 0 0 0 0 0 ;
%       0 0.01 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;
%       0 0 0.010 0 0 0 0 0 0 0 0 0.1 0 0 0 0 0 0 ;
%       0 0 0 0.010 0 0 0 0 0 0 0.1 0 0 0.1 0 0 0 0 ;
%       0 0 0 0 0.010 0 0 0 0 0 0 0 0.01 0.01 0 0.1 0 0 ;
%       0 0 0 0 0 0.010 0 0 0 0 0 0 0.01 0 0 0 0 0.1 ;
%       0 0 0 0 0 0 0.010 0 0 0 0 0 0 0 0.1 0 0 0 ;
%       0 0 0 0 0 0 0 0.010 0 0 0 0 0 0 0 0 0.1 0 ;
%       0 0 0 0 0 0 0 0 0.010 0 0 0 0 0 0 0 0 0.1 ;
%       0 0 0 0 0 0 0 0 0 0.010 0 0 0 0 0 0 0 0 ;
%       0 0 0 0 0 0 0 0 0 0 0.010 0 0 0 0 0 0 0 ;
%       0 0 0 0 0 0 0 0 0 0 0 0.010 0 0 0 0 0 0 ;
%       0 0 0 0 0 0 0 0 0 0 0 0 0.010 0 0 0 0 0 ;
%       0 0 0 0 0 0 0 0 10 0 0 0 0 00.10 0 0 0 0 ;
%       0 0 0 0 0 0 0 0 0 0 0 0 0 0 0.010 0 0 0 ;
%       0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0.010 0 0 ;
%       0 0 0 0 0 0 0  0 0 0 0 0 0 0 0 0 00.10 0 ;
%       0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0.01 ];
  
%R = measurement noise covariance
% R = [ 20 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;
%       10 10 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;
%       10 0 20 0 0 0 0 0 0 0 0 5 5 5 0 0 0 0 ;
%       10 0 0 10 0 0 0 0 0 0 0 5 5 5 0 0 0 0 ;
%       10 0 0 0 20 0 0 0 0 0 0 0 0 0 0 0 0 0 ;
%       10 0 0 0 0 10 0 0 0 0 0 0 0 0 0 0 0 0 ;
%       10 0 0 0 0 0 20 0 0 0 0 0 0 0 0 0 0 0 ;
%       10 0 0 10 0 0 0 10 0 0 0 0 0 0 0 0 0 0 ;
%       10 0 0 0 10 0 0 0 20 0 0 0 0 0 0 0 0 0 ;
%       10 0 0 10 10 0 0 0 0 10 0 0 0 0 0 0 0 0 ;
%       10 0 0 0 10 10 0 0 0 0 20 0 0 0 0 0 0 0 ;
%       10 0 0 0 0 10 10 0 0 0 0 10 0 0 0 0 0 0 ;
%       10 0 10 0 0 0 10 10 0 0 0 0 20 0 0 0 0 0 ;
%       10 0 5 10 0 0 0 10 10 0 0 0 0 10 0 0 0 0 ;
%       10 0 0 5 10 0 0 0 10 0 0 0 0 0 10 0 0 0 ;
%       10 0 0 0 5 10 0 0 0 10 0 0 0 0 0 20 0 0 ;
%       10 0 0 0 0 5 10 0 0 0 10 0 0 0 0 0 10 0 ;
%       10 0 0 0 0 0 5 10 0 0 0 10 0 0 0 0 0 20 ];
R = 9000000*eye(18);

%%%%Just estimates no filter%%%%%
for i = inc:inc:runs     
%for i = 2:1:1999
    k = k+4;
    du = uOut(:,k) - uTrim;
    dx = xOut(1:18,k) - xTrim2;
    
    dxdot = A*dx + toggle*ones(18,13)*vertcat(wind,meas_noise_array(:,k))+ B2*du  ;
    dxdotarray = [dxdotarray, dxdot];
end


%%%%Baseline Behavior no attack%%%%%
%%%Adding Kalman Filter%%%
k = 0;

%Initial State Estimate
dxdot = xOut(:,1);
dxdot = dxdot(1:18);

for i = inc:inc:runs     
    k = k+4;
    ind = ind+1;
    dy = yOut(:,k);
    dy = dy(1:18);
   
%%%%%Kalman Filter%%%%%
    
    %Measurement/Observation Matrix
    %18x18 H variable loaded in the beginning
    %H = C2;

    %State Estimate 
    du = uOut(:,k) - uTrim;
    dx = xOut(1:18,k) - xTrim2;
    
    dxdot = A*dx + toggle*ones(18,13)*vertcat(wind,meas_noise_array(:,k))+ B2*du;

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
    dxdotarraykf = [dxdotarraykf, dxdot];
    %Update the error covariance
    P = P - K*H*P;
    
%%%%%END Kalman Filter%%%%%
 
  disp(k)
end
%%%End Baseline%%%%%


%%%%%BEGIN ATTACK%%%%%%%%%
%%%AGGREGATING OFFSET%%%%%%%%
%%%Adding Kalman Filter%%%
k = 0;
offset = 0;
%Initial State Estimate
dxdot = xOut(:,1);
dxdot = dxdot(1:18);

for i = inc:inc:runs     
    k = k+4;
    ind = ind+1;
    dy = yOut(:,k);
    dy = dy(1:18);
   
%%%%%Kalman Filter%%%%%
    
    %Measurement/Observation Matrix
    %18x18 H variable loaded in the beginning
    %H = C2;

    %State Estimate 
    du = uOut(:,k) - uTrim;
    dx = xOut(1:18,k) - xTrim2;
    
    
    dx = dx+offset;
    offset=offset+0.4;

    
    dxdot = A*dx + toggle*ones(18,13)*vertcat(wind,meas_noise_array(:,k))+ B2*du;

    %Covariance Estimate 
    P = A*P*A' + Q;
    
    %Measurement Estimate 
    dydot    = H*dxdot ;% + D22*du;
   % dydot = [dydot(1:3); zeros(3,1);dydot(5:10);zeros(6,1)];
    
    %Innovation Covariance
    S = H*P*H'+ R;
    
    %Kalman Gain
    K = P*H'/S;
    
    dxdot = dxdot + K*(dy-dydot);
    attack_state_array = [attack_state_array, dxdot];
    %Update the error covariance
    P = P - K*H*P;
    
%%%%%END Kalman Filter%%%%%
 
  disp(k)
end
%   % residual = abs(xplot(k+1)-dxdot(10));
%   dx =xOut(:,k+1);
%   dx = dx(1:18);
%   dxarray = [dxarray,dx];
%   attackdx = dx*2;
%   attackdx(12) = attackdx(12) + 50;
%   dx(12) = dx(12)+50;
%   attack_state_array = [attack_state_array,attackdx];
%   residual = abs(dx-dxdot);
%   attackres = abs(attackdx-dxdot);
%   residual_array(:,ind) = residual;
%   attackres_array(:,ind) = attackres;
%%%%%END ATTACK%%%%%%%%



%figure(100)
% plot(yRef,xRef,'r','LineWidth',2);grid on; xlabel('EAST/WEST (m)');ylabel('NORTH/SOUTH (m)'); hold on;
% plot(xOut(11,1:1999),xOut(10,1:1999),col,'LineWidth',2); hold on; grid on; hold on;
% 
% figure(99)
% plot(yRef,xRef,'r','LineWidth',2);grid on; xlabel('EAST/WEST (m)');ylabel('NORTH/SOUTH (m)'); hold on;
% plot(dxdotarray(11,1:1999),dxdotarray(10,1:1999),col,'LineWidth',2); hold on; grid on; hold on; 
%
% figure(98)
% plot(yRef,xRef,'r','LineWidth',2);grid on; xlabel('EAST/WEST (m)');ylabel('NORTH/SOUTH (m)'); hold on;
% plot(dxdotarraykf(11,1:sr),dxdotarraykf(10,1:sr),col,'LineWidth',2); hold on; grid on; hold on; 

figure(97)
clf
plot(yRef,xRef,'r','LineWidth',2);grid on; xlabel('EAST/WEST (m)');ylabel('NORTH/SOUTH (m)'); hold on;
p1=plot(xOut(11,1:1999),xOut(10,1:1999),'b','LineWidth',2);P1="Actual"; hold on; grid on; hold on;
%plot(dxdotarray(11,1:sr),dxdotarray(10,1:sr),'b','LineWidth',2); hold on; grid on; hold on; 
p2=plot(dxdotarraykf(11,1:sr),dxdotarraykf(10,1:sr),'g','LineWidth',2);P2="Estimate"; hold on; grid on; hold on;
p3=plot(attack_state_array(11,1:sr),attack_state_array(10,1:sr),'r','LineWidth',2);P3="Attack"; hold on; grid on; hold on;
legend([p1,p2,p3], [P1, P2,P3]);

