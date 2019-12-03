clear all; clc;
load LevelTrim2
%load LevelTrim

global H0
H0 = 630.0;
global ax az
global wpcmd completedLoops lpflag lpcounter
lpflag = 1; lpcounter = 0;

load param_stele_new2

param_uncert = 1.0;
param = param_uncert*param;

r2d = 180/pi;
d2r = pi/180;

col = 'b';

load circleTrajLPV_2
% load fig8TrajLPV_3

%% Make path
wpcmd = 2;

ax = 1;
az = 1;

%xRef = refData(:,8);
%yRef = refData(:,9);
%zRef = refData(:,10);
xRef = 1;
yRef = 1;
%zRef = 1;

thrdyn_flag = 0 ;
%thrdyn_flag = 1;
clear pathLength

%% SIMULATION INITIALIZATION:
numLoops = 2000;%2;simulation time / rate = numLoops so 100hz = 0.01 and and time is 20s

gustToggle = 1;
noiseToggle = 1;
delayToggle = 1;
magCase = 1; % 1/2/3
constWind = [2.5; 2.5; 2.5]; %NORTH/EAST/DOWN
completedLoops = 0;
timestep = 0.04;

delaymin = 0.9*0.038;
delaymax = 0.9*0.042;

j = 1;
wind = [gustToggle*ones(3,1); magCase; constWind]; wind(:,2)=wind;
measNoise = noiseToggle*[0.5*ones(1,3)*d2r 2 0.01*ones(1,3) 1*ones(1,3)];

% measNoise = noiseToggle*[0.5*ones(1,3)*d2r 0.4*2 0.4*0.01*ones(1,3) 0.4*2 0.4*2 0.4*2];

if(thrdyn_flag==1)
    xOut = [xTrim2(1:18,1); uTrim(4); 0.0; zeros(5,1)]; xeplot = 0; yeplot = 0; psiplot = 0; Vaplot = 0;
else
    xOut = [xTrim2(1:18,1); zeros(5,1)]; xeplot = 0; yeplot = 0; psiplot = 0; Vaplot = 0;
end

%xOut(9) = 0.0;
%xOut(10) = 0.0;
%xOut(11) = 0.0;
uOut = uTrim; uDelay = uOut;

yTrim = StandardObservations(xTrim2,zeros(3,1),zeros(13,1));
%yTrim = xTrim;
yTrim  = [yTrim(1:6);0.0;yTrim(7:9)];

rng('default')
rng(1)

temp(1,1) = 0; temp(1,2) = 0;
gpshold = 0;

% Initializing integrator states
%--------------------------------------------------------------------------------
integ(1,1) = 0.0; % Roll integrator demand
integ(1,2) = 0.0; % Pitch integrator demand
integ(1,3) = 0.0; % integrator6 state
integ(1,4) = sqrt(xTrim(4)^2 + xTrim(5)^2 + xTrim(6)^2); % Velocity demand
integ(1,5) = xTrim(12); % Altitude demand
integ(1,6) = 0.0; %xTrim(8)*57.3*integ(1,4)*9.81*5.0; % integrator7 state
integ(1,7) = 0.0; % Yaw integrator demand
integ(1,8) = 0.0; %xTrim(3); % Body-axis yaw rate
integ(1,9) = 0.0; %xTrim(3); % Body-axis yaw rate
integ(1,10) = integ(1,4);  % integrator 5 state
integ(1,11) = 0.0;  % integrator 4 state
integ(1,12) = xTrim(12); % integrator 3 state
integ(1,13) = 0.0; % integrator 2 state
integ(1,14) = 0.0; % integrator 1 state

%--------------------------------------------------------------------------------

angl(1,1)= cos(xOut(8,1));
angl(1,2)= sin(xOut(8,1));
angl(1,3)= cos(xOut(7,1));
angl(1,4)= sin(xOut(7,1));
angl(1,5)= cos(xOut(9,1));
angl(1,6)= sin(xOut(9,1));

while completedLoops < numLoops
    j = j + 1;
    
    wind(1:3,j) = gustToggle*randn(3,1);
    
    tdel = rand(1)*(delaymax-delaymin)+delaymin;
    
    if delayToggle==0
        tdel = 0.000001;
    end
    
    delayvec = [0 tdel];
    
    % if(thrdyn_flag==1)
    %     odefundelay = @(t,x) StandardSimDynamics_thr(t,x,real(uDelay),wind(:,j),param,0);
    % else
    %     odefundelay = @(t,x) StandardSimDynamics1(t,x,real(uDelay),wind(:,j),param,0);
    % end
    odefundelay = @(t,x) StandardSimDynamics(t,x,real(uDelay),wind(:,j),param,0);
    
    [~,xd] = ode23(odefundelay,delayvec,xOut(:,j-1));
    xDelay = xd(end,:)';
    dtvec = [0 timestep-tdel];
    
    %if(thrdyn_flag==1)
    %    odefun = @(t,x) StandardSimDynamics_thr(t,x,real(uOut(:,j-1)),wind(:,j),param,0);
    %else
    %    odefun = @(t,x) StandardSimDynamics1(t,x,real(uOut(:,j-1)),wind(:,j),param,0);
    %end
    odefun = @(t,x) StandardSimDynamics(t,x,real(uOut(:,j-1)),wind(:,j),param,0);
    
    [~,xs] = ode23(odefun,dtvec,xDelay);
    xOut(:,j) = xs(end,:)';
    if xOut(9,j)>2*pi
        xOut(9,j) = xOut(9,j)-2*pi;
    end
    noise(:,j) = randn(10,1).*measNoise';
    yOut = xOut;
    % if(thrdyn_flag==1)
    %     yOut(:,j) = StandardSimObservations(xOut(:,j),wind(:,j),noise(:,j)); %#ok<SAGROW>
    % else
    %     yOut(:,j) = StandardSimObservations(xOut(:,j),wind(:,j),noise(:,j)); %#ok<SAGROW>
    % end
    
    if(j>1)
        udot = (xOut(4,j)-xOut(4,j-1))/timestep + noise(4,j);
        hdot = (xOut(12,j)-xOut(12,j-1))/timestep + noise(10,j);
    else
        udot = 0;
        hdot = 0;
    end
    
    Vdot = ax - 9.81*sin(xOut(8,j)); % ax - g*sin(theta)
    hddot = -(az + 9.81);
    
    axplot(j) = ax;
    azplot(j) = az;
    yo(j,:) = [xOut(4,j) xOut(5,j) xOut(6,j) xOut(7,j) xOut(8,j) xOut(9,j) xOut(1,j) xOut(2,j) xOut(3,j) xOut(10,j) xOut(11,j) xOut(12,j) yOut(4,j) Vdot hddot];
    ydots(j,:) = [udot hdot 0.0];
    
    % Simulating GPS and airdata sensor update rate
    
    if(gpshold==1)
        yo(j,1) = Uvel;
        yo(j,2) = Vvel;
        yo(j,3) = Wvel;
        yo(j,10) = xpos;
        yo(j,11) = ypos;
        yo(j,12) = hght;
        yo(j,13) = Vtot;
        
        count = count+1;
        if(count==3)
            gpshold=0;
        end
    else
        Uvel = xOut(4,j);
        Vvel = xOut(5,j);
        Wvel = xOut(6,j);
        xpos = xOut(10,j);
        ypos = xOut(11,j);
        hght = xOut(12,j);
        Vtot = yOut(4,j);
        gpshold = 1;
        count = 0;
    end
    
    % Performance output [p; q; r; Va; phi; theta; psi; xF; yF; h]
    % Control input [dEc; dAc; dRc; dT];
    
    angl(j,1)= cos(xOut(8,j));
    angl(j,2)= sin(xOut(8,j));
    angl(j,3)= cos(xOut(7,j));
    angl(j,4)= sin(xOut(7,j));
    angl(j,5)= cos(xOut(9,j));
    angl(j,6)= sin(xOut(9,j));
    
    [uOut(:,j),integ(j,:)] = pathFollowing_ctrl(timestep,yo(j,:),integ(j-1,:),ydots(j,:),0*wind(:,j)',angl(j,:),noise(:,j)');
    
    % Limit throttle rate (Throttle slew rate=100)
    slewrate = (uOut(4,j)-uOut(4,j-1))/0.04;
    if(abs(slewrate)>1)
        uOut(4,j) = uOut(4,j-1) + uOut(4,j-1)*timestep*sign(slewrate);
    end
    
    wind(:,j+1)=wind(:,j);
    uDelay = uOut(:,j-1);
    
    comLoops(j) = completedLoops;
    completedLoops = completedLoops+1;
    %         uOut(:,j) = uTrim; % uncomment for trim only flight
    
end

%%

% close all;
pplot = xOut(1,:)*r2d;
qplot = xOut(2,:)*r2d;
rplot = xOut(3,:)*r2d;
uplot = xOut(4,:);
vplot = xOut(5,:);
wplot = xOut(6,:);
phiplot = xOut(7,:)*r2d;
thetaplot = xOut(8,:)*r2d;
psiplot = xOut(9,:)*r2d;
xplot = xOut(10,:);
yplot = xOut(11,:);
hplot = xOut(12,:);
Vplot = sqrt(uplot.^2+vplot.^2+wplot.^2);
alphaplot = atand(wplot./uplot);
betaplot = asind(vplot./Vplot);
dEplot = xOut(13,:);
dAplot = xOut(15,:);
dRplot = xOut(17,:);
dTplot = min(uOut(4,:),0.9);
tRef = 0:timestep:timestep*(length(pplot)-1);

% Error computation
ind = 1;

for i=1:length(yplot)
    llim = [ind-25:ind+25];
    i1 = find(llim<=0);
    if(~isempty(i1))
        llim(i1) = llim(i1)+2500;
    end
    i2 = find(llim>2500);
    if(~isempty(i2))
        llim(i2) = llim(i2)-2500;
    end
    ytemp = llim - yplot(i);%yRef(llim) - yplot(i);
    xtemp = llim - xplot(i);%xRef(llim) - xplot(i);
    dist = sqrt(ytemp.*ytemp + xtemp.*xtemp);
    itemp = find(dist==min(dist));
    ind = ind + itemp(1) - 25;
    if(ind<=0)
        ind = ind + 2500;
    end
    if(ind>2500)
        ind = ind - 2500;
    end
    rerr(i) = dist(itemp(1));
end

save PID_simdata_circle_modtrim xOut uOut comLoops rerr

% Rotating the reference trajectory
% angle(1) = -30*pi/180;
% xrot = xRef.*cos(xOut(9,:)-angle(1)) + yRef.*sin(xOut(9,:)-angle(1));
% yrot = -xRef.*sin(xOut(9,:)-angle(1)) + yRef.*cos(xOut(9,:)-angle(1));
% angrot = angle-angle(1)+xOut(9);
% xRef = xrot; yRef = yrot; angle = angrot;

figure(100)
plot(yRef,xRef,'r','LineWidth',2);grid on; xlabel('EAST/WEST (m)');ylabel('NORTH/SOUTH (m)'); hold on;
plot(yplot,xplot,col,'LineWidth',2); hold on; grid on; hold on;

% plot(wp(:,2),wp(:,1),'r','LineWidth',2);grid on; xlabel('EAST/WEST (m)');ylabel('NORTH/SOUTH (m)');

%figure(101)
%set(gcf,'Color','white')
%subplot(311), plot(tRef,pplot,col,tRef,xTrim2(1)*57.3*ones(length(tRef),1),'k','LineWidth',2), grid on; axis tight; hold on
%ylabel(' p (deg/s)')
%subplot(312), plot(tRef,qplot,col,tRef,xTrim2(2)*57.3*ones(length(tRef),1),'k','LineWidth',2), grid on; axis tight; hold on
%ylabel('q (deg/s)')
%subplot(313), plot(tRef,rplot,col,tRef,xTrim2(3)*57.3*ones(length(tRef),1),'k','LineWidth',2), grid on; axis tight; hold on
%ylabel('r (deg/s)'), xlabel('time (sec)');grid on; hold on

Vtrim = sqrt(xTrim2(4)^2 + xTrim2(5)^2 + xTrim2(6)^2);
aoatrim = atan2(xTrim2(6),xTrim2(4));
betatrim = asin(xTrim2(5)/Vtrim);

%figure(102)
%set(gcf,'Color','white')
%ax(1) = subplot(211); plot(tRef,rerr,col,tRef,zeros(length(tRef),1),'k','LineWidth',2); grid on; axis tight; hold on
%ylabel('Radial Error')
%ax(2) = subplot(212); plot(tRef,hplot,col,tRef,zeros(length(tRef),1),'k','LineWidth',2); grid on; axis tight;hold on
%ylabel('h Error');
%linkaxes(ax,'x'); clear ax;

%figure(103)
%set(gcf,'Color','white')
%subplot(311), plot(tRef,Vplot,col,'LineWidth',2), grid on, hold on;
%ylabel('Velocity (m/s)'); axis tight
%subplot(312), plot(tRef,alphaplot,col,'LineWidth',2), grid on,hold on;
%ylabel('\alpha (deg)'); axis tight
%subplot(313), plot(tRef,betaplot,col,'LineWidth',2), grid on,hold on;
%ylabel('\beta (deg)'), xlabel('time (sec)'); axis tight;grid on; hold on

%figure(104)
%set(gcf,'Color','white');
%ax(1) = subplot(411); plot(tRef,dEplot,col,tRef,xTrim2(13)*ones(length(tRef),1),'k','LineWidth',2), grid on; axis tight; hold on
%ylabel('\delta_E'); axis([0 tRef(end) -0.4 0.4]);
%ax(2) = subplot(412); plot(tRef,dAplot,col,tRef,xTrim2(15)*ones(length(tRef),1),'k','LineWidth',2), grid on; axis tight; hold on
%ylabel('\delta_A'); axis([0 tRef(end) -0.4 0.4]);
%ax(3) = subplot(413); plot(tRef,dRplot,col,tRef,xTrim2(17)*ones(length(tRef),1),'k','LineWidth',2), grid on; axis tight; hold on
%ylabel('\delta_R'); axis([0 tRef(end) -0.4 0.4]);
%ax(4) = subplot(414); plot(tRef(1:length(dTplot)),dTplot,col,tRef,uTrim(4)*ones(length(tRef),1),'k','LineWidth',2), grid on; axis tight; hold on
%ylabel('\delta_T'); axis([0 tRef(end) 0 0.9]); grid on; hold on
%linkaxes(ax,'x'); clear ax;

%figure(106)
%set(gcf,'Color','white')
%subplot(311), plot(tRef,phiplot,col,tRef,xTrim2(7)*57.3*ones(length(tRef),1),'k','LineWidth',2), grid on; axis tight; hold on
%ylabel('\phi (deg)')
%subplot(312), plot(tRef,thetaplot,col,tRef,xTrim2(8)*57.3*ones(length(tRef),1),'k','LineWidth',2), grid on; axis tight; hold on
%ylabel('\theta (deg)')
%subplot(313), plot(tRef,psiplot,col,tRef,xTrim2(9)*57.3*ones(length(tRef),1),'k','LineWidth',2), grid on; axis tight; hold on
%ylabel('\psi (deg)'), xlabel('time (sec)'); grid on; hold on

% spreadfigures

fprintf('\nMean Radial error = %6.3f \t Maximum Radial error = %6.3f \n',mean(rerr),norm(rerr,Inf));

return;

tind(1)=1;
for i=1:5
    tind(i+1) = min(find(comLoops==i));
end

for i=1:5
    rerr1(i) = mean(sqrt(xOut(10,tind(i):tind(i+1)).^2+(xOut(11,tind(i):tind(i+1))-110.5).^2)-110.5);
    MPE(i) = mean(rerr(tind(i):tind(i+1)));
end

