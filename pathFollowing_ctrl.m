function [CtrlDmd,Inte]=pathFollowing_ctrl(dt,yo,Inte,Ydots,~,angl,noise)

g=9.81;
psiRef = 0.0;
WPrad = 30;
global wpcmd completedLoops lpflag lpcounter
%Initial position angle and rates
U0=yo(:,1);
V0=yo(:,2);
W0=yo(:,3);
phi0=yo(:,4)+noise(:,5); 
theta0=yo(:,5)+noise(:,6); 
psi0=yo(:,6)+noise(:,7);
P0=yo(:,7)+noise(:,1);
Q0=yo(:,8)+noise(:,2);
R0=yo(:,9)+noise(:,3);
PN=yo(:,10)+noise(:,8);
PE=yo(:,11)+noise(:,9);
h=yo(:,12)+noise(:,10);

cth=angl(:,1);
sth=angl(:,2);
cph=angl(:,3);
sph=angl(:,4);
cps=angl(:,5);
sps=angl(:,6);

psi = psi0*180/pi;
psi(psi>180) = psi - 360; psi(psi<-180) = psi + 360;

Roll_IDemand=Inte(:,1);
Pit_IDemand=Inte(:,2);
integ6=Inte(:,3);
Udot=Ydots(:,1);
hdot=Ydots(:,2);
vacc=Ydots(:,3); % since YAW_SLIP is zero, vacc doesn't have any effect
V_dem_old=Inte(:,4);
h_dem_old=Inte(:,5);
integ7=Inte(:,6);
Yaw_IDemand=Inte(:,7);
rate_in_old=Inte(:,8);
rate_out_old=Inte(:,9);
integ5 = Inte(:,10);
integ4 = Inte(:,11);
integ3 = Inte(:,12);
integ2 = Inte(:,13);
integ1 = Inte(:,14);

%Wind
% Uw=wind(:,1);
% Vw=wind(:,2);
% Ww=wind(:,3);
% Uw=zeros(size(U0));Vw=zeros(size(U0));Ww=zeros(size(U0));

%Instantanious wind Velocity V, Angle of attack alpha, and Sideslip Beta

Temp=25;%degrees C
Press=9.46*10^4;
EAS2TAS=sqrt(1.225/(Press/(287.26*(273.15+Temp))));
% Vt=sqrt((U0+Uw).^2+(V0+Vw).^2+(W0+Ww).^2)+sensnois(:,1);
Vt = yo(:,13); % Includes sensor noise

%Ground Speed
PNdot=U0.*cth.*cps+V0.*(-cph.*sps+sph.*sth.*cps)+W0.*(sph.*sps+cph.*sth.*cps)+noise(:,4);
PEdot=U0.*cth.*sps+V0.*(cph.*cps+sph.*sth.*sps)+W0.*(-sph.*cps+cph.*sth.*sps)+noise(:,4);
cth=cos(theta0);
cph=cos(phi0);
sph=sin(phi0);

% Controller Gains
% %AirspeedScaler
% SpdScl=15;
% %Yaw
% YAW_RLL=1.1;
% YAW_SLIP=.2;
% YAW_INT=1;
% YAW_DAMP=.1;
% %Roll
% Roll_TC=.5;
% Roll_P=.95;%.4;
% Roll_I=0.05;
% Roll_D=.09;%.09;
% Roll_limit=180;
% RollK_P=((Roll_P-Roll_I*Roll_TC)*Roll_TC-Roll_D)/EAS2TAS;
% RollK_I=Roll_I*Roll_TC;
% %Pitch
% Pit_Roll=1.05;
% Pit_TC=.5;
% % Pit_P=.8;
% Pit_P=.8;%.8;%Tuned for wind robustness
% Pit_I=.15;
% % Pit_D=0.02;
% Pit_D=.02;%Tuned for wind performance
% Pit_U_limit=20;
% Pit_D_limit=-25;
% PitK_P=((Pit_P-Pit_I*Pit_TC)*Pit_TC-Pit_D)/EAS2TAS;
% PitK_I=Pit_I*Pit_TC;

if(1)
% Controller gains (Straight line)
%--------------------------------------------------------------------------------------
%AirspeedScaler
SpdScl=15; % SCALING SPEED

%Yaw
YAW_RLL = 1.0;
YAW_SLIP = 0.0;
YAW_INT = 0.0;
YAW_DAMP = 0.1;

%Roll
Roll_TC = 0.5;
Roll_P = 0.4;
Roll_I = 0.04;
Roll_D = 0.02;
RollK_P = ((Roll_P-Roll_I*Roll_TC)*Roll_TC-Roll_D)/EAS2TAS;
RollK_I = Roll_I*Roll_TC;

%Pitch
Pit_Roll = 1.0;
Pit_TC = 0.5;
Pit_P = 0.4; % Gives more robustness to wind and gusts
Pit_I = 0.04;
Pit_D = 0.02;
Pit_U_limit = 20*pi/180;
Pit_D_limit = -25*pi/180;
PitK_P = ((Pit_P-Pit_I*Pit_TC)*Pit_TC-Pit_D)/EAS2TAS;
PitK_I = Pit_I*Pit_TC;

%Throttle
Tcas_TC = 5.0;
T_D = 0.5;
T_I = 0.1; 
T_roll = 10.0;
Nom_T = 0.5691; %0.4716;
end

if(0)
% Controller gains (Circular)
%--------------------------------------------------------------------------------------
%AirspeedScaler
SpdScl=15; % SCALING SPEED

%Yaw
YAW_RLL = 1.0;
YAW_SLIP = 0.0;
YAW_INT = 0.0;
YAW_DAMP = 0.1;

%Roll
Roll_TC = 0.5;
Roll_P = 0.4;
Roll_I = 0.04;
Roll_D = 0.02;
RollK_P = ((Roll_P-Roll_I*Roll_TC)*Roll_TC-Roll_D)/EAS2TAS;
RollK_I = Roll_I*Roll_TC;

%Pitch
Pit_Roll = 1.0;
Pit_TC = 0.4; %0.5
Pit_P = 1.85; %0.4; % Gives more robustness to wind and gusts
Pit_I = 0.06; %0.04;
Pit_D = 0.02;
Pit_U_limit = 20*pi/180;
Pit_D_limit = -25*pi/180;
PitK_P = ((Pit_P-Pit_I*Pit_TC)*Pit_TC-Pit_D)/EAS2TAS;
PitK_I = Pit_I*Pit_TC;

%Throttle
Tcas_TC = 4.0; 
T_D = 0.5;
T_I = 0.1; 
T_roll = 10.0;
Nom_T = 0.4503;
end
%--------------------------------------------------------------------------------------

%Pitch demand
SKE_w = 1;%1; % Speed weight
Pitcmd_I = T_I; % Integrator gain
Pitrat_d = 0.0;
%LI gains
L1_Period = 20;
L1_Damp = 0.75;

%Demanded Velocity/Alt and Constraints
H_dem = 0*ones(size(h_dem_old));
V_dem = 19.66*ones(size(V_dem_old)); %14.9*ones(size(V_dem_old));
Vmin = 9;
Vmax = 25;%22;
maxClimb = 5;
maxSink = 5;
THR_max = 1;
THR_min = 0.3;

%Generall Controller Calcs
AirSpdScl=SpdScl./(Vt/EAS2TAS);

% WPrad=40;
% L1_Period=16;
% L1_Damp=1.75;
% WPrad=20;

%% L1 Controller-----------------------------------------------------------
% %Waypoint following
if(0)
%     wp=[0 0;
%         -2.783 100;
%         -33.2 225.4;
%         -105.5 312.8;
%         -211.5 341.3;
%         -320.3 274.4;
%         -337.9 162.2;
%         -278.4 67.09;
%         -140.1 7.687;
%         126 -5.584;
%         271.4 -61.33;
%         336.1 -155.4;
%         330.1 -255.6;
%         250.2 -332.3;
%         133.5 -328.3;
%         41.47 -241.5;
%         4.38 -116.3];

     wp=[0 0;
        100 -2.783 ;
        225.4 -33.2;
        312.8 -105.5;
        341.3 -211.5;
        274.4 -320.3;
        162.2 -337.9;
        67.09 -278.4;
        7.687 -140.1;
        -5.584 126;
        -61.33 271.4;
        -155.4 336.1;
        -255.6 330.1;
        -332.3 250.2;
        -328.3 133.5;
        -241.5 41.47;
        -116.3 4.38];
    
    wpdist = sqrt(sum((wp(wpcmd,:)-[PN,PE]).^2,2));
    if(wpdist<WPrad)
        wpcmd = wpcmd + 1;
    end
    if(wpcmd>size(wp,1))
        wpcmd = 1;
        completedLoops = completedLoops + 1;
        disp(completedLoops);
    end
    %Control for Waypoint following
    if(wpcmd>1)
    wpcmdold=wpcmd-1;
    else
    wpcmdold=size(wp,1);
    end
    
    wpold=wp(wpcmdold,:);
    wpnew=wp(wpcmd,:);
    
    K_L1=4*L1_Damp^2;
    gndspd=sqrt(PNdot.^2+PEdot.^2);
    L1_dist=1/pi*L1_Damp*L1_Period.*gndspd;
    aair=[PN,PE]-wpold;
    nordir=(wpnew-wpold)./(sqrt(sum((wpnew-wpold).^2,2))*ones(1,size(wpnew,2)));
    
    xtracv=PNdot.*nordir(:,2)-PEdot.*nordir(:,1);
    ltracv=dot([PNdot,PEdot],nordir,2);
    Nu2=atan2(xtracv,ltracv);
    
    xtrackErr=aair(:,1).*nordir(:,2)-aair(:,2).*nordir(:,1);
    
    sinNu1=xtrackErr./L1_dist;
    sinNu1(sinNu1>0.7071)=0.7071;
    sinNu1(sinNu1<-0.7071)=-0.7071;
    Nu1=asin(sinNu1);
    Nu=Nu1+Nu2;
    latAccDem=K_L1.*gndspd.^2./L1_dist.*sin(Nu);
    nu = Nu;
end
%% Loiter Command
%
if(1)
    LoitDir=1;
    Center_wp=[0,110.75];
    wpdist = sqrt(sum(([0,0]-[PN,PE]).^2,2));

    if(wpdist<WPrad & lpflag==0)
        completedLoops = completedLoops + 1;
        lpflag = 1;
        disp(completedLoops);
    end
    
    if(wpdist>WPrad & lpflag==1)
        lpcounter = lpcounter+1;
        if(lpcounter>10)
        lpflag = 0;
        lpcounter=0;
        end
    end
    
    Radius=99.87*EAS2TAS^2;
    omega=2*pi/L1_Period;
    Kx=omega^2;
    Kv=2*L1_Damp*omega;
    
    gndspd=sqrt(PNdot.^2+PEdot.^2);
    L1_dist=1/pi*L1_Damp*L1_Period.*gndspd;
    K_L1=4*L1_Damp^2;
    
    aair=[PN,PE]-Center_wp;
    aairnorm=aair./(sqrt(sum((aair).^2,2))*ones(1,size(aair,2)));
    xtracv=-1*(PNdot.*aairnorm(:,2)-PEdot.*aairnorm(:,1));
    ltracv=-dot([PNdot,PEdot],aairnorm,2);
    Nu=atan2(xtracv,ltracv);
    
    xtracVelCirc=-ltracv;
    xtracErr=sqrt(sum((aair).^2,2))-Radius;
    velTangent=xtracv.*LoitDir;
    
    latAccDemPD=(xtracErr.*Kx+xtracVelCirc.*Kv);
    if ltracv<0 & velTangent<0
        latAccDemPD=max(latAccDemPD,0);
    end
    
    latAccDemCtr=velTangent.^2./max(.5*Radius,Radius+xtracErr);
    
    latAccDemCap=K_L1.*gndspd.*gndspd/L1_dist*sin(Nu);
    latAccDemCirc=LoitDir*(latAccDemPD+latAccDemCtr);
    
    if xtracErr>0 & LoitDir*latAccDemCap<LoitDir*latAccDemCirc
        latAccDem=latAccDemCap;
    else
        latAccDem=latAccDemCirc;
    end
end

%% Heading hold guidance logic

% omegaA = sqrt(2)*pi/L1_Period;
% tgt_bearing = psiRef; % in deg
% nu = tgt_bearing - psi; % psi in deg (within +/- 180)
% nu(nu>360 || nu<-360) = mod(nu,360); nu(nu>180) = nu - 360; nu(nu<-180) = nu + 360;
% nu = nu*pi/180; % nu in radians
% gndspd=sqrt(PNdot.^2+PEdot.^2);
% VomegaA = gndspd*omegaA;
% nu(nu>pi/2) = pi/2; nu(nu<-pi/2) = -pi/2;
% latAccDem = 2*sin(nu)*VomegaA;

%%

Demandedroll=cth.*(atan(latAccDem/(g))*180/pi);
Demandedroll(Demandedroll>45)=45;
Demandedroll(Demandedroll<-45)=-45;
% if(abs(nu)<(5*pi/180) && abs(Demandedroll)<4)
%     Demandedroll = 0;
% end

% Second order complementary filter for velocity
omegaCF = 2.0;
Vdot = yo(:,14);
integ4_in = (Vt-integ5)*omegaCF*omegaCF;
integ4_in(integ5<3.1) = max(integ4_in,0.0);
integ4 = integ4 + integ4_in*dt;
integ5_in = integ4 + Vdot + (Vt-integ5)*omegaCF*sqrt(2);
integ5 = integ5 + integ5_in*dt;
integ5 = max(integ5,3.0);
Vcf = integ5;

% Third order complementary filter for height
omegahCF = 3.0;
hddot = yo(:,15);
integ1_in = (h-integ3)*omegahCF*omegahCF*omegahCF;
integ1 = integ1 + integ1_in*dt;
integ2_in = integ1 + hddot + (h-integ3)*omegahCF*omegahCF*3;
integ2 = integ2 + integ2_in*dt;
integ3_in = integ2 + (h-integ3)*omegahCF*3; 
integ3 = integ3 + integ3_in*dt;
hdotcf = integ2;
hcf = integ3;

%TECS Controller----------------------------------------------------------
Vt_rate_d=(V_dem-V_dem_old)./dt;
V_dem_old=V_dem;
H_rate_d=(H_dem-h_dem_old)./dt;
h_dem_old=H_dem;
%Energy Calculation (assumes known airspeed)
%###########################################################################
%### FUNC: update_energy (AP_TECS.cpp)
STEdot_max=maxClimb*g;
STEdot_min=-maxSink*g;
SPE_dem=H_dem*g;
SKE_dem=0.5*V_dem.^2;
SPEdot_dem=H_rate_d.*g;
SKEdot_dem=Vcf.*Vt_rate_d;
SPE_est=hcf*g;
SKE_est=.5*Vcf.^2;
% ste=SPE_est+SKE_est;
SPEdot=hdotcf*g;
SKEdot=Vcf.*Udot;
% pause

%Throtle Commands
%### FUNC: update_throttle (AP_TECS.cpp)
SPE_e=SPE_dem-SPE_est;
SPE_e(SPE_e>(.5*Vmax*Vmax-SKE_dem))=.5*Vmax*Vmax-SKE_dem(1);
SPE_e(SPE_e<.5*Vmin*Vmin-SKE_dem)=.5*Vmin*Vmin-SKE_dem(1);
STE_e=SPE_e+SKE_dem-SKE_est;
STEdot_dem=SPEdot_dem+SKEdot_dem;
if STEdot_dem>STEdot_max
    STEdot_dem=STEdot_max;
elseif STEdot_dem<STEdot_min
    STEdot_dem=STEdot_min;
end

STEdot_e=STEdot_dem-SPEdot-SKEdot;%Does not include filter from code as not currently including accleromiter noise

KSTE2T=1/(Tcas_TC*(STEdot_max-STEdot_min));
STEdot_dem=STEdot_dem+T_roll.*(1./(cph.*cph)-1);
ff_T=Nom_T+STEdot_dem/(STEdot_max-STEdot_min)*(THR_max-THR_min);
thtldmd=(STE_e+STEdot_e*T_D)*KSTE2T+ff_T;
integ6=integ6+(STE_e*T_I)*dt*KSTE2T;
integ6(integ6>0.3) = 0.3;
integ6(integ6<-0.3) = -0.3;
thtldmd=thtldmd+integ6;
%###########################################################################
%Pitch Demands
SPE_w=2-SKE_w;
SEB_dem=SPE_dem*SPE_w-SKE_dem*SKE_w;
SEBdot_dem=SPEdot_dem*SPE_w-SKEdot_dem*SKE_w;
SEB_e=SEB_dem-(SPE_est*SPE_w-SKE_est*SKE_w);
SEBdot_e=SEBdot_dem-(SPEdot*SPE_w-SKEdot*SKE_w);
integ7_i=SEB_e*Pitcmd_I;
integ7=integ7+integ7_i*dt;
gainInv=Vcf*g*Tcas_TC;
temp=SEB_e+SEBdot_e*Pitrat_d+SEBdot_dem*Tcas_TC;
Demandedpitch=(temp+integ7)./gainInv;
Demandedpitch(Demandedpitch>Pit_U_limit)=Pit_U_limit;
Demandedpitch(Demandedpitch<Pit_D_limit)=Pit_D_limit;
% Demandedpitch=10;

% Roll Controller---------------------------------------------------------
Roll_ratedemand=1/Roll_TC*(Demandedroll-phi0*180/pi);
% if Roll_ratedemand>Roll_limit
%     Roll_ratedemand=Roll_limit;
% elseif Roll_ratedemand<-Roll_limit
%     Roll_ratedemand=-Roll_limit;
% end
% Roll_ratedemand
% % P0*180/pi
% pause
Roll_RateErr=(Roll_ratedemand-P0*180/pi).*AirSpdScl;
Roll_IDemand=Roll_IDemand+dt.*RollK_I.*AirSpdScl.*Roll_RateErr;
Roll_IDemand(Roll_IDemand>15)=15;
Roll_IDemand(Roll_IDemand<-15)=-15;
Roll_DDemand=Roll_D*Roll_RateErr;
Roll_PDemand=RollK_P*Roll_ratedemand;
Ail_Demand=(Roll_PDemand+Roll_DDemand).*AirSpdScl+Roll_IDemand/1;
% Ail_Demand(Ail_Demand>15)=15;
% Ail_Demand(Ail_Demand<-15)=-15;
% Ail_Demand=-Ail_Demand;
% pause
% Pitch Controller--------------------------------------------------------
Pit_Comp=cth.*abs(g./Vt.*sph./cph.*sph.*180/pi).*Pit_Roll;%Need to add limit in case of high pitch angle
% Pit_ratedemand=1./Pit_TC*(Demandedpitch-theta0.*180/pi);
Pit_ratedemand=1./Pit_TC*((Demandedpitch-theta0)*180/pi);
% if Pit_ratedemand>180
%     Pit_ratedemand=180;
% elseif Pit_ratedemand<-180
%     Pit_ratedemand=-180;
% end
Pit_ratedemand1=Pit_ratedemand+Pit_Comp;
Pit_RateErr=(Pit_ratedemand1-Q0*180/pi).*AirSpdScl;
Pit_IDemand=Pit_IDemand+dt*PitK_I.*AirSpdScl.*Pit_RateErr;
Pit_IDemand(Pit_IDemand>5)=5;
Pit_IDemand(Pit_IDemand<-5)=-5;
Pit_DDemand=Pit_D*Pit_RateErr;
Pit_PDemand=PitK_P*Pit_ratedemand1;
ele_Demand=(Pit_PDemand+Pit_DDemand).*AirSpdScl+Pit_IDemand;
% ele_Demand = -ele_Demand; % For Senior Telemaster model with flipped
% elevator
% pause

%Yaw Controller-----------------------------------------------------------
rate_ofset=g./Vt.*sph./cph.*cph.*YAW_RLL;
% rate_ofset'
% Vt'
% phi0'
% if max(isnan(rate_ofset))==1
%     g
%     Vt'
%     phi0'
%     pause
% end
rate_in=(R0-rate_ofset)*180/pi;
rate_out=.996008*rate_out_old+rate_in-rate_in_old;
rate_in_old=rate_in;
rate_out_old=rate_out;
Yaw_IDemand=Yaw_IDemand-YAW_INT.*(YAW_SLIP*vacc+rate_out)*dt;
Yaw_IDemand(Yaw_IDemand>5)=5;
Yaw_IDemand(Yaw_IDemand<-5)=-5;
rdr_Demand=YAW_DAMP.*(Yaw_IDemand-rate_out).*AirSpdScl.*AirSpdScl;
rdr_Demand=rdr_Demand;
% rdr_Demand=-rdr_Demand;

%Inertial calc
Inte=[Roll_IDemand,Pit_IDemand,integ6,V_dem_old,h_dem_old,integ7,Yaw_IDemand,rate_in_old,rate_out_old,integ5,integ4,integ3,integ2,integ1];

%Input max control deflections
servo_lim = 30;
thtldmd(thtldmd>1)=1;
thtldmd(thtldmd<.3)=.3;
ele_Demand(ele_Demand>servo_lim)=servo_lim;
ele_Demand(ele_Demand<-servo_lim)=-servo_lim;
rdr_Demand(rdr_Demand>servo_lim)=servo_lim;
rdr_Demand(rdr_Demand<-servo_lim)=-servo_lim;
Ail_Demand(Ail_Demand>servo_lim)=servo_lim;
Ail_Demand(Ail_Demand<-servo_lim)=-servo_lim;

% Angles to PWM
% ele_PWM = 1512 + ele_Demand*(1930-1512)/15;
% ail_PWM = 1565 + Ail_Demand*(1980-1565)/15;
% rdr_PWM = 1520 + rdr_Demand*(1937-1520)/15;
ele_PWM = 1e-3*ele_Demand*(1930-1512)/servo_lim;
ail_PWM = 1e-3*Ail_Demand*(1980-1565)/servo_lim;
rdr_PWM = 1e-3*rdr_Demand*(1937-1520)/servo_lim;
tht_PWM = thtldmd;

% CtrlDmd=[ele_Demand,thtldmd,Ail_Demand,rdr_Demand];
CtrlDmd=[ele_PWM,ail_PWM,rdr_PWM,tht_PWM];
