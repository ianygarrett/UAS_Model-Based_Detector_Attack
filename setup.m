% evaluate fastest method to solve ODE with discontinuities in input

T = 120; %simulation time (s)
Ts = 1/10; %rate at which we update inputs (s)
Ts_th = 0.9*Ts; %ODE function should update input near Ts (should always be > 0.5*Ts and < Ts)
%N = 3; %number of vehicles in platoon (including leader)

%if N < 2
%    disp('Error: platoon must have at least leader and one follower (N>1)');
%    return;
%end

% simulation parameters relevant to ode function
SIM = struct(...
    'Ts',Ts, ...
    'Ts_th', Ts_th);

n = T/Ts; %number of sample points
t = linspace(0,T-Ts,n); % time values of sample point instances

% vehicle states x = [position1,position2,...,positionN,speed1,...,speedN]'
% (note: defining states this way means we don't have to interleave
% variables in the solver, which is slow)
%x0 = zeros(2*N,n); % states for baseline performance
%x1 = zeros(2*N,n); % states for vectorised performance
%x2 = zeros(2*N,n); % states for maxSteps performance

% set initial values of vehicles
% add random error to initial states (for all vehicles; fix leader after)
%x0(:,1) = randn(2*N,1); x1(:,1) = randn(2*N,1); 
%x2(:,1) = randn(2*N,1); % followers have random errors

%x0(2*N,1) = 30; x1(2*N,1) = 30; x2(2*N,1) = 30; %leader has constant speed
%x0(N,1) = 0; x1(N,1) = 0; x2(N,1) = 0; %leader starts at position = 0


load param_stele_new2
load LevelTrim2 % Loads the trim values pertaining to straight and level flight
magCase = 1; % 1/2/3
constWind = [0; 0; 0]; %NORTH/EAST/DOWN
%wind = [randn(3,1); magCase; constWind]; wind(:,2) = wind;
wind = [zeros(3,1); magCase; constWind]; wind(:,2) = wind;
% Initial state and control input vectors  
xOut = [xTrim2; uTrim(4); 0.0; zeros(3,1)]; %xOut = [xTrim2; uTrim(4); 0.0; zeros(5,1)];
dtvec = [0 0.04]; % 0.04s is the timestep 
%set UAS initial X/Y/H states
%xOut(10)=250; xOut(11)=50; xOut(12)=100;

% indicies for vehicle position/speed and position/speed of their
% predecessor
%IND = struct(...
%    'p', 1:N, ...
%    's', N+1:2*N, ...
%    'p_p', zeros(N,1), ...
%    's_p', zeros(N,1));

%IND.p_p = IND.p + 1; %predecessors for forllowers
%IND.p_p(end) = N; %leader is own predecessor
%IND.s_p = IND.s + 1; %predecessors for forllowers
%IND.s_p(end) = 2*N; %leader is own predecessor

%% baseline: simulate between input updates
% control input for vehicles (we make it global because we don't want to
% have to redefine a new function wrapper for f.m at each iteration)
%global u;
%u = zeros(N,1);
global uOut;

uOut =  uTrim; 
uOut(2)=0.04;
uOut(3)=0.04;
% wrapper so we don't have to use global for IND when passing to f.m
%f_w = @(t,y)f(t,y,IND); 



tic;
for i = 2:n
    %wind(1:3,i) = randn(3,1);
	odefun = @(t,x) StandardSimDynamics(t,x,real(uOut(:,i-1)),wind(:,i),param,0);
    [~,xs] = ode23(odefun,dtvec,xOut(:,i-1));
	xOut(:,i) = xs(end,:)';
    wind(:,i+1) = wind(:,i);
	% Update the control input vector, for e.g., using a controller, or just simulate straight and level equilibrium flight by setting 
    uOut(:,i) = uTrim;
    %uOut(2)=0;
    %uOut(3)=0;
    
    %[~,xds] = ode45(f_w,t(i-1:i),x0(:,i-1));
    %x0(:,i) = xds(end,:)';
    
    % calculate control input for next time instance: kp = 1, h = 1, kd = 2
    % (these provide stable behavior see Yanakiev)
   % u = 1*(x0(IND.p_p,i) - x0(IND.p,i) - 1*x0(IND.s,i)) ...
    %    + 2*(x0(IND.s_p,i) - x0(IND.s,i)); % a = kp(pi+1 - pi - h*si) + kd(si+1 - si)
    %u(N) = 0; %leader acceleration should always be zero
end
t0 = toc;

%figure;xlabel('time [s]');plot(t,xOut);title('position (baseline)');
%figure;xlabel('time [s]');plot(t,xOut(10:11,1:1200));title('X/Y/H inertial positions in m ');
figure;xlabel('time [s]');plot(xOut(10,1:1200),xOut(11,1:1200));title('X/Y/H inertial positions in m ');
%figure;xlabel('time [s]');plot(t,uOut);title('speed (baseline)');
% 
% %% improvement two: max step instead of loop
% % idea: don't allow ODE to jump more than Ts steps ahead; we'll update the
% % input at approximately the right time
% %
% % SUGGESTION: use this for debugging code and baseline for generating final
% % results
% 
% % our function uses persistent variables, clear them
% %clear fms;
% 
% % DEBUG: keep track of when in simulation we actually upate input
% global ta;
% ta = zeros(n,1);
% 
% % set max ode step size to the system sample rate: we get approximately
% % this rate
% opt = odeset('MaxStep',Ts);
% 
% % wrapper so we don't have to use global for IND,SIM when passing to f.m
% %f_w = @(t,y)fms(t,y,IND,SIM); 
% 
% tic;
% wind(1:3,opt) = randn(3,1);
% odefun = @(t,x) StandardSimDynamics(t,x,real(uOut(:,opt-1)),wind(:,opt),param,0);
% [~,xs] = ode23(odefun,dtvec,xOut(:,opt-1));
% xOut(:,opt) = xs(end,:)';
% wind(:,opt+1) = wind(:,opt);
% % Update the control input vector, for e.g., using a controller, or just simulate straight and level equilibrium flight by setting 
% uOut(:,opt) = uTrim;
% %[~,xds] = ode45(f_w,t,x2(:,1),opt);
% %x2 = xds';
% t2 = toc;
% 
% figure;xlabel('time [s]');plot(xOut(10,1:1200),xOut(11,1:1200));title('X/Y/H inertial positions in m ');
% %figure;plot(t,x2(IND.p,:)');title('position (improvement two)');
% %figure;plot(t,x2(IND.s,:)');title('speed (improvement two)');
% 
% %rp = abs(x2(IND.p,:) - x0(IND.p,:));
% %rs = abs(x2(IND.s,:) - x0(IND.s,:));
% %figure;semilogy(t,rp');title('residual of improvement & baseline position');
% %figure;semilogy(t,rs');title('residual of improvement & baseline speed');
% %disp(['speed-up: ' num2str(t0/t2)]);
% 
% figure;plot(diff(ta(1:1100)));title(['difference between actual sample times (s) (should be' num2str(Ts) ')']);