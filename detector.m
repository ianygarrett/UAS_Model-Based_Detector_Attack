LowFidelityModel
%load Model_circle
load circleTrajLPV_2

%sr_hz should come from LowFidelityModel.m
if sr_hz == 25
%     if toggle == 0 %baseline
%         tau = 5.722; %25hz tau, 1 std dev meas noise, no dev model para, 
%         climit = 124.1; %25hz tau, 1 std dev meas noise, no dev model para,
%         mshift = 0.001;
%     end
%     tau = 14.1535;  %with wind
%     climit = 139.99; 
%     mshift = 0.002;
%       tau = 24;  %with noise
%       climit = 96;
%       mshift = 0.001;
      tau = 20.19;  %with model shift
      climit = 75.3;
      mshift = 0.001;
elseif sr_hz == 50
%     if toggle == 0
%         tau = 5.722; %1 std dev meas noise, no dev model para
%         %climit affects upper and lower control limit
%         climit = 122;  %1 std dev meas noise, no dev model para
%         mshift = 0.001;
%     end
%     tau = 14.1535;  %with wind
%     climit = 138.33; 
%     mshift = 0.001;
%       tau = 24;  %with noise
%       climit = 94;
%       mshift = 0.001;
      tau = 20.19;  %with model shift
      climit = 78.3;
      mshift = 0.001;
else 
%     if toggle == 0
%         tau = 5.722; 
%         climit = 125; %mean of xOut, the eer is at climit = 11.6, mshift = 1
%         mshift = 0.001;
%     end
%     tau = 14.1535;  %with wind
%     climit = 140.56; 
%     mshift = 0.001;
%       tau = 24;  %with noise
%       climit = 91;
%       mshift = 0.001;
      tau = 20.19;  %with model shift
      climit = 73.9;
      mshift = 0.001;
end

%%%Residual Detector%%%%%%

%Captures the euclidean distance between the cooridnates as the residual 
total_tau = 0;
attack_total_tau = 0;
ab_error_array = zeros(sr,1);
for i=1:1:sr
    residual = sqrt( (xOut(10,i*4)-dxdotarraykf(10,i))^2 + (xOut(11,i*4)-dxdotarraykf(11,i))^2 );
    atk_residual = sqrt( (xOut(10,i*4)-attack_state_array(10,i))^2 + (xOut(11,i*4)-attack_state_array(11,i))^2 );
    
    ab_error_array(i) = residual;
    
    if residual > tau
        total_tau=total_tau+1;
    end
    
    if atk_residual > tau
        attack_total_tau=attack_total_tau+1;
    end
end

disp("Total Tau Normal: " + total_tau);
disp("EER: " + (total_tau/sr*100) + "%");
disp("Total Tau Attack: " + attack_total_tau);
disp("EER: " + (sr-attack_total_tau)/sr*100 + "%");
ab_error = mean(ab_error_array);
disp("Absoluate Err: " + ab_error);

%Mean and Standard Deviation used for CUSUM
cs_mean = mean(xOut(10,:));
cs_std_dev = std(xOut(10,:));
cs_meany = mean(xOut(11,:));
cs_std_devy = std(xOut(11,:));

cs =cusum(dxdotarraykf(10,:),climit,mshift,cs_mean,cs_std_dev,'all');
csa = cusum(attack_state_array(10,:),climit,mshift,cs_mean,cs_std_dev,'all');
csy =cusum(dxdotarraykf(11,:),climit,mshift,cs_meany,cs_std_devy,'all');
csay = cusum(attack_state_array(11,:),climit,mshift,cs_meany,cs_std_devy,'all');
disp("CUSUM: " +(abs(numel(cs)+numel(csy))/2/sr*100) + "%");
disp("ATK CUSUM: " +(abs(sr-(numel(csa)+numel(csay)/2))/sr*100) + "%");
          %  [regchi,regp] = signrank(residual_array(12,:),yshift);
          %  [atkchi, atkp] = signrank(attackres_array(12,:),yshift);
            %regchi = CHI2TEST(residual_array(10,:),alpha);
            %atkchi = CHI2TEST(attackres_array(10,:),alpha);
            % disp("chi test");
            %  disp(regchi);
            %  disp(atkchi);
yshift = 0.001;
alpha = 0.05;

    %Highlight the point where the cumulative sum drifts more than five standard deviations beyond the target mean. 
    %Set the minimum detectable mean shift to one standard deviation.
    %figure(i)
    %cusum(attack_state_array(i,:),5,1,cs_mean,2*cs_std_dev);
    %figure(19*i)
    %cusum(xOut(i,:),5,1,cs_mean,cs_std_dev);

    
    %chi test across regular residuals
   % disp("regular");
   % disp(chi2gof(residual_array(i,:)));
    %chi test across attack residuals
  %  disp("under attack");
   % disp(chi2gof(attackres_array(i,:)));
%    for ind=1:1:500
%        if residual_array(i,ind) >= tau
%            % disp("Residual Alert!")
%            %disp(residual)
%            total_tau(i)=total_tau(i)+1;
%        end
%        if attackres_array(i,ind) >= tau
%            %disp("Attack Alert!")
%            %disp(residual)
%            attack_total_tau(i)=attack_total_tau(i)+1;
%        end
%    end
%    
% 
% 
% 
% disp("mean modeling error");
% disp(mean(residual_array(12))/sr);
% disp("total tau");
% disp(total_tau(12)/sr*100);
% disp("attack total tau");
% disp((sr-attack_total_tau(12))/sr*100);
% 
% 
% %disp(mean(residual_array));
% %disp(std(residual_array));
% %figure(99)
% %plot(yRef,xRef,'r','LineWidth',2);grid on; xlabel('EAST/WEST (m)');ylabel('NORTH/SOUTH (m)'); hold on;
% %plot(dxdotarray(11,1:sr),dxdotarray(10,1:sr),col,'LineWidth',2); hold on; grid on; hold on; 
% 
% %plot residuals for each of the 18 states
% %for i=1:1:18    
% %    figure(i);
% %    plot(residual_array(i,1:500)); 
% %5end
% 
% %Height Graph
% % figure(97)
% % plot(yRef,xRef,'r','LineWidth',2);grid on; xlabel('EAST/WEST (m)');ylabel('NORTH/SOUTH (m)'); hold on;
% % plot(yplot,xplot,'b','LineWidth',2); hold on; grid on; hold on; 
% % plot(dxdotarray(11,1:sr),dxdotarray(10,1:sr),'r','LineWidth',2); hold on; grid on; hold on;
% 
% %  figure(96)
% %  plot(yRef,xRef,'r','LineWidth',2);grid on; xlabel('Timesteps');ylabel('Height (m)'); hold on;
% %  plot(attack_state_array(12,1:sr),'r','LineWidth',2); hold on; grid on; hold on; 
% %  plot(dxdotarray(12,1:sr),'b','LineWidth',2); hold on; grid on; hold on;
% % %%%%%%%%%%%%%%%%%%%%%
% %Constant offset attack
% %%%%%%%%%%%%%%%%%%%%%%
% % tau = 1;
% % yhat = 0;
% % 
% % %x = xTrim2;
% % dx = xOut(:,1);
% % dx = dx(1:18);
% % %dx(10:12) = xyzRef(1);
% % %set UAS initial X/Y/H states
% % %x(10)=250; x(11)=50; x(12)=100;
% % 
% % %w = [wind(5:7,1);rand(10,1)];
% % y = ones(10,1); 
% % du = uOut(1);
% % 
% % yarray = zeros(10,1);
% % dxdotarray = zeros(18,1);
% % dxarray = zeros(18,1);
% % residual_array = zeros(18,500);
% % attackres_array = zeros(18,500);
% % attack_state_array = zeros(18,1);
% % 
% % %CUSUM threshold
% % cst = 0;
% % atk = 0;
% % k = 0;
% % ind = 0;
% % total_tau = zeros(1,18);
% % attack_total_tau = zeros(1,18);
% % for i = 0.04:0.04:19 %25hz = 0.04 at 60s
% %     k = k+4;
% %     ind = ind+1;
% %     w = [wind(:,k);rand(6,1)];
% %     %w = [wind(:,k);zeros(6,1)];
% %     du = uOut(:,k);
% %     dx = xOut(:,k);
% %     dy = yOut(:,k);
% %     dx = dx(1:18);
% %     dy = dy(1:18);
% %     dxdot = A*dx + B1*w + B2*du;
% %     dydot    = C2*dx + D21*w + D22*du;
% %     dydot = [dydot(1:3); zeros(3,1);dydot(5:10);zeros(6,1)];
% %     
% %     %Kalman Filter
% %     dxdot = dxdot + (dy - dydot);
% %   %  yarray = [yarray,dy];
% %     dxdotarray = [dxdotarray,dxdot];
% %     
% %    % residual = abs(xplot(k+1)-dxdot(10));
% %    dx =xOut(:,k+1);
% %    dx = dx(1:18);
% %    dxarray = [dxarray,dx];
% %    if atk == 5
% %       attackdx = dx +100;
% %       atk = 0;
% %    else
% %       attackdx = dx;
% %    end
% %        
% %    atk = atk +1;
% % 
% %    attack_state_array = [attack_state_array,attackdx];
% %    residual = abs(dx-dxdot);
% %    attackres = abs(attackdx-dxdot);
% %    residual_array(:,ind) = residual;
% %    attackres_array(:,ind) = attackres;
% % 
% %    %disp(k)
% % end
% % 
% % %mean and standard deviation used for cusum
% % cs_mean = 0;
% % cs_std_dev = 0;
% % cs = 0;
% % csa = 0;
% % 
% % for i=1:1:18
% %     tau = abs(mean(dxarray(i,:)));
% %     cs_mean = mean(dxarray(i,:));
% %     cs_std_dev = std(dxarray(i,:));
% %     if i == 10
% %         tau = 0.657;
% %         %climit affects upper and lower control limit
% %         climit = 8.1; 
% %         mshift = 1.25;
% %         yshift = 0.03;
% %         cs =cusum(dxdotarray(10,:),climit,mshift,cs_mean,cs_std_dev,'all');
% %         csa = cusum(attack_state_array(10,:),climit,mshift,cs_mean,cs_std_dev,'all');
% %         disp(numel(cs));
% %         disp(numel(csa));
% %         disp(numel(cs)/sr);
% %         disp((500-numel(csa))/sr);
% %         %[regchi,regp] = signrank(residual_array(10,:),yshift+median(residual_array(10,:)));
% %         %[atkchi, atkp] = signrank(attackres_array(10,:),yshift+median(attackres_array(10,:)));
% %     end
% % 
% %     %Highlight the point where the cumulative sum drifts more than five standard deviations beyond the target mean. 
% %     %Set the minimum detectable mean shift to one standard deviation.
% %     %figure(i)
% %     %cusum(attack_state_array(i,:),5,1,cs_mean,2*cs_std_dev);
% %     %figure(19*i)
% %     %cusum(xOut(i,:),5,1,cs_mean,cs_std_dev);
% % 
% %     
% %     %chi test across regular residuals
% %    % disp("regular");
% %    % disp(chi2gof(residual_array(i,:)));
% %     %chi test across attack residuals
% %   %  disp("under attack");
% %    % disp(chi2gof(attackres_array(i,:)));
% %     for ind=1:1:500
% %         if residual_array(i,ind) >= tau
% %            % disp("Residual Alert!") 
% %             %disp(residual)
% %             total_tau(i)=total_tau(i)+1;
% %         end
% %         if attackres_array(i,ind) >= tau
% %             %disp("Attack Alert!") 
% %             %disp(residual)
% %             attack_total_tau(i)=attack_total_tau(i)+1;
% %         end
% %     end
% % end
% % 
% % %figure(97)
% % %plot(yRef,xRef,'r','LineWidth',2);grid on; xlabel('EAST/WEST (m)');ylabel('NORTH/SOUTH (m)'); hold on;
% % %plot(yplot,xplot,'b','LineWidth',2); hold on; grid on; hold on; 
% % %plot(dxdotarray(11,1:sr),dxdotarray(10,1:sr),'r','LineWidth',2); hold on; grid on; hold on;
% % 
% % disp(total_tau);
% % disp(attack_total_tau);
% 
% %%%%%%%%%%%%%%%%
% %END Constant Offset Attack
% %%%%%%%%%%%%%%%%
% 
