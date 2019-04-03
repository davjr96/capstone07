clearvars;
close all;
numBots = 1;
path = robot;
path.init();
path2 = robot;
path2.init();
bot(1, numBots) = robot;
S=0;
U=0;
oldgamma=0;

ipaddress = '192.168.8.250';
rosinit(ipaddress)

pub = rospublisher('/mobile_base/commands/velocity') ;


decay = 0.8;
M=0;
%Mf = Q*P;% @Pat use the real M formula (xobsv-xhat)^2/(sigmax^2)
history = 30;

a=1;
b=3;
c=a;
for b = 1:numBots
    bot(b).init();
end

k = 1;
%r(t) as an expected reference path

%M = [Mx My Mtheta]' %metric
path.pose = [2 -2 1]';
path.estimate = [2 -2 1]';
path.u = [2 2];
bot(1).pose = [2 -2 1]';
bot(1).estimate = [2 -2 1]';
bot(1).u = [2 2];
path2.pose = [2 -2 1]';
path2.estimate = [2 -2 1]';
path2.u = [2 2];
bot(1).pose = [2 -2 1]';
bot(1).estimate = [2 -2 1]';
bot(1).u = [2 2];

XerrorList = [];
YerrorList = [];
ThetaErrorList = [];
mXHist = [];
%[x y uL uR cos sin omega]'
tMax = 20;
dt = 1;
ti = 1;
%SS = cell(numBots, tMax);
path2X = [];
path2Y = [];
path2T = [];
%Make path
for t = 1:dt:tMax
    update(path2, dt);
   kalman(path2, dt);
   
%     if t ==1
%         fail(path2(1))
%     end
%    
%    if t== 2
%        recover(path2(1))
%    end
   path2.estimate(3) = t/10;
   path2X = [path2X, path2.estimate(1)];
   path2Y = [path2Y, path2.estimate(2)];
   path2T = [path2T, path2.estimate(3)];
   figure(1)
   hold on;
   plot(path2.estimate(1), path2.estimate(2), 'go');
end

%Global time loop
for t = 1:dt:tMax

for i = 1:numBots
   %Update SS
   %error calculations
   if t == 1
    fail(bot(1));
   end
   
   if t == 2
    recover(bot(1));
   end
   
   %SS(i, t) = bot(i).pose;
   update(bot(i), dt);
   kalman(bot(i), dt);
   update(path, dt);
   kalman(path, dt);
   Xerror = ((path.estimate(1, 1) - bot(1).estimate(1, 1)).^2)./((k^2).*bot(1).P(1,1));
   %disp(Xerror)
   XerrorList = [XerrorList, Xerror];
   %disp(XerrorList')
   
   Yerror = ((path.estimate(2) - bot(1).estimate(2)).^2)./((k^2).*bot(1).P(2 , 2));%.^2);
   YerrorList = [YerrorList, Yerror];
   
   ThetaError = ((path.estimate(3) - bot(1).estimate(3)).^2)./((k^2).*bot(1).P(3 , 3));%.^2);
   ThetaErrorList = [ThetaErrorList, ThetaError];
   
   M = [XerrorList ; YerrorList; ThetaErrorList]';
   Mscalar = mean([XerrorList(end),YerrorList(end)]);
   Tinst = 0;
if Mscalar <= 1 %Success Counter
    Tinst = 1;
    S=S+1;
end
if Tinst == 0 %Unsuccessful Counter
   U=U+1; 
end
Gamma = (S-U)/(S+U);%Simple Reputation
Gammaw = (a*S-b*U)/(c*(S+U));%Reputation with DoF factors. a makes Gamma favor Success, b favor unsuccesses, c scales

GammaDecay = (1-decay)*oldgamma+decay*Gamma;%Irrelevant, is equivalent
oldgamma = GammaDecay;

% %Simple Trend Factor
% dM = [dM eval(dMf)];
% % sig = [sig sign(M(end)-M(end-1))];
% Trend = mean(dM./M);
Trend = 0;
Trend2 = 0;
% if t < ti+(history-1)*dt
%     Trend = 1-5*mean(diff(M)/M(end));
% end
if t >= ti+(history-1)*dt %%% Trend as a metric on the finite difference, very simple
    MeanM = mean(M(:,1:2),3);
    DM = diff(MeanM(end-history:end));
    Trend = 1-(mean(DM)/dt);
    DDM = diff(DM);
    Trend2 = mean(DDM)/DM(end);
else
    Trend=1;
end
Trust = Gammaw*Trend;
  % MX = ((bot.estimate(1) - path.estimate(1)).^2)./((k^2)*diag(path.P)); %M of X
   %mXHist = [mXHist, MX];
%%%%%%%%
figure(1)
%plot(path.pose(1), path.pose(2), 'g^');
%plot(bot(i).pose(1),bot(i).pose(2),'r*');%,plot_matrix(2,n),plot_matrix(3,k), 'b'); hold on; %plotting x and y of the actual simulation
hold on;
plot(bot(i).estimate(1), bot(i).estimate(2), 'b^');
legend('Goal Path','Robot Position');
legend('location', 'northwest');
title('Position');
pause(.000001);

%%%ROS Stuff Here
%to refer to the target path one time step ahead: path2X((dt*10)+1);
%path2Y(), path2T()


angle_to_travel = path2T(dt+1) - path2T(dt)
rotate(angle_to_travel , pub)
drive(.25, pub )

end

%new time loop, then try to catch the failure. Compare direction, check
%error between timesteps, and adjust/speed up to reduce error

  
     %disp(M);

     
     
 
     
 figure(4)    
 hold on
  plot(1:length(XerrorList), XerrorList, 'r*');
  plot(1:length(YerrorList), YerrorList, 'b*');
  plot(1:length(ThetaErrorList), ThetaErrorList, 'g*');
 %plot(1:length(mXHist), mXHist, 'g^');
 legend('X Error', 'Y Error', 'Theta Error');
 legend('location', 'northwest');

 title('Error');


%fprintf("t: %f | Gamma: %f | Weighted: %f \n",t,Gamma,Gammaw)
% 
% Plotting Current Trust Value
figure(2);
hold on
plot(t, Gamma, 'go','MarkerSize',5)

plot(t,Trust,'ro','MarkerSize',5)
title('Reputation (gamma)');
legend('Reputation-based Trust', 'Reputation & Trend-based Trust');
%plot(t,GammaDecay,'r^','MarkerSize',5)

% 
% plot(t,Trend*Gamma,'bd','MarkerSize',5)

 figure(3);
% plot(t, S, 'or','MarkerSize',5)
 hold on
 plot(t,Trend,'r.','MarkerSize',5)
 title('Trend');
 
% plot(t,Trend2,'m.','MarkerSize',5)
% plot(t,U,'ob','MarkerSize',5)
% % plot(t,Trend,'k*')
% 
% figure(Mplot);
% plot(t,M(end),'k*','MarkerSize',5)
% hold on
end
rosshutdown;
% figure(RepPlot)
% grid on
% title("Reputation Vs. Time")
% ylabel("Reputation (Gamma)")
% xlabel("Time (S)")
%legend()
% figure(SUPlot)
% grid on
% title("Successful/Unsuccessful Counts Vs. Time")
% ylabel("Counts")
% xlabel("Time (S)")
% legend("Successful","Unsuccessful");
% figure(Mplot)
% grid on
% title("Normalized Error Vs. Time")
% ylabel("Normalized Error (M)")
% xlabel("Time (S)")
% plot([ti,tMax],[1,1],'k--')

%system functions
function fail(rob)
rob.u = [2 1.9];

end

function recover(rob)
rob.u = [2 2.1];
end

function rotate (relative_angle, pub)
    current_angle = 0;
    angular_speed = .1; %rads/s
    r = rosrate(10);
    reset(r);
    velmsg = rosmessage(pub);
    velmsg.Angular.Z = angular_speed;
    while(current_angle < relative_angle)
       time = r.TotalElapsedTime;
       send(pub,velmsg);
       current_angle = current_angle + angular_speed * .1
       waitfor(r);
    end
    velmsg.Angular.Z = 0;
end

function drive(target_distance, pub)
    %vars for traveling forward
    
    linear_speed = .1; %m/s
    distance_travelled = 0;
    r = rosrate(10);
    reset(r);
    velmsg = rosmessage(pub);
    velmsg.Linear.X = linear_speed;
    while(distance_travelled < target_distance)
       time = r.TotalElapsedTime;
       send(pub,velmsg);
       distance_travelled = distance_travelled + linear_speed * .1;
       waitfor(r);
    end
end

