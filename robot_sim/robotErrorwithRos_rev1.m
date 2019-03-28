clearvars;
close all;
numBots = 1;
path = robot;
path.init();
bot(1, numBots) = robot;
S=0;
U=0;
oldgamma=0;
rosOn = 1;
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
path.pose = [2 -2 pi/2]';
path.estimate = [2 -2 pi/2]';
path.u = [2 2];
bot(1).pose = [2 -2 pi/2]';
bot(1).estimate = [2 -2 pi/2]';
bot(1).u = [2 2];
XerrorList = [];
YerrorList = [];
ThetaErrorList = [];
mXHist = [];
%[x y uL uR cos sin omega]'
tMax = 20;
dt = .1;
ti = 1;

%SS = cell(numBots, tMax);
if rosOn == 1
ipaddress = "192.168.8.250";
rosinit(ipaddress);
% velocity = .5;
% turn = bot(1).pose(3);
robot = rospublisher('/mobile_base/commands/velocity');
end
% velmsg = rosmessage(robot);
% velmsg.Linear.X = velocity;
% velmsg.Angular.Z = turn;
% 
% x = 0;
% r = rosrate(10);
% reset(r);
% while x < 5
%     time = r.TotalElapsedTime;
%     send(robot, velmsg);
%     x = x + 1;
%     waitfor(r);
% end
%rosshutdown;

%Make path

%Global time loop
for t = 1:dt:tMax

for i = 1:numBots
   %Update SS
   %error calculations
   if t == 5
    fail(bot(1));
   end
   
   if t == 10
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
   
   
   %Tuning happens here
   M = [XerrorList ; YerrorList; ThetaErrorList]';
   
   Tinst = 0;
if M(end) <= 1 %Success Counter
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
    DM = diff(M(end-history:end));
    Trend = min([1, 1-5*mean(DM)/M(end)]);
    DDM = diff(DM);
    Trend2 = mean(DDM)/DM(end);
  
  % MX = ((bot.estimate(1) - path.estimate(1)).^2)./((k^2)*diag(path.P)); %M of X
   %mXHist = [mXHist, MX];
%%%%%%%%
figure(1)
plot(path.pose(1), path.pose(2), 'g^');
%plot(bot(i).pose(1),bot(i).pose(2),'r*');%,plot_matrix(2,n),plot_matrix(3,k), 'b'); hold on; %plotting x and y of the actual simulation
hold on;
plot(bot(i).estimate(1), bot(i).estimate(2), 'b^');
legend('Goal Path','Robot Position');
legend('location', 'northwest');
title('Position');
pause(.000001);

%title('Actual vs Measurement')
%legend({' = Robot 1', ' = Robot 2'})
%  xlim([-5 40]);
%  ylim([-5 40]);
% xlabel('Position X (m)')
% ylabel('Position Y (m)')

end      
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
%plot(1:length(XerrorList), Gamma, 'go','MarkerSize',5)

plot(t*10,Gammaw,'ro','MarkerSize',5)
title('Reputation (gamma)');
legend('Gamma');
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
if rosOn == 1
velmsg = rosmessage(robot);
velmsg.Linear.X = .3;
velmsg.Angular.Z = (bot(1).estimate(3)/10);

x = 0;
r = rosrate(10);
reset(r);

    time = r.TotalElapsedTime;
    send(robot, velmsg);
    x = x + 1;
    waitfor(r);
end
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


