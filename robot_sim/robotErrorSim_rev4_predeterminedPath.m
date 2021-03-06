clearvars;
close all;
numBots = 3;

path1X = [];
path2X = [];
path3X = [];
XerrorList1 = [];
XerrorList2 = [];
XerrorList3 = [];

path1Y = [];
path2Y = [];
path3Y = [];
YerrorList1 = [];
YerrorList2 = [];
YerrorList3 = [];

path1T = [];
path2T = [];
path3T = [];
ThetaErrorList1 = [];
ThetaErrorList2 = [];
ThetaErrorList3 = [];



path = robot;
path.init();
pathbot1 = robot;
pathbot1.init();
pathbot2 = robot;
pathbot2.init();
pathbot3 = robot;
pathbot3.init();
bot(1, numBots) = robot;
S=0;
U=0;
oldgamma=0;

ipaddress = '192.168.8.250';
rosinit(ipaddress)

pub1 = rospublisher('/tb2_2/mobile_base/commands/velocity') ;
pub2 = rospublisher('/tb2_3/mobile_base/commands/velocity') ;
pub3 = rospublisher('/tb2_5/mobile_base/commands/velocity') ;

vicon_sub_2 = rossubscriber('/vicon/turtlebot_2/turtlebot_2');
vicon_data_2 = receive(vicon_sub_2, 1);
vicon_data_2.Transform.Translation.X
vicon_data_2.Transform.Translation.Y
vicon_data_2.Transform.Rotation.Z

vicon_sub_5 = rossubscriber('/vicon/turtlebot_5/turtlebot_5');
vicon_data_5 = receive(vicon_sub_5, 1);
vicon_data_5.Transform.Translation.X
vicon_data_5.Transform.Translation.Y
vicon_data_5.Transform.Rotation.Z

vicon_sub_3 = rossubscriber('/vicon/turtlebot_3/turtlebot_3');
vicon_data_3 = receive(vicon_sub_3, 1);
vicon_data_3.Transform.Translation.X
vicon_data_3.Transform.Translation.Y
vicon_data_3.Transform.Rotation.Z

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

bot(1).pose = [vicon_data_2.Transform.Translation.X vicon_data_2.Transform.Translation.Y vicon_data_2.Transform.Rotation.Z]';
bot(1).estimate = [vicon_data_2.Transform.Translation.X vicon_data_2.Transform.Translation.Y vicon_data_2.Transform.Rotation.Z]';
bot(1).u = [2 2];

bot(2).pose = [vicon_data_3.Transform.Translation.X vicon_data_3.Transform.Translation.Y vicon_data_3.Transform.Rotation.Z]';
bot(2).estimate = [vicon_data_3.Transform.Translation.X vicon_data_3.Transform.Translation.Y vicon_data_3.Transform.Rotation.Z]';
bot(2).u = [2 2];

bot(3).pose = [vicon_data_5.Transform.Translation.X vicon_data_5.Transform.Translation.Y vicon_data_5.Transform.Rotation.Z]';
bot(3).estimate = [vicon_data_5.Transform.Translation.X vicon_data_5.Transform.Translation.Y vicon_data_5.Transform.Rotation.Z]';
bot(3).u = [2 2];

pathbot1.pose =  [vicon_data_2.Transform.Translation.X vicon_data_2.Transform.Translation.Y vicon_data_2.Transform.Rotation.Z]';
pathbot1.estimate =   [vicon_data_2.Transform.Translation.X vicon_data_2.Transform.Translation.Y vicon_data_2.Transform.Rotation.Z]';
pathbot1.u = [2 2];

pathbot2.pose = [vicon_data_3.Transform.Translation.X vicon_data_3.Transform.Translation.Y vicon_data_3.Transform.Rotation.Z]';
pathbot2.estimate = [vicon_data_3.Transform.Translation.X vicon_data_3.Transform.Translation.Y vicon_data_3.Transform.Rotation.Z]';
pathbot2.u = [2 2];

pathbot3.pose = [vicon_data_5.Transform.Translation.X vicon_data_5.Transform.Translation.Y vicon_data_5.Transform.Rotation.Z]';
pathbot3.estimate = [vicon_data_5.Transform.Translation.X vicon_data_5.Transform.Translation.Y vicon_data_5.Transform.Rotation.Z]';
pathbot3.u = [2 2];



XerrorList = [];
YerrorList = [];
ThetaErrorList = [];
mXHist = [];
%[x y uL uR cos sin omega]'
tMax = 10;
dt = 1;
ti = 1;
%SS = cell(numBots, tMax);
path2X = [];
path2Y = [];
path2T = [];
%Make path
for t = 1:dt:tMax
    update(pathbot1, dt);
   kalman(pathbot1, dt);
   update(pathbot2, dt);
   kalman(pathbot2, dt);
   update(pathbot3, dt);
   kalman(pathbot3, dt);
    if t ==1
        fail(pathbot1)
    end
   
   if t== 3
       recover(pathbot1)
   end
   pathbot1.estimate(3) = t/10;
   path1X = [path1X, pathbot1.estimate(1)];
   path1Y = [path1Y, pathbot1.estimate(2)];
   path1T = [path1T, pathbot1.estimate(3)];
   figure(1)
   hold on;
   plot(pathbot1.estimate(1), pathbot1.estimate(2), 'go');
   
   pathbot2.estimate(3) = t/10;
   path2X = [path2X, pathbot2.estimate(1)];
   path2Y = [path2Y, pathbot2.estimate(2)];
   path2T = [path2T, pathbot2.estimate(3)];
   figure(1)
   hold on;
   plot(pathbot2.estimate(1), pathbot2.estimate(2), 'g^');
   
   pathbot3.estimate(3) = t/10;
   path3X = [path3X, pathbot3.estimate(1)];
   path3Y = [path3Y, pathbot3.estimate(2)];
   path3T = [path3T, pathbot3.estimate(3)];
   figure(1)
   hold on;
   plot(pathbot3.estimate(1), pathbot3.estimate(2), 'go');
   
   
end

%Global time loop
for t = 1:dt:tMax


   %Update SS
   %error calculations
   if t == 1
    fail(bot(1));
   end
   
   if t == 3
    recover(bot(1));
   end
   
   


   %SS(i, t) = bot(i).pose;
   update(bot(1), dt);
   kalman(bot(1), dt);
   update(bot(2), dt);
   kalman(bot(2), dt);
   update(bot(3), dt);
   kalman(bot(3), dt);
   
   
   Xerror = ((path.estimate(1) - bot(1).estimate(1)).^2)./((k^2).*bot(1).P(1,1));
   %disp(Xerror)
   XerrorList = [XerrorList, Xerror];
   %disp(XerrorList')
   
   Yerror1 = ((path.estimate(2) - bot(1).estimate(2)).^2)./((k^2).*bot(1).P(2 , 2));%.^2);
   YerrorList1 = [YerrorList1, Yerror1];
   
   ThetaError1 = ((path.estimate(3) - bot(1).estimate(3)).^2)./((k^2).*bot(1).P(3 , 3));%.^2);
   ThetaErrorList1 = [ThetaErrorList1, ThetaError1];
   
   
   Xerror1 = ((path1X(dt) - bot(1).estimate(1)).^2)./((k^2).*bot(1).P(1,1));
   %disp(Xerror)
   XerrorList1 = [XerrorList1, Xerror1];
   %disp(XerrorList')
   
   Yerror1 = ((path1Y(dt) - bot(1).estimate(2)).^2)./((k^2).*bot(1).P(2 , 2));%.^2);
   YerrorList1 = [YerrorList1, Yerror1];
   
   ThetaError1 = ((path1T(dt) - bot(1).estimate(3)).^2)./((k^2).*bot(1).P(3 , 3));%.^2);
   ThetaErrorList1 = [ThetaErrorList1, ThetaError1];
   
   Xerror2 = ((path2X(dt) - bot(2).estimate(1)).^2)./((k^2).*bot(2).P(1,1));
   %disp(Xerror)
   XerrorList2 = [XerrorList2, Xerror2];
   %disp(XerrorList')
   
   Yerror2 = ((path2Y(dt) - bot(2).estimate(2)).^2)./((k^2).*bot(2).P(2 , 2));%.^2);
   YerrorList2 = [YerrorList2, Yerror2];
   
   ThetaError2 = ((path2T(dt) - bot(2).estimate(3)).^2)./((k^2).*bot(2).P(3 , 3));%.^2);
   ThetaErrorList2 = [ThetaErrorList2, ThetaError2];
   
   Xerror3 = ((path3X(dt) - bot(3).estimate(1)).^2)./((k^2).*bot(3).P(1,1));
   %disp(Xerror)
   XerrorList3 = [XerrorList3, Xerror3];
   %disp(XerrorList')
   
   Yerror3 = ((path3Y(dt) - bot(3).estimate(2)).^2)./((k^2).*bot(3).P(2 , 2));%.^2);
   YerrorList3 = [YerrorList3, Yerror3];
   
   ThetaError3 = ((path3T(dt) - bot(3).estimate(3)).^2)./((k^2).*bot(3).P(3 , 3));%.^2);
   ThetaErrorList3 = [ThetaErrorList3, ThetaError3];
   
   
   
   %%%%%%needto adjust trust
   
% %    M = [XerrorList ; YerrorList; ThetaErrorList]';
% %    
% %    
% %    
% %    Mscalar = mean([XerrorList(end),YerrorList(end)]);
% %    Tinst = 0;
% % if Mscalar <= 1 %Success Counter
% %     Tinst = 1;
% %     S=S+1;
% % end
% % if Tinst == 0 %Unsuccessful Counter
% %    U=U+1; 
% % end
% % Gamma = (S-U)/(S+U);%Simple Reputation
% % Gammaw = (a*S-b*U)/(c*(S+U));%Reputation with DoF factors. a makes Gamma favor Success, b favor unsuccesses, c scales
% % 
% % GammaDecay = (1-decay)*oldgamma+decay*Gamma;%Irrelevant, is equivalent
% % oldgamma = GammaDecay;
% % 
% % % %Simple Trend Factor
% % % dM = [dM eval(dMf)];
% % % % sig = [sig sign(M(end)-M(end-1))];
% % % Trend = mean(dM./M);
% % Trend = 0;
% % Trend2 = 0;
% % % if t < ti+(history-1)*dt
% % %     Trend = 1-5*mean(diff(M)/M(end));
% % % end
% % if t >= ti+(history-1)*dt %%% Trend as a metric on the finite difference, very simple
% %     MeanM = mean(M(:,1:2),3);
% %     DM = diff(MeanM(end-history:end));
% %     Trend = 1-(mean(DM)/dt);
% %     DDM = diff(DM);
% %     Trend2 = mean(DDM)/DM(end);
% % else
% %     Trend=1;
% % end
% % Trust = Gammaw*Trend;
% %   % MX = ((bot.estimate(1) - path.estimate(1)).^2)./((k^2)*diag(path.P)); %M of X
% %    %mXHist = [mXHist, MX];
% % %%%%%%%%
% % figure(1)
% % %plot(path.pose(1), path.pose(2), 'g^');
% % %plot(bot(i).pose(1),bot(i).pose(2),'r*');%,plot_matrix(2,n),plot_matrix(3,k), 'b'); hold on; %plotting x and y of the actual simulation
% % hold on;
% % plot(bot(i).estimate(1), bot(i).estimate(2), 'b^');
% % legend('Goal Path','Robot Position');
% % legend('location', 'northwest');
% % title('Position');
% % pause(.000001);
% % 
% % %%%ROS Stuff Here
% % %to refer to the target path one time step ahead: path2X((dt*10)+1);
% % %path2Y(), path2T()


angle_to_travel1 = path1T(dt+1) - path1T(dt)
rotate(angle_to_travel1 , pub1)
drive(.25, pub1 )

angle_to_travel2 = path2T(dt+1) - path2T(dt)
rotate(angle_to_travel2 , pub2)
drive(.25, pub2 )

angle_to_travel3 = path3T(dt+1) - path3T(dt)
rotate(angle_to_travel3 , pub3)
drive(.25, pub3)

end

%new time loop, then try to catch the failure. Compare direction, check
%error between timesteps, and adjust/speed up to reduce error

  
     %disp(M);

     
     
 
     
%  figure(4)    
%  hold on
%   plot(1:length(XerrorList), XerrorList, 'r*');
%   plot(1:length(YerrorList), YerrorList, 'b*');
%   plot(1:length(ThetaErrorList), ThetaErrorList, 'g*');
%  %plot(1:length(mXHist), mXHist, 'g^');
%  legend('X Error', 'Y Error', 'Theta Error');
%  legend('location', 'northwest');
% 
%  title('Error');
% 
% 
% %fprintf("t: %f | Gamma: %f | Weighted: %f \n",t,Gamma,Gammaw)
% % 
% Plotting Current Trust Value
% figure(2);
% hold on
% plot(t, Gamma, 'go','MarkerSize',5)
% 
% plot(t,Trust,'ro','MarkerSize',5)
% title('Reputation (gamma)');
% legend('Reputation-based Trust', 'Reputation & Trend-based Trust');
%plot(t,GammaDecay,'r^','MarkerSize',5)

% 
% plot(t,Trend*Gamma,'bd','MarkerSize',5)

%  figure(3);
% % plot(t, S, 'or','MarkerSize',5)
%  hold on
%  plot(t,Trend,'r.','MarkerSize',5)
%  title('Trend');
%  
% % plot(t,Trend2,'m.','MarkerSize',5)
% % plot(t,U,'ob','MarkerSize',5)
% % % plot(t,Trend,'k*')
% % 
% % figure(Mplot);
% % plot(t,M(end),'k*','MarkerSize',5)
% hold on

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
rob.u = [2 1.6];

end

function recover(rob)
rob.u = [2 2.4];
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
    
    linear_speed = .25; %m/s
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

