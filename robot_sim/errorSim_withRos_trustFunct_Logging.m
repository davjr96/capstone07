clearvars;
close all;

%%%Time Setup%%%
kMax=10;
ti = 1;
dt = 1;


numBots = 3;
path = robot4;
path.init(1,3,kMax);
bot(1, numBots) = robot4;
rosOn = 0;
decay = 0.8;

for b = 1:numBots
    bot(b).init(numBots, 3,kMax); %%%history we keep, numBots, state space
end

path.pose = [0 0 pi/2]';
path.estimate = [0 -2 pi/2]';
path.u = [2 2];

bot(1).pose = [1 0 pi/2]';
bot(1).estimate = [1 0 pi/2]';
bot(1).u = [2 2];

bot(2).pose = [2 0 pi/2]';
bot(2).estimate = [2 0 pi/2]';
bot(2).u = [2 2];

bot(3).pose = [3 0 pi/2]';
bot(3).estimate = [3 0 pi/2]';
bot(3).u = [2 2];

if rosOn == 1
    ipaddress = "192.168.8.250";
    rosinit(ipaddress);
    pub1 = rospublisher('/tb2_2/mobile_base/commands/velocity') ;
    pub2 = rospublisher('/tb2_3/mobile_base/commands/velocity') ;
    pub3 = rospublisher('/tb2_5/mobile_base/commands/velocity') ;
    
    vicon_sub_2 = rossubscriber('/vicon/turtlebot_2/turtlebot_2');
    vicon_data_2 = receive(vicon_sub_2, 1);
    
    vicon_sub_3 = rossubscriber('/vicon/turtlebot_3/turtlebot_3');
    vicon_data_3 = receive(vicon_sub_3, 1);
    
    vicon_sub_5 = rossubscriber('/vicon/turtlebot_5/turtlebot_5');
    vicon_data_5 = receive(vicon_sub_5, 1);
    
    bot(1).pose = [vicon_data_2.Transform.Translation.X vicon_data_2.Transform.Translation.Y vicon_data_2.Transform.Rotation.Z]';
    bot(1).estimate = [vicon_data_2.Transform.Translation.X vicon_data_2.Transform.Translation.Y vicon_data_2.Transform.Rotation.Z]';
    bot(1).u = [2 2];
    
    bot(2).pose = [vicon_data_3.Transform.Translation.X vicon_data_3.Transform.Translation.Y vicon_data_3.Transform.Rotation.Z]';
    bot(2).estimate = [vicon_data_3.Transform.Translation.X vicon_data_3.Transform.Translation.Y vicon_data_3.Transform.Rotation.Z]';
    bot(2).u = [2 2];
    
    bot(3).pose = [vicon_data_5.Transform.Translation.X vicon_data_5.Transform.Translation.Y vicon_data_5.Transform.Rotation.Z]';
    bot(3).estimate = [vicon_data_5.Transform.Translation.X vicon_data_5.Transform.Translation.Y vicon_data_5.Transform.Rotation.Z]';
    bot(3).u = [2 2];
end

%Make path

%Global time loop
for k = 1:1:kMax
    t=ti+k*dt
    k
    for i = 1:numBots
        %Update SS
        %error calculations
        %    if t == 5
        %     fail(bot(1));
        %    end
        %
        %    if t == 10
        %     recover(bot(1));
        %    end
        
        %SS(i, t) = bot(i).pose;
        update(bot(i), dt);
        kalman(bot(i), dt);
        update(path, dt);
        kalman(path, dt);
        %
        boterrb=zeros(3,3);
        boterrc=zeros(3,3);
        errplaneb=zeros(3,3);
        errplanec=zeros(3,3);
        errc=[];
        errb=[];
        for j=1:numBots
            
            errc=bot(i).getErrorc(path.estimate, bot(i).estimate);
            errb=bot(i).getErrorb(path.estimate, bot(i).estimate);
            
            
            bot(i).errMatc(:,j)=errc;
            bot(i).errMatb(:,j)=errb;
            %    bot.errMatb=boterrb;
        end
        bot(i).updateM(bot(i).errMatc,k, 0);
        bot(i).updateM(bot(i).errMatb,k, 1);
        
        bot(i).updateTrendc(k,dt);
        bot(i).InstantTrustc(k);
        bot(i).updateRepc(k);
        bot(i).updateALTrustc(k);
        %
        bot(i).updateTrendb(k,dt);
        bot(i).InstantTrustb(k)
        bot(i).updateRepb(k);
        bot(i).updateALTrustb(k);
        
        %%%%%%%%
        figure(1)
        % plot(path.pose(1), path.pose(2), 'g^');
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


%fprintf("t: %f | Gamma: %f | Weighted: %f \n",t,Gamma,Gammaw)
%
% Plotting Current Trust Value
% figure(2);
% hold on
% %plot(1:length(XerrorList), Gamma, 'go','MarkerSize',5)
%
% plot(t*10,Gammaw,'ro','MarkerSize',5)
% title('Reputation (gamma)');
% legend('Gamma');
%plot(t,GammaDecay,'r^','MarkerSize',5)

%
% plot(t,Trend*Gamma,'bd','MarkerSize',5)
%
%  figure(3);
% % plot(t, S, 'or','MarkerSize',5)
%  hold on
%  plot(t,Trend,'r.','MarkerSize',5)
%  title('Trend');

% plot(t,Trend2,'m.','MarkerSize',5)
% plot(t,U,'ob','MarkerSize',5)
% % plot(t,Trend,'k*')
%
% figure(Mplot);
% plot(t,M(end),'k*','MarkerSize',5)
% hold on
if rosOn == 1
    rosshutdown;
end


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



