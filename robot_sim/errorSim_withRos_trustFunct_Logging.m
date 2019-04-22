clearvars;
close all;

%%%Time Setup%%%
kMax=10;
dt = 1;

numBots = 3;

pathbot(1,numBots) = robot4;

pathX = zeros(numBots,kMax);
pathY = zeros(numBots,kMax);
pathT = zeros(numBots,kMax);


bot(1, numBots) = robot4;
rosOn = 1;
decay = 0.8;

for b = 1:numBots
    bot(b).init(numBots, 3,kMax); %%%history we keep, numBots, state space
    pathbot(b).init(numBots, 3,kMax);
end

bot(1).pose = [1 0 pi/2]';
bot(1).estimate = [1 0 pi/2]';
bot(1).u = [.1 .1];

bot(2).pose = [2 0 pi/2]';
bot(2).estimate = [2 0 pi/2]';
bot(2).u = [.1 .1];

bot(3).pose = [3 0 pi/2]';
bot(3).estimate = [3 0 pi/2]';
bot(3).u = [.1 .1];

pathbot(1).pose = [1 0 pi/2]';
pathbot(1).estimate = [1 0 pi/2]';
pathbot(1).u = [.1 .1];

pathbot(2).pose = [2 0 pi/2]';
pathbot(2).estimate = [2 0 pi/2]';
pathbot(2).u = [.1 .1];

pathbot(3).pose = [3 0 pi/2]';
pathbot(3).estimate = [3 0 pi/2]';
pathbot(3).u = [.1 .1];


if rosOn == 1
    ipaddress = "192.168.8.250";
    rosinit(ipaddress);
    
    
    pub(1) = rospublisher('/tb2_2/mobile_base/commands/velocity') ;
    pub(2) = rospublisher('/tb2_3/mobile_base/commands/velocity') ;
    pub(3) = rospublisher('/tb2_5/mobile_base/commands/velocity') ;
    
    vicon_sub(1) = rossubscriber('/vicon/turtlebot_2/turtlebot_2');
    vicon_data(1) = receive(vicon_sub(1), 1);
    
    vicon_sub(2) = rossubscriber('/vicon/turtlebot_3/turtlebot_3');
    vicon_data(2) = receive(vicon_sub(2), 1);
    
    vicon_sub(3) = rossubscriber('/vicon/turtlebot_5/turtlebot_5');
    vicon_data(3) = receive(vicon_sub(3), 1);
    
    for i = 1:numBots
        bot(i).pose = [vicon_data(i).Transform.Translation.X vicon_data(i).Transform.Translation.Y vicon_data(i).Transform.Rotation.Z]';
        bot(i).estimate = [vicon_data(i).Transform.Translation.X vicon_data(i).Transform.Translation.Y vicon_data(i).Transform.Rotation.Z]';
        bot(i).u = [.1 .1];
        
        pathbot(i).pose = [vicon_data(i).Transform.Translation.X vicon_data(i).Transform.Translation.Y vicon_data(i).Transform.Rotation.Z]';
        pathbot(i).estimate = [vicon_data(i).Transform.Translation.X vicon_data(i).Transform.Translation.Y vicon_data(i).Transform.Rotation.Z]';
        pathbot(i).u = [.1 .1];
    end
end

%Make path

for t = 1:dt:kMax
    
    for i = 1:numBots
        update(pathbot(i), dt);
        kalman(pathbot(i), dt);
                
        pathX(i, t) = pathbot(i).estimate(1);
        pathY(i, t) = pathbot(i).estimate(2);
        pathT(i, t) = pathbot(i).estimate(3);
        
        figure(1)
        hold on;
        plot(pathbot(i).estimate(1), pathbot(i).estimate(2), 'go');
        
    end
end

%Global time loop
for t = 1:dt:kMax
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
        
        errc=[];
        errb=[];
        
        if rosOn == 1
            vicon_data(i) = receive(vicon_sub(i), 1);
            bot(i).estimate(1) = vicon_data(i).Transform.Translation.X;
            bot(i).estimate(2) = vicon_data(i).Transform.Translation.Y;
            bot(i).estimate(3) = vicon_data(i).Transform.Rotation.Z;
        end
        
        for j=1:numBots
            
            errc=bot(i).getErrorc([pathX(i,t) pathY(i,t) pathT(i,t)]', bot(i).estimate);
            errb=bot(i).getErrorb([pathX(i,t) pathY(i,t) pathT(i,t)]', bot(i).estimate);
            
            bot(i).errMatc(:,j)=errc;
            bot(i).errMatb(:,j)=errb;
        
        end
        bot(i).updateM(bot(i).errMatc,t, 0);
        bot(i).updateM(bot(i).errMatb,t, 1);
        
        bot(i).updateTrendc(t,dt);
        bot(i).InstantTrustc(t);
        bot(i).updateRepc(t);
        bot(i).updateALTrustc(t);
        %
        bot(i).updateTrendb(t,dt);
        bot(i).InstantTrustb(t)
        bot(i).updateRepb(t);
        bot(i).updateALTrustb(t);
        
        angle_to_travel = pathT(i, dt+1) - pathT(i, dt)
        distance_to_travel = sqrt( (pathX(i, dt+1) - pathX(i, dt))^2 +  (pathY(i, dt+1) - pathY(i, dt))^2)
        
        if rosOn == 1
            rotate(angle_to_travel , pub(i))
            drive(distance_to_travel, pub(i) )
        end
        
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



