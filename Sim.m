clearvars;
close all;
%% "steps"
kMax = 500;
tMax = 50;
dt = tMax/kMax;
t = 1;
%% Creating the bots
numBots = 6;
bot(1, numBots) = robot;

%% Creating Reference Signals

%tau=linspace(1,tMax,kMax);
%r1 = [tau;tau.^3+tau.^2+tau+2;zeros(1,length(tau))];
% r1 = [tau;sin(tau)+2;zeros(1,length(tau))];
% for i=1:1:length(r1)
%    frac = r1(2,i)/r1(1,i);
%    r1(3,i)=atan((r1(2,i)-2));
% end
% plot(r1(1,:),r1(2,:));

%% Setting Initial Conditions
bot(1).pose = [5 -10 1]';
bot(1).estimate = [4 -6 1.6]';
bot(1).u = [1 0];
bot(2).pose = [7 7 0]';
bot(2).estimate = [3 5 2]';
bot(2).u = [1 2];
bot(3).pose = [-15 6 2]';
bot(3).estimate = [-13 9 2]';
bot(3).u = [1 2];
bot(4).pose = [1 9 1]';
bot(4).estimate = [0 6 1]';
bot(4).u = [4 2];
bot(5).pose = [-8 -8 1]';
bot(5).estimate = [-8 -8 0]';
bot(5).u = [.5 .5];
bot(6).pose = [1 2.02 0.0399786871232900]';
bot(6).estimate = [0 2 0]';
bot(6).u = [0 0];
%[x y uL uR cos sin omega]'
for b = 1:numBots
    bot(b).init(tMax);
end
%SS = cell(numBots, tMax);

%Global time loop
r1 = [];
ts = [];
for k=1:1:kMax
    ts = [ts t];
    r1 = [r1 [t;t^2/50+2;atan(t/25)]];
for i = 1:numBots
    
   %Update SS
   if t > 1
    logging(bot(i),k);
   end
   %SS(i, t) = bot(i).pose;
   update(bot(i), dt);
   kalman(bot(i), dt);
   control(bot(6),r1(:,end))
  
%%%%%%%%
%Update MSS

% for j = 1:numBots % perspective: bot being updated
%     for n = 1:numBots %this is the bot index that j is looking at
%         if j == n %if bot is looking at itself
%             bot(j).MSS(j,1) = bot(j).pose;
%         elseif n ~=j % if bot(j) is looking at a different bot
%             bot(j).MSS(n,1) = bot(n).pose + L; %add lidar noise
%         end
%     end
% end
%             
%            
% %%%%%%
% colors = ['r','g','b','m','k','c'];
% plot(bot(i).pose(1),bot(i).pose(2),strcat(colors(i),'*'));%,plot_matrix(2,n),plot_matrix(3,k), 'b'); hold on; %plotting x and y of the actual simulation
% hold on;
% plot(bot(i).estimate(1), bot(i).estimate(2),strcat(colors(i),'^'));
% pause(.000001);
% 
% %title('Actual vs Measurement')
% %legend({' = Robot 1', ' = Robot 2'})
%  xlim([-1 20]);
%  ylim([-1 20]);
% % xlabel('Position X (m)')
% % ylabel('Position Y (m)')

end   
t = t+dt;
end

l1 = animatedline(bot(1).pose_hist(1,1),bot(1).pose_hist(2,1),'Color','b','LineWidth',2);
e1 = animatedline(bot(1).est_hist(1,1),bot(1).est_hist(2,1),'Color','b','LineStyle','--');
l2 = animatedline(bot(2).pose_hist(1,1),bot(2).pose_hist(2,1),'Color','r','LineWidth',2);
e2 = animatedline(bot(2).est_hist(1,1),bot(2).est_hist(2,1),'Color','r','LineStyle','--');
l3 = animatedline(bot(3).pose_hist(1,1),bot(3).pose_hist(2,1),'Color','c','LineWidth',2);
e3 = animatedline(bot(3).est_hist(1,1),bot(3).est_hist(2,1),'Color','c','LineStyle','--');
l4 = animatedline(bot(4).pose_hist(1,1),bot(4).pose_hist(2,1),'Color','m','LineWidth',2);
e4 = animatedline(bot(4).est_hist(1,1),bot(4).est_hist(2,1),'Color','m','LineStyle','--');
l5 = animatedline(bot(5).pose_hist(1,1),bot(5).pose_hist(2,1),'Color','k','LineWidth',2);
e5 = animatedline(bot(5).est_hist(1,1),bot(5).est_hist(2,1),'Color','k','LineStyle','--');
% l6 = animatedline(bot(6).pose_hist(1,1),bot(6).pose_hist(2,1),'Color','g','LineWidth',2);
% e6 = animatedline(bot(6).est_hist(1,1),bot(6).est_hist(2,1),'Color','g','LineStyle','--');
% r = animatedline(r1(1),ts(1));
xlabel('Position X')
ylabel('Position Y')
title('Real Position vs. Estimate')
for k = 1:1:kMax    
addpoints(l1,bot(1).pose_hist(1,k),bot(1).pose_hist(2,k))
addpoints(e1,bot(1).est_hist(1,k),bot(1).est_hist(2,k))

addpoints(l2,bot(2).pose_hist(1,k),bot(2).pose_hist(2,k))
addpoints(e2,bot(2).est_hist(1,k),bot(2).est_hist(2,k))

addpoints(l3,bot(3).pose_hist(1,k),bot(3).pose_hist(2,k))
addpoints(e3,bot(3).est_hist(1,k),bot(3).est_hist(2,k))

addpoints(l4,bot(4).pose_hist(1,k),bot(4).pose_hist(2,k))
addpoints(e4,bot(4).est_hist(1,k),bot(4).est_hist(2,k))

addpoints(l5,bot(5).pose_hist(1,k),bot(5).pose_hist(2,k))
addpoints(e5,bot(5).est_hist(1,k),bot(5).est_hist(2,k))

% addpoints(l6,bot(6).pose_hist(1,k),bot(6).pose_hist(2,k))
% addpoints(e6,bot(6).est_hist(1,k),bot(6).est_hist(2,k))
% addpoints(r,r1(1,k),r1(2,k))
drawnow

end
