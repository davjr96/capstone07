ipaddress = '192.168.8.250';
rosinit(ipaddress)

%vars for traveling forward
target_distance = 1;
linear_speed = .1; %m/s
distance_travelled = 0;

%vars for rotating
current_theta = 0;
target_theta = 3.14;
relative_angle = target_theta - current_theta;
current_angle = 0;
angular_speed = .5; %rads/s

robot = rospublisher('/mobile_base/commands/velocity') ;
velmsg = rosmessage(robot);
r = rosrate(10);

reset(r);
velmsg.Angular.Z = angular_speed;
while(current_angle < relative_angle)
   time = r.TotalElapsedTime;
   send(robot,velmsg);
   current_angle = current_angle + angular_speed * .1
   waitfor(r);
end
velmsg.Angular.Z = 0;

reset(r);

velmsg.Linear.X = linear_speed;
while(distance_travelled < target_distance)
   time = r.TotalElapsedTime;
   send(robot,velmsg);
   distance_travelled = distance_travelled + linear_speed * .1
   waitfor(r);
end

rosshutdown;

 
     