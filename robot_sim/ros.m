ipaddress = '192.168.8.250';
rosinit(ipaddress)

current_theta = 0;
target_theta = 3.14;
relative_angle = target_theta - current_theta;
current_angle = 0;
angular_speed = .5; %rads/s
robot = rospublisher('/mobile_base/commands/velocity') ;
velmsg = rosmessage(robot);
velmsg.Angular.Z = angular_speed;
t0 = cputime;
r = rosrate(10);
reset(r);

counter = 1;
while(current_angle < relative_angle)
   time = r.TotalElapsedTime;
   send(robot,velmsg);
   current_angle = angular_speed * (.1 * counter)
   counter = counter + 1;
   waitfor(r);
end
rosshutdown;

 
     