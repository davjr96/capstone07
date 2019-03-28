ipaddress = '192.168.8.250';
rosinit(ipaddress)

robot = rospublisher('/mobile_base/commands/velocity') ;

rotate(3.14, robot)
drive(1 ,robot)
    
rosshutdown;

function rotate (relative_angle, pub)
    current_angle = 0;
    angular_speed = .5; %rads/s
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

function drive(distance, pub)
    %vars for traveling forward
    target_distance = 1;
    linear_speed = .1; %m/s
    distance_travelled = 0;
    r = rosrate(10);
    reset(r);
    velmsg = rosmessage(pub);
    velmsg.Linear.X = linear_speed;
    while(distance_travelled < target_distance)
       time = r.TotalElapsedTime;
       send(pub,velmsg);
       distance_travelled = distance_travelled + linear_speed * .1
       waitfor(r);
    end
end


     