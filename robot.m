classdef robot < handle
   properties
       %kMax = 2500;
       %deltaT = 0.1; %sec
       pose_hist = []
       est_hist = []
       N = 10; %for the resilient part
       L = [1 ,1 ,0;1 ,1 ,0];
       mass = 1;
       I = 5;
       radius = 1; %m
       e = 0;
       esum = 0;
       P = zeros(3,3);
       variance_speed = 0.0001;
       Q = [];

       variance_gps = 1;
       R = [];
       
       
      
       pose = [0 0 1]';
       %[x y theta]
       %pose = [1 1 1 1 1 1 1]';
       estimate = [0 0 1]'; %[x y uL uR cos sin omega]'

       B = [0 0; 0 0; -1 1];
       u = [ 1 1 ]';

%        Fr = 0;
%        Fl = 0;

       E = [];


   end

   methods
       %%%Initialize Sys
       function init(rob, tmax)
           rob.Q = eye(3) * rob.variance_speed;
           rob.R = eye(3)*rob.variance_gps;
%            rob.B = [ 0 0; 0 0; 1/rob.mass+rob.radius/rob.I 1/rob.mass-rob.radius/rob.I; 1/rob.mass-rob.radius/rob.I 1/rob.mass+rob.radius/rob.I; 0 0; 0 0; -1/(2*rob.I) 1/(2*rob.I) ];
           rob.pose_hist = zeros(3,tmax);
           rob.pose_hist(:,1) = rob.pose;
        
           rob.est_hist = zeros(3,tmax);
           rob.est_hist(:,1) = rob.estimate;
        rob.E = zeros(7,rob.N);
       end
       %%%Update Pose
       function update(rob, dt)
       %update the 'actual' pose:
       
       for i1 = 1:3
           if rob.pose(i1) == 0
               rob.pose(i1) = .0000001;
           
           end
       end
       
       rob.pose = rob.pose + dt*[(rob.u(1) + rob.u(2)) * cos(rob.pose(3)) / 2;
           (rob.u(1) + rob.u(2)) * sin(rob.pose(3)) / 2;
           (rob.u(1) - rob.u(2))/(2*rob.radius)];
       rob.pose(3) = mod(rob.pose(3),2*pi);
           
           

%        %normalize sin,cos
%        a = rob.pose(5);
%        b = rob.pose(6);
%        len =sqrt(a^2 + b^2);
%        rob.pose(5) = rob.pose(5) / len;
%        rob.pose(6) = rob.pose(6) / len;
       end
       function control(rob,ref)
           Kp=0.01;
           Ki=0.01;
           turn_factor = 1;
          rob.e=sqrt(norm(ref(1:2)))-sqrt(norm(rob.estimate(1:2)));
          rob.esum=rob.esum+rob.e;
          V = Kp*rob.e+Ki*rob.esum;
          angle_diff = ref(3)-rob.estimate(3);
          rob.u = [V;V];
          if angle_diff > 0
              rob.u = rob.u.*[(1+turn_factor);(1-turn_factor)];
%               rob.u = rob.u.*[(1-turn_factor);(1+turn_factor)];
          elseif angle_diff < 0
              rob.u = rob.u.*[(1-turn_factor);(1+turn_factor)];
%               rob.u = rob.u.*[(1+turn_factor);(1-turn_factor)];
          end
          rob.u
       end
       function logging(rob,k)
           rob.pose_hist(:,k) = rob.pose;
           rob.est_hist(:,k) = rob.estimate;
       end
    function kalman(rob,dt)
         %prediction
         rob.estimate = rob.estimate + dt*[(rob.u(1) + rob.u(2)) * cos(rob.estimate(3)) / 2;
           (rob.u(1) + rob.u(2)) * sin(rob.estimate(3)) / 2;
           (rob.u(1) - rob.u(2))/(2*rob.radius)];
         rob.estimate(3) = mod(rob.estimate(3),2*pi);

         
         %calculate A and W
         A = [1 0 (-dt*(rob.u(2)+rob.u(1))/2)*sin(rob.pose(3)); 0 1 dt/2*(rob.u(2)+rob.u(1))*cos(rob.pose(3)); 0 0 1]; % jacobian of partial derivatives
         W = eye(3);%dt/2*[sin(rob.pose(3)) cos(rob.pose(3));sin(rob.pose(3)) cos(rob.pose(3));1/rob.radius -1/rob.radius]; % jacobian of noise 

         
         rob.P = A * rob.P * A' + W * rob.Q * W';
         %correction
         H = eye(3);

         K = rob.P * H' * inv(H * rob.P * H' + rob.R);

         innovation = rob.pose - H * rob.estimate;
         rob.estimate = rob.estimate + K * innovation;

         

         %finish up by updating P
         rob.P = rob.P - K * H * rob.P;
       end

   end
end

