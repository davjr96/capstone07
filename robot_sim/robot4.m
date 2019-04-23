classdef robot4 < handle
    properties
        %kMax = 2500;
        %deltaT = 0.1; %sec
        
        K = 1;
        
        N = 0; %network size
        n = 0; %dimension of the state vector
        histsize = 30; %number of retained values, also number of timesteps
        
        
        mass = 1;
        I = 5;
        radius = 1; %m
        
        P = zeros(3,3);
        variance_speed = 0.0001;
        Q = [];
        
        variance_gps = 5;
        R = [];
        
        pose = [0 0 1]';
        %[x y theta]
        %pose = [1 1 1 1 1 1 1]';
        estimate = [0 0 1]'; %[x y uL uR cos sin omega]'
        
        B = [0 0; 0 0; -1 1];
        u = [ 1 1 ]'; % wheel dynamics of robot path
        % uPath = [1 1]; %goal path "dynamics"
        
        %        Fr = 0;
        %        Fl = 0;
        
        E = [];
        
        poses={};
        ests={};
        
        %For Behavioral Trust
        Mb={}
        Tinstb={}
        Trendb={}
        Repb={}
        TrustALb={}
        TrustNLb={}
        Sb={}
        Ub={}
        repwtsb=[1 1 1]
        XerrorListb = [];
        YerrorListb = [];
        TerrorListb = [];
        errMatb = [];
        
        %For Communication Trust
        Mc={}
        Tinstc={}
        Trendc={}
        Repc={}
        TrustALc={}
        TrustNLc={}
        Sc={}
        Uc={}
        repwtsc=[1 1 1]
        XerrorListc = [];
        YerrorListc = [];
        TerrorListc = [];
        errMatc = [];
    end
    
    methods
        %%%Initialize Sys
        function init(rob,N,n,kMax)
            rob.N=N;
            rob.n=n;
            rob.Q = eye(3) * rob.variance_speed;
            rob.R = eye(3)*rob.variance_gps;
            rob.ests=cell(1,kMax);
            rob.poses=cell(1,kMax);
            %%%Communication Vars%%%
            rob.Mc = cell(1,kMax);
            rob.Tinstc = cell(1,kMax);
            rob.Trendc = cell(1,kMax);
            rob.Repc = cell(1,kMax);
            rob.TrustALc = cell(1,kMax);
            rob.TrustNLc = cell(1,kMax);
            rob.errMatc = zeros(N,n);
            rob.Sc=cell(1,kMax);
            rob.Uc=cell(1,kMax);
            %            zeros(N,n,rob.histSize);
            %%%Behavior Vars
            rob.Mb = cell(1,kMax);
            rob.Tinstb = cell(1,kMax);
            rob.Trendb = cell(1,kMax);
            rob.Repb = cell(1,kMax);
            rob.TrustALb = cell(1,kMax);
            rob.TrustNLb = cell(1,kMax);
            rob.errMatb = zeros(n,N);
            rob.Sb=cell(1,kMax);
            rob.Ub=cell(1,kMax);
            
            %Sizing
            for h=1:kMax
                rob.poses{h}=zeros(n);
                rob.ests{h}=zeros(n);
                %Comms
                rob.Tinstc{h} = zeros(1,N);
                rob.Trendc{h} = zeros(1,N);
                rob.Repc{h} = zeros(1,N);
                rob.TrustALc{h} = zeros(1,N);
                rob.TrustNLc{h} = zeros(1,N);
                %                rob.errMatc = zeros(N,n);
                rob.Sc{h}=zeros(1,N);
                rob.Uc{h}=zeros(1,N);
                %Behaviour
                rob.Tinstb{h} = zeros(1,N);
                rob.Trendb{h} = zeros(1,N);
                rob.Repb{h} = zeros(1,N);
                rob.TrustALb{h} = zeros(1,N);
                rob.TrustNLb{h} = zeros(1,N);
                %                rob.errMatc = zeros(N,n);
                rob.Sb{h}=zeros(1,N);
                rob.Ub{h}=zeros(1,N);
            end
            
            %            rob.B = [ 0 0; 0 0; 1/rob.mass+rob.radius/rob.I 1/rob.mass-rob.radius/rob.I; 1/rob.mass-rob.radius/rob.I 1/rob.mass+rob.radius/rob.I; 0 0; 0 0; -1/(2*rob.I) 1/(2*rob.I) ];
            
            rob.E = zeros(7,rob.N);
        end
        
        function err=getErrorc(rob, goalPose, robPose)
%             disp("rob")
%             disp(robPose);
%             disp("goal")
%             disp(goalPose);
            Xerrorc = ((goalPose(1) - robPose(1)).^2)./((rob.K^2).*rob.P(1,1));
            %disp(Xerror)
            
            %disp(XerrorList')
            
            Yerrorc = ((goalPose(2) - robPose(2)).^2)./((rob.K^2).*rob.P(2 , 2));%.^2);
            
            ThetaErrorc = ((goalPose(3) - robPose(3)).^2)./((rob.K^2).*rob.P(3 , 3));%.^2);
            
            
            %Tuning happens here
            err = [Xerrorc ; Yerrorc; ThetaErrorc]';
        end
        function err=getErrorb(rob, goalPose, robPose)
            
            Xerrorb = ((goalPose(1) - robPose(1)).^2)./((rob.K^2).*rob.P(1,1));
            %disp(Xerror)
            
            %disp(XerrorList')
            
            Yerrorb = ((goalPose(2) - robPose(2)).^2)./((rob.K^2).*rob.P(2 , 2));%.^2);
            
            ThetaErrorb = ((goalPose(3) - robPose(3)).^2)./((rob.K^2).*rob.P(3 , 3));%.^2);
            
            
            %Tuning happens here
            err = [Xerrorb ; Yerrorb; ThetaErrorb];
            
            
            
        end
        function updateM(rob,ErrMat,k,type)
            if type==0
                rob.Mc{k}=ErrMat;
                %            M=rob.Mc(1:end-1,:,:);
                %            rob.Mc=[ErrMat,M];
            elseif type==1
                rob.Mb{k}=ErrMat;
                %            rob.Mb=[ErrMat,M];  %chnaged semicolon to comma
            end
        end
        
        function reduced=VecReduce(rob,k,type)
            reduced=zeros(1,rob.N);
            if type ==0
                for j=1:rob.N
                    for i=1:rob.n
                        reduced(i)=norm(rob.Mc{k}(i,:));%Place whatever vector norm you like in here
                        %I'm using the default Euclidean Norm (2-norm)
                    end
                end
            elseif type == 1
                for j=1:rob.N
                    for i=1:rob.n
                        reduced(i)=norm(rob.Mb{k}(i,:));%Place whatever vector norm you like in here
                        %I'm using the default Euclidean Norm (2-norm)
                    end
                end
            end
        end
        
        function updateTrendc(rob,k,dt)
            M_scalars=zeros(k,rob.N);
            if k >= rob.histsize
                for i=1:k
                    M_scalars(i,:)=rob.VecReduce(k,0);
                end
                DM = diff(M_scalars(end-rob.histsize:end));
                rob.Trendc{k} = 1-(mean(DM)/dt);
            else
                rob.Trendc{k}=ones(size(rob.Trendc{k}));
            end
        end
        
        function InstantTrustc(rob,k)
            %Selecting relevant M
            M_curr = rob.Mc{k}(:,:);
            disp(M_curr);
            %            s=size(rob.Mc);
            T=zeros(rob.N,rob.n);
            for agent=1:rob.N
                for statevar=1:rob.n
                    if M_curr(agent,statevar) > 1
                        T(agent,statevar)=0;
                    else
                        T(agent,statevar)=1;
                    end
                end
                %Most Conservative Rule
                if sum(T(agent,:)) ~= rob.n
                    rob.Tinstc{k}(agent)=0;
                else
                    rob.Tinstc{k}(agent)=1;
                end
            end
        end
        
        function updateRepc(rob,k)
            for agent=1:rob.N
                if rob.Tinstc{k}(agent)==1
                    rob.Sc{k}(agent) = rob.Sc{k}(agent)+1;
                else
                    rob.Uc{k}(agent) = rob.Uc{k}(agent)+1;
                end
                rob.Repc{k}(agent)=(rob.repwtsc(1)*rob.Sc{k}(agent)-rob.repwtsc(2)*rob.Uc{k}(agent))/(rob.repwtsc(3)*(rob.Sc{k}(agent)+rob.Uc{k}(agent)));
            end
        end
        
        function updateALTrustc(rob,k)
            rob.TrustALc{k}=(rob.Trendc{k}).^2.*(rob.Repc{k});
        end
        
        
        % Behavioral Trust
        
        function updateTrendb(rob,k,dt)
            M_scalars=zeros(k,rob.N);
            if k >= rob.histsize
                for i=1:k
                    M_scalars(i,:)=rob.VecReduce(k,0);
                end
                DM = diff(M_scalars(end-rob.histsize:end));
                rob.Trendb{k} = 1-(mean(DM)/dt);
            else
                rob.Trendb{k}=ones(size(rob.Trendb{k}));
            end
        end
        
        function InstantTrustb(rob,k)
            %Selecting relevant M
            M_curr = rob.Mb{1}(:,:);
            %            s=size(rob.Mc);
            T=zeros(rob.N,rob.n);
            for agent=1:rob.N
                for statevar=1:rob.n
                    if M_curr(agent,statevar) > 1
                        T(agent,statevar)=0;
                    else
                        T(agent,statevar)=1;
                    end
                end
                %Most Conservative Rule
                if sum(T(agent,:)) ~= rob.n
                    rob.Tinstb{k}(agent)=0;
                else
                    rob.Tinstb{k}(agent)=1;
                end
            end
        end
        
        function updateRepb(rob,k)
            for agent=1:rob.N
                if rob.Tinstb{k}(agent)==1
                    rob.Sb{k}(agent) = rob.Sb{k}(agent)+1;
                else
                    rob.Ub{k}(agent) = rob.Ub{k}(agent)+1;
                end
                rob.Repb{k}(agent)=(rob.repwtsc(1)*rob.Sb{k}(agent)-rob.repwtsc(2)*rob.Ub{k}(agent))/(rob.repwtsb(3)*(rob.Sb{k}(agent)+rob.Ub{k}(agent)));
            end
        end
        
        function updateALTrustb(rob,k)
            rob.TrustALb{k}=(rob.Trendb{k}).^2.*(rob.Repb{k});
        end
        
        
        %%%Update Pose
        function setVelocity(rob,error)
            Gain=[-.01 .01 .01;.01 .01 -.01];
            rob.u=Gain*error;
        end
        function update(rob, dt)
            %update the 'actual' pose:
            
            for i1 = 1:3
                if rob.pose(i1) == 0
                    rob.pose(i1) = .0000001;
                    
                end
            end
            rob.pose= rob.pose + dt*[(rob.u(1) + rob.u(2)) * cos(rob.pose(3)) / 2;
                (rob.u(1) + rob.u(2)) * sin(rob.pose(3)) / 2;
                (rob.u(1) - rob.u(2))/(2*rob.radius)];
            
            
            
            
            %        %normalize sin,cos
            %        a = rob.pose(5);
            %        b = rob.pose(6);
            %        len =sqrt(a^2 + b^2);
            %        rob.pose(5) = rob.pose(5) / len;
            %        rob.pose(6) = rob.pose(6) / len;
        end
        function kalman(rob,dt)
            %prediction
            rob.estimate = rob.estimate + dt*[(rob.u(1) + rob.u(2)) * cos(rob.estimate(3)) / 2;
                (rob.u(1) + rob.u(2)) * sin(rob.estimate(3)) / 2;
                (rob.u(1) - rob.u(2))/(2*rob.radius)];
            
            
            
            %calculate A and W
            A = [1 0 (-dt*(rob.u(2)+rob.u(1))/2)*sin(rob.pose(3)); 0 1 dt/2*(rob.u(2)+rob.u(1))*cos(rob.pose(3)); 0 0 1]; % jacobian of partial derivatives
            W = eye(3);%dt/2*[sin(rob.pose(3)) cos(rob.pose(3));sin(rob.pose(3)) cos(rob.pose(3));1/rob.radius -1/rob.radius]; % jacobian of noise
            
            
            rob.P = A * rob.P * A' + W * rob.Q * W';
            %correction
            H = eye(3);
            
            K = rob.P * H' * inv(H * rob.P * H' + rob.R); %#ok<MINV>
            
            innovation = rob.pose - H * rob.estimate;
            rob.estimate = rob.estimate + K * innovation;
            
            
            
            %finish up by updating P
            rob.P = rob.P - K * H * rob.P;
        end
        
        function stateRecorder(rob,k)
           rob.ests{k}=rob.estimate;
           rob.poses{k}=rob.pose;
        end
        
        
    end
    
end