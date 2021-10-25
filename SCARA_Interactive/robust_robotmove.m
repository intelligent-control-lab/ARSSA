%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  ROBOT CONTROL CODE              %
%  CHARLES NOREN                   %
%  2021.2                          %
%                                  %
%  ORIGINALLY BY:                  %
%  CHANGLIU LIU                    %
%  2015.5                          %
%                                  %
% USE CASE:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [status,robotnew]=robust_robotmove(t,robot)

status=0;

if status==0
    goal=[robot.Goal(1,1);robot.Goal(2,1)];
    robot.goalhis{t}=goal;
    
    % CALCULATE THE JACOBIAN
    [Je,H, ~, ~, He]=Jacobi(robot.x(1:robot.nlink,end),robot.x(robot.nlink+1:2*robot.nlink,end),robot.l,robot.nlink,robot.l(end),robot.base,robot.DH);
    % STEPWISE CALCULATION OF THE DYNAMIC MATRICES
    % HERE, WE ARE COMPUTING TWO SETS OF PARAMETERS:
    % 1) THIS IS THE ACTUAL PARAMETERS FOR THE PLANT, CAN BE USED
    % ADDITIONALLY FOR SSA WITH KNOWN PARAMETERS
    [M,~]=rbt_dyna_matrix(robot.x(:,end),robot.param);
    C = calc_chat(robot.x(robot.nlink+1:2*robot.nlink,end), robot.x(robot.nlink+1:2*robot.nlink,end), robot.param);
    % 2) THIS IS THE SET OF ESTIMATED PARAMETERS FOR WHEN SSA OR RSSA DOES
    % NOT HAVE THE ACTUAL PARAMETER INFORMATION PRESENT
    [M_hat,~]=rbt_dyna_matrix(robot.x(:,end),robot.est_param);
    C_hat = calc_chat(robot.x(robot.nlink+1:2*robot.nlink,end), robot.x(robot.nlink+1:2*robot.nlink,end), robot.est_param);
    
    % DESIRED CONTROL
    % This is the STANDARD ADAPTIVE CONTROL FROM SLOTINE AND LI AS
    % REFERENCED IN THE PAPER
    % 1) CALCULATE THE SLIDING VARIABLES
    [qrdot, qrddot, s] = calc_sliding_vars(robot.x(robot.nlink+1:2*robot.nlink,end), goal, robot.wx(1:2,end), robot.wx(4:5,end), Je, He, robot.slotinecontroller.LAMBDA);    
    % 2) CALCULATE THE CONTOL ITSELF
    U = compute_control_slotine(qrddot, qrdot, s, M_hat, C_hat, robot.slotinecontroller.KD);
    robot.u(1:2,t)=U; % STORE THIS FOR USE IN THE SYSTEM
    % 3) ESTIMATE THE PARAMETERS
    robot.est_param = est_params(robot.x(robot.nlink+1:2*robot.nlink,end), robot.x(robot.nlink+1:2*robot.nlink,end), qrdot, qrddot, robot.delta_t, robot.slotinecontroller.Gamma, s, robot.est_parambarrier,  robot.est_param);
    
    %disp(robot.est_param); % DEBUG TO SHOW THE COMPOSITE PARAMETERS
    [~, uwidth] = size(U);
    
    tic % FOR CODE TIMING PURPOSES CHECKS
    
    % CALCULATE THE CLOSEST POINT ON THE ROBOT TO THE ACTOR/AGENT/CURSOR
    [linkid,ratio,robot.profile{t}]=closest_2D(robot.obs.xstar(1:2,t),robot.x(:,t),robot.pos(:,t),robot.l);
    robot.profile{t}.dtime=toc; % COMPUTE HOW LONG IT TAKES TO CALCULATE THE CLOSEST 2D POINT
    tic;
    robot.profile{t}.ssa=0;
    if linkid==1 && ratio==0
        if robot.profile{t}.rmin<0.1
            ratio=0.001;
        end
    end
    
    if linkid==1 && ratio==0
        robot.u(1:2,t)=U;
    else
        [Jm,Hm,robot.mx(1:3,t),robot.mx(4:6,t)]=Jacobi(robot.x(1:robot.nlink,end),robot.x(robot.nlink+1:2*robot.nlink,end),robot.l,linkid,ratio,robot.base,robot.DH);
        [alpha_cond, M_safe, M_safe2, flag] = evolutionary(robot.centerpoint, robot.lowerpoint, robot.upperpoint, Jm, robot.obs.xstar(1:2,t), robot.x(1:2,t), robot.range, robot.grids);
        M_comb = M_safe;
        if flag
            plotLgPhis(M, M_hat, alpha_cond*M_safe, alpha_cond*M_safe2, alpha_cond*M_comb, Jm, robot.x(1:2,t))
        end % A CHECK FOR IF THE LGPHI DIRECTION IS NOT ALIGNED!!!!!!
        
        % CHANGING BETWEEN G^* and Ghat --) NEEDED FOR RSSA VS. NOT RSSA
        
        % IF YOU WANT REGULAR SSA WITH ESTIMATES
        %M_hat = M_hat; %If you want ghat
        % IF YOU WANT RSSA, MUST USE g^*
        M_hat = M_comb; %If you want g^*

        % Discrete safety index
        BJ=robot.B*Jm*pinv(M_hat);
        %D=(robot.A-robot.inf.B{t}(:,1:4))*robot.wx([1,2,4,5],end)-robot.inf.A{t}*robot.obs.xstar(:,end)-robot.inf.B{t}(:,5:6)*robot.obs.goal(:,end)+robot.B*Hm;
        
        D=robot.A*robot.mx([1,2,4,5],end)+robot.B*(Hm-Jm*pinv(M_hat)*C_hat*robot.x(robot.nlink+1:2*robot.nlink,end))-robot.obs.xstar(:,end);
        
        % THIS IS THE FORM OF THE ROBUST SAFETY INDEX
        kphir = 20; % THIS IS OUR CONSTANT THAT WE CAN TUNE
        %kphir = 200; % THIS IS OUR CONSTANT THAT WE CAN TUNE (test3.mat)
        dxi= (robot.est_param' - [robot.est_param1_mid; robot.est_param2_mid; robot.est_param3_mid]);
        XI = diag([1/robot.est_param1_mid, 1/robot.est_param2_mid, 1/robot.est_param3_mid]); % COVARIANCES
        
        % NOW, ONE MUST SPECIFY WHICH PHI THEY WANT
        % ONE MUST ALSO SPECIFY BELOW
        % REGULAR PHI (SAFETY INDEX)
        %[thres,vet]=safety(D,BJ,robot.margin);
        
        % COMPOSITE/ROBUST SAFETY INDEX: PHI R
        [thres,vet]=safety_phir(D,BJ,robot.margin, kphir, XI, dxi);
        
        
        if (vet*U)<thres
            change=thres-vet*U;
            % CHECK FOR USING SSA WITH REAL PARAMETERS OR ESTIMATED
            %U=U+M*vet'*pinv(vet*M*vet')*change;
            %U=U+M_hat*vet'*pinv(vet*M_hat*vet')*change;
            U=(thres*vet/(alpha_cond*norm(vet)^2))';
            robot.profile{t}.ssa=1;
        end
        
        dx=robot.mx([1,2,4,5],end)-robot.obs.xstar(:,end);
        dmin=robot.profile{t}.rmin;
        kd= 0.01;
        % kd=10; (test3.mat)
        % Continuous safety index

        % PHI FORMULATION
        %if 0.15^2-dmin^2-kd*dx'*robot.const.P2*dx/dmin>=0
        % PHI R FORMULATION
        if 0.15^2-dmin^2-kd*dx'*robot.const.P2*dx/dmin + kphir*(dxi' * diag([1/robot.est_param1_mid, 1/robot.est_param2_mid, 1/robot.est_param3_mid])*dxi)>=0
            vet=dx(1:2)'*Jm*pinv(M_hat)./dmin;
            vcirc=dx(3:4)-dx(1:2)*(dx(3:4)'*dx(1:2)/dmin);
            % CHECK FOR USING REAL PARAMETERS OR ESTIMATED PARAMETERS FOR
            % THE THRESHOLD
            thres=robot.margin-2*dx'*robot.const.P2*dx+kd*(dx(1:2)'*(Jm*pinv(M_hat)*C_hat*robot.x(robot.nlink+1:2*robot.nlink,end)-Hm)-norm(vcirc))/dmin;
            %thres=robot.margin-2*dx'*robot.const.P2*dx+kd*(dx(1:2)'*(Jm*pinv(M)*C*robot.x(robot.nlink+1:2*robot.nlink,end)-Hm)-norm(vcirc))/dmin;
            if (vet*U)<=thres
                % Regular SSA
                %change=thres-vet*U;
                
                % HERE, YOU SELECT WHICH SSA YOU WANT TO WORK WITH
                
                % NORMAL SSA WITH KNOWN PARAMETERS
                % MAKE SURE CORRECT GHAT IS USED ON LINE 75
                % MAKE SURE LINE 84 IS COMMENTED OUT
                % UNCOMMENT LINE 122 to use real parameters
                % UNCOMMENT LINE 102 to use real parameters
                % U=U+M*vet'*pinv(vet*M*vet')*change;
                % NORMAL SSA WITH ESTIMATED PARAMETERS
                % MAKE SURE CORRECT GHAT IS USED ON LINE 75                
                % MAKE SURE LINE 83 IS COMMENTED OUT
                % UNCOMMENT LINE 121 to use estimated parameters
                % UNCOMMENT LINE 103 to use estimated parameters                
                %U=U+M_hat*vet'*pinv(vet*M_hat*vet')*change;
                
                % USING RSSA:
                % RSSA
                % MAKE SURE CORRECT GHAT IS USED ON LINE 77                
                % MAKE SURE LINE 104 IS UNCOMMENTED
                U=(thres*vet/(alpha_cond*norm(vet)^2))';
                
                robot.profile{t}.ssa=1;
            end
        end
    end
    robot.profile{t}.ssatime=toc;
    
    % CONTROL LIMIT FOR SIMULATION STABILITY PURPOSES
    if abs(U(1))>robot.umax(1)
        U(1)=U(1)/abs(U(1))*robot.umax(1);
    end
    if abs(U(2))>robot.umax(2)
        U(2)=U(2)/abs(U(2))*robot.umax(2);
    end
    
    % NOT USED, HOLD FROM SSA
%     if robot.x(3,end)*robot.u(1,t)/norm(robot.u(1,t))>=robot.wmax
%         robot.u(1,t)=0;
%     end
%     if robot.x(4,end)*robot.u(2,t)/norm(robot.u(2,t))>=robot.wmax
%         robot.u(2,t)=0;
%     end
    
    [~, uwidth] = size(U);
    robot.u(1:2,t)=U;
    
    
    
end

[~,Q]=ode45(@(t,x)rbt_fwd_dyna_frc(t,x,robot.param,robot.u(:,end)), [t*robot.delta_t,(t+1)*robot.delta_t],robot.x(:,end));
robot.x(:,t+1)=Q(end,:);
robot.pos(:,t+1)=ArmPos(robot.base,robot.DH,robot.x(1:robot.nlink,t+1));%(x1,y1,z1,...,x(N+1),y(N+1),z(N+1))
robot.wx(1:3,t+1)=robot.pos(end-2:end,t+1);
robot.wx(4:5,t+1)=Je*robot.x(robot.nlink+1:2*robot.nlink,t+1);

robotnew=robot;
end