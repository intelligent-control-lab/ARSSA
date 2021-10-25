function robot=robust_robotproperty(id)
switch id
    case 1
        %the constants
        robot.nlink=2;
        robot.umax=[20, 20]; %m/s^2
        robot.margin=0.15;
        robot.delta_t=0.05;
        robot.wmax=1; %rad/s
        robot.seed = rng(0, 'twister');
        %The moments, lengths of the links and DH parameter and base
        
        % With the Robust Safe Set Algorithm, we need a range for each fo
        % the parameters
        
        % ORIGINAL SCARA PARAMETERS:
%         robot.range.IL1 = [0.123 0.123]; % Range of Inertia of Link 1 (L1)
%         robot.range.IL2 = [0.028 0.028]; % Range of Inertia of Link 2 (L2)
%         robot.range.lL1 = [0.320 0.320]; % Range of L1 Length
%         robot.range.lL2 = [0.215 0.215]; % Range of L2 Length
%         robot.range.mL1 = [6.830 6.830]; % Range of Mass of L1
%         robot.range.mL2 = [3.290 3.290]; % Range of Mass of L2
%         robot.range.mM2 = [5.560 5.560]; % Range of Mass of the 2nd motor
%         robot.range.mEE = [1.050 1.050]; % Range of Mass of the end effector
%         robot.range.TC1 = [22.36 22.36]; % Range of Motor 1 Torque Constant
%         robot.range.TC2 = [2.420 2.420]; % Range of Motor 2 Torque Constant

        % RANGE OF PARAMETERS WE ARE USING:
        robot.range.lL1 = [00.25 00.25]; % Range of L1 Length
        robot.range.lL2 = [00.27 00.27]; % Range of L2 Length
        robot.range.mL1 = [26.75 28.75]; % Range of Mass of L1
        robot.range.mL2 = [13.30 14.30]; % Range of Mass of L2
        robot.range.IL1 = [robot.range.mL1(1)*(robot.range.lL1(1)/2)^2 robot.range.mL1(2)*(robot.range.lL1(2)/2)^2]; % Range of Inertia of Link 1 (L1)
        robot.range.IL2 = [robot.range.mL2(1)*(robot.range.lL2(1)/2)^2 robot.range.mL2(2)*(robot.range.lL2(2)/2)^2]; % Range of Inertia of Link 2 (L2)
        robot.range.mM2 = [05.50 05.60]; % Range of Mass of the 2nd motor
        robot.range.mEE = [01.04 01.06]; % Range of Mass of the end effector
        robot.range.TC1 = [22.35 22.37]; % Range of Motor 1 Torque Constant
        robot.range.TC2 = [02.40 02.45]; % Range of Motor 2 Torque Constant
        % CALCULATE THE RANGE (actual params):
        IL1 = computeValInRange(robot.range.IL1(1), robot.range.IL1(2), robot.seed, 0);
        IL2 = computeValInRange(robot.range.IL2(1), robot.range.IL2(2), robot.seed, 0);
        lL1 = computeValInRange(robot.range.lL1(1), robot.range.lL1(2), robot.seed, 0);
        lL2 = computeValInRange(robot.range.lL2(1), robot.range.lL2(2), robot.seed, 0);
        mL1 = computeValInRange(robot.range.mL1(1), robot.range.mL1(2), robot.seed, 0);
        mL2 = computeValInRange(robot.range.mL2(1), robot.range.mL2(2), robot.seed, 0);
        mM2 = computeValInRange(robot.range.mM2(1), robot.range.mM2(2), robot.seed, 0);
        mEE = computeValInRange(robot.range.mEE(1), robot.range.mEE(2), robot.seed, 0);
        TC1 = computeValInRange(robot.range.TC1(1), robot.range.TC1(2), robot.seed, 0);
        TC2 = computeValInRange(robot.range.TC2(1), robot.range.TC2(2), robot.seed, 0);
        
        robot.I    =[IL1 IL2]; %Moments of inertia (kg.m^2)
        robot.l    =[lL1 lL2]; %Length of the links (m)
        robot.m    =[mL1 mL2]; %Mass of the links (Kg)
        robot.M    =[mM2 mEE]; %Mass of the second motor and the end-effector
        robot.Kt   =[TC1 TC2]; %Torque constant of the motors (N.m/V)
        robot.DH   =[0 0 robot.l(1) 0;pi/4 0 robot.l(2) 0]; %theta,d,a,alpha
        robot.base =[-30;-30;0]./100; %origin

        %Parameters in dynamics
        robot.param=[];
        % There is an implicit assumption here that the center of mass of
        % each of the links is located at the length-centroid of the link
        % We don't use the extra masses that are currently written out
        robot.param(1)=robot.I(1)+robot.I(2)+(robot.m(1)/4+robot.m(2))*robot.l(1)^2+...
            robot.m(2)*robot.l(2)^2/4;
        robot.param(2)=robot.I(2)+robot.m(2)*robot.l(2)^2/4;
        robot.param(3)=robot.m(2)*robot.l(1)*robot.l(2)/2;        

        % CALCULATE THE RANGE (estimated params):
        IL1 = computeValInRange(robot.range.IL1(1), robot.range.IL1(2), robot.seed, 1);
        IL2 = computeValInRange(robot.range.IL2(1), robot.range.IL2(2), robot.seed, 1);
        lL1 = computeValInRange(robot.range.lL1(1), robot.range.lL1(2), robot.seed, 1);
        lL2 = computeValInRange(robot.range.lL2(1), robot.range.lL2(2), robot.seed, 1);
        mL1 = computeValInRange(robot.range.mL1(1), robot.range.mL1(2), robot.seed, 1);
        mL2 = computeValInRange(robot.range.mL2(1), robot.range.mL2(2), robot.seed, 1);
        mM2 = computeValInRange(robot.range.mM2(1), robot.range.mM2(2), robot.seed, 1);
        mEE = computeValInRange(robot.range.mEE(1), robot.range.mEE(2), robot.seed, 1);
        TC1 = computeValInRange(robot.range.TC1(1), robot.range.TC1(2), robot.seed, 1);
        TC2 = computeValInRange(robot.range.TC2(1), robot.range.TC2(2), robot.seed, 1);

        robot.est_I    =[IL1 IL2]; %Moments of inertia (kg.m^2)
        robot.est_l    =[lL1 lL2]; %Length of the links (m)
        robot.est_m    =[mL1 mL2]; %Mass of the links (Kg)
        robot.est_M    =[mM2 mEE]; %Mass of the second motor and the end-effector
        robot.est_Kt   =[TC1 TC2]; %Torque constant of the motors (N.m/V)
        robot.est_DH   =[0 0 robot.l(1) 0;pi/4 0 robot.l(2) 0]; %theta,d,a,alpha

        robot.lowerpoint  = [robot.range.IL1(1); robot.range.IL2(1); robot.range.lL1(1); robot.range.lL2(1); robot.range.mL1(1); robot.range.mL2(1); robot.range.mM2(1); robot.range.mEE(1); robot.range.TC1(1); robot.range.TC2(1)];
        robot.upperpoint  = [robot.range.IL1(2); robot.range.IL2(2); robot.range.lL1(2); robot.range.lL2(2); robot.range.mL1(2); robot.range.mL2(2); robot.range.mM2(2); robot.range.mEE(2); robot.range.TC1(2); robot.range.TC2(2)];
        robot.centerpoint = [IL1; IL2; lL1; lL2; mL1; mL2; mM2; mEE; TC1; TC2];
        %Parameters in dynamics -- this time, for estimation
        robot.est_param=[];
        % There is an implicit assumption here that the center of mass of
        % each of the links is located at the length-centroid of the link
        % We don't use the extra masses that are currently written out
        robot.est_param(1)=robot.est_I(1)+robot.est_I(2)+(robot.est_m(1)/4+robot.est_m(2))*robot.est_l(1)^2+...
            robot.est_m(2)*robot.est_l(2)^2/4;
        robot.est_param(2)=robot.est_I(2)+robot.est_m(2)*robot.est_l(2)^2/4;
        robot.est_param(3)=robot.est_m(2)*robot.est_l(1)*robot.est_l(2)/2;
        est_param_1_LB = robot.range.IL1(1)+robot.range.IL2(1)+(robot.range.mL1(1)/4+robot.range.mL2(1))*robot.range.lL1(1)^2+...
            robot.range.mL2(1)*robot.range.lL2(1)^2/4;
        est_param_1_UB = robot.range.IL1(2)+robot.range.IL2(2)+(robot.range.mL1(2)/4+robot.range.mL2(2))*robot.range.lL1(2)^2+...
            robot.range.mL2(2)*robot.range.lL2(2)^2/4;
        est_param_2_LB = robot.range.IL2(1)+robot.range.mL2(1)*robot.range.lL2(1)^2/4;
        est_param_2_UB = robot.range.IL2(2)+robot.range.mL2(2)*robot.range.lL2(2)^2/4;
        est_param_3_LB = robot.range.mL2(1)*robot.range.lL1(1)*robot.range.lL2(1)/2;
        est_param_3_UB = robot.range.mL2(2)*robot.range.lL1(2)*robot.range.lL2(2)/2;
        
        robot.est_parambarrier(1,1) = est_param_1_LB; 
        robot.est_parambarrier(1,2) = est_param_1_UB;
        robot.est_parambarrier(2,1) = est_param_2_LB;
        robot.est_parambarrier(2,2) = est_param_2_UB;
        robot.est_parambarrier(3,1) = est_param_3_LB;
        robot.est_parambarrier(3,2) = est_param_3_UB;
        robot.est_param1_mid = (est_param_1_LB + est_param_1_UB)/2;
        robot.est_param2_mid = (est_param_2_LB + est_param_2_UB)/2;
        robot.est_param3_mid = (est_param_3_LB + est_param_3_UB)/2; 
end

%gridIL1 = linspace(robot.range.IL1(1), robot.range.IL1(2), 4);
%gridIL2 = linspace(robot.range.IL2(1), robot.range.IL2(2), 4);
%gridlL1 = linspace(robot.range.lL1(1), robot.range.lL1(2), 1);
%gridlL2 = linspace(robot.range.lL2(1), robot.range.lL2(2), 1);
gridmL1 = linspace(robot.range.mL1(1), robot.range.mL1(2), 20);
gridmL2 = linspace(robot.range.mL2(1), robot.range.mL2(2), 20);

%[X1, X2, X3, X4, X5, X6] = ndgrid(gridIL1, gridIL2, gridlL1, gridlL2, gridmL1, gridmL2);
[X5, X6] = ndgrid(gridmL1, gridmL2);
%alphaGrid = X1+X2+(X5./4+X6).*X3.^2+X6.*X4.^2/4;
alphaGrid = X5.*(robot.range.lL1(1)/2).^2+X6.*(robot.range.lL2(1)/2).^2+(X5./4+X6).*robot.range.lL1(1).^2+X6.*robot.range.lL2(1).^2/4;
robot.grids.reshapedalphaGrid = vertcat(alphaGrid(:));
%betaGrid = X2+X6.*X4.^2./4;
betaGrid = X6.*robot.range.lL2(1).^2/2;
robot.grids.reshapedbetaGrid = vertcat(betaGrid(:));
%epsilonGrid = X6.*X3.*X4.^2/2;
epsilonGrid = X6.*robot.range.lL1(1).*robot.range.lL2(1).^2/2;
robot.grids.reshapedepsilonGrid = vertcat(epsilonGrid(:));
%robot.abe_funclist = @(x,rs,cx)safety_dot_cond_discret(robot.reshapedalphaGrid, robot.reshapedbetaGrid, robot.reshapedepsilonGrid, x, rs, cx);
% THESE ARE ROBOT CONTROLLER VALUES FOR SLOTINE
% ASSUME A CONTROLLER FOR A 2D Robot
%robot.slotinecontroller.KD = 5*eye(2); 
%robot.slotinecontroller.LAMBDA = 15*eye(2);
%robot.slotinecontroller.Gamma = [60, 0, 0; 0, 100, 0; 0, 0, 20];

robot.slotinecontroller.KD = 5*eye(2); 
robot.slotinecontroller.LAMBDA = 1*eye(2);
robot.slotinecontroller.Gamma = [60, 0, 0; 0, 100, 0; 0, 0, 20];

%The kinematic matrices
robot.A=[eye(robot.nlink) robot.delta_t*eye(robot.nlink);zeros(robot.nlink) eye(robot.nlink)];
robot.B=[0.5*robot.delta_t^2*eye(robot.nlink);robot.delta_t*eye(robot.nlink)];
robot.C=eye(2*robot.nlink);
robot.D=zeros(2*robot.nlink,robot.nlink);
robot.Q=diag([ones(1,robot.nlink) zeros(1,robot.nlink)]);%[1 robot.delta_t 0 0;robot.delta_t robot.delta_t^2 0 0;0 0 1 robot.delta_t;0 0 robot.delta_t robot.delta_t^2];
robot.R=eye(robot.nlink);
robot.Goal=[-0.2,-0.1]';
robot.nG=size(robot.Goal,2);

robot.x(1:2*robot.nlink,1)=[robot.DH(:,1);zeros(robot.nlink,1)];%(theta1,...,thetaN,theta1dot,...,thetaNdot)
robot.pos=ArmPos(robot.base,robot.DH,robot.x(1:robot.nlink,1));%(x1,y1,z1,...,x(N+1),y(N+1),z(N+1))
robot.wx(1:3*robot.nlink,1)=[robot.pos(end-2:end);0;0;0];%endpoint state(x(N+1),y(N+1),z(N+1),x(N+1)dot,y(N+1)dot,z(N+1)dot)
robot.mx=robot.wx;%closest point state

robot.ref.x=robot.x;
robot.innoise=0;
robot.outnoiseself=0;
robot.outnoisestar=0;
robot.obs.xself=[];
robot.obs.xstar=[];
robot.obs.goal=[];
robot.obs.A=robot.A;
robot.obs.B=robot.B;
robot.obs.C=robot.C;
robot.obs.D=robot.D;
robot.obs.Q=robot.Q;
robot.obs.R=robot.R;
robot.score=0;
robot.inf.A={};
robot.inf.B={};
robot.inf.F={};
robot.inf.A{1}=robot.A;
robot.inf.B{1}=[eye(4) robot.B];
robot.inf.F{1}=eye(10);
robot.flag=0;

%For SSA
robot.const.P1=[eye(robot.nlink) zeros(robot.nlink);zeros(robot.nlink) zeros(robot.nlink)];
robot.const.P2=[zeros(robot.nlink) 0.5*eye(robot.nlink);0.5*eye(robot.nlink) zeros(robot.nlink)];
robot.const.P3=[zeros(robot.nlink) zeros(robot.nlink);zeros(robot.nlink) eye(robot.nlink)];

%For collision check
robot.profile={};





