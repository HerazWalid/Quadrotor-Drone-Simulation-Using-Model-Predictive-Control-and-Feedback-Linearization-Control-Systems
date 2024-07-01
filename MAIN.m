%% Project Made By HERAZ Walid as a bachelor graduation project at the Institute of Electrical and Electronic Engineering IGEE_ex_INELEC 2024, Algeria
%% Github : https://github.com/HerazWalid
%% LinkdIn : https://www.linkedin.com/in/walid-heraz-94337a240/
%% 

clear all
close all
clc
echo off
warning off


%% Load the constant values
constants=init_constants();
Ts=constants{7};
controlled_states=constants{14}; % number of controlled states in this script
innerDyn_length=constants{16}; % Number of inner control loop iterations

%% Generate the reference signals
t = 0:Ts*innerDyn_length:100;
t_angles=(0:Ts:t(end))';
r = 2;
f=0.025;
height_i=2;
height_f=5;
[X_ref,X_dot_ref,X_dot_dot_ref,Y_ref,Y_dot_ref,Y_dot_dot_ref,Z_ref,Z_dot_ref,Z_dot_dot_ref,psi_ref]=trajectory_generator(t,r,f,height_i,height_f);
plotl=length(t); % Number of outer control loop iterations


%% Define design parameters
D2R = pi/180;
R2D = 180/pi;
b   = 0.6;   % the length of total square cover by whole body of quadcopter in meter
a   = b/3;   % the legth of small square base of quadcopter(b/4)
H   = 0.06;  % hight of drone in Z direction 
H_m = H+H/2; % hight of motor in z direction
r_p = b/4;   % radius of propeller
%% Conversions
ro = 45*D2R;                   % angle by which rotate the base of quadcopter
Ri = [cos(ro) -sin(ro) 0;
      sin(ro) cos(ro)  0;
       0       0       1];     % rotation matrix to rotate the coordinates of base 
base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base 
           -a/2 -a/2 a/2 a/2;
             0    0   0   0];
base = Ri*base_co;             % rotate base Coordinates by 45 degree 

to = linspace(0, 2*pi);
xp = r_p*cos(to);
yp = r_p*sin(to);
zp = zeros(1,length(to));



%% Load the initial state vector

ut=0;
vt=0;
wt=0;
pt=0;
qt=0;
rt=0;
xt=0;
yt=0; % Initial translational position
zt=0; % Initial translational position
phit=0;    % Initial angular position
thetat=0;  % Initial angular position
psit=psi_ref(1,2);    % Initial angular position

states=[ut,vt,wt,pt,qt,rt,xt,yt,zt,phit,thetat,psit];
states_total=states;

% Assume that first Phi_ref, Theta_ref, Psi_ref are equal to the first
% phit, thetat, psit
ref_angles_total=[phit,thetat,psit];
velocityXYZ_total=[X_dot_ref(1,2),Y_dot_ref(1,2),Z_dot_ref(1,2)];


%% Initial drone state

omega1=110*pi/3; % rad/s
omega2=110*pi/3; % rad/s
omega3=110*pi/3; % rad/s
omega4=110*pi/3; % rad/s 

ct = constants{11};
cq = constants{12};
l  = constants{13};

U1=ct*(omega1^2+omega2^2+omega3^2+omega4^2);
U2=ct*l*(omega2^2-omega4^2);
U3=ct*l*(omega3^2-omega1^2); 
U4=cq*(-omega1^2+omega2^2-omega3^2+omega4^2);

UTotal=[U1,U2,U3,U4];% 4 inputs

global omega_total
omega_total=omega1-omega2+omega3-omega4;

%% Start the global controller

for i_global = 1:plotl-1


    %% Implement the position controller (state feedback linearization)

    [phi_ref, theta_ref, U1]=Feedback_Linearization(X_ref(i_global+1,2),X_dot_ref(i_global+1,2),X_dot_dot_ref(i_global+1,2),Y_ref(i_global+1,2),Y_dot_ref(i_global+1,2),Y_dot_dot_ref(i_global+1,2),Z_ref(i_global+1,2),Z_dot_ref(i_global+1,2),Z_dot_dot_ref(i_global+1,2),psi_ref(i_global+1,2),states);


    Phi_ref=phi_ref*ones(innerDyn_length+1,1);
    Theta_ref=theta_ref*ones(innerDyn_length+1,1);
    
    
    % Make Psi_ref increase continuosly in a linear fashion per outer loop
    Psi_ref=zeros(innerDyn_length+1,1);
    for yaw_step = 1:(innerDyn_length+1)
        Psi_ref(yaw_step)=psi_ref(i_global,2)+(psi_ref(i_global+1,2)-psi_ref(i_global,2))/(Ts*innerDyn_length)*Ts*(yaw_step-1);
    end
    
    ref_angles_total=[ref_angles_total;Phi_ref(2:end) Theta_ref(2:end) Psi_ref(2:end)];

    %% Create the reference vector

    refSignals=zeros(length(Phi_ref(:,1))*controlled_states,1);
    % Format: refSignals=[Phi_ref;Theta_ref;Psi_ref;Phi_ref; ... etc] x inner
    % loop frequency per one set of position controller outputs
    k_ref_local=1;
    for i = 1:controlled_states:length(refSignals)
       refSignals(i)=Phi_ref(k_ref_local,1);
       refSignals(i+1)=Theta_ref(k_ref_local,1);
       refSignals(i+2)=Psi_ref(k_ref_local,1);
       k_ref_local=k_ref_local+1;
    end

    k_ref_local=1; % for reading reference signals
    hz = constants{15}; % horizon period
    for i =1:innerDyn_length
        %% Generate discrete LPV Ad, Bd, Cd, Dd matrices
        [Ad, Bd, Cd, Dd, x_dot, y_dot, z_dot, phit, phi_dot, thetat, theta_dot, psit, psi_dot]=LPV(states);
        velocityXYZ_total=[velocityXYZ_total;[x_dot, y_dot, z_dot]];


        %% Generating the current state and the reference vector
        x_aug_t=[phit;phi_dot;thetat;theta_dot;psit;psi_dot;U2;U3;U4];

        k_ref_local=k_ref_local+controlled_states;

        % Start counting from the second sample period:
        % r=refSignals(Phi_ref_2;Theta_ref_2;Psi_ref_2;Phi_ref_3...) etc.
        if k_ref_local+controlled_states*hz-1 <= length(refSignals)
            r=refSignals(k_ref_local:k_ref_local+controlled_states*hz-1);
        else
            r=refSignals(k_ref_local:length(refSignals));
            hz=hz-1;
        end

        %% Generate simplification matrices for the cost function
        [Hdb,Fdbt,Cdb,Adc] = MPC(Ad,Bd,Cd,Dd,hz);

        %% Calling the optimizer (quadprog)

        % Cost function in quadprog: min(du)*1/2*du'Hdb*du+f'du
        ft=[x_aug_t',r']*Fdbt;

        % normalement Hdb should be positive so we check if matrix Hdb in the cost function is positive definit.
        [~,p] = chol(Hdb);
        if p~=0
           disp('Hdb is NOT positive definite');
        end

   
        [du,fval]=quadprog(Hdb,ft);
        
        % Update the real inputs
        U2=U2+du(1);
        U3=U3+du(2);
        U4=U4+du(3);

        UTotal=[UTotal;U1,U2,U3,U4];

        % Compute the new omegas based on the new U-s.
        U1C=U1/ct;
        U2C=U2/(ct*l);
        U3C=U3/(ct*l);
        U4C=U4/cq;
        
        omega_Matrix=[1 1 1 1;0 1 0 -1;-1 0 1 0;-1 1 -1 1];
        UC_vector=[U1C;U2C;U3C;U4C];
        omegas_vector=inv(omega_Matrix)*UC_vector;

        omega1=sqrt(omegas_vector(1));
        omega2=sqrt(omegas_vector(2));
        omega3=sqrt(omegas_vector(3));
        omega4=sqrt(omegas_vector(4));

        % Compute the total omega
        omega_total=omega1-omega2+omega3-omega4;

        % Simulate the new states
        T = (Ts)*(i-1):(Ts)/30:Ts*(i-1)+(Ts);
        [T,x]=ode45(@(t,x) drone_plant(t,x,[U1,U2,U3,U4]),T,states);
        states=x(end,:);
        disp('Final states:');
        disp(size(states_total));
        states_total=[states_total;states];

        imaginary_check=imag(states)~=0;
        imaginary_check_sum=sum(imaginary_check);
        if imaginary_check_sum~=0
            disp('Imaginary part exists - something is wrong');
        end
    end
end

%% Plot the trajectory
% States: [u,v,w,p,q,r,X,Y,Z,phi,theta,psi] we found them all

% Plot the trajectory
figure;
hg   = gca;
grid on;
view(3);
xlabel('X[m]','FontSize',15);
ylabel('Y[m]','FontSize',15);
zlabel('Z[m]','FontSize',15);
hold(gca, 'on');



%% Design Different parts
% design the base square
 drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
 drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
 alpha(drone(1:2),0.7);
% design 2 parpendiculer legs of quadcopter 
 [xcylinder, ycylinder, zcylinder] = cylinder([H/2 H/2]);
 drone(3) =  surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
 drone(4) =  surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b') ; 
 alpha(drone(3:4),0.6);
% design 4 cylindrical motors 
 drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
 drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
 alpha(drone(5:8),0.7);
% design 4 propellers
 drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
 drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
 drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
 drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
 alpha(drone(9:12),0.3);

%% create a group object and parent surface
  combinedobject = hgtransform('parent',hg );
  set(drone,'parent',combinedobject)


% Initialize plot handles
trajectory_ref = plot3(X_ref(:,2), Y_ref(:,2), Z_ref(:,2), '--b', 'LineWidth', 3);
hold on;


% Extracting position information for plotting the quadrotor
x = states_total(1:innerDyn_length:end,7);
y = states_total(1:innerDyn_length:end,8);
z = states_total(1:innerDyn_length:end,9);
roll = states_total(1:innerDyn_length:end,10);
pitch = states_total(1:innerDyn_length:end,11);
yaw = states_total(1:innerDyn_length:end,12);


 for i = 1:length(x)
     hold on;
     ba = plot3(x(1:i),y(1:i),z(1:i), 'r:','LineWidth',3);
     disp(states_total(:, 7:12));

     translation = makehgtform('translate',...
                               [x(i) y(i) z(i)]);
     rotation1 = makehgtform('xrotate',(pi/180)*(roll(i)));
     rotation2 = makehgtform('yrotate',(pi/180)*(pitch(i)));
     rotation3 = makehgtform('zrotate',yaw(i));
     %scaling = makehgtform('scale',1-i/20);
     set(combinedobject,'matrix',...
          translation*rotation3*rotation2*rotation1);
     drawnow
    pause(0.09);
 end



%% Plot the positions and velocities individually

% X and X_dot
figure;
subplot(2,1,1)
plot(t(1:plotl),X_ref(1:plotl,2),'--b','LineWidth',2)
hold on
subplot(2,1,1)
plot(t(1:plotl),states_total(1:innerDyn_length:end,7),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('x-position [m]','FontSize',15)
legend({'x-ref','x-position'},'Location','northeast','FontSize',15)
subplot(2,1,2)
plot(t(1:plotl),X_dot_ref(1:plotl,2),'--b','LineWidth',2)
hold on
subplot(2,1,2)
plot(t(1:plotl),velocityXYZ_total(1:innerDyn_length:end,1),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('x-velocity [m/s]','FontSize',15)
legend({'x-dot-ref','x-velocity'},'Location','northeast','FontSize',15)

% Y and Y_dot
figure;
subplot(2,1,1)
plot(t(1:plotl),Y_ref(1:plotl,2),'--b','LineWidth',2)
hold on
subplot(2,1,1)
plot(t(1:plotl),states_total(1:innerDyn_length:end,8),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('y-position [m]','FontSize',15)
legend({'y-ref','y-position'},'Location','northeast','FontSize',15)
subplot(2,1,2)
plot(t(1:plotl),Y_dot_ref(1:plotl,2),'--b','LineWidth',2)
hold on
subplot(2,1,2)
plot(t(1:plotl),velocityXYZ_total(1:innerDyn_length:end,2),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('y-velocity [m/s]','FontSize',15)
legend({'y-dot-ref','y-velocity'},'Location','northeast','FontSize',15)

% Z and Z_dot
figure;
subplot(2,1,1)
plot(t(1:plotl),Z_ref(1:plotl,2),'--b','LineWidth',2)
hold on
subplot(2,1,1)
plot(t(1:plotl),states_total(1:innerDyn_length:end,9),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('z-position [m]','FontSize',15)
legend({'z-ref','z-position'},'Location','northeast','FontSize',15)
subplot(2,1,2)
plot(t(1:plotl),Z_dot_ref(1:plotl,2),'--b','LineWidth',2)
hold on
subplot(2,1,2)
plot(t(1:plotl),velocityXYZ_total(1:innerDyn_length:end,3),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('z-velocity [m/s]','FontSize',15)
legend({'z-dot-ref','z-velocity'},'Location','northeast','FontSize',15)

%% Plot the angles individually

% Phi
figure;
subplot(3,1,1)
plot(t_angles(1:length(ref_angles_total(:,1))),ref_angles_total(:,1),'--b','LineWidth',2)
hold on
subplot(3,1,1)
plot(t_angles(1:length(states_total(:,10))),states_total(:,10),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('phi-angle [rad]','FontSize',15)
legend({'phi-ref','phi-angle'},'Location','northeast','FontSize',15)

% Theta
subplot(3,1,2)
plot(t_angles(1:length(ref_angles_total(:,2))),ref_angles_total(:,2),'--b','LineWidth',2)
hold on
subplot(3,1,2)
plot(t_angles(1:length(states_total(:,11))),states_total(:,11),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('theta-angle [rad]','FontSize',15)
legend({'theta-ref','theta-angle'},'Location','northeast','FontSize',15)

% Psi
subplot(3,1,3)
plot(t_angles(1:length(ref_angles_total(:,3))),ref_angles_total(:,3),'--b','LineWidth',2)
hold on
subplot(3,1,3)
plot(t_angles(1:length(states_total(:,12))),states_total(:,12),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('psi-angle [rad]','FontSize',15)
legend({'psi-ref','psi-angle'},'Location','northeast','FontSize',15)


%% Plot the inputs

figure
subplot(4,1,1)
plot(t_angles(1:length(states_total(:,10))),UTotal(:,1))
grid on
xlabel('time [s]','FontSize',15)
ylabel('U1 [N]','FontSize',15)
subplot(4,1,2)
plot(t_angles(1:length(states_total(:,10))),UTotal(:,2))
grid on
xlabel('time [s]','FontSize',15)
ylabel('U2 [Nm]','FontSize',15)
subplot(4,1,3)
plot(t_angles(1:length(states_total(:,10))),UTotal(:,3))
grid on
xlabel('time [s]','FontSize',15)
ylabel('U3 [Nm]','FontSize',15)
subplot(4,1,4)
plot(t_angles(1:length(states_total(:,10))),UTotal(:,4))
grid on
xlabel('time [s]','FontSize',15)
ylabel('U4 [Nm]','FontSize',15)
