clc
clear
close all

addpath(genpath('./utils'));

%Defining object of DynamicsComputations
dynComp = iDynTree.DynamicsComputations();
% expPath = ('./calibData/dumper/icubGazeboSim/');
expPath = ('./robotData/calibIMU/dumper/icub/');
% model.urdf = 'icubGazeboSim';
model.urdf = 'icubGenova02';

%% Load robot urdf
%Loading urdf of corresponding robot
loadModel = dynComp.loadRobotModelFromFile(strcat(model.urdf,'.urdf'),'urdf'); % Loading iCubGenova03 model.urdf
regCheck = dynComp.isValid(); %checking if regressor generator is correctly configured
dofInternal = dynComp.getNrOfDegreesOfFreedom();

%% Setting l_foot  as world frame and r_foot as floating base

    footCheck = dynComp.setFloatingBase('r_foot'); %Setting floating base to the l_foot
    base_link = dynComp.getFloatingBase();
    idx = dynComp.getFrameIndex(base_link);
    world = dynComp.getFrameIndex('l_foot');

    aWorld = [0;0;-9.8]; %expressed as a - g % if world frame is l_foot
%     aWorld = [9.8;0;0]; %expressed as a - g  % if world frame is l_foot_dh_frame

    aWorld = aWorld/norm(aWorld,2); 
    aBaseLink = zeros(3,1);
    
%% Set robot state 
%Obtain joint states from torso, right leg and left leg
torso_state = importdata(strcat(expPath,'torso/state:o/data.log'));
torso.timeStamp = torso_state(:,2) - torso_state(1,2) ;
torso.q = torso_state(:,3:end).*(pi/180);

right_leg_state = importdata(strcat(expPath,'right_leg/state:o/data.log'));
right_leg.timeStamp = right_leg_state(:,2) - right_leg_state(1,2) ;
right_leg.q = right_leg_state(:,3:end).*(pi/180);

left_leg_state = importdata(strcat(expPath,'left_leg/state:o/data.log'));
left_leg.timeStamp = left_leg_state(:,2) - left_leg_state(1,2) ;
left_leg.q = left_leg_state(:,3:end).*(pi/180);


%% IMU acceleration measurement a - g in the form of proper acceleration
% inertial_data = importdata(strcat(expPath,'right_foot_IMU/data.log'));
% inertial.t = inertial_data(:,2)-inertial_data(1,2);
% inertial.idx = inertial_data(:,1) - inertial_data(1,1);
% inertial.data = inertial_data(:,3:end);


% enabled accelerometers and gyros are r_foot_1 and r_foot_2 - check wiki
% for data format in the data log. In our case active accelerometer giving
% raw output is indexed with number 33 (r_foot_2), use conversion factor to
% measure in desired units

%To convert the accelerometer measure in m/s^2 the conversion factor is (fullscale in m/s^2)/(fullscale in raw) = (2*g)/(2^15) ~= 5.9855e-04.
%To convert the gyroscope measure in dps (deg/s) the conversion factor is (fullscale in dps)/(fullscale in raw) = (250)/(2^15) ~= 7.6274e-03.
inertial_data = importdata(strcat(expPath,'right_leg/inertialMTB/data.log'));
inertial.t = inertial_data(:,13)-inertial_data(1,13);
inertial.data = inertial_data(:,14:16);


ttmp = linspace(torso.timeStamp(1),torso.timeStamp(end),length(inertial.t));
inertial.data = interp1(ttmp,inertial.data,torso.timeStamp);
inertial.t = torso.timeStamp;

a.q = inertial.data(:,1:3).*5.9855e-04;
a.timeStamp = inertial.t;



[t,el] = min([length(a.timeStamp),length(torso.timeStamp),length(right_leg.timeStamp),length(left_leg.timeStamp)]);

switch(el)
    case 1
        tC = a.timeStamp(1:end);
    case 2
        tC = torso.timeStamp(1:end);
    case 3
        tC = right_leg.timeStamp(1:end);
    case 4
        tC = left_leg.timeStamp(1:end);
    
end

tmin = tC(1);
tmax = tC(end);
dt = 0.01;
tcalib = linspace(tmin,tmax,(tmax - tmin)/dt);

a.q = interp1(a.timeStamp,a.q,tcalib);
left_leg.q = interp1(left_leg.timeStamp,left_leg.q,tcalib);

%workaround for unknown NAN in  last row
left_leg.q(end,:) = left_leg.q(end-1,:); 
right_leg.q = interp1(right_leg.timeStamp,right_leg.q,tcalib);
torso.q = interp1(torso.timeStamp,torso.q,tcalib);


%% To find min || a_Meas - transpose(com_R_imu)*a_B ||
a_Meas = a.q';

a_B = zeros(3,length(tcalib));
aOffset = zeros(3,length(tcalib));


q = iDynTree.VectorDynSize();
q_dot = iDynTree.VectorDynSize();
q_dotdot = iDynTree.VectorDynSize();
base_vel = iDynTree.Twist();
base_acc = iDynTree.ClassicalAcc();
world_T_base = iDynTree.Transform();
world_gravity = iDynTree.SpatialAcc();


B = 0; %used for SVD
w = zeros(length(tcalib),4); %Measured vector
%% Setting robot state and computing forward kinematics
for i = 1 : length(tcalib)
jointpos = zeros(dofInternal,1);
jointpos(1:6,1) = left_leg.q(i,1:6);
jointpos(7:12,1) = right_leg.q(i,1:6);
jointpos(15,1) = torso.q(i,1);
jointpos(14,1) = torso.q(i,2);
jointpos(13,1) = torso.q(i,3);
q.fromMatlab(jointpos);

    jointvel = zeros(dofInternal,1);
    q_dot.fromMatlab(jointvel);
    
    jointacc = zeros(dofInternal,1);
    q_dotdot.fromMatlab(jointacc);

lin_vel = zeros(3,1);
ang_vel = zeros(3,1);
base_vel.fromMatlab([lin_vel;ang_vel]);

lin_acc = zeros(3,1);
ang_acc = zeros(3,1);
base_acc.fromMatlab([lin_acc;ang_acc]);

state_set(i) = dynComp.setRobotState(q,q_dot,q_dotdot,world_T_base,base_vel,base_acc,world_gravity);


if(state_set(i))
    world_T_body = dynComp.getRelativeTransform(world,idx);
    world_R_body = world_T_body.getRotation().toMatlab();
    aBaseLink = transpose(world_R_body)*aWorld;
end

a_B(1:3,i) = aBaseLink/norm(aBaseLink,2);
a_Meas(1:3,i) = a_Meas(1:3,i)/norm(a_Meas(1:3,i),2) ;
w(i,1:3) = a_Meas(1:3,i)';
w(i,4) = 1;
end

Y = a_B'; %known gravity vector
% Y(nx3) = w(nx4) * R (4x3)

X = inv(transpose(w)*w)*transpose(w)*Y;
com_R_imu  = transpose(X(1:3,1:3))
Offsets = X(4,:)

%last row of R is the offsets
%save('com_R_imu.mat','com_R_imu')
% save('R.mat','com_R_imu')
