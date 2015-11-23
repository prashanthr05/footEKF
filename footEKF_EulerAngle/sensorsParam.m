function [model,tMax,foot_ft,inertial_a,inertial_omega,transforms] = sensorsParam(whichLeg,t_min,t_max,dtKalman,model,expPath,expt,processType)
%% function returns the model parameters for treating the foot as a single rigid body.
%% Setting datasets

if(strcmp(model.urdf,'icubGazeboSim')==1)
expPath     = strcat(expPath,'/dumper/icubGazeboSim/');
FT2 = zeros(1,6);
FT3 = zeros(1,6);
FT4 = zeros(1,6);
FT5 = zeros(1,6);
else
expPath     = strcat(expPath,'/dumper/icub/');
[FT2,FT3,FT4,FT5] = readFTOffsets(expPath);
end

leg_choice  = whichLeg;

offsets.FT2 = FT2;
offsets.FT3 = FT3;
offsets.FT4 = FT4;
offsets.FT5 = FT5;
    
    
left_foot_ft_offset = offsets.FT3;
right_foot_ft_offset = offsets.FT5;


if(strcmp(leg_choice,'left')==1)
    foot_ft_offset = left_foot_ft_offset;
else
    foot_ft_offset = right_foot_ft_offset;
end
 

% Foot F/T analog sensor
if(strcmp(model.urdf,'icubGazeboSim')==1)
    foot_ft_data  = importdata(strcat(expPath,leg_choice,'_foot/analog:o/data.log'));
else
    foot_ft_data  = importdata(strcat(expPath,leg_choice,'_foot/analog_o/data.log'));
end
foot_ft.t = foot_ft_data(:,2)-foot_ft_data(1,2);
foot_ft.idx = foot_ft_data(:,1) - foot_ft_data(1,1);
foot_ft.data(:,3:8) = foot_ft_data(:,3:8) - repmat(foot_ft_offset,size(foot_ft_data,1),1);
foot_ft.f = foot_ft.data(:,3:5);
foot_ft.mu = foot_ft.data(:,6:8);


% Inertial sensor attached to the foot
if(strcmp(model.urdf,'icubGazeboSim')==1)
inertial_data = importdata(strcat(expPath,leg_choice,'_foot_IMU/data.log'));
inertial_a.t = inertial_data(:,2)-inertial_data(1,2);
inertial_a.data = inertial_data(:,6:8);

inertial_omega.t = inertial_data(:,2)-inertial_data(1,2);
inertial_omega.data = inertial_data(:,9:11);

com_R_imu = eye(3);

else
inertial_data = importdata(strcat(expPath,'right_leg/inertialMTB/data.log'));
inertial_a.t = inertial_data(:,13)-inertial_data(1,13);
inertial_a.data = inertial_data(:,14:16);

inertial_omega.t = inertial_data(:,25)-inertial_data(1,25);
inertial_omega.data = inertial_data(:,26:28);


load('com_R_imu.mat','com_R_imu');
end


ttmp = linspace(foot_ft.t(1),foot_ft.t(end),length(inertial_a.t));
inertial_a.data = interp1(ttmp,inertial_a.data,foot_ft.t);
inertial_a.t = foot_ft.t;

ttmp = linspace(foot_ft.t(1),foot_ft.t(end),length(inertial_omega.t));
inertial_omega.data = interp1(ttmp,inertial_omega.data,foot_ft.t);
inertial_omega.t = foot_ft.t;


%% Transformation between IMU sensor frame and body coordinate frame located at CoM
transforms.B_R_imu = com_R_imu;

%% include leg transformation to foot
    [model,transforms] = dynComp(model,inertial_a,inertial_omega,transforms,leg_choice,expPath,t_max,dtKalman);
    

%% world gravity
model.B0_g = [0;0;9.8];
model.phi0 = [0; pi/2; 0];
G_R_B = euler2dcm(model.phi0);
model.G_g = G_R_B*model.B0_g;


%%
tMax = min([foot_ft.t(end),inertial_a.t(end),inertial_omega.t(end),t_max]);


end

