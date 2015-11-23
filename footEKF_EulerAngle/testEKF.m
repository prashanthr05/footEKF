% Department of Robotics, Brain and Cognitive Sciences
% Istituto Italiano di Tecnologia, 11 September 2014
%
% Original code by: Francesco Nori,
% Modified to include local parametrization by: Jorhabib Eljaik G. and
% Naveen Kuppuswamy
%
% Modified to include constraint moment by: Prashanth Ramadoss
%
% This piece of code simulates the problem of estimating dynamic quantities
% for a single rigid body with distributed force/torque measurements and
% distributed gyro/accelerometers measurements. The motion is governed by
% the following differential equation with a local parametrization of the
% orientation in ZYZ Euler angles.
%
% m    dv^B    + S(omega^B) (m       v^B) = f^B_1  + ... + f^B_n + mg^B
%
% I^B domega^B + S(omega^B) (I^B omega^B) = mu^B_1 + ... + mu^B_n
%
%                                  dphi   = T_phi^-1 * omega
%
% where we defined the following quantities:
%
% I^B    : inertia in the body reference frame
% m      : mass of the rigid body
% f^B_i  : i-th force expressed in the body reference frame
% mu^B_i : i-th torque expressed in the body reference frame
% omega^B: angular velocity expressed in the body reference frame
% v^B    : linear velocity in the body reference frame
% phi    : ZYZ Euler angles representing orientation.
% T_phi  : Transformation matrix between omega and rotational velocities

clear
close all
clc

%% adding of paths
utilities    = genpath('./utils');
symb         = genpath('./symbolicFunctions');
dynFuncs     = genpath('./dynamicsFunctions');
plotFuncs   = genpath('./plotFunctions');
urdf = genpath('./model-urdf');

addpath(utilities, symb, dynFuncs, plotFuncs)


%% Model Parameters common across experiments
setup.dtForDyn = 0.0001; % EKF forward dynamics computation time step
setup.dtKalman = 0.01; % EKF computation time step (discretisation)

% setup.urdf = 'icubGazeboSim'; %options 'icubGazeboSim' 'icubGenova02'
setup.urdf = 'icubGenova02';

setup.t_min = 3.0; % time until which to calibrate
setup.t_max = 20.0; % Max time in dataset until which to filter
if(strcmp(setup.urdf,'icubGazeboSim') == 1)
    setup.t_min = 3.0; % time until which to calibrate
    setup.t_max = 300.0;%300; % Max time in dataset until which to filter
end

setup.measurementPlots = 'makePlots'; % options - 'makePlots' , 'noPlots'
setup.filterOutputPlots = 'makePlots';
setup.skipSteps = 50; % no of steps to skip for diplaying kalman execution time in loop

expt = 'foot'; %options - 'foot'
filter = 'ekf';
legChoice = 'right'; %options - use 'left' for gazebo; 'right' for robot


%% toggle with dataset - structuring of folders should be taken care of - refer folder robotData
if(strcmp(setup.urdf,'icubGazeboSim') == 1)
    dataSet = 'fwdTip'; % options 'fwdTip' 'bwdTip' 'standing' 'rotate'  - check folder robotData for more details
    trial = '1'; % change for 'rotate' accordingly referring to folder robotData
    expChoice = strcat(dataSet,trial);
    expPath = strcat('./robotData/gazebo/',dataSet,'/',expChoice);
else
    dataSet = 'standing'; % options 'fwdTip' 'bwdTip' 'standing' 'rotate' 'contact'  - check folder robotData for more details
    surface = 'Firm'; %options 'Firm' '1LayerMat' '2LayerMat'
    trial = '1';
    expChoice = strcat(dataSet,surface);
    expData = strcat(expChoice,trial);
    expPath = strcat('./robotData/robot/',dataSet,'/',expChoice,'/',expData);
    
    if(strcmp(dataSet,'rotate')==1)
        surface = 'RightFoot'; 
        trial = ''; % '' 'Pitch1s' 'Pitch2s' ... 'Pitch5s'
        expChoice = strcat(dataSet,surface);
        expData = strcat(expChoice,trial);
        expPath = strcat('./robotData/robot/',dataSet,'/',expData);
    end
end


%% Filter EKF
fprintf('\nRunning %s \n-------------\n\n',filter);

clearvars -except filter setup expPath expt legChoice
close all



[kalmanQParams,kalmanRParams,kIni,cIni] = setCovariances(filter);
setup.kalmanQParams = kalmanQParams;
setup.kalmanRParams = kalmanRParams;
setup.k = kIni; % initial stiffness (defined but not used for all experiments)
setup.c = cIni; % initial damping
setup.w = [kIni;cIni];

setup.filter = 'ekf';

if(strcmp(setup.urdf,'icubGazeboSim') == 1)
    expID = 1;%:2;
    % 1 - without compliance
    % 2 - with compliance
    
    processSuffix = {'withoutCompliance','withCompliance'};
    n       = [21,27];         % state dimension - (translational vel, rotational vel, w_o, w_c, RPY angle)
    m       = [18,18];         % output dimension
    p       = [0,0];
else
    expID = 2;
    % 1 - without compliance
    % 2 - with compliance
    processSuffix = {'withoutCompliance','withCompliance'};
    n       = [21,27];         % state dimension - (translational vel, rotational vel, w_o, w_c, RPY angle)
    m       = [18,18];         % output dimension
    p       = [0,0];
end


for j = expID
    fprintf('\n Using foot FT and inertial measurements for the experiments \n');
    fprintf('\nProcessing measurement with process %s\n-------------\n\n',processSuffix{j});
    clearvars -except filter j expID processSuffix n m p setup expPath expt legChoice
    close all
    
    %% Function handls for process mode, measurement model and their derivatives
    
    f_func     = str2func(strcat('forwardDynamics_',processSuffix{j}));
    df_dx_func = str2func(strcat('wrapperForwardDynamicsDerivative_',processSuffix{j}));
    dh_dx_func = str2func(strcat('wrapperOutputsDerivatives_',expt,'_',processSuffix{j}));
    h_func = str2func(strcat('output_',expt,'_',processSuffix{j}));
    
    
    %% Kalman Parameters
    kalmanQParams = setup.kalmanQParams;
    % setting up process covariances
    kalman.a_Q  = kalmanQParams(1);%4.5;
    kalman.omega_Q  = kalmanQParams(2);%4.0;%4.75;
    kalman.f_Q  = kalmanQParams(3);%0.5;%6.5;
    kalman.mu_Q =kalmanQParams(4);%2.5;%6.5;
    kalman.phi_Q = kalmanQParams(5);%1.5;%2.50;
    kalman.k_Q =kalmanQParams(6);
    kalman.c_Q =kalmanQParams(7);
    
    
    kalmanRParams = setup.kalmanRParams;
    % setting up measurement covariances
    kalman.sigma_f = kalmanRParams(1);
    kalman.sigma_u = kalmanRParams(2);
    kalman.sigma_a = kalmanRParams(3);
    kalman.sigma_omega = kalmanRParams(4);
    
    
    model = setup;
    t_min = model.t_min;
    t_max = model.t_max;
    
    [yMeas,tMeas,model,RData] = measurement(model.dtKalman,model,model.measurementPlots,t_min,t_max,processSuffix{j},legChoice,expPath,expt);
    T = tMeas(end);
    tKalman = tMeas;
    
    %% Setting up Filter Covariances
    
    %Measurement Noise Covariance
    forceR = 'true';
    
    if(strcmp(forceR,'true')==1 || ~exist('RData'))
        disp('Assuming an R value');
        R = diag([kalman.sigma_a.*ones(1,3), kalman.sigma_omega.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3)]);
        
    else
        
        % modify following two lines to check for without skin option
        disp('Using real data covariance matrix');
        R = RData;
    end
    
    %Process Noise and State Transition Covariance
    switch(processSuffix{j})
        case 'withoutCompliance'
            Q  = diag([kalman.a_Q*ones(3,1);
                kalman.omega_Q*ones(3,1);
                kalman.f_Q*ones(3,1);
                kalman.mu_Q*ones(3,1);
                kalman.f_Q*ones(3,1);
                kalman.mu_Q*ones(3,1);
                kalman.phi_Q*ones(3,1)]);
            kalman.P = 15*diag([kalman.a_Q*ones(3,1);kalman.omega_Q*ones(3,1);kalman.f_Q*ones(3,1);kalman.mu_Q*ones(3,1);kalman.f_Q*ones(3,1);kalman.mu_Q*ones(3,1);kalman.phi_Q*ones(3,1)]);
            x0 = model.x0;
            
        case 'withCompliance'
            Q  = diag([kalman.a_Q*ones(3,1);
                kalman.omega_Q*ones(3,1);
                kalman.f_Q*ones(3,1);
                kalman.mu_Q*ones(3,1);
                kalman.f_Q*ones(3,1);
                kalman.mu_Q*ones(3,1);
                kalman.phi_Q*ones(3,1);
                kalman.k_Q*ones(3,1);
                kalman.c_Q*ones(3,1)]);
            
            kalman.P = 15*diag([kalman.a_Q*ones(3,1);kalman.omega_Q*ones(3,1);kalman.f_Q*ones(3,1);kalman.mu_Q*ones(3,1);kalman.f_Q*ones(3,1);...
                kalman.mu_Q*ones(3,1);kalman.phi_Q*ones(3,1);kalman.k_Q*ones(3,1);kalman.c_Q*ones(3,1)]);
            x0 = model.x0;
    end
    
    
    
    %% KALMAN FILTER IMPLEMENTATION
    %% initialising EKF
    % Initializing estimate and update
    model.dt = model.dtKalman;
    Ph = kalman.P;
    xh        = x0;
    
    
    Xhat      = zeros(n(j),length(tKalman))';
    Xupdt = zeros(length(tKalman),n(j));
    P = zeros(size(Ph,1), size(Ph,2),length(tKalman));
    
    
    
    disp('Starting Kalman Filter prediction');
    drawnow;
    
    %% EKF execution
    for k = 1:length(tKalman)
        tic;
        
        % Update step
        [xh, Ph] = ekf_update1(xh , Ph, yMeas(k,:)', dh_dx_func, R,h_func, [], model);
        
        Xupdt(k,:) = xh;
        Ph = (Ph + Ph')/2;
        xAfterUpdate = xh;
        pAfterUpdate = Ph;
        
        
        % Prediction step
        [xh, Ph] =  ekf_predict1(xh, Ph, df_dx_func, Q, f_func, [], model);
        
        Xhat(k,:) = xh;
        P(:,:,k)  = Ph;
        
        
        if(mod(k,model.skipSteps)==0)
            %                         fprintf('\n %d Steps processing time : ',model.skipSteps);
            %                         disp(toc());
            fprintf('\n Time  : %d \n',t_min + (k*model.dtKalman));
        end
        
    end
    
    
    
    plotFigBaseFolder = sprintf('./plots/%s/',processSuffix{j});
    dataBaseFolder = sprintf('./data/%s/',processSuffix{j});
    
    
    %% Plots
    
    if(~exist(dataBaseFolder))
        mkdir(dataBaseFolder);
    end
    
    finalVersion =0;
    if(finalVersion == 1)
        close all;
    end
    
    save(strcat(dataBaseFolder,'filteredResult.mat'),'tKalman','yMeas','Xupdt','Xhat','P','model');

    if(strcmp(model.filterOutputPlots,'makePlots') == 1)
        plotAndSaveFigs(dataBaseFolder,plotFigBaseFolder,processSuffix{j});
    end
    
     if(j < 4)
                 fprintf('\nPress any key to continue to next experiment\n');
                 pause;
     end
    
    
end



