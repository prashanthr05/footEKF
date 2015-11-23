function [yMeas,tMeas,model,RData] = measurement(dtKalman, model, plots, t_min, t_max, processType, whichLeg, expPath, expt)

if(nargin<1)
    processType = 'withCompliance';
    dtKalman = 0.01;
    model = struct();
    plots = 0;
    t_min = 0;
    t_max = inf;
    whichLeg  = 'right';
    expPath = './robotData/gazebo/bwdTip/bwdTip1';
    expt = 'leg';
end

if (nargin<6)
    processType = 'withCompliance';
    whichLeg  = 'right';
    expPath = './robotData/gazebo/bwdTip/bwdTip1';
    expt = 'leg';
end

%% Setting up fake measurements for check
fakeIMU = 'false';
fakeFT_f = 'false';
fakeFT_mu = 'false';

fakeAccl = [0;0;-9.8];
fakeMu_o = [0;0;0];
fakeF_o = [0;0;0];


[model,tMax,foot_ft,inertial_a,inertial_omega,transforms] = sensorsParam(whichLeg,t_min,t_max,dtKalman,model,expPath,expt,processType);

model.dtKalman = dtKalman;
t = linspace(t_min,tMax,(tMax - t_min)/dtKalman);
tCalib = linspace(0,t_min,(t_min)/dtKalman);

%% Calibrating the measurements
a_calib = interp1(inertial_a.t,inertial_a.data,tCalib);
omega_calib = interp1(inertial_omega.t,inertial_omega.data,tCalib);
f_foot_calib = interp1(foot_ft.t,foot_ft.f,tCalib);
mu_foot_calib = interp1(foot_ft.t,foot_ft.mu,tCalib);

% using conversion factor for real robot inertial readings
if(strcmp(model.urdf,'icubGazeboSim') ~= 1)
    a_calib = a_calib(:,1:3).*5.9855e-04;
    omega_calib = omega_calib(:,1:3).*7.6274e-03;
end

f_mu_foot_calib = transforms.B_adjT_ankle*[f_foot_calib';mu_foot_calib'];

f_foot_calib = f_mu_foot_calib(1:3,:);
mu_foot_calib = f_mu_foot_calib(4:6,:);


fMean = mean(f_foot_calib,2);
muMean = mean(mu_foot_calib,2);
aMean = mean((a_calib'),2);
omegaMean = mean((omega_calib'),2);

aDeviation = a_calib' - repmat(aMean,1,size(a_calib,1));
omegaDeviation = omega_calib' - repmat(omegaMean,1,size(omega_calib,1));
fDeviation = f_foot_calib - repmat(fMean,1,size(f_foot_calib,2));
muDeviation = mu_foot_calib - repmat(muMean,1,size(mu_foot_calib,2));

yCalibBar = [aDeviation;omegaDeviation;fDeviation;muDeviation]';

RData = (1/size(yCalibBar,2))*(yCalibBar'*yCalibBar);


%% raw interpolated data

f_foot_raw = interp1(foot_ft.t,foot_ft.f,t);
mu_foot_raw = interp1(foot_ft.t,foot_ft.mu,t);

a_raw = interp1(inertial_a.t,inertial_a.data,t);
omega_raw = interp1(inertial_omega.t,inertial_omega.data,t);

% using conversion factor for real robot inertial readings
if(strcmp(model.urdf,'icubGazeboSim') ~= 1)
    a_raw = a_raw(:,1:3).*5.9855e-04;
    omega_raw = omega_raw(:,1:3).*7.6274e-03;
end


rawData.f_foot = f_foot_raw;
rawData.mu_foot = mu_foot_raw;
rawData.a = a_raw;
rawData.omega = omega_raw;


%IMU
a = transforms.B_R_imu*a_raw';
omegaCentered = (omega_raw')' - repmat(omegaMean',size(omega_raw,1),1);
omega = (transforms.B_R_imu*omegaCentered')'.*(pi/180);

f_mu_foot = transforms.B_adjT_ankle*[f_foot_raw';mu_foot_raw'];
f_foot = f_mu_foot(1:3,:);
mu_foot = f_mu_foot(4:6,:);



fo = f_foot;
muo = mu_foot;
fc = fo + model.m*a;
muc = muo;


%% Fake Measurements
if(strcmp(fakeIMU,'true')==1)
    aPerfect = fakeAccl*ones(1,size(a,2));
    omegaPerfect = zeros(size(omega));
    a = aPerfect;
    omega = omegaPerfect;
end


if(strcmp(fakeFT_f,'true')==1)
    fgPerfect = model.m*transpose(euler2dcm(model.phi0))*model.G_g*ones(1,size(f_foot,2));
    fcPerfect = - 0.5*fgPerfect;
    foPerfect = -fcPerfect;
    fo = foPerfect;
    fc = fcPerfect;
    
end

if(strcmp(fakeFT_mu,'true')==1)
    muPerfect = fakeMu_o*ones(1,size(mu_foot,2));
    
    muo = muPerfect;
    muc = muPerfect;
end

%%   Seting up measurement and initial conditions



fprintf('\n setting up robot measurements for leg\n');
if(strcmp(processType,'withoutCompliance')==1)
%     model.x0 = [zeros(3,1);zeros(3,1);fo(:,1);muo(:,1);fc(:,1);muc(:,1);model.phi0];
        model.x0 = [zeros(18,1); 0; pi/4; 0]

else if(strcmp(processType,'withCompliance')==1)
        model.x0 = [zeros(3,1);zeros(3,1);fo(:,1);muo(:,1);fc(:,1);muc(:,1);model.phi0;model.k;model.c];
        model.x0 = [zeros(18,1); 0; pi/4; 0; zeros(6,1)]
    end
end


yMeas = [a;omega';fo;muo;fc;muc]';

tMeas = t;

%% Plot Measurements
if (strcmp(model.measurementPlots,'makePlots')==1)
    figure(6);
    %
    subplot(2,2,1);
    plot(t,a_raw);
    xlabel('time (sec)');
    ylabel('a - g (m/s^2)');
    legend('aX', 'aY', 'aZ');
    axis tight;
    title('Raw Proper Acceleration');
    
    subplot(2,2,2);
    plot(t,omega_raw);
    xlabel('time (sec)');
    ylabel('\omega (deg/s)');
    legend('\omega_{X}', '\omega_{Y}', '\omega_{Z}');
    axis tight;
    title('Raw Angular Velocity');
    
    subplot(2,2,3);
    plot(t,a);
    xlabel('time (sec)');
    ylabel('a - g (m/s^2)');
    legend('aX', 'aY', 'aZ');
    axis tight;
    title('Proper Acceleration expressed in Body frame');
    
    subplot(2,2,4);
    plot(t,omega);
    xlabel('time (sec)');
    ylabel('\omega (deg/s)');
    legend('\omega_{X}', '\omega_{Y}', '\omega_{Z}');
    axis tight;
    title('Angular Velocity expressed in Body frame');
    
    
    figure(7);
    %
    subplot(2,2,1);
    plot(t,fo);
    xlabel('time (sec)');
    ylabel('fo (N)');
    legend('foX', 'foY', 'foZ');
    axis tight;
    title('Body Force expressed in Body Frame');
    
    subplot(2,2,2);
    plot(t,muo);
    xlabel('time (sec)');
    ylabel('\mu_{o} (Nm)');
    legend('\muo_{X}', '\muo_{Y}', '\muo_{Z}');
    axis tight;
    title('Body Moment in Body Frame');
    
    subplot(2,2,3);
    plot(t,fc);
    xlabel('time (sec)');
    ylabel('fc (Nm)');
    legend('fcX', 'fcY', 'fcZ');
    axis tight;
    title('Contact Force expressed in Body frame');
    
    subplot(2,2,4);
    plot(t,muc);
    xlabel('time (sec)');
    ylabel('\mu_{c} (Nm)');
    legend('\muc_{X}', '\muc_{Y}', '\muc_{Z}');
    axis tight;
    title('Contact expressed in Body frame');
    
end

end
