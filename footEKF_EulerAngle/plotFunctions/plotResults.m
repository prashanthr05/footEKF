function plotResults(dataBaseFolder,processType,XUpt, XPred,P, tK, yM,source)

if(nargin<3)
    load(strcat(dataBaseFolder,'filteredResult.mat'));
    tK = tKalman;
    yM = yMeas;
    source = 2;
    XUpt = Xupdt;
    XPred = Xhat;
%     sysX = sys;
    tMin = tK(1);
end

idx = 1;

%% wrench above foot
%force
stateVar = 7:9; %proper ordering for proper visualization as subplots
pT.titleText = {'Expectation of force f_{o}^{b}',...

    };
pT.xlabelText = {'Time t(sec)',...
    };
pT.ylabelText = {'E(f) N'};
cols = {'b','g','r'};
plotResultTimeSeries(tK(idx:end),XUpt(idx:end,:),[],P(:,:,idx:end),stateVar,pT,cols,[]);%,sys(idx:end,:));

% moment
stateVar = 10:12; %proper ordering for proper visualization as subplots
pT.titleText = {  'Expectation of Moment \mu_{o}^{b}',...
        };
pT.ylabelText = {
    'E(\mu) Nm'};
cols = {'b','g','r'};
plotResultTimeSeries(tK(idx:end),XUpt(idx:end,:),[],P(:,:,idx:end),stateVar,pT,cols,[]);%,sys(idx:end,:));



%% wrench below foot
stateVar = 13:15;
pT.titleText = {'Expectation of force f_{c}^{b}'};
pT.ylabelText = {'E(f) N'};
cols = {'b','g','r'};
plotResultTimeSeries(tK(idx:end),XUpt(idx:end,:),[],P(:,:,idx:end),stateVar,pT,cols,[]);%,sys(idx:end,:));

% moment
stateVar = 16:18; %proper ordering for proper visualization as subplots
pT.titleText = {  'Expectation of Moment \mu_{c}^{b}'};
pT.ylabelText = {
    'E(\mu) Nm'
    };
cols = {'b','g','r'};
plotResultTimeSeries(tK(idx:end),XUpt(idx:end,:),[],P(:,:,idx:end),stateVar,pT,cols,[]);%,sys(idx:end,:));


%% velocities
% linear

stateVar = 1:3;
pT.titleText = {'Expectation of Translation Velocity v^{b}'};
pT.ylabelText = {'E(v) m/s'};
cols = {'b','g','r'};
plotResultTimeSeries(tK(idx:end),XUpt(idx:end,:),[],P(:,:,idx:end),stateVar,pT,cols,[]);%,sys(idx:end,:));

stateVar = 4:6;
pT.titleText = {'Expectation of Angular Velocity \omega^{b}'};
pT.ylabelText = {'E(\omega) rad/s'....
       };
cols = {'b','g','r'};
plotResultTimeSeries(tK(idx:end),XUpt(idx:end,:),[],P(:,:,idx:end),stateVar,pT,cols,[]);%,sys(idx:end,:));


%% orientation
stateVar = 19:21;
pT.titleText = {'Expectation of Orientation'};
pT.ylabelText = {'E(\phi) degs'
    };
cols = {'b','g','r'};

plotResultTimeSeries(tK(idx:end),rad2deg(XUpt(idx:end,:)),[],P(:,:,idx:end),stateVar,pT,cols,[]);%,sys(idx:end,:));

%% Params
if(strcmp(processType,'withCompliance') == 1)
%Stifness    
    stateVar = 22:24;
    
    XUpt(:,22:24) = XUpt(:,22:24).^2; 
    
    pT.titleText = {'Predicted stiffness matrix elements K_{c}'};
pT.ylabelText = {'k^{2} '};
cols = {'b','g','r'};
plotResultTimeSeries(tK(idx:end),XUpt(idx:end,:),[],P(:,:,idx:end),stateVar,pT,cols,[3,2]);%,sys(idx:end,:));

%Damping
    stateVar = 25:27;
    
    XUpt(:,25:27) = XUpt(:,25:27).^2; 
    
    pT.titleText = {'Predicted damping matrix elements C_{c}'};
pT.ylabelText = {'c^{2} '};
cols = {'b','g','r'};
plotResultTimeSeries(tK(idx:end),XUpt(idx:end,:),[],P(:,:,idx:end),stateVar,pT,cols,[3,2]);%,sys(idx:end,:));
    
    
end
% FRI
if(source==2)
     plot_FRI(XUpt,P,XPred, P,tK, idx,length(tK));%,'b',index+5+figPreN);
 end


end
