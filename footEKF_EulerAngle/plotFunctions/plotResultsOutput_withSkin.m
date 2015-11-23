function plotResultsOutput_withSkin(dataBaseFolder,processType,XUpt, XPred,P, tK, yM,source)

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

% wrench above foot
stateVar = 7:12;
stateVar = [7;10;8;11;9;12]; %proper ordering for proper visualization as subplots
pT.titleText = {'Expectation of force f_o',...
    'Expectation of Moment \mu_o',...
    '',...
    '',...
    '',...
    ''
    };
pT.xlabelText = {'Time t(sec)',...
    'Time t(sec)',...
    'Time t(sec)',...
    'Time t(sec)',...
    'Time t(sec)',...
    'Time t(sec)'
    };
pT.ylabelText = {'E(f_x) N',...
    'E(\mu_x) Nm',...
    'E(f_y) N',...
    'E(\mu_y) Nm',...
    'E(f_z) N'....
    'E(\mu_z) Nm'
    };
cols = {'b','b','g','g','r','r'};
plotFilterResultTimeSeries(tK(idx:end),XUpt(idx:end,:),yM(idx:end,:),P(:,:,idx:end),stateVar,pT,cols,[3,2]);%,sys(idx:end,:));

% wrench below foot
stateVar = 13:18;
stateVar = [13;16;14;17;15;18];
pT.titleText = {'Expectation of force f_c',...
    'Expectation of Moment \mu_c',...
    '',...
    '',...
    '',...
    ''
    };
cols = {'b','b','g','g','r','r'};
plotFilterResultTimeSeries(tK(idx:end),XUpt(idx:end,:),yM(idx:end,:),P(:,:,idx:end),stateVar,pT,cols,[3,2]);%,sys(idx:end,:));


% velocities
stateVar = 1:6;
stateVar = [1;4;2;5;3;6];
pT.titleText = {'Expectation of Translation Velocity v_B',...
    'Expectation of Angular Velocity \omega_B',...
    '',...
    '',...
    '',...
    ''
    };
pT.ylabelText = {'E(v_B_x) m/s',...
    'E(\omega_B_x) rad/s',...
    'E(v_B_y) m/s',...
    'E(\omega_B_y) m/s',...
    'E(v_B_z) m/s'....
    'E(\omega_B_z) rad/s'
    };
cols = {'b','b','g','g','r','r'};
plotFilterResultTimeSeries(tK(idx:end),XUpt(idx:end,:),[],P(:,:,idx:end),stateVar,pT,cols,[3,2]);
% plotFilterResultTimeSeries(tK(idx:end),XUpt(idx:end,:),[],P(:,:,idx:end),stateVar,pT,cols,[3,2]);%,sys(idx:end,:));

% orientation
stateVar = 19:21;
pT.titleText = {'Expectation of Orientation',...
    '',...
    '',...
    ''
    };
pT.ylabelText = {'E(\phi_1) degs',...
    'E(\phi_2) degs',...
    'E(\phi_3) degs'
    };
cols = {'b','g','r'};
plotFilterResultTimeSeries(tK(idx:end),rad2deg(XUpt(idx:end,:)),[],P(:,:,idx:end),stateVar,pT,cols,[]);%,sys(idx:end,:));
% 
%% Params
% if(strcmp(processType,'withCompliance') == 1)
%     
%     stateVar = [22;25;23;26;24;27];
%     
%     XUpt(:,22:24) = XUpt(:,22:24).^2; 
%     XUpt(:,25:27) = XUpt(:,25:27).^2; 
%     
% pT.titleText = {'Predicted Stiffness K_B',...
%     'Predicted Estimate Damping C_B',...
%     '',...
%     '',...
%     '',...
%     ''
%     };
% pT.ylabelText = {'k_{x}^{2} ',...
%     'c_{x}^{2}',...
%     'k_{y}^{2}',...
%     'c_{y}^{2}',...
%     'k_{z}^{2}'....
%     'c_{z}^{2}'
%     };
% cols = {'b','b','g','g','r','r'};
%     plotFilterResultTimeSeries(tK(idx:end),XUpt(idx:end,:),[],P(:,:,idx:end),stateVar,pT,cols,[3,2]);%,sys(idx:end,:));
%     
%     
% end
% FRI
% if(source==2)
%      plot_FRI(XUpt,P,XPred, P,tK, idx,length(tK));%,'b',index+5+figPreN);
%      
%  end


end
