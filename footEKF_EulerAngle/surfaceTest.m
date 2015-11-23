close all;
clc;
clear;

dataBaseFolder = sprintf('./data/withCompliancewithoutSkin/');

load(strcat(dataBaseFolder,'0firm1.mat'));
% load(strcat(dataBaseFolder,'fwdtipfirm.mat'));
xfirm = Xupdt;
tfirm = tKalman;

load(strcat(dataBaseFolder,'1layer1.mat'));
% load(strcat(dataBaseFolder,'fwdtip1layer.mat'));
x1layer = Xupdt;
t1layer = tKalman;

load(strcat(dataBaseFolder,'2layer1.mat'));
% load(strcat(dataBaseFolder,'fwdtip2layer.mat'));
x2layer = Xupdt;
t2layer = tKalman;



tMin = max([tfirm(1,1);t1layer(1,1);t2layer(1,1)]);
tMax = min([tfirm(end);t1layer(end);t2layer(end)]);
dtKalman = model.dtKalman;
t = linspace(tMin,tMax,(tMax - tMin)/dtKalman);

xfirm = interp1(tfirm,xfirm,t);
x1layer = interp1(t1layer,x1layer,t);
x2layer = interp1(t2layer,x2layer,t);


comp = ['x','y','z'];


%% Orientation
figure();
stateVar = 19:21;
numSubFig=length(stateVar);
name = {'Orientation','',''};
  
subFig = [numSubFig,1];

for i = 1:numSubFig
        
      subplot(subFig(1),subFig(2),i);
        plot(t,rad2deg(xfirm(:,stateVar(i))),'g');
        hold on
        plot(t,rad2deg(x1layer(:,stateVar(i))),'r');
        hold on
        plot(t,rad2deg(x2layer(:,stateVar(i))),'b');
        hold on
        xlabel('Time in s','FontSize',14,'FontName','Times');
        ylabel(strcat('\phi_',comp(i),' deg/s'),'FontSize',14,'FontName','Times');
        h = legend('Firm', '1 layer thermoplastic','2 layer thermo plastic');
        xlim([t(1) t(end)])
        title(name{i},'FontSize',14,'FontName','Times');
        h.Box = 'off';
h.Location = 'southeast';
h.FontSize = 7;
end
% h = legend('t = 0.01s','t = 0.1s','t = 1s');
    set(gca,'YTickMode','auto');

 set(gca,'FontSize',12);
 set(gcf,'Renderer','OpenGL');
 print('-djpeg','-r200',strcat(dataBaseFolder,'orientationCompare'),'-opengl')
 
 %% Stiffness
figure();
stateVar = 22:24;
numSubFig=length(stateVar);
name = {'Stiffness','',''};
  
subFig = [numSubFig,1];

for i = 1:numSubFig
        
      subplot(subFig(1),subFig(2),i);
        plot(t,xfirm(:,stateVar(i)).^2,'g');
        hold on
        plot(t,x1layer(:,stateVar(i)).^2,'r');
        hold on
        plot(t,x2layer(:,stateVar(i)).^2,'b');
        hold on
        xlabel('Time in s','FontSize',14,'FontName','Times');
        ylabel(strcat('k^{2}_',comp(i),' Nm/rad'),'FontSize',14,'FontName','Times');
        h = legend('Firm', '1 layer thermoplastic','2 layer thermo plastic');
        xlim([t(1) t(end)])
        title(name{i},'FontSize',14,'FontName','Times');
        h.Box = 'off';
h.Location = 'southeast';
h.FontSize = 7;
end
% h = legend('t = 0.01s','t = 0.1s','t = 1s');
    set(gca,'YTickMode','auto');

 set(gca,'FontSize',12);
 set(gcf,'Renderer','OpenGL');
 print('-djpeg','-r200',strcat(dataBaseFolder,'orientationCompare'),'-opengl')

 
 
  %% Stiffness
figure();
stateVar = 25:27;
numSubFig=length(stateVar);
name = {'Damping','',''};
  
subFig = [numSubFig,1];

for i = 1:numSubFig
        
      subplot(subFig(1),subFig(2),i);
        plot(t,xfirm(:,stateVar(i)).^2,'g');
        hold on
        plot(t,x1layer(:,stateVar(i)).^2,'r');
        hold on
        plot(t,x2layer(:,stateVar(i)).^2,'b');
        hold on
        xlabel('Time in s','FontSize',14,'FontName','Times');
        ylabel(strcat('c^{2}_',comp(i),' Nms/rad'),'FontSize',14,'FontName','Times');
        h = legend('Firm', '1 layer thermoplastic','2 layer thermo plastic');
        xlim([t(1) t(end)])
        title(name{i},'FontSize',14,'FontName','Times');
        h.Box = 'off';
h.Location = 'southeast';
h.FontSize = 7;
end
% h = legend('t = 0.01s','t = 0.1s','t = 1s');
    set(gca,'YTickMode','auto');

 set(gca,'FontSize',12);
 set(gcf,'Renderer','OpenGL');
 print('-djpeg','-r200',strcat(dataBaseFolder,'orientationCompare'),'-opengl')
