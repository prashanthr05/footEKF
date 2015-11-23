close all;
clc;
clear;

dataBaseFolder = sprintf('./data/');

load(strcat(dataBaseFolder,'withoutCompliancefilteredResult.mat'));
% load(strcat(dataBaseFolder,'fwdtipfirm.mat'));
x1 = Xupdt;
t1 = tKalman;

load(strcat(dataBaseFolder,'withoutCompliancefilteredResult2.mat'));
% load(strcat(dataBaseFolder,'fwdtip1layer.mat'));
x2 = Xupdt;
t2 = tKalman;

load(strcat(dataBaseFolder,'withoutCompliancefilteredResult3.mat'));
% load(strcat(dataBaseFolder,'fwdtip2layer.mat'));
x3 = Xupdt;
t3 = tKalman;

load(strcat(dataBaseFolder,'withoutCompliancefilteredResult4.mat'));
% load(strcat(dataBaseFolder,'fwdtip2layer.mat'));
x4 = Xupdt;
t4 = tKalman;

load(strcat(dataBaseFolder,'withoutCompliancefilteredResult5.mat'));
% load(strcat(dataBaseFolder,'fwdtip2layer.mat'));
x5 = Xupdt;
t5 = tKalman;

load(strcat(dataBaseFolder,'withoutCompliancefilteredResult6.mat'));
% load(strcat(dataBaseFolder,'fwdtip2layer.mat'));
x6 = Xupdt;
t6 = tKalman;

tMin = max([t1(1,1);t2(1,1);t3(1,1);t4(1,1);t5(1,1);t6(1,1)]);
tMax = min([t1(end);t2(end);t3(end);t4(end);t5(end);t6(end)]);
dtKalman = model.dtKalman;
t = linspace(tMin,tMax,(tMax - tMin)/dtKalman);

x1 = interp1(t1,x1,t);
x2 = interp1(t2,x2,t);
x3 = interp1(t3,x3,t);
x4 = interp1(t4,x4,t);
x5 = interp1(t5,x5,t);
x6 = interp1(t6,x6,t);

comp = ['x','y','z'];

%% Orientation RMSE
% figure();
% stateVar = 19:21;
% numSubFig=length(stateVar);
% name = {'Orientation RMS Error','',''};
%   
% subFig = [numSubFig,1];
% 
% for i = 1:numSubFig
%         
%     j = stateVar(i);
%         subplot(subFig(1),subFig(2),i);
% y = [orientationRMSE0314(i) ; orientationRMSE0628(i); orientationRMSE0942(i); orientationRMSE1256(i) ; orientationRMSE1571(i); orientationRMSE1884(i); orientationRMSE2198(i); orientationRMSE2512(i); orientationRMSE2826(i); orientationRMSE314(i)];
% bar(y);
%         xlabel('in rad/s   (1) 0.0314    (2) 0.628      (3) 0.942        (4) 1.256    (5) 1.57    (6) 1.884     (7) 2.198      (8) 2.512      (9) 2.826      (10) 3.14');
%         ylabel(strcat('\phi_',comp(i),' m/s'));
%         title(name{i});
% end
% % h = legend('ekf','jekf','dekf');
% % h.Box = 'off';
% % h.Location = 'northwest';
% % h.FontSize = 7;
% 
%  set(gca,'FontSize',12);
%  set(gcf,'Renderer','OpenGL');
%  print('-djpeg','-r200',strcat(dataBaseFolder,'orientationRMSE'),'-opengl')

%% Orientation
figure();
stateVar = 19:21;
numSubFig=length(stateVar);
name = {'Orientation','',''};
  
subFig = [numSubFig,1];

for i = 1:numSubFig
        
      subplot(subFig(1),subFig(2),i);
        plot(t,rad2deg(x1(:,stateVar(i))),'g');
        hold on
        plot(t,rad2deg(x2(:,stateVar(i))),'r');
        hold on
        plot(t,rad2deg(x3(:,stateVar(i))),'b');
        hold on
        
        plot(t,rad2deg(x4(:,stateVar(i))));
        hold on
        
        plot(t,rad2deg(x5(:,stateVar(i))));
        hold on
        
        plot(t,rad2deg(x6(:,stateVar(i))));
        hold on
        
        xlabel('Time in s','FontSize',14,'FontName','Times');
        ylabel(strcat('\phi_',comp(i),' deg'),'FontSize',14,'FontName','Times');
        h = legend('Trial 1', 'Trial 2','Trial 3', 'Trial 4', 'Trial 5' ,'Trial 6');
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
 
%  %% Stiffness
% figure();
% stateVar = 22:24;
% numSubFig=length(stateVar);
% name = {'Stiffness','',''};
%   
% subFig = [numSubFig,1];
% 
% for i = 1:numSubFig
%         
%       subplot(subFig(1),subFig(2),i);
%         plot(t,xfirm(:,stateVar(i)).^2,'g');
%         hold on
%         plot(t,x1layer(:,stateVar(i)).^2,'r');
%         hold on
%         plot(t,x2layer(:,stateVar(i)).^2,'b');
%         hold on
%         xlabel('Time in s','FontSize',14,'FontName','Times');
%         ylabel(strcat('k^{2}_',comp(i),' Nm/rad'),'FontSize',14,'FontName','Times');
%         h = legend('Firm', '1 layer thermoplastic','2 layer thermo plastic');
%         xlim([t(1) t(end)])
%         title(name{i},'FontSize',14,'FontName','Times');
%         h.Box = 'off';
% h.Location = 'southeast';
% h.FontSize = 7;
% end
% % h = legend('t = 0.01s','t = 0.1s','t = 1s');
%     set(gca,'YTickMode','auto');
% 
%  set(gca,'FontSize',12);
%  set(gcf,'Renderer','OpenGL');
%  print('-djpeg','-r200',strcat(dataBaseFolder,'orientationCompare'),'-opengl')
% 
%  
%  
%   %% Stiffness
% figure();
% stateVar = 25:27;
% numSubFig=length(stateVar);
% name = {'Damping','',''};
%   
% subFig = [numSubFig,1];
% 
% for i = 1:numSubFig
%         
%       subplot(subFig(1),subFig(2),i);
%         plot(t,xfirm(:,stateVar(i)).^2,'g');
%         hold on
%         plot(t,x1layer(:,stateVar(i)).^2,'r');
%         hold on
%         plot(t,x2layer(:,stateVar(i)).^2,'b');
%         hold on
%         xlabel('Time in s','FontSize',14,'FontName','Times');
%         ylabel(strcat('c^{2}_',comp(i),' Nms/rad'),'FontSize',14,'FontName','Times');
%         h = legend('Firm', '1 layer thermoplastic','2 layer thermo plastic');
%         xlim([t(1) t(end)])
%         title(name{i},'FontSize',14,'FontName','Times');
%         h.Box = 'off';
% h.Location = 'southeast';
% h.FontSize = 7;
% end
% % h = legend('t = 0.01s','t = 0.1s','t = 1s');
%     set(gca,'YTickMode','auto');
% 
%  set(gca,'FontSize',12);
%  set(gcf,'Renderer','OpenGL');
%  print('-djpeg','-r200',strcat(dataBaseFolder,'orientationCompare'),'-opengl')
