function plotAndSaveFigs(dataBaseFolder,plotFigBaseFolder,processType)
plotResultsOutput_withSkin(dataBaseFolder,processType);
% plotResults(dataBaseFolder,processType);
%plotFigBaseFolder = 'plots/irosMain/';
%dataBaseFolder = './data/acclTests/';



if(~exist(plotFigBaseFolder,'dir'))
    mkdir(plotFigBaseFolder);
end

plotFigBaseName = strcat('./',plotFigBaseFolder,'predicted');

if(strcmp(processType,'withoutCompliance') == 1)
%     
%     selectedFigList = 1:6;
%     FigName = {'UpperWrench'...
%         'LowerWrench',...
%         'Velocities',...
%         'Orientation',...
%         'COP FRI',...
%         'FRI Trajectory'};
selectedFigList = 1; FigName = {'Orientation'};
    %
else if(strcmp(processType,'withCompliance') == 1)
        
        
        
%         selectedFigList = 1:7;
%         FigName = {'UpperWrench'...
%             'LowerWrench',...
%             'Velocities',...
%             'Orientation',...
%             'Stiffness&Damping',...
%             'COP FRI',...
%             'FRI Trajectory'};
    selectedFigList = 1; FigName = {'Orientation'};

    end
end

for i = 1:length(selectedFigList)
    figure(selectedFigList(i))
    set(gca,'FontSize',12);
    set(gcf,'Renderer','OpenGL');
    print('-depsc2','-r200',strcat(plotFigBaseName,FigName{i}),'-opengl');
end
end