function [ output_args ] = plotResultTimeSeries( t, x, y, P,idx, plotText,cols,subFig)%, sysX )
%PLOTFILTERRESULTTIMESERIES PLotting a time series composed of a state of a
%filtered system. Requires the full state definition and the covariance.
%Can be passed on or several state indices.
    
figure();
numSubFig = length(idx);%size(x,2);
if(isempty(subFig))
    subFig = [numSubFig,1];
    
end

for i = 1:numSubFig
%     subplot(subFig(1),subFig(2),i);
if(strcmp(plotText.ylabelText,'E(f) N') == 1 || strcmp(plotText.ylabelText,'E(\mu) Nm') == 1 )
    shadedErrorBar(t,x(:,idx(i)),squeeze(2*sqrt(P(idx(i),idx(i),:)))',cols{i}, 1);
else
    plot(t,x(:,idx(i)),cols{i},'linewidth',1);    
end
    hold on
%     plot(t,sysX(:,idx(i)), '-- m','linewidth',1)
    if(~isempty(y))
        hold on
        plot(t, y(:,i), '--k','linewidth',1);
    end
    
%     xlabel(plotText.xlabelText{i});
%     ylabel(plotText.ylabelText{i});
%     title(plotText.titleText{i});
%     set(gca,'FontSize',12);
%     a = axis();del_a = a(3)-a(4);
%     
% %     axis tight;
% %     axis equal;
%     
%     axis([t(1) t(end) a(3)+0.25*del_a a(4)-0.25*del_a]);
%     
xlim([t(1) t(end)]);
    
%     g = get(gca);set(gca,'yTick',linspace(g.YTick(1),g.YTick(end),3));    
end
xlabel(plotText.xlabelText);
    ylabel(plotText.ylabelText);
    title(plotText.titleText);
    
    legend('x','y','z');


end

