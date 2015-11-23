function [ output_args ] = plotFilter( t, x, y, P,idx, plotText,cols,subFig)%, sysX )
%PLOTFILTERRESULTTIMESERIES PLotting a time series composed of a state of a
%filtered system. Requires the full state definition and the covariance.
%Can be passed on or several state indices.
    
figure();
% numSubFig = subFig;%size(x,2);
noPlots = length(idx);

% for j = 1:numSubFig
%    subplot(1,j);
   for i = 1 : noPlots
%     shadedErrorBar(t,x(:,idx(i)),squeeze(2*sqrt(P(idx(i),idx(i),:)))',cols{i}, 1);
    plot(t,x(:,idx(i)),cols{i},'linewidth',1);    
    hold on
%     plot(t,sysX(:,idx(i)), '-- m','linewidth',1)
    if(~isempty(y))
        hold on
        plot(t, y(:,i), '--k','linewidth',1);
    end
%    end
    title(plotText.titleText);
    xlabel(plotText.xlabelText);
    ylabel(plotText.ylabelText);
    set(gca,'FontSize',12);
    a = axis();del_a = a(3)-a(4);
    
%     axis tight;
%     axis equal;
    
    axis([t(1) t(end) a(3)+0.25*del_a a(4)-0.25*del_a]);
    
    g = get(gca);set(gca,'yTick',linspace(g.YTick(1),g.YTick(end),3));    
end
    
    


end

