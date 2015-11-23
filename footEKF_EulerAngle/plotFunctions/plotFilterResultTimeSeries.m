function [ output_args ] = plotFilterResultTimeSeries( t, x, y, P,idx, plotText,cols,subFig)%, sysX )
%PLOTFILTERRESULTTIMESERIES PLotting a time series composed of a state of a
%filtered system. Requires the full state definition and the covariance.
%Can be passed on or several state indices.
    
figure();
numSubFig = length(idx);%size(x,2);
if(isempty(subFig))
    subFig = [numSubFig,1];
    
end

for i = 1:numSubFig
    subplot(subFig(1),subFig(2),i);
    shadedErrorBar(t,x(:,idx(i)),squeeze(2*sqrt(P(idx(i),idx(i),:)))',cols{i}, 1);
%     plot(t,x(:,idx(i)),cols{i});    
    hold on
%     plot(t,sysX(:,idx(i)), '-- m','linewidth',1)
    if(~isempty(y))
        hold on
        if(idx(i) == 19 || idx(i) == 20 || idx(i) == 21)
        plot(t, y(:,i), '--m','linewidth',1);
        else
        plot(t, y(:,idx(i)), '--m','linewidth',1);
        end
    end
    title(plotText.titleText{i},'FontSize',14,'FontName','Times');
    xlabel(plotText.xlabelText{i},'FontSize',14,'FontName','Times');
    ylabel(plotText.ylabelText{i},'FontSize',14,'FontName','Times');
    set(gca,'FontSize',12);
    a = axis();del_a = a(3)-a(4);
    
    axis([t(1) t(end) a(3)+0.1*del_a a(4)-0.1*del_a]);
    set(gca,'YTickMode','auto');
%     g = get(gca);set(gca,'yTick',linspace(g.YTick(1),g.YTick(end),3));    
end
    
    


end
