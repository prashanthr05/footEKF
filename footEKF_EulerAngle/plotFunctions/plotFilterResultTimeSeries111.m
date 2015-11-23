function [ output_args ] = plotFilterResultTimeSeries( t, x, y, P,idx, plotText,cols,subFig)%, sysX )
%PLOTFILTERRESULTTIMESERIES PLotting a time series composed of a state of a
%filtered system. Requires the full state definition and the covariance.
%Can be passed on or several state indices.
    
figure();
numSubFig = length(idx);%size(x,2);
if(isempty(subFig))
    subFig = [numSubFig,1];
    
end

% for i = 1:numSubFig
%     subplot(subFig(1),subFig(2),i);
%     h = animatedline('Color',cols{i},'LineWidth',2);
% %     shadedErrorBar(t,x(:,idx(i)),squeeze(2*sqrt(P(idx(i),idx(i),:)))',cols{i}, 1);
% %     plot(t,x(:,idx(i)),cols{i});    
% %     hold on
% %     plot(t,sysX(:,idx(i)), '-- m','linewidth',1)
%     if(~isempty(y))
%         hold on
%         if(idx(i) == 19 || idx(i) == 20 || idx(i) == 21)
%         plot(t, y(:,i), '--m','linewidth',1);
%         hold on;
%         else
%         plot(t, y(:,idx(i)), '--m','linewidth',1);
%         hold on;
%         end
%     end
%     a = axis();del_a = a(3)-a(4);
%     xlim([t(1) t(end)]);
% %     axis([t(1) t(end) a(3)+0.1*del_a a(4)-0.1*del_a]);
% % axis([t(1) t(end) a(3)-10 a(4)+10]);
%     set(gca,'YTickMode','auto');
%     for k = 1:length(t)
%         addpoints(h,t(k),x(k,idx(i)));
%         drawnow% limitrate
%     end
% %     drawnow
% 
%     title(plotText.titleText{i},'FontSize',14,'FontName','Times');
%     xlabel(plotText.xlabelText{i},'FontSize',14,'FontName','Times');
%     ylabel(plotText.ylabelText{i},'FontSize',14,'FontName','Times');
%     set(gca,'FontSize',12);
% %     g = get(gca);set(gca,'yTick',linspace(g.YTick(1),g.YTick(end),3));    
% end
    
% 
% subplot(3,1,1);
% p1 = plot(t,x(:,idx(1)),cols{1},'linewidth',2);
% axis([t(1) t(end) -10 10]);
% hold on
% 
% subplot(3,1,2);
% p2 = plot(t,x(:,idx(2)),cols{2},'linewidth',2);
% axis([t(1) t(end) 80 100]);
% hold on
% 
% subplot(3,1,3);
% p3 = plot(t,x(:,idx(3)),cols{3},'linewidth',2);
% axis([t(1) t(end) -10 10]);
% hold on
% 
% x1 = get(p1,'xdata');x2 = get(p2,'xdata');x3 = get(p3,'xdata');
% y1 = get(p1,'ydata');y2 = get(p2,'ydata');y3 = get(p3,'ydata');
% 
% % plot.new();
% 
% for k = 1:length(x1)
%     subplot(3,1,1);set(p1,'xdata',x1(k),'ydata',y1(k)); hold on 
%     subplot(3,1,2);set(p2,'xdata',x2(k),'ydata',y2(k)); hold on 
%     subplot(3,1,3);set(p3,'xdata',x3(k),'ydata',y3(k)); hold on
% %     set([p1; p2; p3],{'xdata'},{x1(k);x2(k);x3(k)},{'ydata'},{y1(k);y2(k);y3(k)}); 
%     drawnow;
%     pause(1/20);
% end


% tic;
for k = 1:length(t)
%     subplot(3,1,1)
%     plot(t(k),x(k,idx(1)),'.','Color','r')%// Choose your own marker here
%     axis([t(1) t(end) -10 10]);
%     hold on
    
%     subplot(3,1,2)
    plot(t(k),x(k,idx(2)),'.','Color','g');
    axis([t(1) t(end) 70 100]);
    hold on

    
%     subplot(3,1,3)
%     plot(t(k),x(k,idx(3)),'.','Color','b');
%     axis([t(1) t(end) -10 10]);
%     hold on


%     
%     %// MATLAB pauses for 0.001 sec before moving on to execue the next 
%     %%// instruction and thus creating animation effect
%     pause(10e-20);     
drawnow;

      frame = getframe(1);
      im = frame2im(frame);
      [imind,cm] = rgb2ind(im,256);
      if k == 1;
          imwrite(imind,cm,'bwdtiporien.gif','gif', 'Loopcount',inf);
      else
          imwrite(imind,cm,'bwdtiporien.gif','gif','WriteMode','append');
      end


end
% display(toc());
% 


end
