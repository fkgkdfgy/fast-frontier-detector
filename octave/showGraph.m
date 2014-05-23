dataFFD = load('-ascii', 'partial_ffd_executions.txt');
times = dataFFD(:,4);
dataFFD = dataFFD(:,2);

dataRafael = load('-ascii', 'exploration_execution_rafael.txt');

dataWolfram = load('-ascii', 'exploration_execution_wolfram.txt');
%dataWolfram = dataWolfram(:,1) * 1000000 + dataWolfram(:,2);

% HACK
limit = length(dataWolfram);
dataFFD = dataFFD(1:limit);
dataRafael = dataRafael(1:limit);
times = times(1:limit);

data = [dataFFD dataRafael dataWolfram];



% plot FFD vs. Rafael vs. Wolfram
bar(times, data);
legend('FFD', 'Rafael', 'Wolfram')
title('FFD vs. Rafael vs. Wolfram', 'fontsize', 15)
xlabel('frames')
ylabel('logscale time (microseconds)')
grid on
set(gca,'yscale','log')
print('-djpg', 'graph_FFD_vs_All.JPG');
print('-depsc2', 'graph_FFD_vs_All.eps');

% plot FFD
figure
plot(times, dataFFD);
legend('FFD')
title('FFD Executions', 'fontsize', 15)
xlabel('frames')
ylabel('time (microseconds)')
grid on
print('-djpg', 'graph_FFD.JPG');
print('-depsc2', 'graph_FFD.eps');


% plot Rafael
figure
plot(times, dataRafael);
legend('Rafael')
title('Rafael Executions', 'fontsize', 15)
xlabel('frames')
ylabel('time (microseconds)')
grid on
print('-djpg', 'graph_Rafael.JPG');
print('-depsc2', 'graph_Rafael.eps');

% plot Wolfram
figure
plot(times, dataWolfram);
legend('Rafael')
title('Wolfram Executions', 'fontsize', 15)
xlabel('frames')
ylabel('time (microseconds)')
grid on
print('-djpg', 'graph_Wolfram.JPG');
print('-depsc2', 'graph_Wolfram.eps');
