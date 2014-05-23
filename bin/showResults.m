% get directory names
maps = dir;
maps = maps(3:end);
xlabels = [];

currIndex = 0;
for i = 1:length(maps)
	
	if maps(i).isdir == 0
		continue;	
	end

	currMap = maps(i).name;
	currIndex = currIndex + 1;
	xlabels = [xlabels; currMap];

	dataFFD = load('-ascii', [currMap '/executions/partial_ffd_executions.txt']);
	times = dataFFD(:,2);
	dataFFD = dataFFD(:,1);

	dataRafael = load('-ascii', [currMap '/executions/exploration_execution_rafael.txt']);

	dataWolfram = load('-ascii', [currMap '/executions/exploration_execution_wolfram.txt']);

	% HACK
	limit = length(dataWolfram);
	dataFFD = dataFFD(1:limit);
	dataRafael = dataRafael(1:limit);
	
	% get stats
	means(currIndex,:) = [mean(dataFFD), mean(dataRafael), mean(dataWolfram)];
	stds(currIndex,:) = [std(dataFFD), std(dataRafael), std(dataWolfram)];
end

%means(2,:) = means(1,:);

bar(means)
legend('FFD', 'Rafael', 'Wolfram')
set(gca,'yscale','log')
title('Running Time in Different Environments', 'fontsize', 15)
xlabel('frames')
ylabel('logscale time (microseconds)' )
grid on

% change x axis labels
x = [1:length(means)];
set(gca, 'XTick', x, 'XTickLabel', xlabels);

% add error bars
hold on
errorb(means, stds);

% output result to files
print('-djpg', 'graph_Results.JPG');
print('-depsc2', 'graph_Results.eps');
