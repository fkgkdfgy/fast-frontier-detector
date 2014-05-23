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
	times = dataFFD(:,4);
	dataFFD = dataFFD(:,2);

	dataRafael = load('-ascii', [currMap '/executions/exploration_execution_rafael.txt']);

	dataWolfram = load('-ascii', [currMap '/executions/exploration_execution_wolfram.txt']);
	%dataWolfram = dataWolfram(:,1) * 1000000 + dataWolfram(:,2);

	% HACK
	limit = min( [length(dataWolfram), length(dataFFD), length(dataRafael) ]);
	dataFFD = dataFFD(1:limit);
	dataRafael = dataRafael(1:limit);
	
	% Fit Data Rafael (WFD) to real scale
	dataRafael = 64*dataRafael;

	% get stats
	means(currIndex,:) = [mean(dataFFD), mean(dataRafael), mean(dataWolfram)];
	stds(currIndex,:) = [std(dataFFD), std(dataRafael), std(dataWolfram)];
end

%means(2,:) = means(1,:);

h = bar(means)

legend('FFD', 'WFD', 'State of the Art')
set(gca,'yscale','log')
%title('Running Time in Different Environments', 'fontsize', 15)
xlabel('environments')
ylabel('logscale time (microseconds)' )
grid on

% change x axis labels
xlabels
xlabels = {'(A)', '(B)', '(C)', '(D)', '(E)'};
x = [1:length(means)];
set(gca, 'XTick', x, 'XTickLabel', xlabels);

% add error bars
hold on
%errorb(means, stds);

set(h(1),'facecolor','white') % use color name
set(h(2),'facecolor','cyan') % or use RGB triple
set(h(3),'facecolor','red') % or use a color defined in the help for 

% output result to files
print('-djpg', 'graph_Results.JPG');
print('-depsc2', 'graph_Results.eps');
