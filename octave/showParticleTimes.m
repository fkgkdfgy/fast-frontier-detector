% get directory names
maps = dir;
maps = maps(3:end);
xlabels = [];


for i = 1:length(maps)
	if maps(i).isdir == 0
		continue;	
	end

	currMap = maps(i).name;

	dataFFD = load('-ascii', [currMap '/executions/partial_ffd_executions.txt']);
	times = dataFFD(:,5:end);


	% get stats
	for i=1:columns(times)
		temp = times(:,i);
		means(i) = mean(temp(find(temp)));
		stds(i) = std(temp(find(temp)));	
	end

	means
	figure
	h = bar(means)

	grid on

	hold on
	h1 = errorbar(means, stds, "#");

	%axis([0 1 50 650], "autox") %axis([0,30,0,700])
	xlim([0 31])
	ylim([0 max(means)+max(stds)+20])
	set(h(1),'facecolor','cyan') % or use RGB triple

	xlabel('particles')
	ylabel('average run-time (microseconds)' )

	% output result to files
	print('-djpg',  [currMap '/executions/graph_particle_times.JPG']);
	print('-depsc2', [currMap '/executions/graph_particle_times.eps']);
	hold off

	%mapName = strsplit(currMap,'.')
	dotPlace = findstr(currMap, ".")
	dotPlace = dotPlace(1)
	mapName = currMap(1:dotPlace-1)
	%copyfile ([currMap '/executions/graph_particle_times.JPG'], ['~/workspace/FullFastFrontierDetector/images/graph_particle_times_' mapName '.JPG'])
	%copyfile ([currMap '/executions/graph_particle_times.eps'], ['~/workspace/FullFastFrontierDetector/images/graph_particle_times_' mapName '.eps'])
end

return
%means = mean(times);
%stds  = std(times);


%legend('FFD', 'WFD', 'State of the Art')
%set(gca,'yscale','log')
%title('Running Time in Different Environments', 'fontsize', 15)
%xlabel('environments')
%ylabel('logscale time (microseconds)' )


% change x axis labels
%xlabels
%xlabels = {'(A)', '(B)', '(C)', '(D)', '(E)'};
%x = [1:length(means)];
%set(gca, 'XTick', x, 'XTickLabel', xlabels);

% add error bars
hold on
h1 = errorbar(means, stds, "#");
%set(h1,"color","red")
%set(h1,"marker","+")
%axis([0 1 50 650], "autox") %axis([0,30,0,700])
%set(h1,"linestyle","none")
%errorb(means, stds);

#set(h(1),'facecolor','white') % use color name
set(h(1),'facecolor','cyan') % or use RGB triple
%set(h(3),'facecolor','red') % or use a color defined in the help for 

% output result to files
print('-djpg', 'graph_particle_times.JPG');
print('-depsc2', 'graph_particle_times.eps');
