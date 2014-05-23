% get directory names
maps = dir;
maps = maps(3:end);
xlabels = [];

allLengths = [];
ctrMaps = 0;
mapNames = {};

for i = 1:length(maps)
	if maps(i).isdir == 0
		continue;	
	end

	currMap = maps(i).name;

	ctrMaps = ctrMaps + 1;
	mapNames(ctrMaps) = currMap;

	contours = load('-ascii', [currMap '/contours/contour_lengths.txt']);
	lengths = contours(:,1);
	times = contours(:,2);

	%fit different sizes from different environments
	maxSize = max(length(allLengths), length(lengths));

	if length(allLengths) < maxSize
		[r,c] = size(allLengths);
		allLengths = [allLengths ; zeros( maxSize - r, c)];
		allLengths = [allLengths lengths];
	else
		tempLengths = [lengths; zeros(maxSize - length(lengths),1)];
		allLengths = [allLengths tempLengths];
	end
	

	contours = sortrows(contours);
	%figure
	h = scatter(contours(:,1), contours(:,2));
	corr = corrcoef(contours(:,1), contours(:,2)); % calculate pearson
	title(['pearson = ' str(corr)]);  

	grid on

	%axis([0 1 50 650], "autox") %axis([0,30,0,700])
	xlim([0 200000])
	ylim([0 150000])
	#set(h(1),'facecolor','cyan') % or use RGB triple

	xlabel('contour lengths')
	ylabel('run-time (microseconds)' )

	% output result to files
	print('-djpg',  [currMap '/executions/graph_contour_lengths.JPG']);
	print('-depsc2', [currMap '/executions/graph_contour_lengths.eps']);
	

	% plot histogram of contour lengths
	figure;
	hist(lengths, 100);
	xlim([0 200000])
	ylim([0 10])
	
	
	% output result to files
	print('-djpg',  [currMap '/executions/graph_contour_lengths_hist.JPG']);
	print('-depsc2', [currMap '/executions/graph_contour_lengths_hist.eps']);	
	
	%
	%dotPlace = findstr(currMap, ".")
	%dotPlace = dotPlace(1)
	%mapName = currMap(1:dotPlace-1)
	%copyfile ([currMap '/executions/graph_particle_times.JPG'], ['~/workspace/FullFastFrontierDetector/images/graph_particle_times_' mapName '.JPG'])

	%copyfile ([currMap '/executions/graph_particle_times.eps'], ['~/workspace/FullFastFrontierDetector/images/graph_particle_times_' mapName '.eps'])
end

% plot histogram of all contour lengths

if (1 == 0) % will NOT be executed
figure;
for i=1:ctrMaps
	currColumn = allLengths(:,i);
	currIndecis = find(currColumn);
	currColumn = currColumn(currIndecis);
	hist(currColumn);
	hold on
	xlim([1 200000]);

end
hold off;
end


figure;
hist(allLengths);
mapNames;
legend(mapNames);
print('-djpg',  ['graph_all_contour_lengths.JPG']);
print('-depsc2', ['graph_all_contour_lengths.eps']);


