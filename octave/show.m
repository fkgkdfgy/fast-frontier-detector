prefix = "contour";
files = dir([prefix '*.txt']);
colors = [1 2 4 5 6]; % red green blue magenta cyan brown
indecis = [];

% get unique contour indecis
%files = dir(['frontier' '*.txt']);
%for i=1:length(files)
%	currFile = files(i);
%	indexUnderscore = findstr(currFile.name, '_')(2);
%	indexPoint = findstr(currFile.name, '.');
%	contourNum = currFile.name(indexUnderscore+1 : indexPoint-1);
%	indecis(i) = str2num(contourNum);
%end
%indecis
%return

for i=1:length(files)
		
	currFile = files(i);
	
	% get number of contours
	indexUnderscore = findstr(currFile.name, '_');
	indexPoint = findstr(currFile.name, '.');
	contourNum = currFile.name(indexUnderscore+1 : indexPoint-1);
	
	% draw base contour 
	currMat = load('-ascii', currFile.name);
	plot(currMat(:,1), currMat(:,2),'*');
	imageName = [currFile.name(1:length(currFile.name)-3), 'png'];
	print('-dpng', imageName);

	% get frontier files of current contour

	hold on	
	
	currFrontiers = dir(['frontier_' contourNum '_*.txt']);
	
	% draw each frontier in current contour
	for j=1:length(currFrontiers)
		currMat = load('-ascii', currFrontiers(j).name);
		plot(currMat(:,1), currMat(:,2),'1*');
		imageName = ['frontiers_' contourNum '.png'];
		print('-dpng', imageName);
	end
	
	hold off
end 
