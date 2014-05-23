function[] = drawAll(num1, num2)
clearplot

colors = [1 2 4 5 6]
colors = [1 1 1 1 1]

% draw contour
fileName = ['/home/matan/workspace/webots/controllers/RV400_controller/contour_' int2str(num1) '.txt'];
A = load('-ascii', fileName);
plot(A(:,1), A(:,2), '+');

hold on

for i=0:(num2-1)
	fileName = ['/home/matan/workspace/webots/controllers/RV400_controller/contour_' int2str(num1) '_' int2str(i) '.txt'];
	A = load('-ascii', fileName);
	plot(A(:,1), A(:,2), [int2str(colors(1 + mod(i,5))) '+']);
end

hold off

%axis([-10, 10, -3000, 3000]);
end

