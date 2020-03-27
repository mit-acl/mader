sol2=load('solutionDeg2.mat');
sol3=load('solutionDeg3.mat');
sol4=load('solutionDeg4.mat');
sol5=load('solutionDeg5.mat');


scatter(2*ones(size(sol2.rootsA(:))), sol2.rootsA(:)); hold on
scatter(3*ones(size(sol3.rootsA(:))), sol3.rootsA(:))
scatter(4*ones(size(sol4.rootsA(:))), sol4.rootsA(:))
scatter(5*ones(size(sol5.rootsA(:))), sol5.rootsA(:))