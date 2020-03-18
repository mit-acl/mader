clear all ; close all ; clc;

t0 = 0;
tfmin = pi; tfmax = pi;
x0min = 0;  y0min = 0;
x0max = 0;  y0max = 0;

xmin = 0; xmax = 2.5;
ymin = 0; ymax = 1.5;

xfmin = 1.7;  yfmin = 0;
xfmax = 2.5;  yfmax = 0;

umin = -pi/2; umax = pi/2;

bounds.phase.initialtime.lower = 0;
bounds.phase.initialtime.upper = 0;
bounds.phase.finaltime.lower = 1;
bounds.phase.finaltime.upper = 1;
bounds.phase.initialstate.lower = [0, 0, 0, 0];
bounds.phase.initialstate.upper = [1, 1, 1, 1];
bounds.phase.state.lower = [0, 0, 0, 0];
bounds.phase.state.upper = [1, 1, 1, 1];
bounds.parameter.lower = -1000*ones(1,12);
bounds.parameter.upper = 1000*ones(1,12);
bounds.phase.finalstate.lower = [0, 0, 0, 0];
bounds.phase.finalstate.upper = [1, 1, 1, 1];
bounds.phase.control.lower = [-1000, -1000, -1000, -1000];
bounds.phase.control.upper = [1000, 1000, 1000, 1000];

bounds.phase.path.lower = [0,0,0,0];
bounds.phase.path.upper = [0,0,0,0];
% bounds.phase.integral.lower = intmin;
% bounds.phase.integral.upper = intmax;

%Provide Guess of Solution 
guess.phase.time = [ 0 ; pi ];
guess.phase.state = [[ 0 ; 1],[ 0 ; 1] ,[ 0 ; 1] ,[ 0 ; 1] ];
guess.phase.control = [1,1,1,1 ; 1 1 1 1];
guess.parameter = [ones(1,12) ];
% guess.phase.integral = pi/2;

%Provide Mesh Refinement Method and Initial Mesh 
mesh.method = 'hp-PattersonRao';
mesh.tolerance = 1e-6;
mesh.maxiteration = 4;
mesh.colpointsmin = 4;
mesh.colpointsmax = 15;

% Assemble Information into Problem Structure

setup.name = 'Brachistochrone - Problem';
setup.functions.continuous = @curveContinuous ;
setup.functions.endpoint = @curveEndpoint ;
setup.bounds = bounds ;
setup.guess = guess ;
setup.mesh = mesh ;
setup.nlp.solver = 'ipopt';
setup.derivatives.supplier ='sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.method = 'RPM-Differentiation';

% Solve Problem Using GPOPS2


output = gpops2 ( setup );
%%
solution = output.result.solution ;
figure
plot (solution.phase.state(:,1),solution.phase.state(:,2),'b--o')
xlabel('x')
ylabel('y')
axis equal
%printeps(1,'OptimalCurve');
figure
plot (solution.phase.time,solution.phase.state(:,1),'--o')
hold on
plot (solution.phase.time,solution.phase.state(:,2),'--o')
plot (solution.phase.time,solution.phase.state(:,3),'--o')
plot (solution.phase.time,solution.phase.state(:,3),'--o')
plot (solution.phase.time,solution.phase.state(:,4),'--o')
% plot (solution.phase.time,solution.phase.control(:,1),'r--o')
legend('\lambda1','\lambda2','\lambda3','\lambda4')
xlabel('t')
%printeps(2,'OptimalControl');