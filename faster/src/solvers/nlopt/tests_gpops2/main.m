clear all ; close all ; clc;

syms t

global deg_pol  dim coeff_pol t

deg_pol=3;
dim=deg_pol; %The control points are in R^dim

num_elem_B=(deg_pol+1)/2;
num_elem_R= ((deg_pol+1)/2)*((deg_pol-1)/2); 

num_of_params= num_elem_B + num_elem_R;

num_of_states=1; %dummy state (all the opt variables are parameters)

bounds.phase.initialtime.lower = 0;
bounds.phase.initialtime.upper = 0;
bounds.phase.finaltime.lower = 1;
bounds.phase.finaltime.upper = 1;
bounds.phase.initialstate.lower = zeros(1,num_of_states);
bounds.phase.initialstate.upper = ones(1,num_of_states);
bounds.phase.state.lower = zeros(1,num_of_states);
bounds.phase.state.upper = ones(1,num_of_states);
bounds.parameter.lower = [zeros(1,num_elem_B)  -1000*ones(1,num_elem_R)];  %The elements of B are >=0
bounds.parameter.upper = 1000*ones(1,num_of_params);
bounds.phase.finalstate.lower = zeros(1,num_of_states);
bounds.phase.finalstate.upper = ones(1,num_of_states);
bounds.phase.control.lower = -1000*ones(1,num_of_states);%[-1000, -1000, -1000, -1000];
bounds.phase.control.upper = 1000*ones(1,num_of_states);%[1000, 1000, 1000, 1000];

% bounds.phase.path.lower = zeros(1,dim+1);%[0,0,0,0];
% bounds.phase.path.upper = zeros(1,dim+1);%[0,0,0,0];
% bounds.phase.integral.lower = intmin;
% bounds.phase.integral.upper = intmax;

%Provide Guess of Solution 
tmp=[0;1];
guess.phase.time = [ 0 ; 1 ];
guess.phase.state = repmat(tmp,1,num_of_states);
guess.phase.control = ones(2,num_of_states);
guess.parameter = [ones(1,num_elem_B)  rand(1,num_elem_R)];
% guess.phase.integral = pi/2;

%Provide Mesh Refinement Method and Initial Mesh 
mesh.method = 'hp-PattersonRao';
mesh.tolerance = 1e-6;
mesh.maxiteration = 10;
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
setup.derivatives.supplier ='sparseCD';  %adigator
setup.derivatives.derivativelevel = 'second';
setup.method = 'RPM-Differentiation'; %'Integration';
% setup.scales.method = 'automatic-bounds';

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
plot (solution.phase.time,solution.phase.state,'--o')
% plot (solution.phase.time,solution.phase.state(:,1),'--o')
% hold on
% plot (solution.phase.time,solution.phase.state(:,2),'--o')
% plot (solution.phase.time,solution.phase.state(:,3),'--o')
% plot (solution.phase.time,solution.phase.state(:,3),'--o')
% plot (solution.phase.time,solution.phase.state(:,4),'--o')
% plot (solution.phase.time,solution.phase.control(:,1),'r--o')
% legend('\lambda1','\lambda2','\lambda3','\lambda4')
xlabel('t')
%printeps(2,'OptimalControl');