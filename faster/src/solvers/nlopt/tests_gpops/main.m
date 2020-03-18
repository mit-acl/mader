clear all ; close all ; clc;

global deg_pol  dim coeff_pol

deg_pol=5;
dim=deg_pol; %The control points are in R^dim
%Note that only the first $(echo deg_pol) numbers of coeff_pol_ will be
%used
%Each column is one polynomial. i.e. col_1=[a b c d ...]'
coeff_pol=[ 0.5   0.6   0.3  0.1  -0.3
           0.2   -0.3  -0.1  0.7  -0.7
           0.3     3    -1   -3   +0.4
            2     -5    -4    2   -0.1
            1      6    -4   -3   +0.6
            -3     6   -0.5  0.2   9
];
% coeff_pol_x=[0.5 0.2 0.3 2 1]';%[a b c d]
% coeff_pol_y=[0.6 -0.3 +3 -5 6]';%[a b c d]
% coeff_pol_z=[0.3 1 -0.1 -1 -4]';%[a b c d]
% coeff_pol_xx=[0.3 1 -0.1 -1 -4]';%[a b c d]
% 
% coeff_pol=[coeff_pol_x coeff_pol_y coeff_pol_z coeff_pol_xx];

num_of_states=deg_pol+1;
num_of_params=dim*num_of_states;

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
bounds.phase.initialstate.lower = zeros(1,num_of_states);
bounds.phase.initialstate.upper = ones(1,num_of_states);
bounds.phase.state.lower = zeros(1,num_of_states);
bounds.phase.state.upper = ones(1,num_of_states);
bounds.parameter.lower = -1000*ones(1,num_of_params);
bounds.parameter.upper = 1000*ones(1,num_of_params);
bounds.phase.finalstate.lower = zeros(1,num_of_states);
bounds.phase.finalstate.upper = ones(1,num_of_states);
bounds.phase.control.lower = -1000*ones(1,num_of_states);%[-1000, -1000, -1000, -1000];
bounds.phase.control.upper = 1000*ones(1,num_of_states);%[1000, 1000, 1000, 1000];

bounds.phase.path.lower = zeros(1,dim+1);%[0,0,0,0];
bounds.phase.path.upper = zeros(1,dim+1);%[0,0,0,0];
% bounds.phase.integral.lower = intmin;
% bounds.phase.integral.upper = intmax;

%Provide Guess of Solution 
tmp=[0;1];
guess.phase.time = [ 0 ; pi ];
guess.phase.state = repmat(tmp,1,num_of_states);
guess.phase.control = ones(2,num_of_states);
guess.parameter = ones(1,num_of_params);
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