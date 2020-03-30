clear all ; close all ; clc;

syms t

global deg_pol  dim coeff_pol t

deg_pol=3;
dim=deg_pol; %The control points are in R^dim

num_elem_B=(deg_pol+1)/2;
num_elem_R= ((deg_pol+1)/2)*((deg_pol-1)/2); 

num_of_params= num_elem_B + num_elem_R;


global num_of_states

num_of_states=num_elem_B + num_elem_R; %dummy state (all the opt variables are parameters)


bounds.phase.initialtime.lower = -1;
bounds.phase.initialtime.upper = -1;
bounds.phase.finaltime.lower = 1;
bounds.phase.finaltime.upper = 1;

bounds.phase.initialstate.lower = -1000*ones(1,num_of_states);
bounds.phase.initialstate.upper = 1000*ones(1,num_of_states);
bounds.phase.state.lower = -1000*ones(1,num_of_states);
bounds.phase.state.upper = 1000*ones(1,num_of_states);
bounds.phase.finalstate.lower = -1000*ones(1,num_of_states);
bounds.phase.finalstate.upper = 1000*ones(1,num_of_states);

% bounds.parameter.lower = [zeros(1,num_elem_B)  -30*ones(1,num_elem_R)];  %The elements of B are >=0
% bounds.parameter.upper = 30*ones(1,num_of_params);

bounds.phase.control.lower = ones(1,num_of_states);%[-1000, -1000, -1000, -1000];
bounds.phase.control.upper = ones(1,num_of_states);%[1000, 1000, 1000, 1000];

bounds.phase.path.lower = zeros(1,dim+1);%[0,0,0,0];
bounds.phase.path.upper = zeros(1,dim+1);%[0,0,0,0];

% bounds.phase.integral.lower = intmin;
% bounds.phase.integral.upper = intmax;

%Provide Guess of Solution 
tmp=[0;1];
guess.phase.time = [ -1 ; 1 ];
tmp=rand(1,num_of_states);
guess.phase.state = [tmp; tmp]; % repmat(tmp,1,num_of_states);
guess.phase.control = rand(2,num_of_states);
% guess.parameter = [ones(1,num_elem_B)  rand(1,num_elem_R)];
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
setup.derivatives.derivativelevel = 'first';
setup.method = 'RPM-Differentiation'; %'Integration';
% setup.scales.method = 'automatic-bounds';

% Solve Problem Using GPOPS2

output = gpops2 ( setup );
%%



solution = output.result.solution ;
 

params=double(solution.parameter);

[B,R]= generateBR(params);
A=getA(B,R);

syms t;
T=[];
for i=flip(0:deg_pol)
    T=[T ;t^(i)];
end
% T=[t*t*t*t*t t*t*t*t t*t*t t*t t 1]';
fplot(A*T,[0,1])

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