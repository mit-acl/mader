clear all ; close all ; clc;

global deg_pol  dim coeff_pol

deg_pol=1;
dim=deg_pol; %The control points are in R^dim
%Note that only the first $(echo deg_pol) numbers of coeff_pol_ will be
%used
%Each column is one polynomial. i.e. col_1=[a b c d ...]'
coeff_pol=[ 0.5   0.6   0.3  0.1  -0.3  0.4  1.0 
           0.2   -0.3  -0.1  0.7  -0.7  5.0  -0.3
           0.3     3    -1   -3   +0.4  1.2  -1.5
            2     -5    -4    2   -0.1 0.2    1.0
            1      6    -4   -3   +0.6  0.3  2.5
            -3     6   -0.5  0.2   9    1.1  0.9
            -1.5   0.1   0.33 0.45  -0.48  -1.4 1.3
            2.5  -1.8    3.1    -2.5  3.5  -0.2  0.4
];


if(deg_pol==3)
  
    coeff_pol_x=[0.2 0.3 2 1]';%[a b c d]
    coeff_pol_y=[-0.3 +3 -5 6]';%[a b c d]
    coeff_pol_z=[1 -0.1 -1 -4]';%[a b c d]
    coeff_pol=[coeff_pol_x coeff_pol_y coeff_pol_z];

end

num_of_states=deg_pol+1;
num_of_params=dim*num_of_states;

deg_is_even =~mod(deg_pol,2)
if (deg_is_even)
    tmp1= 0;
    tmp2= 1;
else
    tmp1= [];
    tmp2= [];
end

bounds.phase.initialtime.lower = 0;
bounds.phase.initialtime.upper = 0;
bounds.phase.finaltime.lower = 1;
bounds.phase.finaltime.upper = 1;
bounds.phase.initialstate.lower = [zeros(1,num_of_states/2) tmp1  zeros(1,num_of_states/2)];
bounds.phase.initialstate.upper = [zeros(1,num_of_states/2) tmp2  ones(1,num_of_states/2)];
bounds.phase.state.lower = zeros(1,num_of_states);
bounds.phase.state.upper = ones(1,num_of_states);
bounds.parameter.lower = -20*ones(1,num_of_params);
bounds.parameter.upper = 20*ones(1,num_of_params);
bounds.phase.finalstate.lower = [zeros(1,num_of_states/2) tmp1 zeros(1,num_of_states/2)];
bounds.phase.finalstate.upper = [ones(1,num_of_states/2) tmp2 zeros(1,num_of_states/2)];
bounds.phase.control.lower = -2000*ones(1,num_of_states);%[-1000, -1000, -1000, -1000];
bounds.phase.control.upper = 2000*ones(1,num_of_states);%[1000, 1000, 1000, 1000];

bounds.phase.path.lower = zeros(1,dim+1);%[0,0,0,0];
bounds.phase.path.upper = zeros(1,dim+1);%[0,0,0,0];
% bounds.phase.integral.lower = intmin;
% bounds.phase.integral.upper = intmax;

%Provide Guess of Solution 
tmp=[0;1];
guess.phase.time = [ 0 ; 1 ];
guess.phase.state = rand(2, num_of_states)%repmat(tmp,1,num_of_states);
guess.phase.control = rand(2,num_of_states);
guess.parameter = rand(1,num_of_params);
% guess.phase.integral = pi/2;

%Provide Mesh Refinement Method and Initial Mesh 
mesh.method = 'hp-LiuRao-Legendre' %  'hp-PattersonRao';
mesh.tolerance = 1e-8;
mesh.maxiterations = 150;
mesh.colpointsmin = 30;  %30
mesh.colpointsmax = 35;

num_intervals=10;            %10
col_points_per_interval=20;  %20

mesh.phase.fraction = (1/num_intervals)*ones(1,num_intervals);
mesh.phase.colpoints = col_points_per_interval*ones(1,num_intervals);

setup.nlp.ipoptoptions.tolerance=1e-7; %Default is 1e-7

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

%%
figure
%Now fit a polynomial
A=[];
for i=1:size( solution.phase.state,2)
    A=[A;polyfit(solution.phase.time(1:end-1), solution.phase.state(1:end-1,i),deg_pol)]
end

syms t;
T=[];
for i=flip(0:deg_pol)
    T=[T ;t^(i)];
end
% T=[t*t*t*t*t t*t*t*t t*t*t t*t t 1]';
fplot(A*T,[0,1])

rootsA=[];
for i=1:size(A,1)
    rootsA=[rootsA ; roots(A(i,:))'];
end
rootsA=real(rootsA);


save(['solutionDeg' num2str(deg_pol) '.mat'],'A','rootsA')

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