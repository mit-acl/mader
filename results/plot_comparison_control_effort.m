close all; clear; clc;
set(0,'defaulttextInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');


min_x=1;
max_x=50;

%% B-Spline 
bag = rosbag('bspline.bag');
goal_topic = readMessages(select(bag, 'Topic', '/SQ01s/goal'));
jerk_bspline=[];
for i=1:size(goal_topic,1)
    if(goal_topic{i}.Pos.X>min_x && goal_topic{i}.Pos.X<max_x)
        jerk=[goal_topic{i}.Jerk.X; goal_topic{i}.Jerk.Y; goal_topic{i}.Jerk.Z];
        jerk_bspline=[jerk_bspline; norm(jerk)];
    end
end

%% Bezier
bag = rosbag('bezier.bag');
goal_topic = readMessages(select(bag, 'Topic', '/SQ01s/goal'));
jerk_bezier=[];
for i=1:size(goal_topic,1)
    if(goal_topic{i}.Pos.X>min_x && goal_topic{i}.Pos.X<max_x)
        jerk=[goal_topic{i}.Jerk.X; goal_topic{i}.Jerk.Y; goal_topic{i}.Jerk.Z];
        jerk_bezier=[jerk_bezier; norm(jerk)];
    end
end

%% MINVO
bag = rosbag('minvo.bag');
goal_topic = readMessages(select(bag, 'Topic', '/SQ01s/goal'));
jerk_minvo=[];
for i=1:size(goal_topic,1)
    if(goal_topic{i}.Pos.X>min_x && goal_topic{i}.Pos.X<max_x)
        jerk=[goal_topic{i}.Jerk.X; goal_topic{i}.Jerk.Y; goal_topic{i}.Jerk.Z];
        jerk_minvo=[jerk_minvo; norm(jerk)];
    end
end
%%

%B-SPLINE costs

% cost_bs=cost_bs(cost_bs>114);
% cost_minvo=cost_minvo(cost_bs>114);

max_jerk=max([jerk_bspline;jerk_bezier;jerk_minvo]);
min_jerk=min([jerk_bspline;jerk_bezier;jerk_minvo]);

ax=figure;
subplot(3,1,1);
bins=80;
histogram(jerk_bspline,bins,'FaceColor','#0072BD', 'Normalization','pdf'); 
xlim([min_jerk,max_jerk])

legend1= strcat("B-Spline: ", num2str(mean(jerk_bspline)), "$\pm $",  num2str(std(jerk_bspline))," $m/s^3$" );
legend({legend1},'FontSize',12)

subplot(3,1,2);
histogram(jerk_bezier,bins,'FaceColor','#D95319', 'Normalization','pdf');
legend2=strcat("B\'ezier: ", num2str(mean(jerk_bezier)), "$\pm $",  num2str(std(jerk_bezier))," $m/s^3$" );
legend({legend2},'FontSize',12)
xlim([min_jerk,max_jerk])

subplot(3,1,3);
histogram(jerk_minvo,bins,'FaceColor','#EDB120', 'Normalization','pdf');
legend3=strcat("MINVO (ours): ", num2str(mean(jerk_minvo)), "$\pm $",  num2str(std(jerk_minvo))," $m/s^3$" );
legend({legend3},'FontSize',12)
xlim([min_jerk,max_jerk])

% legend1= strcat("BSpline convex hull: ", num2str(mean(jerk_bspline)), "$\pm $",  num2str(std(jerk_bspline)) );
% legend2=strcat("B\'ezier convex hull: ", num2str(mean(jerk_bezier)), "$\pm $",  num2str(std(jerk_bezier)) );
% legend3=strcat("MINVO convex hull (ours): ", num2str(mean(jerk_minvo)), "$\pm $",  num2str(std(jerk_minvo)) );

% 
% legend({legend1,legend2,legend3},'FontSize',12)

xlabel("Control Effort ($m/s^3$)")
% 
% title('\textbf{Histogram (200 runs)}')

