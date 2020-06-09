close all; clear; clc;
set(0,'defaulttextInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');


min_x=1;
max_x=50;




clc;
[vel_minvo dist_minvo time_minvo n_times_stopped_minvo]=readBagsThatStartWith('MINVO');
[vel_bezier dist_bezier time_bezier n_times_stopped_bezier]=readBagsThatStartWith('BEZIER');

[vel_bspline dist_bspline time_bspline n_times_stopped_bspline]=readBagsThatStartWith('B_SPLINE');

%%
close all; clc

vel_minvo(vel_minvo<0)
%%Plot distances
n_bins=5;
subplot(3,3,1); xlabel('Distance (m)')
histogram(dist_minvo,n_bins,'FaceColor','#0072BD'); %, 'Normalization','pdf'
subplot(3,3,4); xlabel('Distance (m)')
histogram(dist_bezier,n_bins,'FaceColor','#0072BD'); 
subplot(3,3,7); xlabel('Distance (m)')
histogram(dist_bspline,n_bins,'FaceColor','#0072BD'); 

%%Plot times
n_bins=5;
subplot(3,3,2); xlabel('Time(s)')
histogram(time_minvo,n_bins,'FaceColor','#0072BD'); 
subplot(3,3,5); xlabel('Time(s)')
histogram(time_bezier,n_bins,'FaceColor','#0072BD'); 
subplot(3,3,8); xlabel('Time(s)')
histogram(time_bspline,n_bins,'FaceColor','#0072BD'); 

%%Plot velocity x
n_bins=100;
subplot(3,3,3); xlabel('Vel ($m/s$)')
histogram(vel_minvo(1,:),n_bins,'FaceColor','#0072BD', 'Normalization','pdf'); 
subplot(3,3,6); xlabel('Vel ($m/s$)')
histogram(vel_bezier(1,:),n_bins,'FaceColor','#0072BD', 'Normalization','pdf'); 
subplot(3,3,9); xlabel('Vel ($m/s$)')
histogram(vel_bspline(1,:),n_bins,'FaceColor','#0072BD', 'Normalization','pdf'); 

disp("velocities")
[mean(abs(vel_minvo),2) mean(abs(vel_bezier),2) mean(abs(vel_bspline),2)]


disp("distances")
[mean(dist_minvo), mean(dist_bezier), mean(dist_bspline)]

disp("times")
[mean(time_minvo), mean(time_bezier), mean(time_bspline)]

disp("n_times_stopped")
[mean(n_times_stopped_minvo) mean(n_times_stopped_bezier) mean(n_times_stopped_bspline) ]

% ax=figure;
% subplot(3,1,1);
% % bins=80;
% histogram(dist_minvo,'FaceColor','#0072BD', 'Normalization','pdf'); 
% xlim([min_jerk,max_jerk])
%%
figure
subplot(3,1,1)
 scatter3(vel_minvo(1,:),vel_minvo(2,:),vel_minvo(3,:)); title('MINVO')
subplot(3,1,2)
 scatter3(vel_bezier(1,:),vel_bezier(2,:),vel_bezier(3,:)); title('Bezier')
subplot(3,1,3)
 scatter3(vel_bspline(1,:),vel_bspline(2,:),vel_bspline(3,:)); title('BSpline')
%%

%B-SPLINE costs

% cost_bs=cost_bs(cost_bs>114);
% cost_minvo=cost_minvo(cost_bs>114);
% 
% max_jerk=max([jerk_bspline;jerk_bezier;jerk_minvo]);
% min_jerk=min([jerk_bspline;jerk_bezier;jerk_minvo]);
% 
% ax=figure;
% subplot(3,1,1);
% bins=80;
% histogram(jerk_bspline,bins,'FaceColor','#0072BD', 'Normalization','pdf'); 
% xlim([min_jerk,max_jerk])
% 
% legend1= strcat("B-Spline: ", num2str(mean(jerk_bspline)), "$\pm $",  num2str(std(jerk_bspline))," $m/s^3$" );
% legend({legend1},'FontSize',12)
% 
% subplot(3,1,2);
% histogram(jerk_bezier,bins,'FaceColor','#D95319', 'Normalization','pdf');
% legend2=strcat("B\'ezier: ", num2str(mean(jerk_bezier)), "$\pm $",  num2str(std(jerk_bezier))," $m/s^3$" );
% legend({legend2},'FontSize',12)
% xlim([min_jerk,max_jerk])
% 
% subplot(3,1,3);
% histogram(jerk_minvo,bins,'FaceColor','#EDB120', 'Normalization','pdf');
% legend3=strcat("MINVO (ours): ", num2str(mean(jerk_minvo)), "$\pm $",  num2str(std(jerk_minvo))," $m/s^3$" );
% legend({legend3},'FontSize',12)
% xlim([min_jerk,max_jerk])

% legend1= strcat("BSpline convex hull: ", num2str(mean(jerk_bspline)), "$\pm $",  num2str(std(jerk_bspline)) );
% legend2=strcat("B\'ezier convex hull: ", num2str(mean(jerk_bezier)), "$\pm $",  num2str(std(jerk_bezier)) );
% legend3=strcat("MINVO convex hull (ours): ", num2str(mean(jerk_minvo)), "$\pm $",  num2str(std(jerk_minvo)) );

% 
% legend({legend1,legend2,legend3},'FontSize',12)

xlabel("Control Effort ($m/s^3$)")
% 
% title('\textbf{Histogram (200 runs)}')

function [vel dist time n_times_stopped]=readBagsThatStartWith(name)

    
    vel=[]; %each column has the velocity
    dist=[]; %one element per bag
    time=[]; %one element per bag
    n_times_stopped=[];
    
    files = dir(['./',name,'*.bag']);
    filenames = {files.name};
    
    stopped=1; %Starts the sim stopped; 
    
    for i=1:size(filenames,2)
        disp(['reading bag ', filenames{i}])
        bag = rosbag(filenames{i});
        topics = readMessages(select(bag, 'Topic', '/SQ01s/goal'));
        total_distance=0.0;
        n_times_stopped_per_bag=0;
        
        times_filtered=[];
        
        for j=1:size(topics,1)
             %%%%%%%%%%%%%%%%%%%%%%%%%
             min_x=1;
             max_x=47;
             if(topics{j}.Pos.X>min_x && topics{j}.Pos.X<max_x)
              vel_j=[topics{j}.Vel.X; topics{j}.Vel.Y; topics{j}.Vel.Z];
              times_filtered=[times_filtered topics{j}.Header.Stamp.Sec + topics{j}.Header.Stamp.Nsec*(1e-9)];
              if(norm(vel_j)<1e-7 && stopped==0)
                  stopped=1;
                  n_times_stopped_per_bag=n_times_stopped_per_bag+1;
              end
              if(norm(vel_j)>1e-7)
                 stopped=0;
              end
              
              vel=[vel vel_j];
             end
             %%%%%%%%%%%%%%%%%%%%%%%%%

            pos=[topics{j}.Pos.X; topics{j}.Pos.Y; topics{j}.Pos.Z];

            if(j>1)
                total_distance=total_distance+ norm(pos-last_pos);
            end

            last_pos=pos;
        end
        
         n_times_stopped=[n_times_stopped n_times_stopped_per_bag];

        t_init=min(times_filtered);
        t_final=max(times_filtered);

        time=[time t_final-t_init];
        dist=[dist total_distance];
    end


end
