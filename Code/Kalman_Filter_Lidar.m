%% Ahmed Elsaharti - 2019

clear all
close all
clc
clf
%% Set g^2 for matching
g2=1000;

%%  Initialize ground truth vector for comparison
G_T=browsefile('ground truth');


%% Animated plot initialization 
plot_KF_Pose=[];
plot_dumb_Pose=[];
data_points={};
data_points_KF={};

%% Import map and pre-process units
map=browsefile('map');
map(:,2:end)=map(:,2:end); %convert to mm
map(:,1)=deg2rad(map(:,1)); %convert to rad

%% Set distance between wheels
b=160;

%% Set error constants
Kr=0.1;
Kl=0.1;

prompt = 'Enter robot x position in mm';
x_position = input(prompt);

prompt = 'Enter robot y position in mm';
y_position = input(prompt);

prompt = 'Enter robot theta relative to x in degrees';
robot_theta = deg2rad(input(prompt));

%% Create initial position vector
pose=[x_position;y_position;robot_theta];
dumb_pose=[x_position;y_position;robot_theta];
plot_KF_Pose=[pose(1),pose(2)];
plot_dumb_Pose=[dumb_pose(1),dumb_pose(2)];
plot(pose(1),pose(2),'s','MarkerFaceColor', 'b','MarkerSize',10)
hold on

%% Set initial covariance matrix to zero
P_t_1=zeros(3);

prompt = 'Enter number of scans/movements after initial move';
EndOfTime = input(prompt);

%% Main loop
for i=1:EndOfTime
    
prompt = 'Enter robot right wheel movement in mm';
delta_s_r = input(prompt);

prompt = 'Enter robot left wheel movement in mm';
delta_s_l = input(prompt);

delta_s_t=(delta_s_r+delta_s_l)/2;
delta_theta_t=(delta_s_r-delta_s_l)/b;

%% Step 1 - Position prediction:
x_t_1=pose(1);
y_t_1=pose(2);
theta_t_1=pose(3);

x_hat_t=pose+[((delta_s_r+delta_s_l)/2)*cos(theta_t_1+((delta_s_r-delta_s_l)/(2*b)));((delta_s_r+delta_s_l)/2)*sin(theta_t_1+((delta_s_r-delta_s_l)/(2*b)));(delta_s_r-delta_s_l)/b];
theta_t=x_hat_t(3);



        %       Dumb System Simulation 1:
                dumb_pose=dumb_pose+[((delta_s_r+delta_s_l)/2)*cos(dumb_pose(3)+((delta_s_r-delta_s_l)/(2*b)));((delta_s_r+delta_s_l)/2)*sin(dumb_pose(3)+((delta_s_r-delta_s_l)/(2*b)));(delta_s_r-delta_s_l)/b];


%% Step 2 - Covariance matrix update:
F_x=[1 0 -delta_s_t*sin(theta_t+(delta_theta_t/2));
     0 1  delta_s_t*cos(theta_t+(delta_theta_t/2));
     0 0  1];

F_u=[0.5*cos(theta_t+delta_theta_t/2)-((delta_s_t/(2*b))*sin(theta_t+delta_theta_t/2)),    0.5*cos(theta_t+delta_theta_t/2)+((delta_s_t/(2*b))*sin(theta_t+delta_theta_t/2));
     0.5*sin(theta_t+delta_theta_t/2)+((delta_s_t/(2*b))*cos(theta_t+delta_theta_t/2)),    0.5*sin(theta_t+delta_theta_t/2)-((delta_s_t/(2*b))*cos(theta_t+delta_theta_t/2));
     1/b                                                                              ,   -1/b];

Q_t=[Kr*abs(delta_s_r) 0;
     0                 Kl*abs(delta_s_l)];
 
P_hat_t=F_x*P_t_1*F_x'+F_u*Q_t*F_u';

%% Step 3 - Observations (Lidar Readings):

lidar_readings=browsefile(['observation ',num2str(i)]);
[alpha_r,Covariance_mat]=splitandmerge(lidar_readings);
number_of_detected_lines=length(alpha_r);


                    %%% Animation stuff
                    data = map;
                    data(:,1)=deg2rad(data(:,1));
                    data(:,2:end)=data(:,2:end);
                    original_map=data(:,3:6);

                    for ii=1:length(original_map)
                       xs=[original_map(ii,1),original_map(ii,3)];
                       ys=[original_map(ii,2);original_map(ii,4)];
                       plot(xs,ys,'-k','LineWidth',2)
                       hold on
                    end                    
                    
                    T_robot_to_world = [cos(dumb_pose(3)) -sin(dumb_pose(3)) 0 dumb_pose(1)
                                        sin(dumb_pose(3)) cos(dumb_pose(3)) 0 dumb_pose(2)
                                        0 0 1 0
                                        0 0 0 1];

                    %   Transform Lidar readings to world frame

                    data = lidar_readings;
                    for i=1:length(data)
                        Point=[data(i,1:2)';0;1];
                        Point_New=T_robot_to_world*Point;
                        data(i,1)=Point_New(1);
                        data(i,2)=Point_New(2);
                    end
                    xy=data;                    
                    data_points=[data_points;data(:,1),data(:,2)];
                    axis equal

                    T_robot_to_world_KF = [cos(pose(3)) -sin(pose(3)) 0 pose(1)
                                        sin(pose(3)) cos(pose(3)) 0 pose(2)
                                        0 0 1 0
                                        0 0 0 1];
                     data = lidar_readings;               
                    for i=1:length(data)
                                            Point=[data(i,1:2)';0;1];
                                            Point_New=T_robot_to_world_KF*Point;
                                            data(i,1)=Point_New(1);
                                            data(i,2)=Point_New(2);
                    end
                                        xy=data;
                                        data_points_KF=[data_points_KF;data(:,1),data(:,2)];
                    %%% End of animation stuff

for ii=1:number_of_detected_lines
    z_i_t{ii}=[alpha_r(ii,1);alpha_r(ii,2)];
    R_i_t{ii}=Covariance_mat{ii};
end

%% Step 4 - Measurement predictions :
number_of_map_lines=length(map);
for ii=1:number_of_map_lines
    z_j_t{ii}=[map(ii,1)-x_hat_t(3);
               map(ii,2)-(x_hat_t(1)*cos(map(ii,1))+x_hat_t(2)*sin(map(ii,1)))];
    H_j{ii}=[              0               0    -1;
             -cos(map(ii,1)) -sin(map(ii,1))     0];
end

%% Step 5 - Matching :

%initializing Z_t, V_t, H_t and R_t
Z_t=[];
V_t=[];
H_t=[];
R_t=[];
matches=0;
for ii=1:number_of_detected_lines
    for jj=1:number_of_map_lines
        V_ij_t=z_i_t{ii}-z_j_t{jj};
        E_IN_ij_t=H_j{jj}*P_hat_t*H_j{jj}'+R_i_t{ii};
        actualg2=V_ij_t'*inv(E_IN_ij_t)*V_ij_t;
        if actualg2<=g2
            disp(['MATCH ',num2str(ii),' ',num2str(jj)])
            matches=matches+1;
            Z_t=[Z_t;z_i_t{ii}];
            V_t=[V_t;V_ij_t];
            H_t=[H_t;H_j{jj}];
            R_t=blkdiag(R_t,R_i_t{ii});
        end
    end
end

if matches==0
    error('No line match found')
end

%% Step 6 - K.F. Estimate :

E_IN_t=H_t*P_hat_t*H_t'+R_t;
K_t=P_hat_t*H_t'*inv(E_IN_t);
x_t=x_hat_t+K_t*V_t;
pose=x_t;
P_t_1=P_hat_t-K_t*E_IN_t*K_t';
DisplayKalmanPose=[pose(1);pose(2);rad2deg(pose(3))]
DisplayDumbPose=[dumb_pose(1);dumb_pose(2);rad2deg(dumb_pose(3))]
plot_KF_Pose=[plot_KF_Pose;pose(1),pose(2)];
plot_dumb_Pose=[plot_dumb_Pose;dumb_pose(1),dumb_pose(2)];


end


%% Start Animations
for jj=2:EndOfTime
for ii=1:length(data_points{jj})
plot(data_points{jj-1}(ii,1),data_points{jj-1}(ii,2),'.r');
end
pause(0.1)
for ii=1:length(data_points_KF{jj})
plot(data_points_KF{jj}(ii,1),data_points_KF{jj}(ii,2),'.g');
end

plot(G_T(jj,1),G_T(jj,2),'o-','MarkerFaceColor', 'k');
plot(plot_dumb_Pose(jj,1),plot_dumb_Pose(jj,2),'o-','MarkerFaceColor', 'r');
plot(plot_KF_Pose(jj,1),plot_KF_Pose(jj,2),'o-','MarkerFaceColor', 'g');
plot(plot_KF_Pose(jj-1:jj,1),plot_KF_Pose(jj-1:jj,2),'-g');
plot(plot_dumb_Pose(jj-1:jj,1),plot_dumb_Pose(jj-1:jj,2),'-r');
plot(G_T(jj-1:jj,1),G_T(jj-1:jj,2),'-k');
pause(0.1)

end
for ii=1:length(data_points{jj})
plot(data_points{EndOfTime}(ii,1),data_points{EndOfTime}(ii,2),'.r');
end
pl_dumb=plot(plot_dumb_Pose(jj,1),plot_dumb_Pose(jj,2),'o','MarkerFaceColor', 'r');
pl_KF=plot(plot_KF_Pose(jj,1),plot_KF_Pose(jj,2),'o','MarkerFaceColor', 'g');
plot(plot_dumb_Pose(jj+1,1),plot_dumb_Pose(jj+1,2),'o-','MarkerFaceColor', 'r');
plot(plot_KF_Pose(jj+1,1),plot_KF_Pose(jj+1,2),'o-','MarkerFaceColor', 'g');
plot(plot_KF_Pose(jj:jj+1,1),plot_KF_Pose(jj:jj+1,2),'-g');
plot(plot_dumb_Pose(jj:jj+1,1),plot_dumb_Pose(jj:jj+1,2),'-r');
plot(G_T(jj:jj+1,1),G_T(jj:jj+1,2),'-k');
pl_GT=plot(G_T(jj+1,1),G_T(jj+1,2),'o','MarkerFaceColor', 'k');
legend([pl_dumb pl_KF pl_GT],{'Dumb Estimate','Kalman Filter Estimate','Ground Truth'})