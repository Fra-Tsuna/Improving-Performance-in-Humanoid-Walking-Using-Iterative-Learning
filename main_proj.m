close all; clear; clc;

%% Constants definition
z_com = 0.8; %average height CoM, m.
g0 = 9.81; %m/s**2
g=[0; 0; -g0];

b=sqrt(z_com/g0); %s, time constant.
eta=1/b;

N=20; % # foot positions.
t_step=1.2;

deltax=0.3;
deltay=0.25;

R_delta=eye(2); %Rotation matrix from step to step

kl=1; %Learning factor
kf=1; %Forgetting factor
k_DCM=4;

Ts=0.001; %Discrete time sampling interval
T_ILC=0.01; %Learning time
T_iter=2*t_step; %Duration of walking cycle

t=(0:Ts:t_step-Ts); %Time in a step
T=N*t_step;

Tg=(0:Ts:T-Ts); %Global time

colors=['r','g','b'];
lw = 1;

%% Footprints Generation

%-robot feet are point feet
%-constant angular momentum L, dL=0.
%-no double support phase, instantaneous transition between left and right SS 
%-no impacts during transitions

footprints=zeros(3,N);

footprint0_sx = [0,deltay/2,0]';
footprint0_dx = [0,-deltay/2,0]';  

max_height = 0;


for i=1:N
    if i==1
        footprints(:,i)=footprint0_dx;
%     elseif i == N
%         footprints(:,i)=[deltax*(i-1); deltay/2; rand*max_height];
    elseif mod(i,2)~=0
        footprints(:,i)=[deltax*(i-1); -deltay/2; rand*max_height];
    else
        footprints(:,i)=[deltax*(i-1); deltay/2; rand*max_height];
    end
end

%% Waypoints generation

eCMPs = footprints;
ZMPs = footprints;

VRP_des = footprints + [0; 0; z_com];

%% VRP desired trajectory with DS

% find VRP waypoints for start, end of each transition phase, interpolate,
% compute DCM trajectory
% divide t_step in T_DS+T_SS
VRP_trajectory = zeros(3,N,length(t));
SS_ratio = 0.9/t_step;
T_SS = SS_ratio*t_step;
T_DS = (1-SS_ratio)*t_step;
for  i=1:N
    % VRP_DS_ini_i+1 = VRP_SS_end_i
    if i < N
        VRP_DS_end_i = VRP_des(:,i+1);
    else
        VRP_DS_end_i = VRP_des(:,i);
    end
    for t_ss = 1:int16(T_SS/Ts)
        VRP_trajectory(:,i,t_ss) = VRP_des(:,i);
    end
    for t_ds = 1:T_DS/Ts
        VRP_trajectory(:,i,int32(T_SS/Ts+t_ds)) = (1 - t_ds*Ts/T_DS)*...
            VRP_trajectory(:,i,t_ss)+...
            (t_ds*Ts/T_DS)*VRP_DS_end_i;
    end
end

f=figure(1);
f.Position = [0 0 1920 1080];

for j=1:3
    subplot(1,3,j)
    hold off
    hold on
    for i=1:N
        plot(t+t_step*(i-1),reshape(VRP_trajectory(j,i,:),1,[]),'b','LineWidth',lw);
    end
    grid on
    hold off
    xlim([0,24])
    if j==1
        title("VRP desired trajectory x component")
    elseif j==2
        title("VRP desired trajectory y component")
        ylim([-1,1])
    else
        title("VRP desired trajectory z component")
        ylim([0,1])
    end
end

%% DCM desired trajectory with DS

DCM_trajectory = zeros(3,N,length(t));
DCM_DS_end_N = VRP_trajectory(:,end,end);
for i = N:-1:1
    VRP_DS_end_i = VRP_trajectory(:,i,end);
    if i == N
        DCM_DS_end_i = DCM_DS_end_N;
    else
        DCM_DS_end_i = DCM_trajectory(:,i+1,1);
    end
    VRP_DS_ini_i = VRP_trajectory(:,i,length(Ts:Ts:T_SS));
    for t_ds = 1:length(Ts:Ts:T_DS)
        
        alpha = 1-t_ds*Ts/T_DS- b/T_DS + exp((t_ds*Ts-T_DS)/b)*b/T_DS;
        beta = t_ds*Ts/T_DS + b/T_DS - exp((t_ds*Ts-T_DS)/b)*(1+b/T_DS);
        gamma = exp((t_ds*Ts-T_DS)/b);
        
        DCM_trajectory(:,i,int32(T_SS/Ts+t_ds)) = alpha*VRP_DS_ini_i +...
            beta*VRP_DS_end_i +...
            gamma*DCM_DS_end_i;
    end
    DCM_SS_end_i = DCM_trajectory(:,i,length(Ts:Ts:T_SS)+1);
    for t_ss = 1:length(Ts:Ts:T_SS)
        alpha = 1-t_ss*Ts/T_SS- b/T_SS + exp((t_ss*Ts-T_SS)/b)*b/T_SS;
        beta = t_ss*Ts/T_SS + b/T_SS - exp((t_ss*Ts-T_SS)/b)*(1+b/T_SS);
        gamma = exp((t_ss*Ts-T_SS)/b);
        
        DCM_trajectory(:,i,t_ss) = (alpha + beta)*VRP_des(:,i) + gamma*DCM_SS_end_i;
        
    end
end

f=figure(2);
f.Position = [0 0 1920 1080];

for j=1:3
    subplot(1,3,j);
    hold off
    hold on
    for i=1:N
        plot(t+t_step*(i-1),reshape(DCM_trajectory(j,i,:),1,[]),'b','LineWidth',lw);
        
    end
    grid on
    hold off
    xlim([0,24])
    if j==1
        title("DCM desired x component")
    elseif j==2
        title("DCM desired y component")
        ylim([-1,1])
    else
        title("DCM desired z component")
        ylim([0,1])
    end
end

%% CoM desired trajectory
CoM_trajectory = zeros(3,N,length(t));
for i = 1:N
    for ts = 1:length(t)
        if i == 1 && ts == 1
            prev_CoM = [0,0,z_com]';
            prev_DCM = DCM_trajectory(:,1,1);
        elseif ts == 1
            prev_Com = CoM_trajectory(:,i-1,end);
            prev_DCM = DCM_trajectory(:,i-1,end);
        else
            prev_CoM = CoM_trajectory(:,i,ts-1);
            prev_DCM = DCM_trajectory(:,i,ts-1);
        end
        CoM_trajectory(:,i,ts) = prev_CoM - eta*(prev_CoM-prev_DCM)*Ts;
    end
end
CoM_traj_des = permute(reshape(permute(CoM_trajectory, [1 3 2]),3, T_iter/Ts,[]),[1 3 2]);
%% plot on plane

f=figure(3);
f.Position = [0 0 1920 1080];
for i=1:N
    hold on;
    plot(reshape(VRP_trajectory(1,i,:),1,[]),reshape(VRP_trajectory(2,i,:),1,[]),'b','LineWidth',lw); 
    plot(reshape(DCM_trajectory(1,i,:),1,[]),reshape(DCM_trajectory(2,i,:),1,[]),'r','LineWidth',lw);
end
legend('VRP_d','DCM_d')
axis equal
xlim([0,deltax*4])
grid on
hold off

title("VRP and DCM desired on 2d plane")

    

%% DCM disturbed
dis = 0.9;
b = dis*b;

DCM_disturbed = zeros(3,N,length(t));
DCM_DS_end_N = VRP_trajectory(:,end,end);
for i = N:-1:1
    VRP_DS_end_i = VRP_trajectory(:,i,end);
    if i == N
        DCM_DS_end_i = DCM_DS_end_N;
    else
        DCM_DS_end_i = DCM_disturbed(:,i+1,1);
    end
    VRP_DS_ini_i = VRP_trajectory(:,i,length(Ts:Ts:T_SS));
    for t_ds = 1:length(Ts:Ts:T_DS)
        
        alpha = 1-t_ds*Ts/T_DS- b/T_DS + exp((t_ds*Ts-T_DS)/b)*b/T_DS;
        beta = t_ds*Ts/T_DS + b/T_DS - exp((t_ds*Ts-T_DS)/b)*(1+b/T_DS);
        gamma = exp((t_ds*Ts-T_DS)/b);
        
        DCM_disturbed(:,i,int32(T_SS/Ts+t_ds)) = alpha*VRP_DS_ini_i +...
            beta*VRP_DS_end_i +...
            gamma*DCM_DS_end_i;
    end
    DCM_SS_end_i = DCM_disturbed(:,i,length(Ts:Ts:T_SS)+1);
    for t_ss = 1:length(Ts:Ts:T_SS)
        alpha = 1-t_ss*Ts/T_SS- b/T_SS + exp((t_ss*Ts-T_SS)/b)*b/T_SS;
        beta = t_ss*Ts/T_SS + b/T_SS - exp((t_ss*Ts-T_SS)/b)*(1+b/T_SS);
        gamma = exp((t_ss*Ts-T_SS)/b);
        
        DCM_disturbed(:,i,t_ss) = (alpha + beta)*VRP_des(:,i) + gamma*DCM_SS_end_i;
        
    end
end
b = b/dis;

f=figure(4);
f.Position = [0 0 1920 1080];


for j=1:3
    subplot(1,3,j);
    hold off
    hold on
    for i=1:N
        plot(t+t_step*(i-1),reshape(DCM_trajectory(j,i,:),1,[]),'b--','LineWidth',lw);
        plot(t+t_step*(i-1),reshape(DCM_disturbed(j,i,:),1,[]),'g','LineWidth',lw);
    end
    grid on
    hold off
    xlim([0,24])
    if j==1
        title("DCM disturbed vsdesired x component")
    elseif j==2
        title("DCM disturbed vsdesired y component")
        ylim([-1,1])
    else
        title("DCM disturbed vsdesired z component")
        ylim([0,1])
    end
end

%% VRP_OILC based on 3 Cycles
tic
VRP_traj_des = permute(reshape(permute(VRP_trajectory, [1 3 2]),3, T_iter/Ts,[]),[1 3 2]);
DCM_traj_des = permute(reshape(permute(DCM_trajectory, [1 3 2]),3, T_iter/Ts,[]),[1 3 2]);
VRP_c_traj = VRP_trajectory + (1 + k_DCM*b)*(DCM_disturbed - DCM_trajectory);
VRP_c_traj = permute(reshape(permute(VRP_c_traj, [1 3 2]),3, T_iter/Ts,[]),[1 3 2]); %commanded VRP, computed from measured DCM,
                                                         %desired VRP and
                                                         %desired DCM.

VRP_adj=zeros(2,N/2,T_iter/Ts); %Structure for VRP adjusted
DCM_adj=zeros(2,N/2,T_iter/Ts); %Structure for DCM adjusted
DCM = zeros(2,N/2,T_iter/Ts);
CoM = zeros(2,N/2,T_iter/Ts);
CoM_dot = zeros(2,N/2,T_iter/Ts);
DCM_dis = zeros(2,N/2,T_iter/Ts);
CoM_dis = zeros(2,N/2,T_iter/Ts);
CoM_dot_dis = zeros(2,N/2,T_iter/Ts);

phi_c=1;
n_phi=N*2; %Number of transitions, 4 transitions for step so N*2.
N_iter = T_iter/Ts;
N_k = T_iter/T_ILC;
N_ILC = T_ILC/Ts;
c = b/T_ILC;
c_inv = T_ILC/b;
alpha_0 = 1 - c + exp(-c_inv)*c;
beta_0 = c - exp(-c_inv)*(1+c);
gamma_0 = exp(-c_inv);

% noise generators
b_dis = b*dis;
vel_dis = [0, 0.02]';
schedule_direction = [4000 10000 13000];
vel_dis_mat = [zeros(2, schedule_direction(1)),...
    vel_dis.*ones(2,schedule_direction(2)-schedule_direction(1)),...
    zeros(2,schedule_direction(3)-schedule_direction(2)),...
    -vel_dis.*ones(2, length(Tg)-schedule_direction(3))];
vel_dis_mat = permute(reshape(vel_dis_mat,2,[],N/2),[1 3 2]);

zeta = 1+k_DCM*b;
e_k_DCM_T = exp(-k_DCM*T_iter);    
kf = 1;
kl = 1;
for s = 1:2
    for i=1:N/2 %1
        for k = 1:N_k %8
            for index_t_p = 1:N_ILC
                index_tt = (k-1)*N_ILC+index_t_p; %simulation step in current iteration
                t_passed = (index_t_p-1)*Ts; %ILC time %9
                g_t = N_iter*(i-1)+index_tt; %global time %2
                tt = (index_tt-1)*Ts; %time in current iteration
                
                phi_c = 1+floor((i-1+tt/T_iter)*4);
                if (i==1 && index_tt==1) %3
                    Vl=VRP_traj_des(1:2,i,1:N_ILC:N_iter-N_ILC+1); %4
                    VRP_adj(:,i,index_tt)=Vl(:,1); %5
                    DCM_adj(:,i,index_tt)=DCM_traj_des(1:2,i,index_tt); %6
                else %7
                    if (t_passed == 0) %10
                        if (phi_c + 4 <= n_phi) %11
                            decreased = 0;
                            if index_tt>1
                                index_tt = index_tt-1;
                                decreased = 1;
                            end
                            VRP_adj(:,i,index_tt)=Vl(:,2); %12
                            VRP_adj(:,i+1,index_tt)=VRP_traj_des(1:2,i+1,index_tt)+...
                                kf*R_delta*(VRP_adj(:,i,index_tt)-VRP_traj_des(1:2,i,index_tt))+...
                                kl*R_delta*(VRP_traj_des(1:2,i,index_tt)-VRP_c_traj(1:2,i,index_tt)); %13
                            Vl_l=[Vl(:,2:end),VRP_adj(:,i+1,index_tt)]; %14
                            if decreased == 1
                                index_tt = index_tt+1;
                            end
                        else %15
                            Vl_l=[Vl(:,2:end),Vl(:,end)]; %16
                        end
                        Vl=Vl_l; %17
                    end
                    VRP_adj(:,i,index_tt)=(1-t_passed/T_ILC)*Vl(:,1)+t_passed/T_ILC*Vl(:,2); %18
                    if phi_c + 4 <= n_phi %19
                        DCM_f = DCM_traj_des(1:2,i+1,index_tt-index_t_p+1); %20
                    else %21
                        DCM_f = Vl(:,end); %22
                    end
                    DCM_mid = DCM_f; %23
                    for j=N_k:-1:3 %24
                        DCM_mid = alpha_0*Vl(:,j-1)+...
                            beta_0*Vl(:,j)+gamma_0*DCM_mid; %25
    
                    end
                    alpha_t = 1-t_passed/T_ILC - c + exp((t_passed-T_ILC)/b)*c;
                    beta_t = t_passed/T_ILC + c - exp((t_passed-T_ILC)/b)*(1+c);
                    gamma_t = exp((t_passed-T_ILC)/b);
                      
                    DCM_adj(:,i,index_tt) = alpha_t*Vl(:,1)+...
                        beta_t*Vl(:,2)+gamma_t*DCM_mid; %26
                end
                % introduce error from here
                % for the DCM controller
                DCM_adj_dot = eta*(DCM_adj(:,i,index_tt)-VRP_adj(:,i,index_tt));
                if index_tt == 1
                    if i ==1
                        prev_DCM = DCM_traj_des(1:2,1,1);
                        prev_DCM_adj = DCM_traj_des(1:2,1,1);
                        prev_DCM_dis = DCM_disturbed(1:2,1,1);
                        prev_CoM = [0, 0]';
                        prev_CoM_dis = prev_CoM;
                    else
                        prev_DCM = DCM(:,i-1,end);
                        prev_DCM_adj = DCM_adj(:,i-1,end);
                        prev_DCM_dis = DCM_dis(:,i-1,end);
                        prev_CoM = CoM(:,i-1,end);
                        prev_CoM_dis = CoM_dis(:,i-1,end);
                    end
                else
                    prev_DCM = DCM(:,i,index_tt-1);
                    prev_DCM_adj = DCM_adj(:,i,index_tt-1);
                    prev_DCM_dis = DCM_dis(:,i,index_tt-1);
                    prev_CoM = CoM(:,i,index_tt-1);
                    prev_CoM_dis = CoM_dis(:,i,index_tt-1);
                end
                DCM_dot = -k_DCM*(prev_DCM - DCM_adj(:,i,index_tt)) +...
                    DCM_adj_dot;
    
                %DCM controller
                DCM(:,i,index_tt) = prev_DCM + DCM_dot*Ts;
    
                % CoM commanded from VRP_adj through DCM from the
                % controller
                CoM(:,i,index_tt) = prev_CoM - eta*(prev_CoM - prev_DCM)*Ts;
                CoM_dot(:,i,index_tt) = -eta*(CoM(:,i,index_tt)-DCM(:,i,index_tt));
    
                % simulated disturbed CoM measurements
                CoM_dis(:,i,index_tt) = prev_CoM_dis - 1/b_dis*(prev_CoM_dis - prev_DCM_dis)*Ts;
                CoM_dot_dis(:,i,index_tt) = -1/b_dis*(CoM_dis(:,i,index_tt)-DCM(:,i,index_tt)) + vel_dis_mat(:, i, index_tt);
    
                % DCM disturbed from CoM disturbed
                DCM_dis(:,i,index_tt) = CoM_dis(:,i,index_tt) + b*CoM_dot_dis(:,i,index_tt);
    
                VRP_c_traj(1:2,i,index_tt) = VRP_adj(:,i,index_tt) +...
                    zeta*(DCM_dis(:,i,index_tt)-DCM_adj(:,i,index_tt));
            end
        end
    end
    if kf == 1 && kl == 1
        kf = 0;
        kl = 0;
        VRP_c_no_ILC = VRP_c_traj;
    end
end
toc

%% Error measurements
e_c = zeros(2,N/2);
e_c_no_ILC = zeros(2,N/2);
% DCM_s_disturbed = permute(reshape(permute(DCM_disturbed, [1 3 2]),3, T_iter/Ts,[]),[1 3 2]);
% VRP_c_no_ILC = VRP_traj_des(1:2,:,:) + (1 + k_DCM*b)*(DCM_s_disturbed(1:2,:,:) - DCM_traj_des(1:2,:,:));

for i = 1:N/2
    e_c(:,i) = sum(abs(VRP_traj_des(1:2,i,:)-VRP_c_traj(1:2,i,:))*Ts,3)/T_iter;
    e_c_no_ILC(:,i) = sum(abs(VRP_traj_des(1:2,i,:)-VRP_c_no_ILC(1:2,i,:))*Ts,3)/T_iter;
end

e_c_norm = vecnorm(e_c, 2, 1);
e_c_no_ILC_norm = vecnorm(e_c_no_ILC, 2, 1);

f=figure(5);
f.Position = [0 0 1920 540];
hold off
for j=1:3
%     subplot(1,3,j);
    hold on
    if j<3
%         plot(1:N/2,e_c(j,:), 'r-o',LineWidth=lw);
%         plot(1:N/2,e_c_no_ILC(j,:), 'g--o',LineWidth=lw);
    else
        plot(1:N/2,e_c_norm, 'r-o',LineWidth=lw);
        plot(1:N/2,e_c_no_ILC_norm, 'g--o',LineWidth=lw);
    end
    
    legend({'e_{c,ILC}','e_{c,no-ILC}'},FontSize=16);
    
    box on
    grid on
    hold off
    xlim([1,N/2])
    if j == 1
%         title("Commanded Error with and without ILC on x axis")
    elseif j == 2
%         title("Commanded Error with and without ILC on y axis")
    else
%         title("Commanded Error norm with and without ILC")
    end
end

%% Final plots

f=figure(6);
f.Position = [0 0 960 540];
hold off
for j=3:3
%     if j<=2
%         subplot(1,2,j);
%     end
    hold on
    xlim([0,1.2*N])
    for i=1:N/2
        if j<3
            plot(Tg((i-1)*fix(T_iter/Ts)+1:i*fix(T_iter/Ts)),reshape(VRP_adj(j,i,:),1,[]),'g','LineWidth',lw)
            plot(Tg((i-1)*fix(T_iter/Ts)+1:i*fix(T_iter/Ts)),reshape(VRP_c_traj(j,i,:),1,[]),'b','LineWidth',lw)
            plot(Tg((i-1)*fix(T_iter/Ts)+1:i*fix(T_iter/Ts)),reshape(VRP_traj_des(j,i,:),1,[]),'r','LineWidth',lw)
            legend({'VRP_{adj}','VRP_{c}', 'VRP_{des}'},FontSize=16);
%         if j<3
%             j = j-2;
%             plot(Tg((i-1)*fix(T_iter/Ts)+1:i*fix(T_iter/Ts)),reshape(vel_dis_mat(j,i,:),1,[]),'b',LineWidth=lw);
%             j = j+2;
%             j = j-2;
%             plot(Tg((i-1)*fix(T_iter/Ts)+1:i*fix(T_iter/Ts)),reshape(DCM_adj(j,i,:),1,[]),'g',LineWidth=lw);
%             plot(Tg((i-1)*fix(T_iter/Ts)+1:i*fix(T_iter/Ts)),reshape(DCM(j,i,:),1,[]),'b',LineWidth=lw);
%             plot(Tg((i-1)*fix(T_iter/Ts)+1:i*fix(T_iter/Ts)),reshape(DCM_traj_des(j,i,:),1,[]),'r',LineWidth=lw);
%             j = j+2;
%             legend('DCM_{adj}','DCM_{c}','DCM_{des}');
%         elseif j<7
%             j = j-4;
%             plot(Tg((i-1)*fix(T_iter/Ts)+1:i*fix(T_iter/Ts)),reshape(CoM_dis(j,i,:),1,[]),'g',LineWidth=lw);
%             plot(Tg((i-1)*fix(T_iter/Ts)+1:i*fix(T_iter/Ts)),reshape(CoM(j,i,:),1,[]),'b',LineWidth=lw);
%             plot(Tg((i-1)*fix(T_iter/Ts)+1:i*fix(T_iter/Ts)),reshape(CoM_traj_des(j,i,:),1,[]),'r',LineWidth=lw);
%             legend('CoM_{dis}','CoM','CoM_{des}');
%             j = j+4;
        else
            if i ==1
                f=figure(7);
                f.Position = [0 0 1920 1080];
                hold on
            end
            plot(reshape(CoM(1,i,:),1,[]),reshape(CoM(2,i,:),1,[]),colors(1),'LineWidth',2*lw);
            plot(reshape(DCM(1,i,:),1,[]),reshape(DCM(2,i,:),1,[]),colors(2),'LineWidth',2*lw);
            plot(reshape(VRP_c_traj(1,i,:),1,[]),reshape(VRP_c_traj(2,i,:),1,[]),colors(3),'LineWidth',2*lw);
            legend({'CoM','DCM','VRP'},FontSize=16);
            xlim([0, deltax*4])
            axis equal
        end
    end
    box on
    grid on;
    hold off;
    if j==1
%         title("VRP trajectories x component");
    elseif j==2
%         title("VRP trajectories y component");
        ylim([-0.3,0.3])
    elseif j==3
%         title("Wind effect on x component");
    elseif j==4
%         title("Wind effect on y component");
        ylim([-0.1,0.1])
    elseif j==5
%         title("CoM commanded trajectory x component");
    elseif j==6
%         title("CoM commanded trajectory y component");
        ylim([-1,1])
    elseif j==7
%         title("CoM, DCM and VRP on plane");
    end
end
