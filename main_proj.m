close all; clear; clc;

%% Constants definition
m = 60; %kg
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

imgs=0;

%% Footprints Generation

%-robot feet are point feet
%-constant angular momentum L, dL=0.
%-no double support phase, instantaneous transition between left and right SS 
%-no impacts during transitions

footprints=zeros(3,N);

footprint0_sx = [0,deltay,0]';
footprint0_dx = [0,0,0]';  

max_height = 0;


for i=1:N
    if i==1
        footprints(:,i)=footprint0_dx;
%     elseif i == N
%         footprints(:,i)=[deltax*(i-1); deltay/2; rand*max_height];
    elseif mod(i,2)~=0
        footprints(:,i)=[deltax*(i-1); 0; rand*max_height];
    else
        footprints(:,i)=[deltax*(i-1); deltay; rand*max_height];
    end
end

%% Waypoints generation

eCMPs = footprints;
ZMPs = footprints;

VRP_des = footprints + [0; 0; z_com];

%% DCM of paper 1

% DCM_eos_des = zeros(3,N);
% 
% for i=1:N
%     j=N+1-i;
%     if i==1
%         DCM_eos_des(:,j) = VRP_des(:,j);
%     else
%     DCM_eos_des(:,j)= VRP_des(:,j+1) +... 
%         exp(-eta*t_step)*(DCM_eos_des(:,j+1) - VRP_des(:,j+1));
%     end
% end
% 
% DCM_trajectories=zeros(3,N,length(t));
% 
% for k=1:length(t)
%     for i=1:N
%         DCM_trajectories(:,i,k)=VRP_des(:,i)+exp(eta*(t(k)-t_step))*...
%               (DCM_eos_des(:,i)-VRP_des(:,i));
%     end
% end
% 
% imgs=imgs+1;
% f=figure(imgs);
% f.Position = [20 200 1500 400];
% 
% for j=1:3
%     subplot(1,3,j);
%     for i=1:N
%         plot(t+t_step*(i-1),reshape(DCM_trajectories(j,i,:),1,[]),colors(j));
%         hold on;
%     end
%     grid on;
%     hold off;
%     if j==1
%         title("DCM measured x component");
%     elseif j==2
%         title("DCM measured y component");
%     else
%         title("DCM measured z component");
%     end
% end

%% dot DCM of paper 1

% dot_DCM_trajectories=zeros(3,N,length(t));
% for k=1:length(t)
%     for i=1:N
%         dot_DCM_trajectories(:,i,k)=eta*exp(eta*(t(k)-t_step))*...
%               (DCM_eos_des(:,i)-VRP_des(:,i));
%     end
% end
% 
% imgs=imgs+1;
% f=figure(imgs);
% f.Position = [20 200 1500 400];
% 
% for j=1:3
%     subplot(1,3,j);
%     for i=1:N
%         tx=t+t_step*(i-1);
%         plot(tx,reshape(dot_DCM_trajectories(j,i,:),1,[]),colors(j));
%         hold on;
%         if(i<N)
%             y1=dot_DCM_trajectories(j,i,length(tx));
%             y2=dot_DCM_trajectories(j,i+1,1);
%             plot([tx(end) tx(end)],[y1 y2],colors(j));
%             hold on;
%         end
%     end
%     grid on;
%     hold off;
%     if j==1
%         title("dotDCM measured x component");
%     elseif j==3
%         title("dotDCM measured y component");
%     else
%         title("dotDCM measured z component");
%     end
% end

%% CoM of paper 1
% %Euler integration

% CoM=zeros(3,N,length(t));
% 
% prev=[0, deltay/2, z_com]';
% prevDCM=DCM_trajectories(:,1,1);
% 
% for i=1:N
%     if i > 1
%         prev = CoM(:,i-1,length(t));
%         prevDCM = DCM_trajectories(:,i-1,length(t));
%     end
%     for k=1:length(t)
%         if k==1
%            CoM(:,i,k) = prev;
%         else
%             prev = CoM(:,i,k-1);
%             prevDCM = DCM_trajectories(:,i,k-1);
%             CoM(:,i,k) = prev - eta*(prev - prevDCM)*t(2);
%         end
%     end
% end
% 
% imgs=imgs+1;
% f=figure(imgs);
% f.Position = [20 200 1500 400];
% 
% for j=1:3
%     subplot(1,3,j);
%     for i=1:N
%         plot(t+t_step*(i-1),reshape(CoM(j,i,:),1,[]),colors(j));
%         hold on;
%     end
%     grid on;
%     hold off;
%     if j==1
%         title("CoM x measured component");
%     elseif j==2
%         title("CoM y measured component");
%     else
%         title("CoM z measured component");
%     end
% end

%% dot CoM of paper 1
% 
% dot_CoM = zeros(3,N,length(t));
% 
% for i=1:N
%     for k=1:length(t)
%         dot_CoM(:,i,k) = -eta*(CoM(:,i,k)-DCM_trajectories(:,i,k));
%     end
% end
% 
% imgs=imgs+1;
% f=figure(imgs);
% f.Position = [20 200 1500 400];
% 
% for j=1:3
%     subplot(1,3,j);
%     for i=1:N
%         plot(t+t_step*(i-1),reshape(dot_CoM(j,i,:),1,[]),colors(j));
%         hold on;
%     end
%     grid on;
%     hold off;
%     if j==1
%         title("dot CoM measured x component");
%     elseif j==2
%         title("dot CoM measured y component");
%     else
%         title("dot CoM measured z component");
%     end
% end

%% VRP desired trajectory of paper 2 (to convert in DS-SS)
% 
% VRP_trajectory = zeros(3,N,length(t));
% 
% for i=1:N
%     for k=1:length(t)
%         if i < N
%             VRP_trajectory(:,i,k) = (1-t(k)/t_step)*VRP_des(:,i) + (t(k)/t_step)*VRP_des(:,i+1);
%         else
%             VRP_trajectory(:,i,k) = VRP_des(:,i);
%         end
%     end
% end
% 
% imgs=imgs+1;
% f=figure(imgs);
% f.Position = [20 200 1500 400];
% 
% for j=1:3
%     subplot(1,3,j);
%     for i=1:N
%         plot(t+t_step*(i-1),reshape(VRP_trajectory(j,i,:),1,[]),colors(j));
%         hold on;
%     end
%     grid on;
%     hold off;
%     if j==1
%         title("VRP desired trajectory x component");
%     elseif j==2
%         title("VRP desired trajectory y component");
%     else
%         title("VRP desired trajectory z component");
%     end
% end
%% DCM desired trajectory of paper 2 (put this before
% 
% DCM_trajectory = zeros(3,N,length(t));
% 
% for i=1:N
%     j=N+1-i;
%     if i == 1
%         DCM_trajectory(:,j,length(t)) = VRP_trajectory(:,j,length(t));
%     else
%         DCM_trajectory(:,j,length(t)) = DCM_trajectory(:,j+1,1);
%     end
%     for k = 1:length(t)
%         alpha = 1-t(k)/t_step - b/t_step + exp((t(k)-t_step)/b)*b/t_step;
%         beta = t(k)/t_step + b/t_step - exp((t(k)-t_step)/b)*(1+b/t_step);
%         gamma = exp((t(k)-t_step)/b);
%         DCM_trajectory(:,j,k) = alpha*VRP_trajectory(:,j,1) +...
%             beta*VRP_trajectory(:,j,length(t))+...
%             gamma*DCM_trajectory(:,j,length(t));
%     end
% end
% 
% imgs=imgs+1;
% f=figure(imgs);
% f.Position = [20 200 1500 400];
% 
% for j=1:3
%     subplot(1,3,j);
%     for i=1:N
%         plot(t+t_step*(i-1),reshape(DCM_trajectory(j,i,:),1,[]),colors(j));
%         hold on;
%     end
%     grid on;
%     hold off;
%     if j==1
%         title("DCM desired x component");
%     elseif j==2
%         title("DCM desired y component");
%     else
%         title("DCM desired z component");
%     end
% end

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

imgs=imgs+1;
f=figure(imgs);
f.Position = [20 200 1500 400];

for j=1:3
    subplot(1,3,j);
    for i=1:N
        plot(t+t_step*(i-1),reshape(VRP_trajectory(j,i,:),1,[]),colors(j));
        hold on;
    end
    grid on;
    hold off;
    if j==1
        title("VRP desired trajectory x component");
    elseif j==2
        title("VRP desired trajectory y component");
    else
        title("VRP desired trajectory z component");
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

imgs=imgs+1;
f=figure(imgs);
f.Position = [20 200 1500 400];

for j=1:3
    subplot(1,3,j);
    for i=1:N
        plot(t+t_step*(i-1),reshape(DCM_trajectory(j,i,:),1,[]),colors(j));
        hold on;
    end
    grid on;
    hold off;
    if j==1
        title("DCM desired x component");
    elseif j==2
        title("DCM desired y component");
    else
        title("DCM desired z component");
    end
end

%% plot on plane

imgs=imgs+1;
f=figure(imgs);
f.Position = [20 200 1500 400];
for i=1:N
    hold on;
    plot(reshape(VRP_trajectory(1,i,:),1,[]),reshape(VRP_trajectory(2,i,:),1,[]),colors(3)); 
    plot(reshape(DCM_trajectory(1,i,:),1,[]),reshape(DCM_trajectory(2,i,:),1,[]),colors(1));
end
legend('VRP_d','DCM_d');
axis equal
xlim([0,deltax*4]);
grid on;
hold off;

title("VRP and DCM desired on 2d plane");

    

%% DCM disturbed
b_dis = 0.9*b;
b = b_dis;

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
b = b/0.9;

imgs=imgs+1;
f=figure(imgs);
f.Position = [20 200 1500 400];

for j=1:3
    subplot(1,3,j);
    hold on;
    for i=1:N
        plot(t+t_step*(i-1),reshape(DCM_disturbed(j,i,:),1,[]),colors(j));
        plot(t+t_step*(i-1),reshape(DCM_trajectory(j,i,:),1,[]),[colors(j),'--']);
    end
    grid on;
    hold off;
    if j==1
        title("DCM disturbed vs desired x component");
    elseif j==2
        title("DCM disturbed vs desired y component");
    else
        title("DCM disturbed vs desired z component");
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
DCM_adj_dis=zeros(2,N/2,T_iter/Ts);
DCM = zeros(2,N/2,T_iter/Ts);

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

c_dis = b_dis/T_ILC;
c_inv_dis = T_ILC/b_dis;
alpha_0_dis = 1 - c_dis + exp(-c_inv_dis)*c_dis;
beta_0_dis = c_dis - exp(-c_inv_dis)*(1+c_dis);
gamma_0_dis = exp(-c_inv_dis);

zeta = 1+k_DCM*b;
e_k_DCM_T = exp(-k_DCM*T_iter);

a_3 = -kl*zeta;
a_4 = kf+kl*(zeta*alpha_0-1);
a_2 = alpha_0*a_4 - e_k_DCM_T*...
    (alpha_0+a_4*(beta_0+gamma_0*alpha_0));
a_1 = e_k_DCM_T+(alpha_0-e_k_DCM_T*...
    (beta_0+gamma_0*alpha_0))*a_3;
                
syms s
D_T = -eta*e_k_DCM_T*int(exp(k_DCM*s), s, 0, T_iter);

for i=1:N/2 %1
    for k = 1:N_k %8
        for index_t_p = 1:N_ILC
            index_tt = (k-1)*N_ILC+index_t_p; %simulation step in current iteration
            t_passed = (index_t_p-1)*Ts; %ILC time %9
            tt = (index_tt-1)*Ts; %global time %2
            
            phi_c = 1+floor((i-1+tt/T_iter)*4);
            if (i==1 && index_tt==1) %3
                Vl=VRP_traj_des(1:2,i,1:N_ILC:N_iter-N_ILC+1); %4
                VRP_adj(:,i,index_tt)=Vl(:,1); %5
                DCM_adj(:,i,index_tt)=DCM_traj_des(1:2,i,index_tt); %6
                DCM_adj_dis(:,i,index_tt)=DCM_traj_des(1:2,i,index_tt);
            else %7
                if (t_passed == 0) %10
                    if (phi_c + 4 <= n_phi) %11
                        VRP_adj(:,i,index_tt)=Vl(:,2); %12
                        decreased = 0;
                        if index_tt>1
                            index_tt = index_tt-1;
                            decreased = 1;
                        end
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
                    DCM_f = DCM_traj_des(1:2,i+1,k+1); %20
                else %21
                    DCM_f = Vl(:,end); %22
                end
                DCM_mid = DCM_f; %23
                for j=N_k:-1:3 %24
                    DCM_mid = alpha_0*Vl(:,j-1)+...
                        beta_0*Vl(:,j)+gamma_0*DCM_mid; %25
                    DCM_mid_dis = alpha_0_dis*Vl(:,j-1)+...
                        beta_0_dis*Vl(:,j)+gamma_0_dis*DCM_mid;
                end
                alpha_t = 1-t_passed/T_ILC - c + exp((t_passed-T_ILC)/b)*c;
                beta_t = t_passed/T_ILC + c - exp((t_passed-T_ILC)/b)*(1+c);
                gamma_t = exp((t_passed-T_ILC)/b);
                
                alpha_t_dis = 1-t_passed/T_ILC - c_dis + exp((t_passed-T_ILC)/b_dis)*c_dis;
                beta_t_dis = t_passed/T_ILC + c_dis - exp((t_passed-T_ILC)/b_dis)*(1+c_dis);
                gamma_t_dis = exp((t_passed-T_ILC)/b_dis);
                
                DCM_adj(:,i,index_tt) = alpha_t*Vl(:,1)+...
                    beta_t*Vl(:,2)+gamma_t*DCM_mid; %26
                
                % CoM commanded from VRP_adj through DCM from the
                % controller
                
                % FFT1 = (1 + kl*zeta*beta_0)*VRP_traj_des(:,i+1,1) +...
                    % (kl-kf)*VRP_traj_des(:,i,1) +...
                    % kl*gamma_0*zeta*DCM_traj_des(:,i+1,1)
                % FFT2 = beta_0*VRP_traj_des(:,i+2,1) +...
                    % gamma_0*DCM_traj_des(:,i+2,1)
                % FFT3 = ?
                % FFT4 = alpha_0*FFT1 + FFT2 - e_k_DCM_T*FFT3
                
%                 DCM(:,i+1,1) = a_2*VRP_adj(:,i,1)+...
%                     a_1*DCM(:,i,1)-D_T/b; %+FFT4
                
                
                % CoM actual measurement
                
                % DCM measurement from CoM
                
                % disturbed values
                DCM_adj_dis(:,i,index_tt) = alpha_t_dis*Vl(:,1)+...
                    beta_t_dis*Vl(:,2)+gamma_t_dis*DCM_mid_dis;
                VRP_c_traj(1:2,i,index_tt) = VRP_adj(:,i,index_tt) +...
                    zeta*(DCM_adj_dis(:,i,index_tt)-DCM_adj(:,i,index_tt));
            end
        end
    end
end
toc

%% Error measurements
e_c = zeros(2,N/2);
e_c_no_ILC = zeros(2,N/2);
VRP_c_no_ILC = VRP_traj_des(1:2,:,:) + (1 + k_DCM*b)*(DCM_adj_dis - DCM_traj_des(1:2,:,:));

for i = 1:N/2
    e_c(:,i) = sum(abs(VRP_traj_des(1:2,i,:)-VRP_c_traj(1:2,i,:))*Ts,3)/T_iter;
    e_c_no_ILC(:,i) = sum(abs(VRP_traj_des(1:2,i,:)-VRP_c_no_ILC(1:2,i,:))*Ts,3)/T_iter;
end

e_c_norm = vecnorm(e_c, 2, 1);
e_c_no_ILC_norm = vecnorm(e_c_no_ILC, 2, 1);

imgs=imgs+1;
f=figure(imgs);
f.Position = [20 200 1800 400];

for j=1:3
    subplot(1,3,j);
    hold on;
    if j<3
        plot(1:N/2,e_c(j,:), [colors(1), '-o']);
        plot(1:N/2,e_c_no_ILC(j,:), [colors(2), '--o']);
    else
        plot(1:N/2,e_c_norm, [colors(1), '-o']);
        plot(1:N/2,e_c_no_ILC_norm, [colors(2), '--o']);
    end
    
    legend('e_{c,ILC}','e_{c,no-ILC}');
    
    grid on;
    hold off;
    if j == 1
        title("Commanded Error with and without ILC on x axis");
    elseif j == 2
        title("Commanded Error with and without ILC on y axis");
    else
        title("Commanded Error norm with and without ILC");
    end
end

%% DS SS (from paper 1)

% choose desired T_DS duration (fraction of t_step)
% delta_T_DS_ini = alpha_DS_ini * T_DS % alpha=0.5
% delta_T_DS_ini = (1-alpha_DS_ini) * T_DS % alpha=0.5

% refine DCM trajectory:
% for each step i:
    % DS phase
    % DCM_iniDS_i = VRP_i-1 +...
        % exp(-eta*delta_T_DS_ini)*(DCM_ini_i - VRP_i-1) 
    % DCM_eoDS_i = VRP_i + exp(eta*delta_T_DS_end)*(DCM_ini_i - VRP_i)
    
    % interpolate DCM using
    % P =   [1/T_DS^3, 1/T_DS^2, -2/T_DS^3, 1/T_DS^2;...
          %  -3/T_DS^2, -2/T_DS, 3/T_DS^2, -1/T_DS;...
          %  0, 1, 0, 0;...
          %  1, 0, 0, 0]*...
          % [DCM_iniDS_i, DCM_iniDS_i_dot, DCM_eoDS_i, DCM_eoDS_i_dot]'
    % for t from 0 to T_DS:
        % [DCM_DS_i(t), DCM_DS_i_dot(t)]' = [t^3, t^2, t, 1;...
                                       % 3*t^2, 2*t, 1, 0] * P
        % VRP trajectory computed using 
        % VRP_DS_i(t) = DCM_DS_i(t) - b*DCM_DS_i_dot(t)
                                       
    % SS phase
    % VRP_i constant
    % DCM_i interpolated exponentially % both already computed
    
    
% results: refined VRP and DCM trajectories


%% CoM

%% Final plots

imgs=imgs+1;
f=figure(imgs);
f.Position = [20 100 1300 600];

for j=1:4
    subplot(2,2,j);
    hold on;
    for i=1:N/2
        if j<3
            plot(Tg((i-1)*fix(T_iter/Ts)+1:i*fix(T_iter/Ts)),reshape(VRP_c_traj(j,i,:),1,[]),colors(j));
            plot(Tg((i-1)*fix(T_iter/Ts)+1:i*fix(T_iter/Ts)),reshape(VRP_adj(j,i,:),1,[]),[colors(j),'--']);
            legend('VRP_{c}','VRP_{adj}');
        else
            j = j-2;
            plot(Tg((i-1)*fix(T_iter/Ts)+1:i*fix(T_iter/Ts)),reshape(DCM_adj(j,i,:),1,[]),colors(j));
            plot(Tg((i-1)*fix(T_iter/Ts)+1:i*fix(T_iter/Ts)),reshape(DCM_traj_des(j,i,:),1,[]),[colors(j),'--']);
            j = j+2;
            legend('DCM_{adj}','DCM_{des}');
        end
    end

    grid on;
    hold off;
    if j==1
        title("VRP desired vs adjusted trajectory  x component");
    elseif j==2
        title("VRP desired vs adjusted trajectory y component");
    elseif j==3
        title("DCM desired vs adjusted trajectory x component");
    elseif j==4
        title("DCM desired vs adjusted trajectory y component");
    end
end
