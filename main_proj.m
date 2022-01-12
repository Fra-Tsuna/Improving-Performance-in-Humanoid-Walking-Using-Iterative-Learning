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

R_delta=eye(2);

kl=1;
kf=1;
k_DCM=4;

Ts=0.001;
T_ILC=0.01;
T_iter=2*t_step;

t=(0:Ts:t_step-Ts); 
T=N*t_step;

Tg=(0:Ts:T-Ts);

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
    elseif mod(i,2)~=0
        footprints(:,i)=[deltax*(i-1); 0; rand*max_height];
    else
        footprints(:,i)=[deltax*(i-1); deltay; rand*max_height];
    end
end

%% DCM "measured"

eCMPs = footprints;
ZMPs = footprints;

VRP_des = footprints + [0; 0; z_com];

DCM_eos_des = zeros(3,N);

for i=1:N
    j=N+1-i;
    if i==1
        DCM_eos_des(:,j) = VRP_des(:,j);
    else
    DCM_eos_des(:,j)= VRP_des(:,j+1) +... 
        exp(-eta*t_step)*(DCM_eos_des(:,j+1) - VRP_des(:,j+1));
    end
end

DCM_trajectories=zeros(3,N,length(t));

for k=1:length(t)
    for i=1:N
        DCM_trajectories(:,i,k)=VRP_des(:,i)+exp(eta*(t(k)-t_step))*...
              (DCM_eos_des(:,i)-VRP_des(:,i));
    end
end

imgs=imgs+1;
f=figure(imgs);
f.Position = [20 200 1500 400];

for j=1:3
    subplot(1,3,j);
    for i=1:N
        plot(t+t_step*(i-1),reshape(DCM_trajectories(j,i,:),1,[]),colors(j));
        hold on;
    end
    grid on;
    hold off;
    if j==1
        title("DCM measured x component");
    elseif j==2
        title("DCM measured y component");
    else
        title("DCM measured z component");
    end
end

%% dot DCM "measured"

dot_DCM_trajectories=zeros(3,N,length(t));
for k=1:length(t)
    for i=1:N
        dot_DCM_trajectories(:,i,k)=eta*exp(eta*(t(k)-t_step))*...
              (DCM_eos_des(:,i)-VRP_des(:,i));
    end
end

imgs=imgs+1;
f=figure(imgs);
f.Position = [20 200 1500 400];

for j=1:3
    subplot(1,3,j);
    for i=1:N
        tx=t+t_step*(i-1);
        plot(tx,reshape(dot_DCM_trajectories(j,i,:),1,[]),colors(j));
        hold on;
        if(i<N)
            y1=dot_DCM_trajectories(j,i,length(tx));
            y2=dot_DCM_trajectories(j,i+1,1);
            plot([tx(end) tx(end)],[y1 y2],colors(j));
            hold on;
        end
    end
    grid on;
    hold off;
    if j==1
        title("dotDCM measured x component");
    elseif j==2
        title("dotDCM measured y component");
    else
        title("dotDCM measured z component");
    end
end

%% CoM "measured"
%Euler integration

CoM=zeros(3,N,length(t));

prev=[0, deltay/2, z_com]';
prevDCM=DCM_trajectories(:,1,1);

for i=1:N
    if i > 1
        prev = CoM(:,i-1,length(t));
        prevDCM = DCM_trajectories(:,i-1,length(t));
    end
    for k=1:length(t)
        if k==1
           CoM(:,i,k) = prev;
        else
            prev = CoM(:,i,k-1);
            prevDCM = DCM_trajectories(:,i,k-1);
            CoM(:,i,k) = prev - eta*(prev - prevDCM)*t(2);
        end
    end
end

imgs=imgs+1;
f=figure(imgs);
f.Position = [20 200 1500 400];

for j=1:3
    subplot(1,3,j);
    for i=1:N
        plot(t+t_step*(i-1),reshape(CoM(j,i,:),1,[]),colors(j));
        hold on;
    end
    grid on;
    hold off;
    if j==1
        title("CoM x measured component");
    elseif j==2
        title("CoM y measured component");
    else
        title("CoM z measured component");
    end
end

%% dot CoM "measured"

dot_CoM = zeros(3,N,length(t));

for i=1:N
    for k=1:length(t)
        dot_CoM(:,i,k) = -eta*(CoM(:,i,k)-DCM_trajectories(:,i,k));
    end
end

imgs=imgs+1;
f=figure(imgs);
f.Position = [20 200 1500 400];

for j=1:3
    subplot(1,3,j);
    for i=1:N
        plot(t+t_step*(i-1),reshape(dot_CoM(j,i,:),1,[]),colors(j));
        hold on;
    end
    grid on;
    hold off;
    if j==1
        title("dot CoM measured x component");
    elseif j==2
        title("dot CoM measured y component");
    else
        title("dot CoM measured z component");
    end
end

%% VRP desired trajectory

VRP_trajectory = zeros(3,N,length(t));

for i=1:N
    for k=1:length(t)
        if i < N
            VRP_trajectory(:,i,k) = (1-t(k)/t_step)*VRP_des(:,i) + (t(k)/t_step)*VRP_des(:,i+1);
        else
            VRP_trajectory(:,i,k) = VRP_des(:,i);
        end
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
%% DCM desired trajectory

DCM_trajectory = zeros(3,N,length(t));

for i=1:N
    j=N+1-i;
    if i == 1
        DCM_trajectory(:,j,length(t)) = VRP_trajectory(:,j,length(t));
    else
        DCM_trajectory(:,j,length(t)) = DCM_trajectory(:,j+1,1);
    end
    for k = 1:length(t)
        alpha = 1-t(k)/t_step - b/t_step + exp((t(k)-t_step)/b)*b/t_step;
        beta = t(k)/t_step + b/t_step - exp((t(k)-t_step)/b)*(1+b/t_step);
        gamma = exp((t(k)-t_step)/b);
        DCM_trajectory(:,j,k) = alpha*VRP_trajectory(:,j,1) +...
            beta*VRP_trajectory(:,j,length(t))+...
            gamma*DCM_trajectory(:,j,length(t));
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

%% VRP_OILC

VRP_traj_des=reshape(reshape(VRP_trajectory,3,[]),3,N/2,[]);
DCM_traj_des=reshape(reshape(DCM_trajectory,3,[]),3,N/2,[]);
VRP_c_traj = VRP_trajectory + (1 + k_DCM*b)*(DCM_trajectories - DCM_trajectory);
VRP_c_traj = reshape(reshape(VRP_c_traj,3,[]),3,N/2,[]);

VRP_adj=zeros(2,N/2,T_iter/Ts);
DCM_adj=zeros(2,N/2,T_iter/Ts);

phi_c=1;
n_phi=N; %Numero transizioni = 2 transizione per iterazione

for s=1:length(Tg)
    i=floor(Tg(s)/T_iter)+1;
    tt=mod(Tg(s),T_iter);
    phi_c = 1+floor((i-1+tt/T_iter)*2);
    if (i==1 && tt==0)
        Vl=VRP_traj_des(1:2,i,1:T_ILC/Ts:(T_iter-T_ILC)/Ts+1);
        VRP_adj(:,i,tt/Ts+1)=Vl(:,1);
        DCM_adj(:,i,tt/Ts+1)=DCM_traj_des(1:2,i,tt/Ts+1);
    else
        tt_fixed = floor(tt/Ts)+1;
        k = floor(tt/T_ILC);
        t_passed = mod(tt,T_ILC);
        if (t_passed == 0)
            if (phi_c + 2 <= n_phi)
                VRP_adj(:,i,tt_fixed)=Vl(:,2);
                decreased = 0;
                if tt_fixed>1
                    tt_fixed = tt_fixed-1;
                    decreased = 1;
                end
                    VRP_adj(:,i+1,tt_fixed)=VRP_traj_des(1:2,i+1,tt_fixed)+...
                        kf*R_delta*(VRP_adj(:,i,tt_fixed)-VRP_traj_des(1:2,i,tt_fixed))+...
                        kl*R_delta*(VRP_traj_des(1:2,i,tt_fixed)-VRP_c_traj(1:2,i,tt_fixed));
                    Vl_l=[Vl(:,2:end),VRP_adj(:,i+1,tt_fixed)];
                if decreased == 1
                    tt_fixed = tt_fixed+1;
                end
            else
                Vl_l=[Vl(:,2:end),Vl(:,end)];
            end
            Vl=Vl_l;
        end
        VRP_adj(:,i,tt_fixed)=(1-t_passed/T_ILC)*Vl(:,1)+t_passed/T_ILC*Vl(:,2);
        if phi_c + 2 <= n_phi
            DCM_f = DCM_traj_des(1:2,i+1,k+1);
        else
            DCM_f = Vl(:,end);
        end
        DCM_mid = DCM_f;
        alpha_0 = 1 - b/T_ILC + exp(-T_ILC/b)*b/T_ILC;
        beta_0 = b/T_ILC - exp(-T_ILC/b)*(1+b/T_ILC);
        gamma_0 = exp(-T_ILC/b);
        for j=(T_iter/T_ILC):-1:3
            DCM_mid = alpha_0*Vl(:,j-1)+...
                beta_0*Vl(:,j)+gamma_0*DCM_mid;
        end
        alpha_t = 1-t_passed/T_ILC - b/T_ILC + exp((t_passed-T_ILC)/b)*b/T_ILC;
        beta_t = t_passed/T_ILC + b/T_ILC - exp((t_passed-T_ILC)/b)*(1+b/T_ILC);
        gamma_t = exp((t_passed-T_ILC)/b);
        DCM_adj(:,i,tt_fixed) = alpha_t*Vl(:,1)+...
            beta_t*Vl(:,2)+gamma_t*DCM_mid;
    end
end

%% plots

imgs=imgs+1;
f=figure(imgs);
f.Position = [20 200 1500 400];

for j=1:2
    subplot(1,2,j);
    for i=1:N/2
        plot(Tg((i-1)*fix(T_iter/Ts)+1:i*fix(T_iter/Ts)),reshape(DCM_adj(j,i,:),1,[]),colors(j));
        hold on;
    end
    grid on;
    hold off;
    if j==1
        title("VRP adjusted trajectory x component");
    elseif j==2
        title("VRP adjusted trajectory y component");
    end
end
