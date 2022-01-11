close all; clear; clc;

%% Constants definition
m = 60; %kg
z_com = 0.8; %average height CoM, m.
g0 = 9.81; %m/s**2
g=[0; 0; -g0];

b=sqrt(z_com/g0); %s, time constant.
eta=1/b;

N=5; % # foot positions.
t_step=1;

deltax=0.3;
deltay=0.25;

t=(0:0.01:t_step); 
T=N*t_step;

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

%% DCM reference generation and plot

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
        title("DCM x component");
    elseif j==2
        title("DCM y component");
    else
        title("DCM z component");
    end
end

%% Derivative of DCM reference generation and plot

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
        title("dotDCM x component");
    elseif j==2
        title("dotDCM y component");
    else
        title("dotDCM z component");
    end
end

%% CoM trajectory
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
        title("CoM x component");
    elseif j==2
        title("CoM y component");
    else
        title("CoM z component");
    end
end

%% dot_CoM trajectory

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
        title("dot CoM x component");
    elseif j==2
        title("dot CoM y component");
    else
        title("dot CoM z component");
    end
end

%% VRP trajectory

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
        title("VRP trajectory x component");
    elseif j==2
        title("VRP trajectory y component");
    else
        title("VRP trajectory z component");
    end
end

