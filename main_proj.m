close all; clear; clc;

%% Constants Definition
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

t=[0:0.01:t_step]; 
T=N*t_step;

colors=['r','g','b'];

%% Footprints Generation

%-robot feet are point feet
%-constant angular momentum L, dL=0.
%-no double support phase, instantaneous transition between left and right SS 
%-no impacts during transitions

footprints=zeros(3,N);

footprint0_sx = [0,deltay,0]';
footprint0_dx = [0,0,0]';  


for i=1:N
    if i==1
        footprints(:,i)=footprint0_dx;
    elseif mod(i,2)~=0
        footprints(:,i)=[deltax*(i-1); 0; 0];
    else
        footprints(:,i)=[deltax*(i-1); deltay; 0];
    end
end

%% DMC reference generation and plot

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

for j=1:3
    figure(j);
    for i=1:N
        plot(t+t_step*(i-1),reshape(DCM_trajectories(j,i,:),1,[]),colors(j));
        hold on;
    end
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

for j=1:3
    figure(j+3);
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
    hold off;
    if j==1
        title("dotDCM x component");
    elseif j==2
        title("dotDCM y component");
    else
        title("dotDCM z component");
    end
end

%% CoM reference generation and plot