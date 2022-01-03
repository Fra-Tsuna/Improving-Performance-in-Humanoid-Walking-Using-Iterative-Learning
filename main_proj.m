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

t=[0:0.01:t_step]; 
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
%% CoM from diff eq
% syms CoMx(t) DCMx(t) CoMy(t) DCMy(t) CoMz(t) DCMz(t)
% syms ETA real
% odex = diff(CoMx,t) == -ETA*(CoMx-DCMx);
% odey = diff(CoMy,t) == -ETA*(CoMy-DCMy);
% odez = diff(CoMz,t) == -ETA*(CoMz-DCMz);
% condx = [CoMx(0) == 0];
% condy = [CoMy(0) == deltay/2];
% condz = [CoMz(0) == z_com];
% Solx(t) = dsolve(odex,condx)
% Soly(t) = dsolve(odey,condy)
% Solz(t) = dsolve(odez,condz)

%% CoM reference generation and plot

CoM_trajectories=zeros(3,N,length(t));
CoM_start = [0, deltay/2, z_com]';

for i=1:N
    cumsum = 0;
    if i > 1
        CoM_start = CoM_trajectories(:,i-1,length(t));
    end
    for k=1:length(t)
        CoM_trajectories(:,i,k) = exp(-eta*t(k))*(CoM_start + cumsum);
        cumsum = cumsum + eta*exp(eta*t(k))*DCM_trajectories(:,i,k)*t(2);
    end
end

imgs=imgs+1;
f=figure(imgs);
f.Position = [20 200 1500 400];

for j=1:3
    subplot(1,3,j);
    for i=1:N
        plot(t+t_step*(i-1),reshape(CoM_trajectories(j,i,:),1,[]),colors(j),...
             t+t_step*(i-1),reshape(DCM_trajectories(j,i,:),1,[]),[colors(j),'--']);
        hold on;
    end
    grid on;
    hold off;
    if j==1
        title("CoM(line) vs DCM(dotted) x component");
    elseif j==2
        title("CoM(line) vs DCM(dotted) y component");
    else
        title("CoM(line) vs DCM(dotted) z component");
    end
end

%% dot CoM reference generation and plot

dot_CoM_trajectories=zeros(3,N,length(t));

for i=1:N
    for k=1:length(t)
        dot_CoM_trajectories(:,i,k) = -eta*(CoM_trajectories(:,i,k)-DCM_trajectories(:,i,k));
    end
end

imgs=imgs+1;
f=figure(imgs);
f.Position = [20 200 1500 400];

for j=1:3
    subplot(1,3,j);
    for i=1:N
        plot(t+t_step*(i-1),reshape(dot_CoM_trajectories(j,i,:),1,[]),colors(j));
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

%% Checking CoM consistency
close all;
%controllare se soddisfa eq LIP xc"=eta * (xc - xz)

CoMdotdot_temp=zeros(3,5,100);
for i=1:N
    CoMdotdot_temp(:,i,:)=diff(dot_CoM_trajectories(:,i,:), 1, 3);
end

CoMdotdot=zeros(3,5,101);
CoMdotdot(:,:,[2:101])=CoMdotdot_temp;



% imgs=imgs+1;
% f=figure(imgs);
% f.Position = [20 200 1500 400];

%for j=1:3
%     subplot(1,3,j);
%     for i=1:N
%         plot(t+t_step*(i-1),reshape(100*CoMdotdot(j,i,:),1,[]),colors(j))
%         hold on;
%     end
%     grid on;
%     hold off;
% end
% 
% imgs=imgs+1;
% f=figure(imgs);
% f.Position = [20 200 1500 400];
% 
% for j=1:3
%     subplot(1,3,j);
%     for i=1:N
%         plot(t+t_step*(i-1),reshape((eta*eta)*(CoM_trajectories(j,i,:)-VRP_des(j,i)),1,[]),colors(j))
%         hold on;
%     end
%     grid on;
%     hold off;
% end

CoM=zeros(3,N,length(t));

prev=[0, deltay/2, z_com]';
prevDCM=DCM_trajectories(:,1,1);

for i=1:N
    if i > 1
        prev = CoM(:,i-1,length(t))
        prevDCM = DCM_trajectories(:,i-1,length(t))
    end
    for k=1:length(t)
        if k > 1
            prev = CoM(:,i,k-1);
            prevDCM = DCM_trajectories(:,i,k-1);
        end
        CoM(:,i,k) = prev - eta*(prev - prevDCM)*t(2);
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
end