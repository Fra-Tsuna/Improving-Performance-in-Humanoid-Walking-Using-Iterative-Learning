load("selectedFootsteps.txt")
load("footstepPlan.txt")
load("zmpx.txt")
load("zmpy.txt")
load left_z.txt
load right_z.txt
load vrp_trajectory
load dcm_trajectory
load com_trajectory
load VRPCommanded
% load current.comVel 
% load desired.comVel
load current.vrpPos
% load desired.comPos
% load current.comPos
load desired.vrpPos
lw = 1;
%%
load left_desired
load left_current
load right_current
load right_desired
figure(1)
subplot(1,3,1)
hold on
plot(left_desired(:,1))
plot(left_current(:,1))
ylim([0,10])
hold off
subplot(1,3,2)
hold on
plot(left_desired(:,2))
plot(left_current(:,2))
ylim([-0.5,0.5])
hold off
subplot(1,3,3)
hold on
plot(left_desired(:,3))
plot(left_current(:,3))
ylim([-0.5,0.5])
hold off

figure(2)
subplot(1,3,1)
hold on
plot(right_desired(:,1))
plot(right_current(:,1))
ylim([0,10])
hold off
subplot(1,3,2)
hold on
plot(right_desired(:,2))
plot(right_current(:,2))
ylim([-0.5,0.5])
hold off
subplot(1,3,3)
hold on
plot(right_desired(:,3))
plot(right_current(:,3))
ylim([-0.5,0.5])
hold off
%%
figure(3);
% subplot(1,3,1)
hold on
plot(vrp_trajectory(:,1),'LineWidth',lw)
plot(desired(:,1),'LineWidth',lw)
plot(VRPCommanded(:,1),'LineWidth',lw)
plot(current(:,1),'LineWidth',lw)
% title("DCM and VRP desired x component")
hold on
start = 1000;
plot([1150, 750], [2.3, 2.8], 'r','LineWidth',lw)
rectangle('position', [start 1.5 300 0.8], 'EdgeColor', 'r')
grid on
box on
ylim([-0.2 6])
xlim([1 2480])
legend({'VRP_d', 'VRP_l', 'VRP_c', 'VRP_m'},'FontSize',16)
hold off
axes('Position',[.15 .48 .37 .43])
hold on
hold on
plot(vrp_trajectory(start:start+300,1),'LineWidth',2*lw)
plot(desired(start:start+300,1),'LineWidth',2*lw)
plot(VRPCommanded(start:start+300,1),'LineWidth',2*lw)
plot(current(start:start+300,1),'LineWidth',2*lw)
xlim([1 300])
grid on
box on
set(gca,'Yticklabel',[]) 
set(gca,'Xticklabel',[])
hold off

%%
figure(4);
% subplot(1,3,1)
hold on
plot(vrp_trajectory(:,2),'LineWidth',lw)
plot(desired(:,2),'LineWidth',lw)
plot(VRPCommanded(:,2),'LineWidth',lw)
plot(current(:,2),'LineWidth',lw)
% title("DCM and VRP desired x component")
hold on
start = 1500;
len = 480;
plot([start+200, 750], [0.15, 0.3], 'r','LineWidth',lw)
rectangle('position', [start -0.15 len 0.3], 'EdgeColor', 'r',LineWidth=lw)
grid on
box on
ylim([-0.2 0.6])
xlim([1 2480])
legend({'VRP_d', 'VRP_l', 'VRP_c', 'VRP_m'},'FontSize',16)
hold off
axes('Position',[.15 .48 .37 .43])
hold on
hold on
plot(vrp_trajectory(start:start+len,2),'LineWidth',2*lw)
plot(desired(start:start+len,2),'LineWidth',2*lw)
plot(VRPCommanded(start:start+len,2),'LineWidth',2*lw)
plot(current(start:start+len,2),'LineWidth',2*lw)
xlim([1 len])
grid on
box on
set(gca,'Yticklabel',[]) 
set(gca,'Xticklabel',[])
hold off
%%
figure(5)
% subplot(1,3,3)
hold on
plot(vrp_trajectory(:,3),'LineWidth',lw)
plot(desired(:,3),'LineWidth',lw)
plot(VRPCommanded(:,3),'LineWidth',lw)
plot(current(:,3),'LineWidth',lw)
legend('VRP_d', 'VRP_l', 'VRP_c', 'VRP_m')
% plot(left_z, Color='r')
% plot(right_z, Color='b')
hold off
% title("DCM and VRP desired z component")
ylim([0 1])

%%
figure(5);
hold on
plot3(com_trajectory(:,1), com_trajectory(:,2), com_trajectory(:,3),'LineWidth',lw)
plot3(dcm_trajectory(:,1), dcm_trajectory(:,2), dcm_trajectory(:,3),'LineWidth',lw)
plot3(vrp_trajectory(:,1), vrp_trajectory(:,2), vrp_trajectory(:,3),'LineWidth',lw)
hold off
axis equal
xlim([-1.5 5])
ylim([-0.5 0.5])
zlim([0 1])

%%
figure(6)
hold on
plot3(current(:,1), current(:,2), current(:,3),'LineWidth',lw)
plot3(com_trajectory(:,1), com_trajectory(:,2), com_trajectory(:,3),'LineWidth',lw)
hold off
axis equal
xlim([-1.5 5])
ylim([-0.5 0.5])
zlim([0 1])
%% 
figure(3);
% subplot(1,3,1)
hold on
plot(com_trajectory(:,1),'LineWidth',lw)
plot(current(:,1),'LineWidth',lw)
% title("DCM and VRP desired x component")
hold on
start = 1000;
plot([1150, 750], [2.3, 2.8], 'r','LineWidth',lw)
rectangle('position', [start 1.5 300 0.8], 'EdgeColor', 'r')
grid on
box on
ylim([-0.2 6])
xlim([1 2700])
legend({'CoM_d', 'CoM_m'},'FontSize',16)
hold off
axes('Position',[.15 .48 .37 .43])
hold on
hold on
plot(com_trajectory(start:start+300,1),'LineWidth',2*lw)
plot(current(start:start+300,1),'LineWidth',2*lw)
xlim([1 300])
grid on
box on
set(gca,'Yticklabel',[]) 
set(gca,'Xticklabel',[])
hold off

%%
figure(3);
% subplot(1,3,1)
hold on
plot(com_trajectory(:,2),'LineWidth',lw)
plot(current(:,2),'LineWidth',lw)
% title("DCM and VRP desired x component")
hold on
start = 1500;
len = 480;
plot([start+200, 750], [0.15, 0.3], 'r','LineWidth',lw)
rectangle('position', [start -0.15 len 0.3], 'EdgeColor', 'r',LineWidth=lw)
grid on
box on
ylim([-0.2 0.6])
xlim([1 2700])
legend({'CoM_d', 'CoM_m'},'FontSize',16)
hold off
axes('Position',[.15 .48 .37 .43])
hold on
hold on
plot(com_trajectory(start:start+len,2),'LineWidth',2*lw)
plot(current(start:start+len,2),'LineWidth',2*lw)
xlim([1 len])
grid on
box on
set(gca,'Yticklabel',[]) 
set(gca,'Xticklabel',[])
hold off
%%
e_c = zeros(2,10);
e_m = zeros(2,10);
% e_m_no_ILC = zeros(2,10);

for i = 1:10
%     e_c(:,i) = sum(abs(vrp_trajectory(((i-1)*240+1):(i*240),1:2)-VRPCommanded(((i-1)*240+1):(i*240),1:2))*0.01,1)/2.4;
%     e_m_no_ILC(:,i) = sum(abs(vrp_trajectory(((i-1)*240+1):(i*240),1:2)-current(((i-1)*240+1):(i*240),1:2))*0.01,1)/2.4;
    e_m(:,i) = sum(abs(vrp_trajectory(((i-1)*240+1):(i*240),1:2)-current(((i-1)*240+1):(i*240),1:2))*0.01,1)/2.4;% e_c_no_ILC
end

% e_m_norm_no_ILC = vecnorm(e_m_no_ILC, 2, 1);
e_m_norm = vecnorm(e_m, 2, 1);
% e_c_no_ILC_norm = vecnorm(e_c_no_ILC, 2, 1);

figure(7);


hold on

plot(1:10,e_m_norm, '-o','LineWidth',lw);
plot(1:10,e_m_norm_no_ILC, '-o','LineWidth',lw);
legend({'e_{m,ILC}', 'e_{m,NO-ILC}'}, FontSize=16);
xlim([1,10])
ylim([0 0.1])
box on
grid on
hold off

%%
f = figure(8);
f.Position = [0 0 1920 540];
load acceleration
plot(acceleration(:,2),LineWidth=2*lw)
xlim([1,2700])
ylim([-22 22])
grid on
box on
legend({'wind force'},FontSize=16)