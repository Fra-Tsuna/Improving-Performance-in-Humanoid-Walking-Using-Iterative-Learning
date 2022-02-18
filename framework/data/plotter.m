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
hold off
subplot(1,3,2)
hold on
plot(left_desired(:,2))
plot(left_current(:,2))
hold off
subplot(1,3,3)
hold on
plot(left_desired(:,3))
plot(left_current(:,3))
hold off

figure(2)
subplot(1,3,1)
hold on
plot(right_desired(:,1))
plot(right_current(:,1))
hold off
subplot(1,3,2)
hold on
plot(right_desired(:,2))
plot(right_current(:,2))
hold off
subplot(1,3,3)
hold on
plot(right_desired(:,3))
plot(right_current(:,3))
hold off
%%
figure(3);
subplot(1,3,1)
dcm_trajectory = flip(dcm_trajectory, 1);
hold on
plot(vrp_trajectory(:,1),'LineWidth',lw)
plot(desired(:,1),'LineWidth',lw)
plot(VRPCommanded(:,1),'LineWidth',lw)
plot(current(:,1),'LineWidth',lw)
legend('VRP_d', 'VRP_l', 'VRP_c', 'VRP_m')
hold off
title("DCM and VRP desired x component")
ylim([0 10])
subplot(1,3,2)
hold on
plot(vrp_trajectory(:,2),'LineWidth',lw)
plot(desired(:,2),'LineWidth',lw)
plot(VRPCommanded(:,2),'LineWidth',lw)
plot(current(:,2),'LineWidth',lw)
legend('VRP_d', 'VRP_l', 'VRP_c', 'VRP_m')
hold off
ylim([-1 1])
title("DCM and VRP desired y component")
subplot(1,3,3)
hold on
plot(vrp_trajectory(:,3),'LineWidth',lw)
plot(desired(:,3),'LineWidth',lw)
plot(VRPCommanded(:,3),'LineWidth',lw)
plot(current(:,3),'LineWidth',lw)
legend('VRP_d', 'VRP_l', 'VRP_c', 'VRP_m')
% plot(left_z, Color='r')
% plot(right_z, Color='b')
hold off
title("DCM and VRP desired z component")
% ylim([0 0.71])

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
figure(5)
hold on
plot3(current(:,1), current(:,2), current(:,3),'LineWidth',lw)
plot3(com_trajectory(:,1), com_trajectory(:,2), com_trajectory(:,3),'LineWidth',lw)
hold off
axis equal
xlim([-1.5 5])
ylim([-0.5 0.5])
zlim([0 1])
%% 
f = figure(3);
subplot(1,3,1)
hold on
plot(current(:,1),'LineWidth',lw)
plot(com_trajectory(:,1),'LineWidth',lw)
legend('CoM', 'CoM_d')
hold off
ylim([0 8])
title("CoM current and desired x component")
subplot(1,3,2)
hold on
plot(current(:,2),'LineWidth',lw)
plot(com_trajectory(:,2),'LineWidth',lw)
legend('CoM', 'CoM_d')
hold off
ylim([-1 1])
title("CoM current and desired y component")
subplot(1,3,3)
hold on
plot(current(:,3),'LineWidth',lw)
plot(com_trajectory(:,3),'LineWidth',lw)
ylim([0, 0.8])
legend('CoM', 'CoM_d')
% plot(left_z, Color='r')
% plot(right_z, Color='b')
hold off
title("CoM current and desired z component")

%%
e_c = zeros(2,10);
e_m = zeros(2,10);
e_c_no_ILC = zeros(2,10);

for i = 1:10
    e_c(:,i) = sum(abs(vrp_trajectory(((i-1)*240+1):(i*240),1:2)-VRPCommanded(((i-1)*240+1):(i*240),1:2))*0.01,1)/2.4;
    e_m(:,i) = sum(abs(vrp_trajectory(((i-1)*240+1):(i*240),1:2)-current(((i-1)*240+1):(i*240),1:2))*0.01,1)/2.4;
    % e_c_no_ILC
end

e_c_norm = vecnorm(e_c, 2, 1);
e_m_norm = vecnorm(e_m, 2, 1);
% e_c_no_ILC_norm = vecnorm(e_c_no_ILC, 2, 1);

figure(6);

for j=1:3
    subplot(1,3,j);
    hold on;
    if j<3
%         plot(1:10,e_c(j,:), 'b-o','LineWidth',lw);
        plot(1:10,e_m(j,:), 'b-o','LineWidth',lw);
%         plot(1:10,e_c_no_ILC(j,:), [colors(2), '--o']);
    else
%         plot(1:10,e_c_norm, 'b-o','LineWidth',lw);
        plot(1:10,e_m_norm, 'b-o','LineWidth',lw);
%         plot(1:10,e_c_no_ILC_norm, [colors(2), '--o']);
    end
    
    legend('e_{m,ILC}');
    
    grid on;
%     hold off;
    if j == 1
        title("Commanded Error with and without ILC on x axis");
    elseif j == 2
        title("Commanded Error with and without ILC on y axis");
    else
        title("Commanded Error norm with and without ILC");
    end
end