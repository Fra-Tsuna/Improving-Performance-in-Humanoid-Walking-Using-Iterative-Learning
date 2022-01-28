load("framework/data/selectedFootsteps.txt")
load("framework/data/footstepPlan.txt")
load("framework/data/zmpx.txt")
load("framework/data/zmpy.txt")
load framework/data/left_z.txt
load framework/data/right_z.txt
load framework/data/vrp_trajectory
load framework/data/dcm_trajectory
f = figure(1);
f.Position = [20 200 1500 500];
subplot(1,3,1)
dcm_trajectory = flip(dcm_trajectory, 1);
hold on
plot(dcm_trajectory(:,1))
plot(vrp_trajectory(:,1),Color='r')
legend('DCM', 'VRP')
hold off
title("DCM and VRP desired x component")
% ylim([0 0.71])
subplot(1,3,2)
hold on
plot(dcm_trajectory(:,2))
plot(vrp_trajectory(:,2),Color='r')
legend('DCM', 'VRP')
hold off
ylim([-1 1])
title("DCM and VRP desired y component")
subplot(1,3,3)
hold on
plot(dcm_trajectory(:,3))
plot(vrp_trajectory(:,3),Color='r')
legend('DCM', 'VRP')
% plot(left_z, Color='r')
% plot(right_z, Color='b')
hold off
title("DCM and VRP desired z component")
% ylim([0 0.71])